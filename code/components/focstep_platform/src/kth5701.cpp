#include "kth5701.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "common/foc_utils.h"

static const char *TAG = "KTH5701";

/* 芯片协议规定寄存器号在总线上要左移 2 位 (只读/只写命令字节的低 6 位是寄存器域) */
#define REG_SHIFT(reg) ((uint8_t)((reg) << 2))

KTH5701::KTH5701(i2c_port_t port, gpio_num_t scl_io, gpio_num_t sda_io,
                 uint8_t dev_addr, uint32_t clk_hz)
    : _port(port), _scl(scl_io), _sda(sda_io), _addr(dev_addr), _clk_hz(clk_hz)
{
}

KTH5701::~KTH5701()
{
    if (_installed) {
        deinit();
    }
}

/* ---------------- 底层事务 ----------------
 * 参考实现用 Linux 的 i2c_master_send + i2c_master_recv = **两段独立事务**
 * (各自带 START/STOP), 不是 repeated-start。这里照做。
 * i2c_dev_write/read 经 i2cdev 内部总线锁与设备互斥锁, 故无需自建锁。
 */

esp_err_t KTH5701::tx_write(const uint8_t *buf, size_t len)
{
    if (!_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    /* i2c_dev_write 自带 START...STOP, 与芯片要求的"独立写事务"一致 */
    return i2c_dev_write(&_dev, NULL, 0, buf, len);
}

esp_err_t KTH5701::tx_read(uint8_t *buf, size_t len)
{
    if (!_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    /* i2c_dev_read 自带 START...STOP (独立读事务), 由 i2cdev 填充 7 位地址与读方向 */
    return i2c_dev_read(&_dev, NULL, 0, buf, len);
}

void KTH5701::init()
{
    /* 满足基类接口 (Sensor::init 返回 void)。真正的初始化在 begin()。 */
    (void)begin();
}

esp_err_t KTH5701::begin()
{
    /* i2cdev 全局初始化 (幂等)。总线按需创建: 首个设备 (KTH5701 或 MCP4725)
     * 初始化时建 i2c_master bus, 后续设备复用同一总线 (pin 校验一致)。 */
    esp_err_t ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, TAG, "i2cdev_init failed");

    _dev.port = _port;
    _dev.addr = _addr;
    _dev.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    _dev.cfg.sda_io_num = _sda;
    _dev.cfg.scl_io_num = _scl;
    _dev.cfg.sda_pullup_en = true;
    _dev.cfg.scl_pullup_en = true;
    _dev.cfg.master.clk_speed = _clk_hz;
    ret = i2c_dev_create_mutex(&_dev);
    ESP_RETURN_ON_ERROR(ret, TAG, "i2c_dev_create_mutex failed");

    _installed = true;

    /* 上电/复位后芯片需要 4~10ms 才能响应 (参考实现 mdelay(4), probe 里 mdelay(10)) */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ⚠️ Sensor::init() 是 protected, 这里在子类里调它是合法的。
     *    它内部会连续调 4 次 getSensorAngle() 来消除首帧跳变。 */
    Sensor::init();

    ESP_LOGI(TAG, "init done: addr=0x%02X clk=%" PRIu32 "Hz", _addr, _clk_hz);
    return ESP_OK;
}

void KTH5701::deinit()
{
    /* 从 i2cdev 总线移除设备并释放设备互斥锁。总线句柄由 i2cdev 统一管理,
     * 此处不删除 (其他设备可能仍在用)。 */
    if (_installed) {
        i2c_dev_delete_mutex(&_dev);
        _dev = {};
    }
    _installed = false;
}

/* ---------------- 寄存器层 ---------------- */

esp_err_t KTH5701::reg_read(uint8_t reg, uint16_t *val, uint8_t *status)
{
    uint8_t wbuf[2] = {KTH5701_CMD_READ_REG, REG_SHIFT(reg)};
    uint8_t rbuf[3] = {0};

    esp_err_t ret = tx_write(wbuf, sizeof(wbuf));
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    ret = tx_read(rbuf, sizeof(rbuf));
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }

    _last_status = rbuf[0];
    if (status) {
        *status = rbuf[0];
    }
    if (val) {
        *val = (uint16_t)((rbuf[1] << 8) | rbuf[2]);
    }
    return ESP_OK;
}

esp_err_t KTH5701::reg_write(uint8_t reg, uint16_t val)
{
    /* ⚠️ 字节序很反直觉: [命令, 数据高, 数据低, 寄存器号<<2] —— 寄存器号在**最后** */
    uint8_t wbuf[4] = {
        KTH5701_CMD_WRITE_REG,
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFF),
        REG_SHIFT(reg),
    };
    uint8_t sta = 0;

    esp_err_t ret = tx_write(wbuf, sizeof(wbuf));
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    ret = tx_read(&sta, 1);
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    _last_status = sta;
    return ESP_OK;
}

esp_err_t KTH5701::chip_id(uint16_t *id)
{
    uint16_t v = 0;
    esp_err_t ret = reg_read(KTH5701_REG_CHIP_ID, &v, nullptr);
    if (ret == ESP_OK && id) {
        *id = v;
    }
    return ret;
}

esp_err_t KTH5701::read_status(uint8_t *status)
{
    uint16_t v = 0;
    uint8_t sta = 0;
    esp_err_t ret = reg_read(KTH5701_REG_STATUS, &v, &sta);
    if (ret == ESP_OK && status) {
        *status = (uint8_t)v;
    }
    return ret;
}

esp_err_t KTH5701::set_mode(uint8_t cmd)
{
    uint8_t sta = 0;
    esp_err_t ret = tx_write(&cmd, 1);
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    ret = tx_read(&sta, 1);
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    _last_status = sta;
    return ESP_OK;
}

esp_err_t KTH5701::read_xyz(int16_t *x, int16_t *y, int16_t *z, uint8_t *status)
{
    uint8_t buf[16] = {0};
    size_t len = sizeof(buf);
    esp_err_t ret = data_read(KTH5701_AXIS_ALL, buf, &len);
    if (ret != ESP_OK) {
        return ret;
    }
    /* 帧序 Z, Y, X, T (各 2B, 大端), buf[0] = status */
    if (z) {
        *z = (int16_t)((buf[1] << 8) | buf[2]);
    }
    if (y) {
        *y = (int16_t)((buf[3] << 8) | buf[4]);
    }
    if (x) {
        *x = (int16_t)((buf[5] << 8) | buf[6]);
    }
    if (status) {
        *status = buf[0];
    }
    return ESP_OK;
}

/* ---------------- 角度 ----------------
 * ⚠️ 芯片**不输出角度**, 只给磁场分量。这里做 atan2 + 标定。
 *    参考实现把 Y 轴磁场直接当角度用 (还带符号扩展 bug), 不可照抄。
 */
float KTH5701::getSensorAngle()
{
    uint8_t buf[KTH5701_XY_FRAME_LEN] = {0};
    size_t len = sizeof(buf);

    /* 只读 X/Y —— 角度计算所需最少项。帧序按 ZYXT 过滤后: [status, Y, X] */
    esp_err_t ret = data_read(KTH5701_AXIS_XY, buf, &len);
    if (ret != ESP_OK || len < KTH5701_XY_FRAME_LEN) {
        return -1.0f; /* 负值 = 出错, Sensor::update() 据此跳过本次 */
    }
    _last_status = buf[0];

    float raw_y = (float)(int16_t)((buf[1] << 8) | buf[2]);
    float raw_x = (float)(int16_t)((buf[3] << 8) | buf[4]);

    /* 硬铁偏移 + 软铁增益 */
    float x = (raw_x - _x_off) * _x_gain;
    float y = (raw_y - _y_off) * _y_gain;

    float a = atan2f(y, x); /* [-π, π] */

    /* 先减零位再镜像 —— 方向取反时零位也跟着镜像, 否则换磁铁方向要重标两次 */
    a = (a - _zero_offset) * (float)_direction;

    /* 归一化到 [0, 2π) */
    a = fmodf(a, _2PI);
    if (a < 0.0f) {
        a += _2PI;
    }
    return a;
}

float KTH5701::get_mech_angle() const
{
    /* angle_prev 是上一次成功的 getSensorAngle() 结果, 已含方向与零位 */
    return angle_prev;
}

float KTH5701::get_multi_turn_angle() const
{
    return (float)full_rotations * _2PI + angle_prev;
}

esp_err_t KTH5701::data_read(uint8_t axis, uint8_t *buf, size_t *len)
{
    /* 需要读回的字节数 = 1 (status) + 2 * 使能的项数 */
    uint8_t n = 0;
    for (int i = 0; i < 4; i++) {
        if (axis & (1 << i)) {
            n++;
        }
    }
    size_t need = 1 + 2 * (size_t)n;
    if (*len < need) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t wbuf = (uint8_t)(KTH5701_CMD_DATA_READ | axis);
    esp_err_t ret = tx_write(&wbuf, 1);
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    ret = tx_read(buf, need);
    if (ret != ESP_OK) {
        _error_count++;
        return ret;
    }
    *len = need;
    return ESP_OK;
}

/* ---------------- 标定与诊断 ---------------- */

void KTH5701::set_calibration(float x_off, float y_off, float x_gain, float y_gain)
{
    _x_off = x_off;
    _y_off = y_off;
    _x_gain = x_gain;
    _y_gain = y_gain;
}

void KTH5701::get_calibration(float *x_off, float *y_off, float *x_gain, float *y_gain) const
{
    if (x_off) {
        *x_off = _x_off;
    }
    if (y_off) {
        *y_off = _y_off;
    }
    if (x_gain) {
        *x_gain = _x_gain;
    }
    if (y_gain) {
        *y_gain = _y_gain;
    }
}

const char *KTH5701::status_str(char *buf, size_t len) const
{
    uint8_t s = _last_status;
    snprintf(buf, len, "0x%02X [%s%s%s%s%s mode=%u]",
             s,
             (s & 0x01) ? "DRDY " : "",
             (s & 0x02) ? "softRst " : "",
             (s & 0x04) ? "magnDet " : "",
             (s & 0x08) ? "buttDet " : "",
             (s & 0x10) ? "FAILING " : "",
             (unsigned)(s >> 5) & 0x07);
    return buf;
}
