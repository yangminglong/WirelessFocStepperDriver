#pragma once

/*
 * KTH5701 三轴霍尔磁编码器驱动 (自研)
 *
 * ── 为什么自己写 ──────────────────────────────────────────────
 * 1. esp_simplefoc 的 MagneticSensorI2C/SPI **未被编译**（arduino-foc 的 CMakeLists
 *    SRC_FILES 里没有它们）, 且它们 #include "Arduino.h"/Wire.h —— IDF 下无 shim。
 *    ⇒ 必须自写 Sensor 子类 (模板: esp_simplefoc/port/angle_sensor/as5600.cpp)。
 * 2. github.com/hgddingjun/KTH5701 是 **GPL-2.0** 的 MediaTek Linux 驱动。本工程
 *    **只取其协议规格（命令字/帧格式/寄存器访问时序）作为接口事实**，代码独立自写，
 *    以避免 GPL-2.0 传染到商用固件。参考实现本身也不完整（无角度算法、INT 处理不符
 *    数据手册语义），移植收益有限。
 *
 * ── 芯片给什么 ────────────────────────────────────────────────
 * **芯片不输出角度**。测量帧给的是 X/Y/Z 磁场分量 + 温度。
 * 参考实现的 Android 侧把 Y 轴磁场当"角度"显示，因 Y 随转角是正弦, 指针看着能转,
 * 但那是错觉。⇒ atan2、椭圆标定、零位、方向、多圈 全部由本驱动实现。
 *
 * ── 协议要点 (已核实) ─────────────────────────────────────────
 *   I2C 地址  0x68 | (A1?2:0) | (A0?1:0)  → 0x68/0x69/0x6A/0x6B
 *   读寄存器  写 [0x50, reg<<2]  → 读 3B: [status, hi, lo]
 *   写寄存器  写 [0x60, hi, lo, reg<<2] → 读 1B: [status]
 *   读测量帧  写 [0x40|axis]     → 读 1 + 2*popcount(axis) B, [0]=status
 *   命令字    连续 0x10 / 唤醒睡眠 0x20 / 单次 0x30 / IDLE 0x80 / RESET 0xF0
 *   axis 低4位 ZYXT 位掩码, 0x0F = 全选
 *   芯片 ID   寄存器 0x0D == 0x0203
 *
 *   ⚠️ 参考实现用**两段独立事务** (Linux i2c_master_send + i2c_master_recv),
 *      不是 repeated-start。本驱动照此实现 (tx_write 后 tx_read)。
 *
 * ── INT ──────────────────────────────────────────────────────
 *   数据手册: Wake-up&Sleep 档下被测项变化超阈值 → **INT 高有效、锁存**,
 *   **读一次数据即清零**。⇒ ext1 配 ESP_EXT1_WAKEUP_ANY_HIGH, ISR 里必须读数据。
 *   ⚠️ 参考实现每次触发就翻转触发极性、从不读数据清锁存 —— 是权宜写法, 不可照抄。
 *   上板自检必须确认"读数据后 INT 归低", 否则深睡档不成立。
 */

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "i2cdev.h"
#include "common/base_classes/Sensor.h"

/* ---- 命令字 ---- */
#define KTH5701_CMD_CONTINUOUS   0x10 /* 连续测量 (~25µA) */
#define KTH5701_CMD_WAKEUP_SLEEP 0x20 /* 唤醒睡眠 (~1.4µA) —— 深睡档用 */
#define KTH5701_CMD_SINGLE       0x30 /* 单次测量 */
#define KTH5701_CMD_IDLE         0x80
#define KTH5701_CMD_DATA_READ    0x40
#define KTH5701_CMD_READ_REG     0x50
#define KTH5701_CMD_WRITE_REG    0x60
#define KTH5701_CMD_RESET        0xF0

/* axis 位掩码 (ZYXT) */
#define KTH5701_AXIS_T 0x01
#define KTH5701_AXIS_X 0x02
#define KTH5701_AXIS_Y 0x04
#define KTH5701_AXIS_Z 0x08
#define KTH5701_AXIS_ALL 0x0F

/* 只读 X/Y —— 角度计算所需的最少项。帧序按 ZYXT 过滤后: [status, Y_hi, Y_lo, X_hi, X_lo] */
#define KTH5701_AXIS_XY (KTH5701_AXIS_Y | KTH5701_AXIS_X)
#define KTH5701_XY_FRAME_LEN (1 + 2 * 2)

#define KTH5701_REG_CHIP_ID 0x0D
#define KTH5701_CHIP_ID 0x0203

/* 状态寄存器 */
#define KTH5701_REG_STATUS 0x06

class KTH5701 : public Sensor {
public:
    /*
     * @param port     I2C 端口
     * @param scl_io / sda_io
     * @param dev_addr 0x68 | (A1?2:0) | (A0?1:0)
     * @param clk_hz   100000 或 400000
     */
    KTH5701(i2c_port_t port, gpio_num_t scl_io, gpio_num_t sda_io,
            uint8_t dev_addr, uint32_t clk_hz);
    ~KTH5701();

    /* ⚠️ 基类 `Sensor::init()` 返回 **void** 且是 protected,
     *    所以不能把它 override 成 esp_err_t。
     *    这里分两个: `begin()` 做真正的初始化并返回错误码供上层判断;
     *    `init()` 满足基类接口, 内部调 begin()。 */
    esp_err_t begin();
    void init() override;
    void deinit();

    /* ---- Sensor 接口 ---- */
    /* 返回 [0, 2π); **出错必须返回负值** —— Sensor::update() 据此跳过本次 */
    float getSensorAngle() override;

    /* ---- 寄存器层 (供自检命令使用) ---- */
    esp_err_t reg_read(uint8_t reg, uint16_t *val, uint8_t *status);
    esp_err_t reg_write(uint8_t reg, uint16_t val);
    esp_err_t chip_id(uint16_t *id);
    esp_err_t read_xyz(int16_t *x, int16_t *y, int16_t *z, uint8_t *status);
    esp_err_t read_status(uint8_t *status);
    esp_err_t set_mode(uint8_t cmd);
    /* 读测量帧。axis = KTH5701_AXIS_* 位掩码; *len 传入缓冲区大小, 返回实际读取字节数。
     * 帧序按 ZYXT 依次排列且**只含 axis 里使能的项**, buf[0] 恒为 status。 */
    esp_err_t data_read(uint8_t axis, uint8_t *buf, size_t *len);

    /* ---- 标定 ---- */
    /* 硬铁偏移 (原始 LSB) 与软铁增益 (归一化到单位圆) */
    void set_calibration(float x_off, float y_off, float x_gain, float y_gain);
    void get_calibration(float *x_off, float *y_off, float *x_gain, float *y_gain) const;
    void set_direction(int8_t dir) { _direction = (dir < 0) ? -1 : 1; }
    int8_t direction() const { return _direction; }
    void set_zero_offset(float rad) { _zero_offset = rad; }

    /* ---- 多圈 (推拉门必须跨深睡记住行程) ---- */
    int32_t get_turns() const { return full_rotations; }
    void set_turns(int32_t t) { full_rotations = t; }
    /* 机械角 (单圈, 已含零位与方向) 与多圈角。
     * 定义放 .cpp —— 头文件里用 _2PI 会引入 foc_utils.h 依赖。 */
    float get_mech_angle() const;
    float get_multi_turn_angle() const;

    /* ---- 诊断 ---- */
    uint32_t error_count() const { return _error_count; }
    uint8_t last_status() const { return _last_status; }
    const char *status_str(char *buf, size_t len) const;

private:
    /* 底层事务: 两段独立事务 (START..STOP), 与芯片协议一致 */
    esp_err_t tx_write(const uint8_t *buf, size_t len);
    esp_err_t tx_read(uint8_t *buf, size_t len);

    /* i2cdev 管理的设备描述符。总线 (i2c_master bus) 由 i2cdev 统一创建/复用,
     * 与 MCP4725 共享同一条物理总线 (GPIO14/15), 见 doc.md §5.4。 */
    i2c_dev_t _dev = {};
    i2c_port_t _port;
    gpio_num_t _scl;
    gpio_num_t _sda;
    uint8_t _addr;
    uint32_t _clk_hz;
    bool _installed = false;

    /* 标定系数 */
    float _x_off = 0.0f, _y_off = 0.0f;
    float _x_gain = 1.0f, _y_gain = 1.0f;
    int8_t _direction = 1;
    float _zero_offset = 0.0f;

    uint32_t _error_count = 0;
    uint8_t _last_status = 0;
};
