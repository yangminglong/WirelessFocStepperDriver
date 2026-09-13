#include "ipropi_current_sense.h"
#include "ipropi_sense.h"

#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "CS_IPROPI";

int IpropiCurrentSense::init()
{
    if (_drv == nullptr) {
        ESP_LOGE(TAG, "driver 为空");
        return 0;
    }
    /* ADC 由 ipropi_sense.c 初始化 (main 里在 foc_motor_init 之前已 init) */
    if (ipropi_read_mv(0, nullptr) == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "ipropi 未初始化 —— 请先调 ipropi_init()");
        return 0;
    }

    /* 链接驱动: 基类会据此取 driver->type() == DriverType::Stepper,
     * 从而在 getABCurrents() 里跳过 Clarke (两相直接就是 α/β)。 */
    linkDriver(_drv);

    /* 不做 driverAlign 的物理对齐: 它靠"给一相通电看另一相反应"来判定接线,
     * 而我们的符号是从指令方向重建的, 对齐过程反而会与重建逻辑打架。 */
    skip_align = true;

    initialized = true;
    ESP_LOGI(TAG, "init ok: driver_type=%d (2=Stepper), 符号由指令方向重建",
             (int)driver_type);
    return 1;
}

PhaseCurrent_s IpropiCurrentSense::getPhaseCurrents()
{
    PhaseCurrent_s cur = {0.0f, 0.0f, 0.0f};

    if (!_enabled) {
        return cur;
    }

    float ma1 = 0.0f, ma2 = 0.0f;
    if (ipropi_read_both_ma(&ma1, &ma2) != ESP_OK) {
        _fail++;
        /* 读失败返回 0 —— 让环把它当成"测到 0 电流", 比返回陈旧值安全 */
        return cur;
    }

    /* ★ IPROPI 无符号 (手册 §7.3.3.1: 反向电流计 0), 符号来自指令方向。
     *   见 ipropi_current_sense.h 顶部说明与已知限制。 */
    cur.a = ma1 * 0.001f * (float)_drv->sign_a(); /* SimpleFOC 用安培 */
    cur.b = ma2 * 0.001f * (float)_drv->sign_b();
    cur.c = 0.0f; /* 两相系统没有第三相 */
    return cur;
}

void IpropiCurrentSense::disable()
{
    _enabled = false;
    _fail = 0;
}
