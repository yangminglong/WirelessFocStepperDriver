#include "platform_console.h"
#include "board_pins.h"
#include "kth5701.h"
#include "foc_motor.h"
#include "ipropi_sense.h"
#include "bus_voltage.h"
#include "homing.h"
#include "net_ota.h"
#include "power_state.h"
#include "wakeup.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CMD";

/* 编码器句柄 —— 由 platform_console_init() 自己从平台取, 不需要应用注入 */
static KTH5701 *s_enc = nullptr;

/* circle 命令的统计 */
static float s_xmin, s_xmax, s_ymin, s_ymax;

/* ---------------- 自检 1: 芯片 ID ---------------- */
static int do_id(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_enc) {
        printf("encoder not bound\n");
        return 1;
    }
    uint16_t id = 0;
    esp_err_t ret = s_enc->chip_id(&id);
    if (ret != ESP_OK) {
        printf("I2C 读失败 (%s)。检查: 地址 0x%02X / 上拉 / 门控供电\n",
               esp_err_to_name(ret), CONFIG_FOCSTEP_ENCODER_I2C_ADDR);
        return 1;
    }
    char buf[64];
    s_enc->status_str(buf, sizeof(buf));
    printf("chip_id = 0x%04X  (期望 0x%04X)  %s\n", id, KTH5701_CHIP_ID,
           (id == KTH5701_CHIP_ID) ? "✅" : "❌ 不匹配 —— 别往下走");
    printf("status  = %s\n", buf);
    return (id == KTH5701_CHIP_ID) ? 0 : 1;
}

/* ---------------- 自检 2: 寄存器回读, 裁决冲突 ---------------- */
static int do_regs(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_enc) {
        return 1;
    }
    /* 这组值来自 GPL 参考实现(真板跑通), 但与本项目从数据手册记的寄存器号冲突。
     * 回读能对上的话说明该组可用; 对不上说明寄存器号另有其值。 */
    const struct { uint8_t reg; uint16_t val; } table[] = {
        {0x1C, 0x1636},
        {0x1D, 0x0002},
        {0x1E, 0x8000},
        {0x1F, 0x0000},
        {0x28, 0x0000},
        {0x29, 0x0000},
    };
    printf("reg   写入     回读     判定\n");
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        uint16_t rd = 0;
        esp_err_t w = (table[i].val == 0)
                          ? ESP_OK /* 0 表示"只读不写", 避免覆盖未知寄存器 */
                          : s_enc->reg_write(table[i].reg, table[i].val);
        vTaskDelay(pdMS_TO_TICKS(2));
        esp_err_t r = s_enc->reg_read(table[i].reg, &rd, nullptr);
        if (w != ESP_OK || r != ESP_OK) {
            printf("0x%02X  --        --       I2C 错误\n", table[i].reg);
            continue;
        }
        if (table[i].val == 0) {
            printf("0x%02X  (只读)    0x%04X   ——\n", table[i].reg, rd);
        } else {
            printf("0x%02X  0x%04X   0x%04X   %s\n", table[i].reg, table[i].val, rd,
                   (rd == table[i].val) ? "✅ 回读一致" : "❌ 不一致");
        }
    }
    printf("\n提示: 0x1C~0x1E 是参考实现的初始化序列; 0x1F/0x28/0x29 是数据手册口径的\n"
           "      阈值/过采样/measTime 位置。两边对不上时以回读结果为准。\n");
    return 0;
}

static int do_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint8_t s = 0;
    if (!s_enc || s_enc->read_status(&s) != ESP_OK) {
        printf("读状态失败\n");
        return 1;
    }
    char buf[64];
    s_enc->status_str(buf, sizeof(buf));
    printf("status 0x06 = %s\n", buf);
    return 0;
}

static int do_xyz(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    int16_t x = 0, y = 0, z = 0;
    uint8_t st = 0;
    if (!s_enc || s_enc->read_xyz(&x, &y, &z, &st) != ESP_OK) {
        printf("读 XYZ 失败\n");
        return 1;
    }
    printf("X=%6d  Y=%6d  Z=%6d   |XY|=%.0f   status=0x%02X\n",
           x, y, z, sqrtf((float)x * x + (float)y * y), st);
    return 0;
}

/* ---------------- 自检 3/4: 手转轴看 XY 是否走出一个圆 ---------------- */
static int do_circle(int argc, char **argv)
{
    int n = (argc > 1) ? atoi(argv[1]) : 200;
    if (n <= 0) {
        n = 200;
    }
    if (!s_enc) {
        return 1;
    }
    s_xmin = s_ymin = 1e9f;
    s_xmax = s_ymax = -1e9f;

    printf("采样 %d 次 —— 请**缓慢匀速转一整圈**轴...\n", n);
    for (int i = 0; i < n; i++) {
        int16_t x = 0, y = 0, z = 0;
        uint8_t st = 0;
        if (s_enc->read_xyz(&x, &y, &z, &st) == ESP_OK) {
            s_xmin = fminf(s_xmin, (float)x);
            s_xmax = fmaxf(s_xmax, (float)x);
            s_ymin = fminf(s_ymin, (float)y);
            s_ymax = fmaxf(s_ymax, (float)y);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    float cx = (s_xmin + s_xmax) * 0.5f;
    float cy = (s_ymin + s_ymax) * 0.5f;
    float rx = (s_xmax - s_xmin) * 0.5f;
    float ry = (s_ymax - s_ymin) * 0.5f;
    printf("X: [%.0f, %.0f]  中心 %.0f  半径 %.0f\n", s_xmin, s_xmax, cx, rx);
    printf("Y: [%.0f, %.0f]  中心 %.0f  半径 %.0f\n", s_ymin, s_ymax, cy, ry);
    if (rx > 1.0f && ry > 1.0f) {
        float ecc = (rx > ry) ? (rx / ry) : (ry / rx);
        printf("椭圆度 = %.4f  %s\n", ecc,
               (ecc < 1.10f) ? "✅ 良好" : "⚠️ 偏大 (>1.10) —— 磁铁偏心/倾斜或软铁干扰");
    } else {
        printf("❌ 没采到有效行程 —— 确认轴转了一圈, 且磁场量程没饱和\n");
    }
    printf("\n提示: 这里看的是 **XY 是否走出一个圆**。芯片**不输出角度**,\n"
           "      所以不要期望某个寄存器直接给出角度值。\n");
    return 0;
}

static int do_cal(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_enc) {
        return 1;
    }
    float cx = (s_xmin + s_xmax) * 0.5f;
    float cy = (s_ymin + s_ymax) * 0.5f;
    float rx = (s_xmax - s_xmin) * 0.5f;
    float ry = (s_ymax - s_ymin) * 0.5f;
    if (rx < 1.0f || ry < 1.0f) {
        printf("先跑 circle 采到行程再标定\n");
        return 1;
    }
    /* 软铁: 把两轴归一到同一个半径 */
    float gx = ry / rx;
    float gy = rx / ry;
    s_enc->set_calibration(cx, cy, gx, gy);
    printf("已应用 硬铁偏移=(%.0f, %.0f)  软铁增益=(%.4f, %.4f)\n", cx, cy, gx, gy);
    printf("⚠️ 这组值只在本次运行有效。定稿后请写回 Kconfig/参数表。\n");
    return 0;
}

/* ---------------- 自检 5: 母线电压 ---------------- */
static int do_vbus(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    int mv = 0;
    esp_err_t ret = bus_voltage_read_mv(&mv);
    if (ret != ESP_OK) {
        printf("读母线失败: %s\n", esp_err_to_name(ret));
        return 1;
    }
    printf("VBUS = %d mV  (门控导通 %dms 后采样)\n", mv, CONFIG_FOCSTEP_VBUS_GATE_SETTLE_MS);
    printf("使能下限 %d mV ⇒ %s\n", CONFIG_FOCSTEP_VBUS_MIN_ENABLE_MV,
           bus_voltage_allow_motor_enable(mv) ? "允许使能电机 ✅" : "禁止使能 ❌");
    printf("提示: 与万用表比对。门控关断时外部分压节点应为 0V, 若为 24V 说明\n"
           "      门控只做在高边(正确)之外还另有通路。\n");
    return 0;
}

static int do_gate(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: gate <0|1>\n");
        return 1;
    }
    int on = atoi(argv[1]);
    bus_voltage_gate(on != 0);
    printf("母线分压门控 → %s\n", on ? "导通" : "关断");
    if (!on) {
        printf("⚠️ 现在量 GPIO2 (ADC 节点) 应为 **0V**。若不是, 检查是否误把开关做在低边。\n");
    }
    return 0;
}

/* ---------------- 自检 6: VREF 门控两态 ---------------- */
static int do_vref(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: vref <0|1>   0 = nSLEEP 拉低, 1 = nSLEEP 拉高\n");
        return 1;
    }
    int on = atoi(argv[1]);
    /* ⚠️ 这里直接写 GPIO16 仅用于**自检**。正常运行期 GPIO16 由 power_state.c 独占。 */
    gpio_set_level((gpio_num_t)PIN_DRV_nSLEEP, on ? 1 : 0);
    printf("nSLEEP(GPIO%d) → %d\n", PIN_DRV_nSLEEP, on);
    if (on) {
        printf("请量 **VREF 引脚** ≈ 2.34V (10k+22k 从 3.4V 分压)。\n");
        printf("⚠️ 抬 nSLEEP 前务必先跑 brake 命令确认 EN/PH 没认反!\n");
    } else {
        printf("请量 **VREF 引脚** = 0V。\n");
        printf("这一态若不为 0, 说明 P-MOS 门控没关断 ⇒ 分压持续耗 106µA ≈ 0.42mW@24V,\n"
               "是整机 0.25mW 待机预算的 1.7 倍。\n");
    }
    return 0;
}

/* ---------------- 自检 6.5: EN/PH 裁决 ---------------- */
static int do_brake(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    foc_motor_brake();
    printf("两相 EN 已拉低 = Brake (低边慢衰减)。\n");
    printf("★ 现在用手转轴, 应该有**明显阻尼**, 且电机**不会主动往一个方向转**。\n");
    printf("  若电机使劲朝一个方向转 ⇒ EN/PH 认反了 (把 PH 当 EN 拉低),\n"
           "  打开 Kconfig 的 FOCSTEP_PHEN_SWAPPED 重烧再试。\n");
    return 0;
}

static int do_jog(int argc, char **argv)
{
    float v = (argc > 1) ? strtof(argv[1], nullptr) : 1.0f;
    if (!s_enc) {
        return 1;
    }
    printf("点动 %.2fV, 2 秒 —— 观察转向\n", v);
    foc_motor_set_mode(FOC_MODE_IDLE);
    foc_motor_enable(true);
    /* 用通用力矩模式做点动 —— 平台层没有"助动"这个概念, 那是应用语义 */
    foc_motor_set_torque(v);
    vTaskDelay(pdMS_TO_TICKS(2000));
    foc_motor_set_torque(0.0f);
    foc_motor_set_mode(FOC_MODE_IDLE);
    printf("停止。角度 = %.4f rad\n", s_enc->get_mech_angle());
    printf("⚠️ 确认: 电机转的方向 与 atan2 角度增大的方向 是否一致。\n"
           "  不一致就改 Kconfig 的 FOCSTEP_ENCODER_DIRECTION。\n");
    return 0;
}

static int do_align(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return (foc_motor_align() == ESP_OK) ? 0 : 1;
}

static int do_char(int argc, char **argv)
{
    float v = (argc > 1) ? strtof(argv[1], nullptr) : 2.0f;
    return (foc_motor_characterise(v) == ESP_OK) ? 0 : 1;
}

static int do_ipropi(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    float ma1 = 0, ma2 = 0;
    esp_err_t ret = ipropi_read_both_ma(&ma1, &ma2);
    if (ret != ESP_OK) {
        printf("读 IPROPI 失败: %s\n", esp_err_to_name(ret));
        return 1;
    }
    int mv1 = 0, mv2 = 0;
    ipropi_read_mv(0, &mv1);
    ipropi_read_mv(1, &mv2);
    printf("相1: %6.0f mA (%d mV)   相2: %6.0f mA (%d mV)\n", ma1, mv1, ma2, mv2);
    printf("换算: A_IPROPI=%d µA/A, R_IPROPI=%d Ω (均在 Kconfig)\n",
           CONFIG_FOCSTEP_A_IPROPI_UA_PER_A, CONFIG_FOCSTEP_R_IPROPI_OHM);
    printf("⚠️ 手册正文写 450、应用示例写 455 µA/A —— 用已知负载比对后按实测回填。\n");
    printf("⚠️ V_IPROPI 被内部钳位到 V_VREF ⇒ 可测上限 ≈ %.2f A, 再高读数不再上升。\n",
           (double)(2.34f / (CONFIG_FOCSTEP_R_IPROPI_OHM * 450e-6f)));
    return 0;
}

/* ---------------- 自检 10: INT 锁存语义 ---------------- */
static int do_int(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!s_enc) {
        return 1;
    }
    printf("进入 Wake-up&Sleep 档...\n");
    s_enc->set_mode(KTH5701_CMD_WAKEUP_SLEEP | KTH5701_AXIS_ALL);
    vTaskDelay(pdMS_TO_TICKS(20));

    int lvl = gpio_get_level((gpio_num_t)PIN_ENCODER_INT);
    printf("初始 INT(GPIO%d) = %d\n", PIN_ENCODER_INT, lvl);
    printf("★ 现在用手轻转轴...\n");

    bool seen_high = false;
    for (int i = 0; i < 100; i++) { /* 5 秒 */
        if (gpio_get_level((gpio_num_t)PIN_ENCODER_INT)) {
            seen_high = true;
            printf("  INT 拉高 (第 %d 次轮询)\n", i);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!seen_high) {
        printf("❌ 5 秒内 INT 未拉高。可能: 阈值太大 / 轴没动够 / 极性不是高有效。\n");
        return 1;
    }

    /* ★ 关键: 读一次数据, 看 INT 是否归低 */
    int16_t x, y, z;
    uint8_t st;
    s_enc->read_xyz(&x, &y, &z, &st);
    vTaskDelay(pdMS_TO_TICKS(10));
    int after = gpio_get_level((gpio_num_t)PIN_ENCODER_INT);
    printf("读数据后 INT = %d\n", after);
    if (after == 0) {
        printf("✅ 锁存语义成立: **读数据清 INT** ⇒ 深睡档可用 (ext1 ANY_HIGH)。\n");
    } else {
        printf("❌ 读数据**没有**清 INT ⇒ 深睡会被反复唤醒, §10.3 深睡档不成立。\n"
               "   退路: 改用轮询, 或找其它清中断方式(写状态寄存器/重配模式)。\n");
    }
    s_enc->set_mode(KTH5701_CMD_CONTINUOUS | KTH5701_AXIS_ALL);
    return after == 0 ? 0 : 1;
}

static int do_stat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    power_state_snapshot_t s;
    power_state_snapshot(&s);
    foc_motor_status_t f;
    foc_motor_status(&f);
    printf("平台状态 = %s (fault=%u, 状态迁移 %" PRIu32 " 次)\n",
           power_state_name(s.state), s.fault_code, s.transitions);
    printf("母线     = %d mV\n", bus_voltage_last_mv());
    printf("angle   = %.4f rad  vel = %.3f rad/s  turns = %" PRIi32 "\n",
           (double)f.angle_rad, (double)f.velocity, f.turns);
    printf("target  = %.4f rad   enabled=%d  sensor_ok=%d\n",
           (double)f.target_rad, f.enabled, f.sensor_ok);
    printf("loopFOC = %" PRIu32 " µs (1kHz 周期 1000µs)\n", f.loop_us);
    return 0;
}

static int do_pos(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "save") == 0) {
        return (foc_motor_save_position() == ESP_OK) ? 0 : 1;
    }
    if (argc > 1 && strcmp(argv[1], "restore") == 0) {
        return (foc_motor_restore_position() == ESP_OK) ? 0 : 1;
    }
    if (argc > 1 && strcmp(argv[1], "invalidate") == 0) {
        foc_motor_invalidate_position("手动作废 (自检命令)");
        return 0;
    }
    float rmin = 0, rmax = 0;
    foc_motor_get_travel_range(&rmin, &rmax);
    printf("当前位置 : %.4f rad  (圈数 %" PRIi32 ")\n",
           (double)foc_motor_get_angle_rad(), foc_motor_turns());
    printf("位置目标 : %.4f rad   模式 %d\n",
           (double)foc_motor_get_target_rad(), (int)foc_motor_get_mode());
    printf("电压上限 : %.2f V   位置可信: %s\n",
           (double)foc_motor_get_voltage_limit(),
           foc_motor_position_trusted() ? "是 ✅" : "**否 ❌ (需回零)**");
    printf("行程限位 : [%.4f, %.4f] rad\n", (double)rmin, (double)rmax);
    printf("\n用法: pos | pos save | pos restore | pos invalidate\n");
    return 0;
}

static int do_home(int argc, char **argv)
{
    /* 参数: home [open|closed], 默认 closed (全关端作基准零点) */
    home_end_t end = HOME_END_CLOSED;
    if (argc > 1 && strcmp(argv[1], "open") == 0) {
        end = HOME_END_OPEN;
    } else if (argc > 1 && strcmp(argv[1], "closed") == 0) {
        end = HOME_END_CLOSED;
    } else if (argc > 1) {
        printf("用法: home [open|closed]\n");
        return 1;
    }

    printf("⚠️ 即将以 %d mV 的回零力矩朝 %s 端推, 会顶到机械限位。\n"
           "   确保机构上**没有异物也没有人**。\n",
           CONFIG_FOCSTEP_HOME_TORQUE_MV,
           (end == HOME_END_CLOSED) ? "全关" : "全开");
    float a = 0.0f;
    homing_result_t r = homing_run(end, &a);
    printf("结果: %s  接触点 %.4f rad\n", homing_result_str(r), (double)a);
    if (r == HOME_OK) {
        foc_motor_save_position();
        printf("✓ 基准已建立并存盘。此时可以跑位置指令。\n");
    }
    return (r == HOME_OK) ? 0 : 1;
}

static int do_learn(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("⚠️ 自学习会**依次顶两端机械限位** (回零力矩 %d mV), 全程约 10~60 秒。\n"
           "   确保机构上没有人, 且两端都有可靠的机械限位。\n",
           CONFIG_FOCSTEP_HOME_TORQUE_MV);
    homing_result_t r = homing_learn_range();
    printf("结果: %s\n", homing_result_str(r));
    if (r == HOME_OK) {
        homing_print_status();
    }
    return (r == HOME_OK) ? 0 : 1;
}

static int do_stall(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "reset") == 0) {
        ipropi_stall_reset();
        printf("堵转锁存已解除\n");
        return 0;
    }
    printf("堵转锁存 : %s\n", ipropi_stall_active() ? "🔴 已触发" : "正常");
    printf("阈值     : %d mA (矢量幅值峰值), 持续 %d ms\n",
           CONFIG_FOCSTEP_STALL_CURRENT_MA, CONFIG_FOCSTEP_STALL_MS);
    printf("实测上限 : ≈%.2f A (V_IPROPI 被钳位到 V_VREF, 再高读数不再上升)\n",
           (double)(2.34f / (CONFIG_FOCSTEP_R_IPROPI_OHM * 450e-6f)));
    printf("\n用法: stall | stall reset\n");
    printf("整定: 跑 normal 负载, 用 `ipropi` 反复读, 记录稳态峰值, 取其 1.5~2 倍。\n");
    return 0;
}

static int do_ota(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    net_ota_print_info();
    return 0;
}

static int do_net(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "on") == 0) {
        esp_err_t r = net_ota_start();
        printf("net_ota_start → %s\n", esp_err_to_name(r));
        if (r == ESP_OK) {
            printf("⚠️ 控制面开着会打破深睡, 用完请 `net off`\n");
        }
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "off") == 0) {
        net_ota_stop();
        printf("控制面已关闭\n");
        return 0;
    }
    printf("平台态控制面: %s\n", net_ota_is_running() ? "运行中" : "关闭");
    printf("用法: net on | net off | ota\n");
    return 0;
}

static int do_sleep(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("请求进深睡 (由主循环实际执行)...\n");
    power_state_request(PS_SLEEP);
    wakeup_log_config();
    /* 真正的 esp_deep_sleep_start() 由主循环执行 (要先 disarm 其它唤醒源) */
    return 0;
}

/* ---------------- 注册 ---------------- */

static bool s_ready = false;

bool platform_console_ready(void)
{
    return s_ready;
}

esp_err_t platform_console_register(const char *name, const char *help,
                                    esp_console_cmd_func_t fn)
{
    if (!s_ready) {
        ESP_LOGE(TAG, "REPL 未就绪, 无法注册命令 '%s' (要先调 platform_console_init)",
                 name ? name : "?");
        return ESP_ERR_INVALID_STATE;
    }
    esp_console_cmd_t c = {};
    c.command = name;
    c.help = help;
    c.func = fn;
    esp_err_t ret = esp_console_cmd_register(&c);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册命令 '%s' 失败: %s", name, esp_err_to_name(ret));
    }
    return ret;
}

/* 平台自己的命令用这个 —— 注册失败是编程错误, 直接 abort 更早暴露 */
static void reg_platform(const char *name, const char *help, esp_console_cmd_func_t fn)
{
    ESP_ERROR_CHECK(platform_console_register(name, help, fn));
}

esp_err_t platform_console_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "focstep>";
    /* 命令台任务走**低优先级** —— FOC 循环是最高优先级, 不能被日志拖慢 */
    repl_cfg.task_priority = 2;
    repl_cfg.max_cmdline_length = 256;

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t dev_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_uart(&dev_cfg, &repl_cfg, &repl),
                        TAG, "repl uart init failed");
#else
    esp_console_dev_usb_serial_jtag_config_t dev_cfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_usb_serial_jtag(&dev_cfg, &repl_cfg, &repl),
                        TAG, "repl usj init failed");
#endif

    /* 平台自己拿编码器句柄 —— 不再需要应用层注入 */
    s_enc = static_cast<KTH5701 *>(foc_motor_encoder_handle());

    s_ready = true;

#if CONFIG_FOCSTEP_PLATFORM_CONSOLE_ENABLE
    /* ---- 平台级自检命令 (硬件 bring-up 用, 与应用无关) ---- */
    reg_platform("id",     "读芯片 ID (0x0D, 期望 0x0203)",              do_id);
    reg_platform("regs",   "覆写并回读 0x1C/0x1D/0x1E, 裁决寄存器号冲突",  do_regs);
    reg_platform("status", "读状态寄存器 0x06",                          do_status);
    reg_platform("xyz",    "读 X/Y/Z 三轴原始值",                        do_xyz);
    reg_platform("circle", "circle [n] 采样统计 XY 圆度",                 do_circle);
    reg_platform("cal",    "用 circle 的结果算硬铁/软铁并应用",            do_cal);
    reg_platform("vbus",   "读母线电压",                                 do_vbus);
    reg_platform("gate",   "gate <0|1> 手动开关母线分压门控",              do_gate);
    reg_platform("vref",   "vref <0|1> 手动开关 nSLEEP, 量 VREF",         do_vref);
    reg_platform("brake",  "两相 EN 拉低(brake), 裁决 EN/PH 是否认反",     do_brake);
    reg_platform("jog",    "jog <v> 小电压点动, 确认转向",                do_jog);
    reg_platform("align",  "initFOC 电角对齐",                           do_align);
    reg_platform("char",   "char <v> characteriseMotor 实测相电阻",       do_char);
    reg_platform("ipropi", "读两相 IPROPI 电流",                         do_ipropi);
    reg_platform("stall",  "stall [reset] 堵转判定状态与阈值",             do_stall);
    reg_platform("home",   "home [open|closed] 无限位回零",              do_home);
    reg_platform("learn",  "自学习: 顶两端限位, 建立基准与行程范围",        do_learn);
    reg_platform("pos",    "pos [save|restore|invalidate] 位置/可信度",   do_pos);
    reg_platform("int",    "观察 INT 锁存语义 (读数据是否清中断)",         do_int);
    reg_platform("stat",   "电源状态机快照",                              do_stat);
    reg_platform("net",    "net on|off 平台态控制面 (WiFi+HTTP+mDNS+OTA)", do_net);
    reg_platform("ota",    "打印运行/待升级分区与版本",                    do_ota);
    reg_platform("sleep",  "立即进深睡",                                 do_sleep);
    ESP_LOGI(TAG, "平台自检命令已注册 (共 23 条)");
#endif

    ESP_RETURN_ON_ERROR(esp_console_start_repl(repl), TAG, "repl start failed");
    ESP_LOGI(TAG, "命令台就绪, 输入 help 查看可用命令");
    return ESP_OK;
}
