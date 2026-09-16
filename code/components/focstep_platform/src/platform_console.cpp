#include "platform_console.h"
#include "board_pins.h"
#include "kth5701.h"
#include "foc_motor.h"
#include "ipropi_sense.h"
#include "bus_voltage.h"
#include "homing.h"
#include "net_ota.h"
#include "pa_wake.h"
#include "power_state.h"
#include "wakeup.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <inttypes.h> /* PRIu32: 本工具链上 uint32_t == long unsigned int, %u 会 -Werror=format */

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
    printf("VBUS = %d mV  (门控导通 %dµs 后采样; 导通期耗 312µA)\n",
           mv, CONFIG_FOCSTEP_VBUS_GATE_SETTLE_US);
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
    /* ⚠️ 这里直接写 GPIO18 仅用于**自检**。正常运行期 GPIO18 由 power_state.c 独占
     *    —— 它一根脚管三件事: DRV 使能 / VREF 门控 / CAN 的 Rs (docs/doc.md §5.3)。 */
    gpio_set_level((gpio_num_t)PIN_DRV_nSLEEP, on ? 1 : 0);
    printf("nSLEEP(GPIO%d) → %d\n", PIN_DRV_nSLEEP, on);
    if (on) {
        printf("请量 **VREF 引脚** ≈ 2.34V (10k+22k 从 3.4V 分压)。\n");
        printf("⚠️ 抬 nSLEEP 前务必先跑 brake 命令验 EN/PH 接法!\n");
    } else {
        printf("请量 **VREF 引脚** = 0V。\n");
        printf("这一态若不为 0, 说明 P-MOS 门控没关断 ⇒ 分压持续耗 106µA@3.4V\n"
               "(折算 24V 输入侧 ≈17.6µA), 占深睡档基线 (25~65µA@24V) 的 27~70%%。\n");
    }
    return 0;
}

/* ---------------- 自检 6.5: EN/PH 验线 ---------------- */
static int do_brake(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    foc_motor_brake();
    printf("两相 EN 已拉低 = Brake (低边慢衰减)。\n");
    printf("★ 现在用手转轴, 应该有**明显阻尼**, 且电机**不会主动往一个方向转**。\n");
    printf("  若电机使劲朝一个方向转 ⇒ 两根接反了 (GPIO19/21 实际接到 IN2/PH),\n"
           "  打开 Kconfig 的 FOCSTEP_PHEN_SWAPPED 重烧再试 (台面救回, 不必重画板)。\n");
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
    range_state_t rst = foc_motor_range_state();
    float zero = 0.0f, end = 0.0f, span = 0.0f;
    foc_motor_get_range(&zero, &end, &span);
    printf("当前位置 : %.4f rad  (圈数 %" PRIi32 ")\n",
           (double)foc_motor_get_angle_rad(), foc_motor_turns());
    printf("位置目标 : %.4f rad   模式 %d\n",
           (double)foc_motor_get_target_rad(), (int)foc_motor_get_mode());
    printf("电压上限 : %.2f V   位置可信: %s\n",
           (double)foc_motor_get_voltage_limit(),
           foc_motor_position_trusted() ? "是 ✅" : "**否 ❌ (需标定/回零)**");
    printf("行程状态 : %s\n", foc_motor_range_state_str(rst));
    if (rst != RANGE_NONE) {
        printf("零点(0%%)  : %.4f rad\n", (double)zero);
    }
    if (rst == RANGE_BOTH) {
        printf("满行程(100%%): %.4f rad   span %.4f rad (带符号) ⇒ 开度增大 = 朝%s\n",
               (double)end, (double)span, (span > 0) ? "正" : "负");
    } else if (rst == RANGE_ZERO_ONLY) {
        printf("           **缺满行程点 ⇒ 0%%/100%% 还不成立, 开度指令会被拒**\n");
    }
    printf("\n用法: pos | pos save | pos restore | pos invalidate\n");
    return 0;
}

/* 方向字符串 → 枚举 ("pos"/"+" 与 "neg"/"-") */
static bool parse_dir(const char *s, home_dir_t *out)
{
    if (strcmp(s, "pos") == 0 || strcmp(s, "+") == 0) {
        *out = HOME_DIR_POS;
        return true;
    }
    if (strcmp(s, "neg") == 0 || strcmp(s, "-") == 0) {
        *out = HOME_DIR_NEG;
        return true;
    }
    return false;
}

/* 最近一次 home 测到的接触点。**由调用方 (命令台) 持有** —— 平台模块不带会话状态,
 * 免得出现"测量后又被复位/被另一次测量覆盖"的失效规则。 */
static bool s_contact_valid = false;
static float s_contact_rad = 0.0f;

/* "零点/满行程点" → 方向: 只有标定完成后才成立 (span 的符号就是方向的定义)。
 * 标定前没有 span ⇒ 只能拿 pos/neg 说话, 这正是"两个量各自在能知道的那一侧给"。 */
static bool dir_for_endpoint(bool want_zero, home_dir_t *out)
{
    if (foc_motor_range_state() != RANGE_BOTH) {
        return false;
    }
    float span = 0.0f;
    foc_motor_get_range(nullptr, nullptr, &span);
    home_dir_t zero_dir = (span > 0.0f) ? HOME_DIR_NEG : HOME_DIR_POS;
    *out = want_zero ? zero_dir : (home_dir_t)(-(int)zero_dir);
    return true;
}

static int do_home(int argc, char **argv)
{
    /* 参数是**方向** (pos/neg), 或标定完成后可用的**端点名** (zero/max)。
     * 只测, 不改行程 —— 落定要走 `learn zero|end`。 */
    home_dir_t dir = HOME_DIR_NEG;
    if (argc > 1) {
        if (parse_dir(argv[1], &dir)) {
            /* ok */
        } else if (strcmp(argv[1], "zero") == 0 || strcmp(argv[1], "closed") == 0 ||
                   strcmp(argv[1], "max") == 0 || strcmp(argv[1], "open") == 0) {
            bool want_zero = (strcmp(argv[1], "zero") == 0 || strcmp(argv[1], "closed") == 0);
            if (!dir_for_endpoint(want_zero, &dir)) {
                printf("'%s' 要等标定完成后才能翻译成方向 (标定前还不知道哪一侧是零点)\n"
                       "  ⇒ 现在请直接用 home pos / home neg\n", argv[1]);
                return 1;
            }
            printf("(按当前行程解读: %s = 朝%s)\n", want_zero ? "零点/全关" : "满行程/全开",
                   homing_dir_str(dir));
        } else {
            printf("用法: home [pos|neg|zero|max]   (默认 neg)\n"
                   "  pos/neg   = 朝角度增大/减小的方向顶机械限位 (**只测, 不改行程**)\n"
                   "  zero|max  = 顶零点/满行程点那一侧 (需先标定完成才有意义)\n");
            return 1;
        }
    }

    printf("⚠️ 即将以 %d mV 的回零力矩朝%s推, 会顶到机械限位。\n"
           "   确保机构上**没有异物也没有人**。\n",
           CONFIG_FOCSTEP_HOME_TORQUE_MV, homing_dir_str(dir));
    float contact_rad = 0.0f;
    homing_result_t r = homing_probe(dir, &contact_rad);
    printf("结果: %s  接触点 %.4f rad\n", homing_result_str(r), (double)contact_rad);
    if (r == HOME_OK) {
        s_contact_rad = contact_rad;
        s_contact_valid = true;
        printf("✓ 已记住这个接触点 ⇒ 确认合理后用 `learn zero` 或 `learn end` 落定\n");
        printf("  (本命令只**测**, 行程一点没动。落定那一步不动电机)\n");
    }
    return (r == HOME_OK) ? 0 : 1;
}

static int do_learn(int argc, char **argv)
{
    /* learn 语法 (两个量各自在"能知道的那一侧"给):
     *   learn both [<零点方向>]   —— 一次到底: 顶两端; 满行程点方向 = 零点反向
     *   learn zero | learn end    —— 落定**上一次 home 测到的接触点** (不动电机)
     *   learn auto [on|off]       —— 无人值守自动标定 (受策略门控) */
    if (argc > 1 && strcmp(argv[1], "auto") == 0) {
        homing_policy_t pol;
        homing_get_policy(&pol);
        if (argc > 2) {
            if (strcmp(argv[2], "on") == 0) {
                pol.auto_calib = true;
            } else if (strcmp(argv[2], "off") == 0) {
                pol.auto_calib = false;
            } else {
                printf("用法: learn auto [on|off]\n");
                return 1;
            }
            homing_set_policy(&pol);
            printf("自动标定策略: %s (掉电不保持 —— 默认值在 Kconfig)\n",
                   pol.auto_calib ? "**允许**" : "禁止");
            return 0;
        }
        printf("自动标定: 策略 %s, 失败重试 %u 次, 零点朝%s (方向来自策略: "
               "Kconfig FOCSTEP_HOME_AUTO_INVERTED)\n",
               pol.auto_calib ? "允许" : "**禁止 (learn auto on 可临时打开)**",
               (unsigned)pol.retries, homing_dir_str(pol.zero_dir));
        if (!pol.auto_calib) {
            printf("用法: learn auto on   (会让门自己跑到底, 现场确认机构通畅再用)\n");
            return 1;
        }
        printf("⚠️ 会以 %d mV 力矩让门自己跑到底 (两端各两次)。确保机构通畅且无人。\n",
               CONFIG_FOCSTEP_HOME_TORQUE_MV);
        homing_result_t ra = homing_auto();
        printf("结果: %s (策略见 homing_policy_t; 策略默认值在 Kconfig)\n",
               homing_result_str(ra));
        if (ra == HOME_OK) {
            homing_print_status();
        }
        return (ra == HOME_OK) ? 0 : 1;
    }

    /* 落定上一次 home 的接触点 (不动电机) —— 这是单端重标的正路 */
    if (argc > 1 && (strcmp(argv[1], "zero") == 0 || strcmp(argv[1], "end") == 0)) {
        if (argc > 2) {
            printf("用法: learn %s   (不带方向: 落定的是上一次 home 测到的点)\n", argv[1]);
            return 1;
        }
        if (!s_contact_valid) {
            printf("❌ 还没有测到的接触点 ⇒ 先 `home pos|neg` 顶一次限位\n");
            return 1;
        }
        bool as_zero = (strcmp(argv[1], "zero") == 0);
        printf("落定: 把已测接触点 %.4f rad 写为%s (不动电机, 另一端保持不变)\n",
               (double)s_contact_rad, as_zero ? "零点(0%)" : "满行程点(100%)");
        homing_result_t rr = homing_apply_contact(s_contact_rad, as_zero);
        printf("结果: %s\n", homing_result_str(rr));
        if (rr == HOME_OK) {
            s_contact_valid = false; /* 用掉就失效, 防止被重复落定 */
            homing_print_status();
        }
        return (rr == HOME_OK) ? 0 : 1;
    }

    home_dir_t zero_dir = HOME_DIR_NEG; /* 常规安装: 零点在负端 */

    if (argc > 1) {
        if (strcmp(argv[1], "both") == 0) {
            if (argc > 2 && !parse_dir(argv[2], &zero_dir)) {
                printf("方向只能是 pos 或 neg\n");
                return 1;
            }
            if (argc > 3) {
                printf("用法: learn both [<零点方向>]   (满行程点必然是反向, 不用给)\n");
                return 1;
            }
        } else {
            /* 旧写法 learn closed|open 已废弃: 它把"端点"与"方向"绑死, 反装机构会朝
             * 错误方向顶限位 (还会把那一点标成端点)。宁可报错也不要猜。 */
            printf("用法: learn [both|zero|end] | learn auto [on|off]\n"
                   "  learn both            一次到底: 依次顶两端, 建立零点+满行程点\n"
                   "  learn both pos        同上, 但零点在**正**端 (反装机构)\n"
                   "  learn zero            把上一次 `home` 测到的接触点**落定为零点**\n"
                   "  learn end             同上, 落定为**满行程点** (要求已有零点)\n"
                   "  流程: home pos|neg  →  看接触点是否合理  →  learn zero|end\n"
                   "⚠️ 旧写法 closed/open 与 `learn end <方向>` 都已废弃:\n"
                   "   标定前能给的只有**方向**, 测量之后能给的只有**端点身份**, 不要互相推导\n");
            return 1;
        }
    }

    printf("⚠️ 会依次顶两端机械限位 (回零力矩 %d mV), 全程约 10~60 秒。\n"
           "   确保机构上没有人, 且两端限位可靠。\n", CONFIG_FOCSTEP_HOME_TORQUE_MV);
    printf("   端点分配: 零点朝%s, 满行程点朝%s (反向)。失败按策略重试\n",
           homing_dir_str(zero_dir), homing_dir_str((home_dir_t)(-(int)zero_dir)));

    /* homing_learn_range 的契约本就含"按策略重试"; 与自动路径的唯一区别是
     * 失败后果 (这里不拉 PS_FAULT —— 操作员就在旁边, 拉故障会挡住他重测)。 */
    homing_result_t r = homing_learn_range(zero_dir);
    printf("结果: %s\n", homing_result_str(r));
    if (r == HOME_OK) {
        s_contact_valid = false; /* 行程已重写, 之前那个测量点不再对应任何落定意图 */
        homing_print_status();
    }
    return (r == HOME_OK) ? 0 : 1;
}

/* ---------------- 移动到指定位置 ---------------- */

static int do_goto(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: goto <0..1>   归一化开度 (0 = 零点/全关, 1 = 满行程点/全开)\n");
        return 1;
    }
    float p = strtof(argv[1], nullptr);
    if (p < 0.0f || p > 1.0f) {
        printf("开度必须在 0..1 之间 (要越过零点/超出满行程点请用 goto_x)\n");
        return 1;
    }
    if (!foc_motor_position_trusted()) {
        printf("❌ 位置不可信 ⇒ 拒绝位置指令 (先 `learn` 或 `mark`)\n");
        return 1;
    }
    if (foc_motor_range_state() != RANGE_BOTH) {
        printf("❌ 行程不完整 (%s) ⇒ 拒绝开度指令: 0%%/100%% 只在零点与满行程点都标定后成立\n",
               foc_motor_range_state_str(foc_motor_range_state()));
        return 1;
    }
    if (power_state_current() != PS_ACTIVE) {
        printf("⚠️ 当前 %s 态 (电机未使能) —— 电机不会动; 先 `wake`\n",
               power_state_name(power_state_current()));
    }
    float zero = 0.0f, span = 0.0f;
    foc_motor_get_range(&zero, nullptr, &span);
    esp_err_t r = foc_motor_move_to_ext(p);
    if (r == ESP_OK) {
        printf("目标 %.4f rad = 零点 %.3f + %.1f%% × span %.3f\n",
               (double)foc_motor_get_target_rad(), (double)zero, (double)(p * 100.0f),
               (double)span);
    } else {
        printf("❌ 未下达: %s (目标仍是 %.4f rad)\n", esp_err_to_name(r),
               (double)foc_motor_get_target_rad());
    }
    return (r == ESP_OK) ? 0 : 1;
}

/* 越界开度: 允许 0..1 之外 (越过零点 / 超出满行程点) —— 探边与越零点调试用 */
static int do_goto_x(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: goto_x <开度>   允许 0..1 之外 (例如 -0.05 = 越过零点 5%%)\n"
               "  ⚠️ 越过端点意味着继续朝机械限位推, 机构会顶死 —— 只用于探边/调试\n");
        return 1;
    }
    float p = strtof(argv[1], nullptr);
    if (p < -1.0f || p > 2.0f) {
        printf("本命令只接受 -1..2 (再远就不是调试而是撞机了)\n");
        return 1;
    }
    esp_err_t r = foc_motor_move_to_ext(p);
    if (r == ESP_OK) {
        printf("目标 %.4f rad (开度 %.1f%%)\n", (double)foc_motor_get_target_rad(),
               (double)(p * 100.0f));
        if (p < 0.0f || p > 1.0f) {
            printf("⚠️ 越界指令已下达 —— 门会走到 0%%/100%% 之外 (碰到机械限位就是硬顶)\n");
        }
    } else {
        printf("❌ 未下达: %s\n", esp_err_to_name(r));
    }
    return (r == ESP_OK) ? 0 : 1;
}

static int do_goto_rad(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: goto_rad <rad>   绝对多圈角 (自检/回零用)\n");
        return 1;
    }
    float rad = strtof(argv[1], nullptr);
    if (power_state_current() != PS_ACTIVE) {
        printf("⚠️ 当前 %s 态 (电机未使能) —— 电机不会动; 先 `wake`\n",
               power_state_name(power_state_current()));
    }
    foc_motor_set_target_rad(rad);
    printf("目标 %.4f rad (绝对多圈角)\n", (double)rad);
    printf("⚠️ 本指令**不过行程门禁也不检查位置可信度** —— 可指向行程外, 自检用\n");
    return 0;
}

/* ---------------- 手动标定端点 ---------------- */

static int do_mark(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: mark <zero|end>\n"
               "  把**当前位置**定为端点 —— 不推限位, 直接标 (手推到机械端点后标最省事)。\n"
               "  mark zero : 立即成为系统零点 (0%% 开度) 且位置可信\n"
               "  mark end  : 立即成为满行程点 (100%% 开度), 要求已有零点\n"
               "  典型流程: 手推到全关 → mark zero; 手推到全开 → mark end\n");
        return 1;
    }
    bool as_zero;
    if (strcmp(argv[1], "zero") == 0 || strcmp(argv[1], "closed") == 0) {
        as_zero = true;
    } else if (strcmp(argv[1], "end") == 0 || strcmp(argv[1], "max") == 0 ||
               strcmp(argv[1], "open") == 0) {
        as_zero = false;
    } else {
        printf("未知端点: %s (用 zero 或 end)\n", argv[1]);
        return 1;
    }
    esp_err_t r = as_zero ? homing_mark_zero() : homing_mark_end();
    printf("结果: %s\n", esp_err_to_name(r));
    if (r == ESP_OK) {
        homing_print_status();
    }
    return (r == ESP_OK) ? 0 : 1;
}

/* ---------------- 方向 (0%/100% 与正负方向的对应关系) ---------------- */

static int do_dir(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "invert") == 0) {
        esp_err_t r = foc_motor_invert_travel();
        printf("结果: %s\n", esp_err_to_name(r));
        if (r == ESP_OK) {
            homing_print_status();
        }
        return (r == ESP_OK) ? 0 : 1;
    }
    range_state_t st = foc_motor_range_state();
    float zero = 0.0f, end = 0.0f, span = 0.0f;
    foc_motor_get_range(&zero, &end, &span);
    printf("行程状态 : %s\n", foc_motor_range_state_str(st));
    if (st == RANGE_BOTH) {
        printf("零点/满行程: %.4f / %.4f rad, span %.4f\n",
               (double)zero, (double)end, (double)span);
        printf("开度增大 : 朝%s\n", (span > 0) ? "正(角度增大)" : "负(角度减小)");
    } else {
        printf("⚠️ 行程不完整 ⇒ 无反可反 (反向就是交换零点与满行程点这两个端点)\n");
    }
    printf("说明     : `dir invert` = 交换零点与满行程点 (纯数据变换, 不碰编码器/Motor 约定)。\n"
           "           反转**编码器**方向仍是 Kconfig ENCODER_DIRECTION (会让已标定行程按指纹作废)\n");
    printf("\n用法: dir | dir invert\n");
    return 0;
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

/* ---------------- 自检 7: 无线唤醒 (BLE 周期广播) ---------------- */

static const char *wl_state_str(const pa_wake_status_t *st)
{
    if (st->mode == PA_WAKE_OFF) {
        return "关闭";
    }
    if (st->synced) {
        return "已同步";
    }
    return st->slow_scan ? "慢扫(对端不在场?)" : "扫描中";
}

static void wl_print_status(void)
{
    pa_wake_status_t st;
    pa_wake_status(&st);

    printf("无线唤醒: 模式=%s 状态=%s\n",
           (st.mode == PA_WAKE_PA) ? "PA 监听" : "关闭", wl_state_str(&st));
    printf("  SID=%u skip=%u per_adv_ival=%u ms | T 实得=%" PRIu32 " ms 期望=%" PRIu32 " ms\n",
           st.sid, st.skip, st.per_adv_ival_ms, st.t_ms, st.t_want_ms);
    printf("  host=%s synced=%d rssi=%d | 失步=%" PRIu32 " 收包=%" PRIu32 " 指令=%" PRIu32 "\n",
           st.host_ready ? "就绪" : "未就绪", (int)st.synced, st.rssi,
           st.lost_cnt, st.pkt_ok, st.cmd_cnt);
    printf("  丢弃: crc=%" PRIu32 " 重复帧=%" PRIu32 " 非本机=%" PRIu32
           " 版本不符=%" PRIu32 " 噪声=%" PRIu32 "\n",
           st.drop_crc, st.drop_dup, st.drop_not_me, st.drop_badver, st.drop_noise);
    printf("  ⚠️ PA 是**单向**链路(发送端无回执) ⇒ 改 T 是否生效只能在**本端**看实得值\n");
}

static int do_wl(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "off") == 0) {
        esp_err_t r = pa_wake_set_mode(PA_WAKE_OFF);
        printf("关闭无线监听 → %s\n", esp_err_to_name(r));
        printf("⚠️ 关闭后只能本地唤醒; 本地唤醒或复位后会重新开窗 (出厂行为)\n");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "pa") == 0) {
        esp_err_t r = pa_wake_set_mode(PA_WAKE_PA);
        printf("开启 PA 监听 → %s (看同步日志, 或 wl status)\n", esp_err_to_name(r));
        return 0;
    }
    if (argc > 2 && strcmp(argv[1], "t") == 0) {
        uint32_t t = (uint32_t)strtoul(argv[2], nullptr, 10);
        esp_err_t r = pa_wake_set_T_ms(t);
        printf("请求 T=%u ms → %s\n", (unsigned)t, esp_err_to_name(r));
        printf("注意: T 只能落在 per_adv_ival 的整数倍上, 实得值见 wl status\n");
        return 0;
    }
    /* 注: "本地注入一条业务指令" 是**应用**工具 (命令名属于应用协议),
     *     因此不在平台命令台里 —— 见应用命令 `pa_send` (driver/main/app_console.c)。 */

    wl_print_status();
    printf("用法: wl [status] | wl off | wl pa | wl t <ms>\n");
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
    reg_platform("brake",  "两相 EN 拉低(brake), 验线: 手转应有阻尼",    do_brake);
    reg_platform("jog",    "jog <v> 小电压点动, 确认转向",                do_jog);
    reg_platform("align",  "initFOC 电角对齐",                           do_align);
    reg_platform("char",   "char <v> characteriseMotor 实测相电阻",       do_char);
    reg_platform("ipropi", "读两相 IPROPI 电流",                         do_ipropi);
    reg_platform("stall",  "stall [reset] 堵转判定状态与阈值",             do_stall);
    reg_platform("home",   "home [pos|neg|zero|max] 顶限位测接触点 (只测)", do_home);
    reg_platform("learn",  "learn [both [pos|neg]|zero|end|auto] 标定行程", do_learn);
    reg_platform("mark",   "mark <zero|end> 把当前位置定为端点",          do_mark);
    reg_platform("dir",    "dir [invert] 开度方向 (反向 = 交换两端点)",    do_dir);
    reg_platform("goto",   "goto <0..1> 按开度移动 (0=零点/全关, 1=满行程)", do_goto);
    reg_platform("goto_x", "goto_x <开度> 允许越界 (越过零点/超出满行程)", do_goto_x);
    reg_platform("goto_rad", "goto_rad <rad> 移动到绝对多圈角 (自检用)",   do_goto_rad);
    reg_platform("pos",    "pos [save|restore|invalidate] 位置/可信度",   do_pos);
    reg_platform("int",    "观察 INT 锁存语义 (读数据是否清中断)",         do_int);
    reg_platform("stat",   "电源状态机快照",                              do_stat);
    reg_platform("net",    "net on|off 平台态控制面 (WiFi+HTTP+mDNS+OTA)", do_net);
    reg_platform("ota",    "打印运行/待升级分区与版本",                    do_ota);
    reg_platform("sleep",  "立即进深睡",                                 do_sleep);
    reg_platform("wl",     "wl [status|off|pa|t <ms>|send <cmd>] 无线唤醒", do_wl);
    ESP_LOGI(TAG, "平台自检命令已注册 (共 29 条)");
#endif

    ESP_RETURN_ON_ERROR(esp_console_start_repl(repl), TAG, "repl start failed");
    ESP_LOGI(TAG, "命令台就绪, 输入 help 查看可用命令");
    return ESP_OK;
}
