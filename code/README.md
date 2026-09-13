# FocStepper 固件

> 版本 v0.1（2026-09-14）· 状态：**代码完成，板未回，未上板验证**
> 设计依据：[`docs/doc.md`](../docs/doc.md)（v0.5）· 无线唤醒选型：[`docs/wireless_wakeup_options.md`](../docs/wireless_wakeup_options.md)

两个 ESP-IDF 工程 + 一份共享协议。**验收标准是 `idf.py build` 通过**——
上板动作全部写成了自检命令，等板回来一次跑完。

```
code/
├── components/
│   ├── foc_link_protocol/          PA 载荷协议（两端共用，避免线格式漂移）
│   └── focstep_platform/           ★ 平台层：跨项目复用的底层驱动
├── driver/                         ESP32-C6 驱动板固件
│   └── main/                       应用层（推拉门）
└── wake_sender/                    无线唤醒发送端（PA 广播端）
```

---

## 1. 构建

```bash
export IDF_PATH=/home/hanson/.espressif/v5.5.3/esp-idf
export IDF_TOOLS_PATH=/home/hanson/.espressif/tools
export ESP_ROM_ELF_DIR=/home/hanson/.espressif/tools/esp-rom-elfs/20241011
export IDF_PYTHON_ENV_PATH=/home/hanson/.espressif/tools/python/v5.5.3/venv
export PATH="/home/hanson/.espressif/tools/ninja/1.12.1:\
/home/hanson/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20251107/riscv32-esp-elf/bin:\
/home/hanson/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20251107/xtensa-esp-elf/bin:\
$IDF_PYTHON_ENV_PATH/bin:$PATH"

for p in driver wake_sender; do
  (cd $p && rm -f sdkconfig && idf.py build)
done
```

> ⚠️ `~/.espressif/tools/activate_idf_v5.5.3.sh` 把 `idf.py` 定义成 **shell alias**，
> 非交互 shell（脚本/CI）里不生效，必须用上面的显式方式。
>
> ⚠️ **改完 `sdkconfig.defaults` 必须 `rm -f sdkconfig`** —— 否则新条目与新增
> Kconfig 符号都不会生效（后者直接编译报未声明）。本项目踩过两次。

---

## 2. 分层：平台层 vs 应用层

**平台层 `components/focstep_platform/` 不含任何应用语义**，换项目整个目录拷走即可。

| | 内容 | 行数 |
|---|---|---|
| **平台层** | `board_pins` / `kth5701` / `ipropi_*` / `bus_voltage` / `foc_motor` / `homing` / `power_state` / `wakeup` / `led_ws2812` / `platform_button` / `can_link` / `net_ota` / `platform_events` / `platform_console` | 4848 |
| **应用层** | `app_main.c` / `app_door.c` / `app_console.c` | 614 |

三条边界纪律（重构时立，已校验）：

1. **平台层不引用应用级 Kconfig**（`ASSIST_TORQUE` / `IDLE_TO_SLEEP_MS` / … 都在 `driver/main/Kconfig.projbuild`）
2. **底层模块只发事件，不做决策**。按键、堵转、CAN 指令、nFAULT 都发 `FOCSTEP_EVT_*`，应用订阅后决定怎么做 —— 消除层次倒置（原来"按键"模块直接调 `power_state_*()`）
3. **平台内部模块直接互相调用**，不再绕道函数指针钩子表（那张 `power_state_hooks_t` 单订阅、字段定形，换个应用就得改结构体）

应用层怎么接：

```c
platform_events_init();
esp_event_handler_instance_register(FOCSTEP_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL);
platform_console_init();                                    // 建 REPL + 平台自检命令
platform_console_register("open", "开门", my_open_cb);      // 追加自己的命令
```

**§10.3 的四态是应用概念**：平台只有 `SLEEP / ACTIVE / FAULT` 三态；"唤醒接管/运行"是 `ACTIVE` 的两种子模式，差别在"电机做什么 + CAN Rs"——两者都由应用控制（见 `app_door.c`）。同理，平台只提供**灯色**，不规定"什么颜色代表什么状态"。

> ⚠️ `wake_sender` 的 `EXTRA_COMPONENT_DIRS` 只指向 `../components/foc_link_protocol` ——
> 写成 `../components` 会把驱动板的平台层也编进来（发送端没有电机，且平台引用的是驱动板项目的 Kconfig）。

---

## 3. 组件选型：优先用官方组件

**原则：设计一个功能之前，先查组件仓库有没有现成的。** 只有确认没有才自己写。

已核实的仓库现状（2026-09-14）：

| 功能 | 组件 | 说明 |
|---|---|---|
| FOC 电机控制 | **`espressif/esp_simplefoc` 1.4.1** | 传递拉入 `arduino-foc`(SimpleFOC v2.4.0 移植) + `iqmath` + `i2c_bus` |
| I2C 总线 | **`espressif/i2c_bus` 1.5.2** | 由 esp_simplefoc 传递依赖，KTH5701 复用 |
| WS2812 底层 | **`espressif/led_strip` 3.0.3** | RMT 驱动 |
| **灯效/闪灯模式** | **`espressif/led_indicator` 2.1.2** | 闪灯时序与模式表是它的本职 —— 原本自己写的闪灯状态机已删除 |
| **按键消抖/事件** | **`espressif/button` 4.2.1** | 短按/双击/长按识别。GPIO9 **严禁加消抖电容**，所以消抖只能做在固件里，正是它的用途 |
| **局域网发现** | **`espressif/mdns` 1.12.0** | OTA 页面走 `focstep-xxxx.local` |
| OTA 写入 | IDF 内置 `esp_ota_ops` / `esp_http_server` | 上传页 `ota_page.html` 随固件 EMBED 进去 |
| 其余 | IDF 内置 | `twai`(CAN)、`esp_adc`、`esp_console`(命令台)、`nvs_flash`、`esp_pm` |

**确认没有官方组件、必须自研的**（已查组件仓库）：

| 功能 | 结论 |
|---|---|
| **KTH5701 驱动** | 仓库无此组件 ⇒ 自研 `kth5701.cpp`（见 §10）。另有一个 GPL-2.0 的 Linux 驱动可参考协议，**不取代码** |
| **DRV8874 驱动** | 无官方组件。但本设计不需要 —— `StepperDriver2PWM` 直接产生 PH/EN 波形 |
| 电流采样 | `LowsideCurrentSense` 在本拓扑不可用（它绑定 MCPWM 定时器，步进驱动走 LEDC）⇒ 自研 `ipropi_sense.c` + `IpropiCurrentSense`（见 §8）。**注意 `CurrentSense` 本身是抽象接口，可以自己写子类 —— 这曾导致一个错误结论** |
| 电源状态机 | 应用专属，无组件可代 |

> 顺带核实过：`espp/*`（magnetic_encoder / bldc_motor / twai / pid）是**第三方**组件（espp 组织），
> 不是 Espressif 官方，未采用。`ulipe/espfoc` 同为第三方且维护活跃度低。

---

## 4. 引脚与硬件真源

**唯一真源是 [`driver/main/board_pins.h`](driver/main/board_pins.h)**，抄自 `doc.md` §四。
其他文件一律引用它，不得硬编码 GPIO 号。

关键约束（详见该头文件注释）：

| 项 | 要点 |
|---|---|
| LP 域 GPIO0~7 | **已用满，无空闲 GPIO**。新增功能只能复用既有信号 |
| ADC1 | GPIO0~6 七路，扣掉晶振后实际 5 路，本项目用满 3 路（GPIO2 母线 + GPIO4/5 IPROPI） |
| Strapping | GPIO8 **严禁加下拉**（与 GPIO9 同为 0 是非法 boot 组合） |
| JTAG | GPIO4~7 被 IPROPI/PMODE/门控占用 ⇒ **外部 JTAG 不可用**，只能走 USB-Serial-JTAG |

---

## 5. ★ 上板前必须知道的三个裁决点

这三处**文档与手册/源码存在冲突**，代码里做成了可配置项，**必须上板用自检命令裁决**，
不能假定哪一方对：

### 5.1 EN/PH 引脚是否认反（有危险）

- 手册引脚命名是 **EN/IN1** 和 **PH/IN2**（EN 在 IN1 上）
- `doc.md` §四 把 GPIO10 标成 `IN2(EN)`、GPIO11 标成 `IN1(PH)` —— **两者交叉**

PH/EN 模式下**只有 EN=0 是 Brake**。认反了，"先切 brake"会变成"EN 仍为 1、PH=0"
= **满压反转驱动**，唤醒瞬间猛冲。

⇒ 裁决：`brake` 命令 → 手转轴应有明显阻尼且电机不主动转。
反了就把 Kconfig 的 `FOCSTEP_PHEN_SWAPPED` 打开重烧。

### 5.2 KTH5701 寄存器初始化序列

参考实现（真板跑通）写 `0x1C=0x1636 / 0x1D=0x0002 / 0x1E=0x8000`；
本项目从数据手册记的是 `0x1D=wakeDiff / 0x28=过采样 / 0x29=measTime`。**两边对不上。**

⇒ 裁决：`regs` 命令回读比对。默认按参考实现那组走（`FOCSTEP_ENCODER_REG_INIT`）。

### 5.3 CAN 波特率

**`doc.md` 全文未提及 CAN 波特率** —— 默认 500k 是本次新增的占位项。
定稿前需确认。

---

## 6. 上板自检顺序

按顺序敲（命令台 `focstep>`，输入 `help` 可看全部）：

| # | 命令 | 验什么 | 不过怎么办 |
|---|---|---|---|
| 1 | `id` | 芯片 ID `0x0D == 0x0203` | **不通过就别往下走**，查 I2C 地址/上拉/供电 |
| 2 | `regs` | 覆写并回读 0x1C/0x1D/0x1E | 见 §5.2 |
| 3 | `circle 200` | 手转一圈，看 **XY 是否走出一个圆** | 圆度 < 1.10 合格 |
| 4 | `cal` | 用上一步的 min/max 算硬铁/软铁并应用 | — |
| 5 | `vbus` | 母线电压，与万用表比对 | 差得多就查分压比/门控 |
| 6 | `gate 0` | 门控关断后 ADC 节点应为 **0V** | 若为 24V ⇒ 门控做在低边了 |
| 7 | `brake` | **两相 EN 拉低 = Brake**，手转有阻尼、电机不主动转 | 见 §5.1 |
| 8 | `vref 0` / `vref 1` | **VREF 引脚 0V / ≈2.34V** | 不归 0 ⇒ P-MOS 没关断，**待机预算当场破** |
| 9 | `jog 1.0` | 小电压点动，确认转向与 atan2 方向一致 | 反了就改 `FOCSTEP_ENCODER_DIRECTION` |
| 10 | `align` | initFOC 电角对齐（**门要松开**） | — |
| 11 | `char 2.0` | 实测相电阻 → **回填 Kconfig `FOCSTEP_PHASE_RESISTANCE`** | — |
| 12 | `ipropi` | 两相电流，与万用表比对 → 标定 `A_IPROPI` | 手册正文 450 / 示例 455 自相矛盾 |
| 12b | `stall` | 堵转判定状态；跑 normal 负载记稳态**矢量幅值**峰值，取其 1.5~2× 回填 `FOCSTEP_STALL_CURRENT_MA` | 阈值必须 < ITRIP(≈1.49A) |
| 12c | `learn` | **无限位自学习**：顶两端机械限位，建立基准+行程范围并存 NVS。⚠️ 回零力矩默认仅 800mV，先确认机构上没人 | 报 `UNREPEATABLE` ⇒ 调小力矩或查机构 |
| 12d | `pos` | 位置/基准/可信度。跑完 `learn` 后应显示「位置可信: 是 ✅」 | 显示 ❌ ⇒ 再跑 `learn` |
| 13 | `int` | **INT 锁存语义：读数据是否清中断** | ❌ 则深睡档不成立，须改轮询 |
| 14 | `sleep` | 进深睡，量总电流 | 对比 `vref 1` 态，差值应 ≈**106µA** |
| 15 | `ota` | 打印运行/待升级分区与版本 | — |
| 16 | `net on` | 起 Wi-Fi+HTTP+mDNS，浏览器开 `http://focstep-xxxx.local/` **实测升级一次** | 装门前必须验通 |

> 第 13 项决定 §10.3 深睡档能否成立；第 8/14 项决定 0.25mW 待机预算是否守得住。
> 这两条是**整个低功耗方案的地基**。
>
> 第 16 项不是"锦上添花" —— 板子装进门里之后再拆成本极高，**OTA 通路必须先验通**。
> 验完记得 `net off`，否则待机功耗对不上。

---

## 7. 相对文档的设计偏差（都有依据，建议回改文档）

| 位置 | 文档写的 | 代码实际 | 依据 |
|---|---|---|---|
| `doc.md` §10.1 | PWM（**MCPWM** ×4） | **LEDC** ×4→2 | `esp_hal_stepper.cpp` 用 LEDC（20kHz/9-bit/LEDC_TIMER_0） |
| `doc.md` §10.1 | `StepperDriver4PWM` + **PMODE=高** | **`StepperDriver2PWM` + PMODE=低** | 手册真值表：PWM 模式 `(0,0)=Coast`，PH/EN 模式 `EN=0=Brake`。手册明确 *"In coast mode… cannot be sensed"* ⇒ 4PWM 会让 IPROPI 只在导通期有效 |
| `doc.md` §五.4 | VREF = 10k+22k 分压（**无门控**） | **加 nSLEEP 门控** | 原方案常态耗 106µA ≈ 0.42mW@24V，是待机预算 0.25mW 的 **1.7 倍** |
| `doc.md` §六 待机表 | 三项相加 = 0.25mW | **漏了 VREF 分压那 106µA** | 同上 |
| `doc.md` §十 :538 | 电流环走 DRV8874 IPROPI | **可行，已实现**（`FOCSTEP_TC_FOC_CURRENT`），但符号需重建、过零点有死区 | 见 §8 |

---

## 8. 电流环：`foc_current` 是**可以做的**（已修正）

> ⚠️ **本节曾给出错误结论**，说 foc_current"结构性做不到"。**那是错的。**
> 错因：把 `LowsideCurrentSense` —— 一个绑定 MCPWM 定时器 + 三电阻采样的**具体实现**
> —— 当成了拿到 `CurrentSense` 的唯一途径。实际上 `CurrentSense` 是**抽象接口，
> 只有两个纯虚**（`init` / `getPhaseCurrents`），和 `Sensor` 一样可以自己写子类。

### 事实（已逐条核对源码）

| 断言 | 核对结果 |
|---|---|
| 基类是否为步进内建了分支？ | ✅ `CurrentSense::getABCurrents()`：*"if so there is no need to Clarke transform"*，两相直接 `alpha=a, beta=b` |
| `driver_type` 怎么来的？ | ✅ `StepperDriver::type()` 返回 `DriverType::Stepper`，`linkDriver()` 自动取 |
| Park 变换通用吗？ | ✅ `getDQCurrents()` 与驱动类型无关 |
| `driverAlign()` 支持步进吗？ | ✅ 有专门分支 → `alignStepperDriver()` |
| `StepperMotor` 真的锁死 voltage 吗？ | ❌ **不是**。`StepperMotor.cpp:27` 只是**构造默认值**，`torque_controller` 是 `FOCMotor` 的 **public 成员** |
| `FOCMotor::loopFOC()` 处理 foc_current 吗？ | ✅ `:604` 的 switch 里有完整分支（双 PI + 交叉耦合补偿） |

⇒ `getFOCCurrents()`（foc_current 唯一用到的接口）**开箱即用**。
本工程已实现 `IpropiCurrentSense`（`ipropi_current_sense.cpp/.h`），
Kconfig 选 `FOCSTEP_TC_FOC_CURRENT` 即可启用，**已编译验证**。

### 真正的代价：符号，以及由此而来的死区

DRV8874 手册 §7.3.3.1：`IPROPI = (I_LS1 + I_LS2) × A_IPROPI`，且
*"ILSx is only valid when the current flows from drain to source... if current flows from
source to drain, the value of ILSx for that channel is **zero**"*
⇒ **IPROPI 恒 ≥ 0，只给幅值，没有符号。**

符号只能由**指令方向**重建（`SignTrackingStepperDriver` 在 `setPwm` 里记录）。
慢衰减期间电流继续沿原方向流动，所以"上一次指令的符号"仍然成立 —— 这也是选 PH/EN
而不是 PWM 模式的又一个理由（PWM 模式的 Coast 会切断电流通路）。

**由此而来的本质缺陷：**

| # | 限制 | 影响 |
|---|---|---|
| 1 | **过零点附近符号无意义** | 这是最本质的缺陷。因 \|I\|→0 也随之减小，误差有界，但零附近环不干净 |
| 2 | 符号有 **1 个 FOC 周期滞后**（1kHz 下 1ms） | 门机这种慢负载可忽略 |
| 3 | 两相**非同时采样**（两次 oneshot 相隔 ~40µs） | 4% 相位偏差 |
| 4 | `AERR` **±6%**（1–2A 档）；<0.4A 是 ±30mA 固定偏置 | 传感器偏粗，环的精度天花板 |
| 5 | 超过 ITRIP 读数**被钳位**（≈1.49A） | 环无法要求更高电流（但 DRV8874 自己会斩波） |

### 三档力矩环，按需选

| Kconfig | 前提 | 说明 |
|---|---|---|
| `FOCSTEP_TC_VOLTAGE` | 无 | 最简，开环力矩。**首次上板建议先用它**跑通 FOC 与限位 |
| `FOCSTEP_TC_ESTIMATED`（默认） | 实测相电阻 | `Uq = i_q·R + 反电动势`。拿电流环大部分收益，零额外硬件 |
| `FOCSTEP_TC_FOC_CURRENT` | 实测相电阻 + 整定 PID | 双 PI 实测闭环。**实验性**，先读上面的缺陷表 |

### 本阶段仍从 S0 起步

`S0`（两路 IPROPI → ADC → 堵转/堵门检测）依然是**第一步**，理由不变：
它独立有用（§10.4 的必需件），且**正是 foc_current 的前提**。
电流环的 PI 整定必须等上板、能看阶跃响应之后再做。

---

## 9. 无限位回零（sensorless homing）

**`doc.md` §10.4 ② 要求的"上电自学习限位"，现在实现了。** 不依赖任何限位开关。

### 判据：两条并行，任一成立即判接触

```
① 电流相对基线的抬升   peak > baseline × HOME_RISE_PCT   ← 早期检测, 接触瞬间即响应
② 位置停滞             位移 < 2 mrad                      ← 保底, 电流不可用时仍工作
   任一满足并持续 HOME_CONFIRM_MS ⇒ 判定接触
```

- **判据①必须用相对基线**。用绝对阈值会踩坑：回零力矩下自由运行的电流本就接近
  `ITRIP`，绝对阈值在接触前就已满足 ⇒ 判据形同虚设。（初版就是这么写的）
- **不能用"位置误差大"**：目标故意设在行程外，误差从一开始就是大的。正确表述是
  **"还在下指令，但实际位置不动了"**。
- 两者都是**比值/差分判据**，免疫 A_IPROPI 容差、温度、轨压漂移。

> ⚠️ `HOME_TORQUE_MV` **同时决定判据①有没有动态范围**，而且方向和"保护机构"一致：
> 力矩越小 → 自由运行电流越低 → 到钳位余量越大 → 检测越灵敏，且顶得越轻。
> 以 R=1.2Ω 为例：`1500mV`→1.25A，余量仅 1.19×（判据几乎不可用）；
> `800mV`→0.67A，余量 2.23×；`600mV`→0.50A，余量 2.98×。

### 与 TMC2209 StallGuard4 的对比

StallGuard4 测的是**负载角**（线圈磁场与转子磁场的夹角）：重载 → 负载角增大 →
`SG_RESULT` 下降，到 90° 即最大负载点。阈值 `SGTHRS`，`SG_RESULT < 2×SGTHRS` 时
DIAG 拉高。**本质是反电动势测量。**

| 维度 | TMC2209 StallGuard4 | 本方案 |
|---|---|---|
| 物理量 | 负载角（反电动势） | 电流相对基线的抬升 + 位置停滞 |
| 输出 | 连续量 `SG_RESULT` (0–510)，可感知负载**连续变化** | 二值 |
| **最低速度** | **须明显高于 ~1 rev/s**。低速下反电动势太小，测量不稳，"机械负载几乎不影响结果" | **无限制**，可以极慢 |
| 依赖编码器 | 不需要 | **需要**（本板有） |
| 绝对位置 | 无 | **有**（单圈绝对 + 多圈 NVS） |
| 阈值标定 | 必须，且**绑定速度** | 必须，但与速度无关 |
| 触发时机 | 负载角增大即响应 | 电流抬升即响应（已对齐） |

**结论**：

- **回零这个用途上本方案不差，两项更好**：① **速度不受限** —— 这是决定性的，
  回零本就该慢，而 StallGuard 恰好要求快，快撞限位是要出事的；
  ② **有绝对位置**，判定不依赖任何模拟量的绝对精度。
- **一项原本更差、现已对齐**：StallGuard 能在负载刚开始增大时就停；本方案原来靠
  "位置停住"判定，机构已被顶住。改用**相对基线的电流抬升**后，响应时机基本对齐。
- **仍不如的一项**：StallGuard 给的是**连续负载量**，能用于 CoolStep 那种连续力矩
  调节；本方案只给二值"到/没到"。对本应用（回零）无影响。

### 流程

1. 电压上限降到 `FOCSTEP_HOME_TORQUE_MV`（默认 **1500mV**）
   > ⚠️ **"力小"和"速度慢"是两回事**。本实现限的是电压上限（决定力矩），
   > 不是把速度调慢 —— 慢速大力同样会顶坏东西。
2. 目标设到行程外 1.2×，朝该端推
3. 轮询 100ms：位置不动 + 电流抬升，持续确认 → 判定接触
4. **退避** `HOME_BACKOFF`（松开机械限位，避免长期顶住）
5. **重复性检查**：再做一次，两次接触点之差须 ≤ `HOME_REPEAT_TOL_MRAD`
   > **这一步不能省** —— 偏一步的基准会让机构永远偏，而且现场很难发现。
6. `learn` 会依次顶两端，学到 `[全关角, 全开角]` 并存 NVS

### ★ 一个必须知道的固有限制

`KTH5701` 是**单圈绝对**编码器。深睡期间 C6 不在计数，而**手拉门是本产品的核心用法**
⇒ 多圈计数会发散。

**这个歧义是根本性的**：只有单圈绝对传感器，**分不清"动了 0.3 圈"与"动了 1.3 圈"**
—— 单圈读数一样。

所以策略只能是：**动过就标记不可信，下一条运动指令前先回零**。具体做法：

- 深睡存盘时存 **(圈数, 当时的单圈角)** 一对（不是只存圈数）
- 唤醒时比对单圈角，超 `FOCSTEP_POS_TRUST_TOL_MRAD`（默认 30 mrad）即判"被动过"
- 被编码器 INT 唤醒（= 睡着时被拉动了）也主动作废位置
- **位置不可信时 `foc_motor_move_to()` 直接拒绝执行** —— 否则会以错误基准冲到机械限位

### 想完全避免回零的话

如果**总行程 < 电机一圈**（机构减速比足够小、或编码器装在减速之后），单圈绝对角就能
唯一确定全程位置 ⇒ 永不需要回零。本板 42 步进直驱皮带轮通常做不到
（1m 行程约 10 圈），所以回零是必需的。

---

## 10. KTH5701 驱动：自研，不移植 GPL

`driver/main/kth5701.cpp/.h` **独立自写**。参考仓库
`github.com/hgddingjun/KTH5701` 是 **GPL-2.0** 的 MediaTek Linux 内核驱动 +
Android app，**只取协议规格（命令字/帧格式/寄存器访问时序）作为接口事实**，
不复制代码 —— 避免 GPL-2.0 传染到商用固件。

参考实现本身也不完整，这是不移植的另一个理由：

- **芯片不输出角度**：它给的是 X/Y/Z 磁场 + 温度。参考的 Android 侧把 Y 轴磁场
  当"角度"显示（Y 随转角是正弦，指针看着能转，但是错觉），且那段 Java 还有
  `(byte)` 强转的符号 bug（高字节 ≥0x80 时一半量程全错）。内核侧 `x_min/x_max/y_min/y_max`
  声明了从未使用 ⇒ **标定从未实现**。
- **INT 处理不符数据手册语义**：它每次触发就翻转触发极性、从不读数据清锁存。
  手册是"高有效、**锁存**、读数据清零" ⇒ 必须 `ext1 ANY_HIGH` + **ISR 里读一次数据**。

⇒ **atan2、椭圆标定、零位、方向、多圈，全部由本驱动实现**，多圈存 NVS 以跨深睡。

---

## 11. 已知未完成项

| 项 | 说明 |
|---|---|
| ~~OTA 未接~~ | ✅ 已接入：`net_ota.c` + `ota_page.html`（官方 `esp_ota_ops`/`esp_http_server`）。⚠️ **上板第一件事就是验它** —— 装进门里再拆成本极高 |
| **EAD 未做** | PA 载荷未加密。`receiver_id/session/sequence` 只能防误触发，不能防伪造。量产前必须补 |
| ~~Wi-Fi 控制面未接~~ | ✅ 已接入：`net_ota.c`（官方 `esp_wifi`/`mdns`）。**默认关闭**（`FOCSTEP_WIFI_ENABLE=n`），因为开着会打破深睡；用 `net on` / `net off` 临时启用 |
| **无线唤醒接收未接** | 驱动板当前只有本地唤醒源（编码器 INT）。PA 接收端需把 `tests/ble_sync/ble_periodic_sync` 的逻辑并进来 |
| ~~多圈限位自学习未做~~ | ✅ 已实现：`homing.c` + `home`/`learn`/`pos` 命令（见 §9）。⚠️ 回零力矩、重复性容差、可信度容差**都必须上板实测整定** |
| **相电阻/相电感未实测** | `FOCSTEP_PHASE_RESISTANCE` 是占位值，须 `char` 命令实测回填 |
