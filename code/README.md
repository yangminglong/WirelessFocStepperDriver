# FocStepper 固件

> 状态：**代码完成，板未回，未上板验证**
> 设计依据：[`docs/doc.md`](../docs/doc.md) · 无线唤醒选型：[`docs/wireless_wakeup_options.md`](../docs/wireless_wakeup_options.md)
> ⚠️ `doc.md` 是**原理图与固件的共同真源**：文档改了必须同步固件（`board_pins.h` / Kconfig），反之亦然。

两个 ESP-IDF 工程 + 一份共享协议。**验收标准是 `idf.py build` 通过**——
上板动作全部写成了自检命令，等板回来一次跑完。

```text
code/
├── components/
│   ├── foc_link_protocol/          承载层：PA 帧格式（两端共用；**只搬字节**）
│   ├── foc_door_link/              应用层：无线指令集（开/关/停/唤醒周期）
│   └── focstep_platform/           ★ 平台层：跨项目复用的底层驱动
├── driver/                         ESP32-C6 驱动板固件
│   └── main/                       应用层（推拉门）
└── wake_sender/                    无线唤醒发送端（PA 广播端）
```

---

## 1. 构建

两个工程都是 **esp32c6**，验收标准是各自 `idf.py build` 通过。

### Linux

```bash
export IDF_PATH=/home/hanson/.espressif/v5.5.5/esp-idf
export IDF_TOOLS_PATH=/home/hanson/.espressif/tools
export ESP_ROM_ELF_DIR=/home/hanson/.espressif/tools/esp-rom-elfs/20241011
export IDF_PYTHON_ENV_PATH=/home/hanson/.espressif/tools/python/v5.5.5/venv
export PATH="/home/hanson/.espressif/tools/ninja/1.12.1:\
/home/hanson/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20260121/riscv32-esp-elf/bin:\
/home/hanson/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin:\
$IDF_PYTHON_ENV_PATH/bin:$PATH"

for p in driver wake_sender; do
  (cd $p && rm -f sdkconfig && idf.py build)
done
```

> ⚠️ `~/.espressif/tools/activate_idf_v5.5.5.sh` 把 `idf.py` 定义成 **shell alias**，
> 非交互 shell（脚本/CI）里不生效，必须用上面的显式方式。
>
> ⚠️ **两端必须同版本**：`dependencies.lock` 记的是 IDF 5.5.5，用别的版本编会报 mismatch。

### Windows（eim 装的 IDF，在 Git Bash 里）

本机 IDF 5.5.5 在 `Q:\espressif\.espressif\v5.5.5\esp-idf`，工具链在 `C:\Espressif\tools`
（真值见 `C:\Espressif\tools\Microsoft.v5.5.5.PowerShell_profile.ps1`）：

```bash
export IDF_TOOLS_PATH='C:\Espressif\tools'
export IDF_PATH='Q:\espressif\.espressif\v5.5.5\esp-idf'
export IDF_PYTHON_ENV_PATH='C:\Espressif\tools\python\v5.5.5\venv'
export ESP_ROM_ELF_DIR='C:/Espressif/tools/esp-rom-elfs/20241011/'
export PATH="/c/Espressif/tools/ccache/4.12.1/ccache-4.12.1-windows-x86_64:\
/c/Espressif/tools/cmake/3.30.2/bin:\
/c/Espressif/tools/ninja/1.12.1:\
/c/Espressif/tools/riscv32-esp-elf/esp-14.2.0_20260121/riscv32-esp-elf/bin:\
/c/Espressif/tools/python/v5.5.5/venv/Scripts:$PATH"

idf.py() { MSYS_NO_PATHCONV=1 "$IDF_PYTHON_ENV_PATH/Scripts/python.exe" \
           "$TEMP/idfpy_wrapper.py" "$@"; }
```

`idfpy_wrapper.py` 见下面第 3 条。三条**必须知道**的（都实际踩过）：

| # | 坑 | 现象 | 做法 |
| --- | --- | --- | --- |
| 1 | `export.sh` 拒绝 MSYS | 直接 `ERROR: MSys/Mingw is not supported` | **别用它** —— `idf.py` 本质就是 venv 的 `python.exe` 跑 `esp-idf/tools/idf.py`，直接调即可 |
| 2 | Git Bash 不认 `C:/...` 形式的 PATH 项 | `which` 静默跳过它们，落到系统 PATH 里的别的版本（如 `P:\CMake 3.31.12` 顶掉 Espressif 的 3.30.2） | PATH 写 **POSIX 形式 `/c/...`**；`IDF_PATH` / `IDF_TOOLS_PATH` 这类给 Windows python 读的变量保持 Windows 形式 |
| 3 | `MSYSTEM` 摘不掉 | `idf.py` 只打一句 "or continue at your own risk" 就退出 0，**什么都没编** | 用包装层：在自己进程里 `os.environ.pop('MSYSTEM', None)` 再拉起 `idf.py` |

第 3 条的原因在 `idf.py` 的 `__main__`：

```python
if 'MSYSTEM' in os.environ:
    print_warning('MSys/Mingw is no longer supported. ... or continue at your own risk.')
elif ...
else:
    main()          # ← MSYSTEM 存在时永远走不到这里
```

那句 "or continue at your own risk" 是**假的** —— 它并不 continue。而 MSYS2 运行时会**强制**
把 `MSYSTEM` 注入给每个原生 Windows 子进程，所以 `unset MSYSTEM` / `env -u MSYSTEM`
都无效（已验证），只能靠包装层。`%TEMP%\idfpy_wrapper.py` 内容就是那三行。

> ⚠️ **改完 `sdkconfig.defaults` 必须 `rm -f sdkconfig`** —— 否则新条目与新增
> Kconfig 符号都不会生效（后者直接编译报未声明）。本项目踩过两次。
>
> ⚠️ `sdkconfig` **不入库**（见 `.gitignore`），所以删它只会丢掉本机的 menuconfig 改动。
> 只想验证编译、没动过 `sdkconfig.defaults` 时，**不必删**。

---

## 2. 分层：平台层 vs 应用层

**平台层 `components/focstep_platform/` 不含任何应用语义**，换项目整个目录拷走即可。

|  | 内容 | 行数 |
| --- | --- | --- |
| **平台层** | `board_pins` / `kth5701` / `ipropi_*` / `bus_voltage` / `foc_motor` / `homing` / `power_state` / `wakeup` / `led_ws2812` / `platform_button` / `can_link` / `net_ota` / `platform_events` / `platform_console` | 4848 |
| **应用层** | `app_main.c` / `app_door.c` / `app_console.c` | 614 |

三条边界纪律（已校验）：

1. **平台层不引用应用级 Kconfig**（`ASSIST_TORQUE` / `IDLE_TO_SLEEP_MS` / … 都在 `driver/main/Kconfig.projbuild`）
2. **底层模块只发事件，不做决策**。按键、堵转、CAN 指令、nFAULT 都发 `FOCSTEP_EVT_*`，应用订阅后决定怎么做 —— 按键模块不得直接调 `power_state_*()`，那是层次倒置
3. **平台内部模块直接互相调用**，不设函数指针钩子表（`power_state_hooks_t` 那类表单订阅、字段定形，换个应用就得改结构体）

应用层怎么接：

```c
platform_events_init();
esp_event_handler_instance_register(FOCSTEP_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL);
platform_console_init();                                    // 建 REPL + 平台自检命令
platform_console_register("open", "开门", my_open_cb);      // 追加自己的命令
```

**§10.3 的四态是应用概念**：平台只有 `SLEEP / ACTIVE / FAULT` 三态；"唤醒接管/运行"是 `ACTIVE` 的两种子模式，差别在"电机做什么"——由应用控制（见 `app_door.c`）。同理，平台只提供**灯色**，不规定"什么颜色代表什么状态"。（**CAN 的 Rs 不在应用控制范围内**——它随 nSLEEP 硬件派生，见 `doc.md` §5.3。）

> ⚠️ `wake_sender` 的 `EXTRA_COMPONENT_DIRS` 只指向**两个共享协议**组件
> （`../components/foc_link_protocol` 与 `../components/foc_door_link`）——
> 写成 `../components` 会把驱动板的平台层也编进来（发送端没有电机，且平台引用的是驱动板项目的 Kconfig）。
>
> **协议分两层**（纪律，见 §13.5 第 5 条）：承载层 `foc_link_protocol` 只搬字节、不定义业务命令；
> 命令集 `foc_door_link` 属于产品。换产品只换后者；只有**帧格式**变了才动 `FOC_LINK_VERSION`。

---

## 3. 组件选型：优先用官方组件

**原则：设计一个功能之前，先查组件仓库有没有现成的。** 只有确认没有才自己写。

已核实的仓库现状（2026-09-18）：

| 功能 | 组件 | 说明 |
| --- | --- | --- |
| FOC 电机控制 | **`espressif/esp_simplefoc` 1.4.1** | 传递拉入 `arduino-foc`(SimpleFOC v2.4.0 移植) + `iqmath` + `i2c_bus` |
| **I2C 总线 / 器件** | **`espressif/i2c_bus` 1.5.2** | 由 esp_simplefoc 传递依赖（给它的 AS5600 路径用）。**本工程的编码器与 DAC 不经过它** |
| **I2C 器件驱动** | **`esp-idf-lib/i2cdev`** | 建在 `driver/i2c_master` 之上，编码器（KTH5701）与 DAC（MCP4725）共用一条总线 —— 见 `components/focstep_platform/idf_component.yml` |
| **DAC (VREF)** | **`esp-idf-lib/mcp4725`** | MCP4725 动态 VREF，见 `vref_dac.c` / doc.md §5.4 |
| WS2812 底层 | **`espressif/led_strip` 3.0.3** | RMT 驱动 |
| **灯效/闪灯模式** | **`espressif/led_indicator` 2.1.2** | 闪灯时序与模式表是它的本职 —— 不必自己写闪灯状态机 |
| **按键消抖/事件** | **`espressif/button` 4.2.1** | 短按/双击/长按识别。GPIO9 **严禁加消抖电容**，所以消抖只能做在固件里，正是它的用途 |
| **局域网发现** | **`espressif/mdns` 1.12.0** | OTA 页面走 `focstep-xxxx.local` |
| OTA 写入 | IDF 内置 `esp_ota_ops` / `esp_http_server` | 上传页 `ota_page.html` 随固件 EMBED 进去 |
| 其余 | IDF 内置 | `twai`(CAN)、`esp_adc`、`esp_console`(命令台)、`nvs_flash`、`esp_pm` |

**确认没有官方组件、必须自研的**（已查组件仓库）：

| 功能 | 结论 |
| --- | --- |
| **KTH5701 驱动** | 仓库无此组件 ⇒ 自研 `kth5701.cpp`（见 §10）。另有一个 GPL-2.0 的 Linux 驱动可参考协议，**不取代码** |
| **DRV8874 驱动** | 无官方组件。但本设计不需要 —— `StepperDriver2PWM` 直接产生 PH/EN 波形 |
| 电流采样 | `LowsideCurrentSense` 在本拓扑不可用（它绑定 MCPWM 定时器，步进驱动走 LEDC）⇒ 自研 `ipropi_sense.c` + `IpropiCurrentSense`（见 §8）。**注意 `CurrentSense` 本身是抽象接口，可以自己写子类** |
| 电源状态机 | 应用专属，无组件可代 |

> 顺带核实过：`espp/*`（magnetic_encoder / bldc_motor / twai / pid）是**第三方**组件（espp 组织），
> 不是 Espressif 官方，未采用。`ulipe/espfoc` 同为第三方且维护活跃度低。

---

## 4. 引脚与硬件真源

**唯一真源是 [`components/focstep_platform/include/board_pins.h`](components/focstep_platform/include/board_pins.h)**，抄自 `doc.md` §四。
其他文件一律引用它，不得硬编码 GPIO 号。

关键约束（详见该头文件注释）：

| 项 | 要点 |
| --- | --- |
| LP 域 GPIO0~7 | **已用满，无空闲 GPIO**。新增功能只能复用既有信号 |
| ADC1 | GPIO0~6 七路，扣掉晶振后实际 5 路，本项目用满 3 路（GPIO2 母线 + GPIO4/5 IPROPI） |
| Strapping | GPIO8 **严禁加下拉**（与 GPIO9 同为 0 是非法 boot 组合） |
| JTAG | GPIO4~7 是 RISC-V JTAG 复用脚，而本板把它们分给了 ADC（IPROPI GPIO4/5）、WS2812 DIN（GPIO6）、母线门控（GPIO7）⇒ **外部 JTAG 不可用**，只能走 USB-Serial-JTAG |

---

## 5. ★ 上板前必须知道的裁决点

§5.2 / §5.3 两处是**文档与手册/源码存在冲突**，代码里做成了可配置项，**必须上板用自检命令裁决**，
不能假定哪一方对；§5.1 已定稿，自检项保留用于**验打样是否接对**：

### 5.1 EN/PH 引脚（自检用于验线）

- 手册引脚命名是 **EN/IN1** 和 **PH/IN2**（**EN 在 IN1 上、PH 在 IN2 上**）
- `doc.md` 已按此定线：**GPIO19/21 → IN1(EN)**、**GPIO20/22 → IN2(PH)**
- 固件默认（`FOCSTEP_PHEN_SWAPPED=n`）与原理图一致，**正常打样不必动**

PH/EN 模式下**只有 EN=0 是 Brake**。若打样接反，"先切 brake"会变成"EN 仍为 1、PH=0"
= **满压反转驱动**，唤醒瞬间猛冲。

⇒ 验线：`brake` 命令 → 手转轴应有明显阻尼且电机不主动转。
接反了就把 Kconfig 的 `FOCSTEP_PHEN_SWAPPED` 打开重烧（台面救回，不必重画板）。

### 5.2 KTH5701 寄存器初始化序列

参考实现（真板跑通）写 `0x1C=0x1636 / 0x1D=0x0002 / 0x1E=0x8000`；
本项目从数据手册记的是 `0x1D=wakeDiff / 0x28=过采样 / 0x29=measTime`。**两边对不上。**

⇒ 裁决：`regs` 命令回读比对。默认按参考实现那组走（`FOCSTEP_ENCODER_REG_INIT`）。
文档侧已在 §九「待核实」**#14** 立档，裁决后回写 `doc.md`。

### 5.3 CAN 波特率

`doc.md` 已在 §九 待定决策 **#3** 立档：固件默认 **500k 是占位值**。
定稿需与组网规划一致，**改则文档与固件同步**。

### 5.4 ⚠️ CAN 与调试口是**构建期二选一**（不是运行时开关）

GPIO16/17 由 **UART0 控制台（4P 排针）与 CAN 收发器共用**，而 **TWAI 一旦 start 就占用
这两个脚、把 UART0 顶掉** ⇒ 只能做成 Kconfig 档位（`FOCSTEP_CAN_ENABLE`，**默认 `n`**）：

| 档 | `FOCSTEP_CAN_ENABLE` | 控制台 sdkconfig | 4P 排针 | CAN |
| --- | --- | --- | --- | --- |
| **调试档（默认）** | `n` | `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`（不动） | ✅ 可用 | ❌ |
| **CAN 档** | `y` | 改 `ESP_CONSOLE_USB_SERIAL_JTAG=y` + `USJ_ENABLE_USB_SERIAL_JTAG=y` + `USJ_NO_AUTO_LS_ON_CONNECTION=y` | ❌ | ✅ |

⚠️ **只改一半会得到"日志消失"或"CAN 不通"** —— 两档必须配套改。

建议**默认档**下再加 `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y`，让 Type-C 也能
镜像日志（**只读**），这样"看日志"两个口都能做，日常不必来回换档。

**硬件侧的配套**：U6 的 RXD 与 GPIO17 之间串了 **1.5kΩ**（`doc.md` §5.3）——
它让收发器在线时 USB-UART 适配器**仍能拉低 GPIO17**。**漏贴这个电阻，4P 调试口就收不到数据**
（且两个推挽输出对顶互灌十几~几十 mA）。

**使用纪律**：用 4P 调试口时 **CAN 总线必须拔掉**，否则 UART 日志会经收发器灌上总线、扰乱全网。

---

## 6. 上板自检顺序

按顺序敲（命令台 `focstep>`，输入 `help` 可看全部）：

| # | 命令 | 验什么 | 不过怎么办 |
| --- | --- | --- | --- |
| 1 | `id` | 芯片 ID `0x0D == 0x0203` | **不通过就别往下走**，查 I2C 地址/上拉/供电 |
| 2 | `regs` | 覆写并回读 0x1C/0x1D/0x1E | 见 §5.2 |
| 3 | `circle 200` | 手转一圈，看 **XY 是否走出一个圆** | 圆度 < 1.10 合格 |
| 4 | `cal` | 用上一步的 min/max 算硬铁/软铁并应用 | — |
| 5 | `vbus` | 母线电压，与万用表比对 | 差得多就查分压比/门控 |
| 6 | `gate 0` | 门控关断后 ADC 节点应为 **0V** | 若为 24V ⇒ 门控做在低边了 |
| 7 | `brake` | **两相 EN 拉低 = Brake**，手转有阻尼、电机不主动转 | 见 §5.1 |
| 8 | `vref 1.5` / `vref 0` | **VREF 引脚 ≈2.35V / 0V（0 档并进 PD）**；换算按 `FOCSTEP_ITRIP_MA` / `R_IPROPI` / `A_IPROPI` 现算 | 0V 后总电流不回落 ⇒ DAC PD 没生效，**待机预算当场破**。⚠️ 门处于带电运行态时本命令会拒绝执行 |
| 9 | `jog 1.0` | 小电压点动，确认转向与 atan2 方向一致 | 反了就改 `FOCSTEP_ENCODER_DIRECTION` |
| 10 | `align` | initFOC 电角对齐（**门要松开**） | — |
| 11 | `char 2.0` | 实测相电阻 → **回填 Kconfig `FOCSTEP_PHASE_RESISTANCE`** | — |
| 12 | `ipropi` | 两相电流，与万用表比对 → 标定 `A_IPROPI` | 手册正文 450 / 示例 455 自相矛盾 |
| 12b | `stall` | 堵转判定状态；跑 normal 负载记稳态**矢量幅值**峰值，取其 1.5~2× 回填 `FOCSTEP_STALL_CURRENT_MA` | 阈值必须 < ITRIP(≈1.494A) |
| 12c | `learn both [pos]` | **标定行程**：顶两端机械限位，建立**零点(0%)**与**满行程点(100%)**并存 NVS。只给**零点的方向**（`learn both pos` = 零点在正端/反装机构），满行程点必然是反向。⚠️ 回零力矩默认仅 800mV，先确认机构上没人 | 报 `UNREPEATABLE` ⇒ 调小力矩/查机构；报 `RANGE` ⇒ 复核方向与机构 |
| 12c' | `home pos\|neg` → `learn zero\|end` | **单端重标**两步走：先顶限位**测**接触点（不动行程），确认日志里的接触点与移动量合理，再**落定**（不动电机）。拆两步是为了拦住"撞到异物被误判成接触点"直接写进标定 | `learn zero\|end` 报"还没有测到的接触点" ⇒ 先跑 `home` |
| 12d | `pos` | 行程**三态**（未标定 / 只有零点 / 零点+满行程点）+ 位置可信度。标定完应显示「零点+满行程点 + 可信 ✅」 | 未标定或不可信 ⇒ 先 `learn`/`mark` |
| 12e | `mark zero` / `mark end` | 手推到机械端点后**免拆卸重标**（不推限位，要求静止）。`mark zero` 立即把该点定为系统零点**并让位置可信** | 报 `INVALID_STATE` ⇒ 门还在动，等停稳；`mark end` 要求先有零点 |
| 12f | `goto 0.35` / `goto_x -0.05` / `goto_rad 3.14` | `goto` = 归一化开度（**夹在 0..1**，走完整门禁）；`goto_x` = 允许**越界**（越过零点/超出满行程）；`goto_rad` = 绝对角（不过门禁，自检用） | 报 `INVALID_STATE` ⇒ 位置不可信或行程不完整 |
| 12g | `dir` / `dir invert` | 开度方向：`dir invert` **交换零点与满行程点**（= 正方向取反，纯数据变换） | 「开/关」与门的开合相反时用它；**不要**去改编码器方向（那会按指纹作废行程） |
| 13 | `int` | **INT 锁存语义：读数据是否清中断** | ❌ 则深睡档不成立，须改轮询 |
| 14 | `sleep` | 进深睡，量总电流 | 睡眠态应已发 DAC PD（VREF=0）：量 VREF=0 且总电流落入深睡档基线（≈25~65µA@24V） |
| 15 | `ota` | 打印运行/待升级分区与版本 | — |
| 16 | `net on` | 起 Wi-Fi+HTTP+mDNS，浏览器开 `http://focstep-xxxx.local/` **实测升级一次** | 装门前必须验通 |
| 17 | `sleep`（**轻睡档**，Kconfig 选 `FOCSTEP_SLEEP_MODE_LIGHT`） | 整机电流、唤醒延迟、**有没有 1 秒内反复唤醒** | 反复唤醒 ⇒ INT 锁存没清干净（见 §12） |
| 18 | `wl` / `wl pa` / `pa_send` | 无线唤醒：PA 同步、`T` 实得值、`pa_send` 验指令链、接收端电流 | 见 **§13.6** 的 10 项 |

> 第 13 项决定 §10.3 深睡档能否成立；第 8/14 项决定**深睡档待机预算（`doc.md` §六：25~65µA @24V）**是否守得住（VREF 走 MCP4725 PD，等效 ≈0）。
> 这两条是**整个低功耗方案的地基**。
>
> 第 16 项不是"锦上添花" —— 板子装进门里之后再拆成本极高，**OTA 通路必须先验通**。
> 验完记得 `net off`，否则待机功耗对不上。
>
> 第 17 项只在选了轻睡档时需要跑；DeepSleep 档跑第 14 项即可。两种睡眠档的差别见 §12。

---

## 7. 关键选型依据（改动前先读这里）

> 下面每条都已是 `doc.md` 的现行口径（§四 引脚表、§10.1 驱动架构与电流环、
> §五.4 + §六 的 VREF 方案与待机表）。**改动前先读这一节。**

| 位置 | 设计选择 | 依据 |
| --- | --- | --- |
| `doc.md` §四、§10.1 | **LEDC ×2 + 2 路方向电平** | `esp_hal_stepper.cpp` 用 LEDC（20kHz/9-bit/LEDC_TIMER_0）；LEDC 走 GPIO 矩阵，任意脚可用 |
| `doc.md` §10.1 | **`StepperDriver2PWM` + PMODE=低（PH/EN）** | 手册真值表：PWM 模式 `(0,0)=Coast`，PH/EN 模式 `EN=0=Brake`。手册明确 *"In coast mode… cannot be sensed"* ⇒ 4PWM 会让 IPROPI 只在导通期有效 |
| `doc.md` §五.4 | **MCP4725 动态 VREF**：12-bit DAC 直驱（I2C 0x60），ITRIP 档位表 0.3~1.8A，待机进 PD（VREF=0，60nA typ / 2µA max） | 固定分压两宗罪：① 无法运行时按档降 ITRIP（低电流档分辨率全靠降 VREF）；② 待机无断耗通路（210µA vs PD 60nA） |
| `doc.md` §六 待机表 | 有「**MCP4725 PD ≈ 0**」一行（60nA，等效 ≈0） | 睡眠态漏发 DAC PD = 210µA 常挂 3.4V 轨，待机预算当场破 |
| `doc.md` §10.1 | 电流环走 IPROPI，**`FOCSTEP_TC_FOC_CURRENT`** | 可行，符号需重建、过零点有死区 —— 见 §8 |

---

## 8. 电流环：`foc_current` 是**可以做的**

> ⚠️ **别被 `LowsideCurrentSense` 误导。** 它是一个绑定 MCPWM 定时器 + 三电阻采样的
> **具体实现**，不是拿到 `CurrentSense` 的唯一途径。`CurrentSense` 是**抽象接口，
> 只有两个纯虚**（`init` / `getPhaseCurrents`），和 `Sensor` 一样可以自己写子类。

### 事实（已逐条核对源码）

| 断言 | 核对结果 |
| --- | --- |
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
| --- | --- | --- |
| 1 | **过零点附近符号无意义** | 这是最本质的缺陷。因 \|I\|→0 也随之减小，误差有界，但零附近环不干净 |
| 2 | 符号有 **1 个 FOC 周期滞后**（1kHz 下 1ms） | 门机这种慢负载可忽略 |
| 3 | 两相**非同时采样**（两次 oneshot 相隔 ~40µs） | 4% 相位偏差 |
| 4 | `AERR` **±6%**（1–2A 档）；<0.4A 是 ±30mA 固定偏置 | 传感器偏粗，环的精度天花板 |
| 5 | 超过 ITRIP 读数**被钳位**（≈1.494A） | 环无法要求更高电流（但 DRV8874 自己会斩波） |

### 三档力矩环，按需选

| Kconfig | 前提 | 说明 |
| --- | --- | --- |
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

```text
① 电流相对基线的抬升   peak > baseline × HOME_RISE_PCT   ← 早期检测, 接触瞬间即响应
② 位置停滞             位移 < 2 mrad                      ← 保底, 电流不可用时仍工作
   任一满足并持续 HOME_CONFIRM_MS ⇒ 判定接触
```

- **判据①必须用相对基线**。用绝对阈值会踩坑：回零力矩下自由运行的电流本就接近
  `ITRIP`，绝对阈值在接触前就已满足 ⇒ 判据形同虚设。
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
| --- | --- | --- |
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
- **响应时机已对齐**：StallGuard 能在负载刚开始增大时就停；本方案的**相对基线电流抬升**
  判据在接触瞬间即响应，不必等"位置停住"（那时机构已被顶住）。
- **仍不如的一项**：StallGuard 给的是**连续负载量**，能用于 CoolStep 那种连续力矩
  调节；本方案只给二值"到/没到"。对本应用（回零）无影响。

### 流程

1. 电压上限降到 `FOCSTEP_HOME_TORQUE_MV`（默认 **800mV**）
   > ⚠️ **"力小"和"速度慢"是两回事**。本实现限的是电压上限（决定力矩），
   > 不是把速度调慢 —— 慢速大力同样会顶坏东西。
2. 目标设到行程外 1.2×，朝该端推
3. 轮询 100ms：位置不动 + 电流抬升，持续确认 → 判定接触
4. **退避** `HOME_BACKOFF`（松开机械限位，避免长期顶住）
5. **重复性检查**：再做一次，两次接触点之差须 ≤ `HOME_REPEAT_TOL_MRAD`
   > **这一步不能省** —— 偏一步的基准会让机构永远偏，而且现场很难发现。
6. 标定行程：`learn` 默认依次顶两端，把接触点写进**零点(0%)**与**满行程点(100%)**并存 NVS

### 行程模型：零点 + 满行程点（不是 min/max）

行程由**两个语义端点**定义，**不按角度大小排序**：

```text
零点 (0% 开度) ──span──► 满行程点 (100% 开度)      target = 零点 + 开度 × span
```

- 两个端点各存一个**绝对多圈角**（rad），`span = 满行程点 − 零点` **带符号**
  ⇒ "正方向"就是 span 的符号，**反装机构**（零点的角比满行程点更大）天然表达得出来
- 三态：`未标定` / `只有零点` / `零点+满行程点`。「全开/全关」**只在第三态成立**，
  缺一个就直接拒绝开度指令（不设占位角兜底 —— 占位角与真实机构无关，照它跑就是用错误基准冲限位）

### 标定行程的入口

| 命令 | 做什么 | 什么时候用 |
| --- | --- | --- |
| `learn both [pos\|neg]` | 顶限位标两端，带**两次逼近 + 重复性检查**。只给零点方向 | 首次装机 / 换电机 / 换机构 |
| `home <方向>` → `learn zero` / `learn end` | 单端重标两步：`home` 只**测**（不动行程），`learn zero\|end` 只**写**（不动电机），**另一端的物理位置保持不变**（span 按新端点重算） | 只有一端够得着 / 只有一端动过 |
| `mark zero` / `mark end` | **把当前位置定为端点**，不推限位（要求电机静止） | 手推到机械端点后免拆卸重标；或限位够不着时 |
| `learn auto [on\|off]` | 无人值守自动标定：受**策略门控**（`auto_calib` 默认关，Kconfig `FOCSTEP_HOME_AUTO_ENABLE`），方向也来自策略（`FOCSTEP_HOME_AUTO_INVERTED`，因为无人值守时"哪侧是零点"推不出来），失败**按 `FOCSTEP_HOME_RETRY` 重试**并报 `PS_FAULT_CALIB` | 上电自学习（待接：目前只能手动触发） |
| `pos` / `status` | 看行程三态与位置可信度 | 每次开工先看一眼 |

**方向 vs 端点语义：两个量各自在"能知道的那一侧"给**：

- **标定前**能知道的只有**方向** —— "哪一侧是零点"正是待求量 ⇒ 命令给 `pos`/`neg`
- **标定后**能派生的是**端点身份与方向** —— span 的符号就是方向的定义
  （`span > 0` = 零点在角度小的一侧）⇒ 命令可以用 `zero`/`max`（`home zero` 会自动翻成方向）

所以只有 2 种安装形态，不是 4 种：`learn both` 默认零点在负端，反装机构用 `learn both pos`
（零点在正端）—— 满行程点的方向**必然是零点的反向**（两端都是顶出来的机械限位，而推拉机构
只有两个），不需要也不应该手给。把两者绑死（"全关端"必朝负、"全开端"必朝正）会让
反装机构根本表达不出来 ⇒ 不设 `learn closed|open` 写法。

**单端重标为什么拆成两步**：`home`（只测，不动行程）+ `learn zero|end`（只写，不动电机）。
一次到底时，门中途撞到异物被接触判据误判出来的**假端点会直接写进标定**；拆开后操作员先看到
"接触[位置停滞]、自起点只移动 0.03 rad" 这种离谱数字，可以否掉再重测。

**写入校验**（在 `foc_motor` 里，是唯一入口）：跨度 < 0.1 rad ⇒ 报
`RANGE` 且**整笔不写**（旧的标定继续有效，提示复核方向/机构）；
`mark end` / `learn end` 在**还没有零点**时直接拒绝（没有零点就没有 100% 可言）；
若本次落定把 span **变号**（= 0%/100% 与门开合的对应关系翻转，正是 `dir invert` 的语义）
⇒ 大声告警但**不拦**（标定是有意的人为动作）。

**失败处理与安全前提**（`homing_policy_t`，默认值来自 Kconfig，运行时可改）：

- 自动标定默认**禁止**（`FOCSTEP_HOME_AUTO_ENABLE=n`）——它会让门自己跑到底顶限位，
  必须在现场确认机构通畅后显式打开（`learn auto on` 或 `homing_set_policy()`）
- 失败 → 按 `FOCSTEP_HOME_RETRY`（默认 1）重试；仍失败则**保留原有标定不动**、
  位置仍不可信、报 `PS_FAULT_CALIB`（灯闪 7 次）并广播 `FOCSTEP_EVT_HOMED`

### 行程的两条铁律（固件会主动拦）

1. **行程 = 编码器机械角坐标系里的两个端点** ⇒ `FOCSTEP_ENCODER_DIRECTION` /
   `FOCSTEP_ENCODER_ZERO_OFFSET` 一改，旧行程就不再对应机械端点。
   所以标定时把这两个值当**指纹**一起存 NVS，开机比对不上就
   **自动作废行程 + 标记位置不可信**（必须重标）—— 而不是静默沿用：
   后者会"能跑但整段错位"，现场基本发现不了。
2. **从未标定时拒绝一切开度指令**（`move_to()` / 应用的开-关都会被拦）。
   行程只有**标定**一个来源 —— 不设 `ENCODER_MIN/MAX_ANGLE` 一类占位角配置项：
   占位角与真实机构无关，照它跑就是用错误基准冲限位。

> 「行程是否已建立」的判据是 **NVS 里的持久化行程**，不是"本次开机跑过 learn 没有"
> —— 复位后会正确显示三态。两个端点的**绝对角也一起持久化**，复位后 `pos` 直接看得到。
>
> ⚠️ 要"移动跨过零点"（探边、顶过全关位置）用 `goto_x <负开度>`：
> 那是**故意允许越界**的接口，走同一个门禁（位置可信 + 行程完整）但**不夹紧**开度。
> `goto` 始终夹在 0..1，应用的开/关走的就是它。

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

## 11. 未完成项 / 风险

| 项 | 说明 |
| --- | --- |
| **EAD 未做** | PA 载荷未加密。`receiver_id/session/sequence` 只能防误触发，不能防伪造。量产前必须补 |
| **上电自动标定的触发未接** | 策略与入口已有（`homing_auto()` + `FOCSTEP_HOME_AUTO_ENABLE`，默认**关**），但**上电时自动调用**还没接：它会让门自己跑到底，需要先定"什么条件下才允许自动跑"（本地唤醒后？还是只有上位机下令？）。目前只能手动 `learn auto on` / `learn both` |
| **相电阻/相电感未实测** | `FOCSTEP_PHASE_RESISTANCE` 是占位值，须 `char` 命令实测回填 |
| **无线唤醒未上板验证** | 平台层 `pa_wake.c`（PA 扫描/同步/载荷解析/周期调整）+ 应用侧监听窗口已接，出厂默认"上电即监听 + 超时回落"，详见 **§13** —— 但**尚未上板验证** |
| **回零参数未整定** | `homing.c` + `home`/`learn`/`mark`/`dir`/`pos` 命令已实现，行程是**零点 + 满行程点**两个语义端点（见 §9）—— 但回零力矩、重复性容差、可信度容差**都必须上板实测整定** |

> **已接入、上板必须优先验的两项**：
> **OTA**（`net_ota.c` + `ota_page.html`，官方 `esp_ota_ops`/`esp_http_server`）——
> 装进门里再拆成本极高，第一件事就是验它；
> **Wi-Fi 控制面**（`net_ota.c`，官方 `esp_wifi`/`mdns`）—— **默认关闭**
> （`FOCSTEP_WIFI_ENABLE=n`），因为开着会打破深睡；用 `net on` / `net off` 临时启用。

---

## 12. 睡眠档：DeepSleep / LightSleep（`FOCSTEP_SLEEP_MODE`）

板级"深睡"（`power_state = PS_SLEEP`）有两种 MCU 实现，**Kconfig 二选一**：

|  | `FOCSTEP_SLEEP_MODE_DEEP`（默认） | `FOCSTEP_SLEEP_MODE_LIGHT` |
| --- | --- | --- |
| 唤醒 | **即复位** ⇒ 从 `app_main` 重跑 | **不复位** ⇒ 从 `esp_light_sleep_start()` 之后继续 |
| 位置 | 睡前写 NVS（`foc_motor_save_position`），醒来 `restore_position` 做一致性检查 | 只记 RAM（`mark_sleep_angle`），醒来 `check_sleep_angle` 用**同一套判据** |
| GPIO | **浮空** ⇒ 靠板上上下拉维持安全电平 | **保持** ⇒ 靠固件进睡前显式写入 |
| 待机电流 | 最低（§六 的 **25~65µA @24V** 按这个口径算） | 更高 ⇒ **该数字要按实测重算** |
| FOC 任务 | 无需处理（复位重来） | 进睡**暂停**、回 ACTIVE 恢复（否则系统永远到不了 idle） |

**为什么不用 IDF 推荐的"自动轻睡"**（`esp_pm_configure()` + tickless idle）：自动轻睡的进入
时机取决于"所有任务都空闲"，对要求待机电流确定的门机产品不够可控。本工程在 PS_SLEEP 里
**显式调 `esp_light_sleep_start()`**，代价是自己承担下面三条纪律。

⚠️ **三条硬纪律**（代码已实现，改代码时别破坏）：

1. **唤醒后必须清 INT 锁存**：`wakeup_resume_from_light_sleep()` → `foc_motor_encoder_clear_int()`。
   KTH5701 的 INT 是"高有效 + 锁存 + 读一次数据清零"，不清就再睡 ⇒ 同一中断立刻把你再叫醒。
   自检命令 `int` 就是验这一条。
2. **唤醒后要重置时间基**：直接调 `esp_light_sleep_start()` 时 IDF **不补偿 FreeRTOS tick**
   （只有自动轻睡才走 `pm_step_tick`）⇒ 主循环的 `last` 与 FOC 任务的时间基都要重置，
   否则 `vTaskDelayUntil` 会连续追打（`foc_motor_pause_loop()` 里立标志，循环恢复时自己重置）。
   副作用：**睡眠期间 tick 不前进**，基于 tick 的超时（接管超时、无指令回睡）在睡眠期间不推进
   —— 这正是想要的语义。
3. **醒来要重做"位置是否可信"判定**：轻睡期间 MCU 不计数，门被拉动超过容差 ⇒ 多圈计数发散，
   与深睡口径一致：标记不可信，下一条运动指令前先回零。

**上板必须实测的三项**（§6 第 17 项）：轻睡整机电流、唤醒延迟、**有没有 1 秒内反复唤醒**。

**还没上板验证、先记在这里的**：

- 轻睡醒来后 **I2C 与外设**是否需要重新初始化。清 INT 那次 `data_read()` 就是醒来后第一次
  实打实的 I2C 访问 —— 若失败，日志会出现"读数据清 INT 失败"。真需要 re-init 的话，
  补在 `wakeup_resume_from_light_sleep()` 里。
- **UART 控制台**在轻睡期间会丢字符（同类原因已让本工程放弃 USB-Serial-JTAG）。
  台面调试时把 `FOCSTEP_SLEEP_ENABLE` 关掉即可一直连着串口。
- 以后若启用 Wi-Fi/OTA 控制面（`net on`），它必须持有 `ESP_PM_NO_LIGHT_SLEEP` 之类的 PM 锁，
  否则轻睡会打断连接。当前 `FOCSTEP_WIFI_ENABLE=n`，暂不涉及。
- `foc_motor_mark_sleep_angle()` 会读一次编码器数据 ⇒ **会清掉此刻可能已锁存的 INT**。
  阈值检测会重新拉高，最多晚一个检测周期；若实测发现"入睡前刚好被拉一下却没醒"，就是这里。

> ⚠️ 启用**无线唤醒（PA）**时本档会被 Kconfig 禁掉：监听期必须让 BLE 控制器自己排唤醒
> （PM 自动轻睡），显式 `esp_light_sleep_start()` 与它不共存。见 §13。

---

## 13. 无线唤醒（PA 监听档）

驱动板可被 BLE 周期广播（PA）远程唤醒 / 开 / 关 / 停。这套能力**改动三处口径**，
上板前先把本节读完。

### 13.1 三档待机口径（不要再只记一个数）

| 档 | 何时 | 待机 | 能不能被远程唤醒 |
| --- | --- | --- | --- |
| **深睡档** | `wl off`，或监听窗口超时回落 | ≈**25~65µA** ≈ 0.6~1.56mW@24V（**不是** 10µA：TVS 高温漏电 10~50µA 已入账，见 `doc.md` §9.2 #7） | ❌ 只能本地（编码器 INT/RTC/干接点） |
| **PA 监听档** | 上电默认 / `wl pa` | **亚 mA**：`f + 0.16/T`（C6 实测模型；`f` 待实测） | ✅ T=0.48s → ≈0.4mA，T=3.12s → ≈0.11mA |
| 运行态 | 门在动 / 接管中 | 电机主导 | ✅ 同步保持（否则运动中收不到"停"） |

### 13.2 模式与监听窗口

- 运行时模式：`wl off` / `wl pa`（命令台）。**不写 NVS** —— 出厂行为是"上电即开窗"，
  所以 `wl off` 只关当前窗口：**本地唤醒或复位后会重新开窗**（有意为之：
  人在现场时通常也希望远程可达）。
- 窗口时长 = 应用 Kconfig `FOCSTEP_PA_LISTEN_HOLD_MS`（默认 10 分钟）。
  "活动" = 收到本机非心跳指令 / 本地手拉 / 按键 / CAN / 命令台；
  **发送端心跳不算活动**（否则窗口永不回落）。
- 上电是否开窗 = 应用 Kconfig `FOCSTEP_PA_BOOT_LISTEN`（默认 y）。

> ⚠️ **这条语义尚未拍板**（`doc.md` §九 待定决策 **#4**）：窗口只认**本地**活动 ⇒
> **没人碰 10 分钟后门就回落深睡档、远程打不开**，而"没人碰"正是门的常态。
> 是接受"本地活动后 10 分钟"的机会式可达，还是拉长窗口/接 RTC 定时唤醒维持，需产品级确认。

### 13.3 唤醒周期 T

`T = per_adv_ival × (skip+1)`，最坏响应 ≈ T。发送端用 `SET_T` 指令动态改它
（发送端策略：被运动唤醒 → T=0.5s；倒计时无动作 → T=3s，见其 Kconfig）。

- **只能落在 `per_adv_ival` 的整数倍上**：发射端 240ms 时 0.5s→**实得 480ms**、
  3s→**实得 3120ms**。采多少只能在**接收端**看（`wl status`）——
  ⚠️ PA 是**单向**链路，发送端没有回执。
- 改 T 走 terminate → 重建，**中间有 0.5~1s 监听空窗** ⇒ 发送端侧已做"变化才发"。

### 13.4 睡眠架构（最容易踩的一处）

监听期**必须**让 BLE 控制器自己排唤醒 ⇒ 用 **PM 自动轻睡**，**不能**用显式
`esp_light_sleep_start()`（那条路只认我们配的冷唤醒源，不会给 PA 窗口留时间）。因此：

- Kconfig 已把 `FOCSTEP_SLEEP_MODE_LIGHT` 与 PA **做成互斥**（开了 PA 就只能是 DeepSleep 档）；
- 监听期由 `app_main` 的"监听档"分支接管：**不显式睡**、主循环节拍降到
  `FOCSTEP_PA_TICK_MS`（默认 1s）、FOC 任务由 `power_state` 在 PS_SLEEP 暂停；
- **代价**：监听期本地手拉唤醒的检测延迟 ≤ `PA_TICK_MS`（1s）。调小能更快，但每次唤醒
  都要付控制器固定的轻睡退出开销（实测常数 ~3.2ms × ~50mA），100ms 节拍会把省电全吃掉。

### 13.5 四条纪律（改代码别破坏）

1. **建同步必须传回调**：`ble_gap_periodic_adv_sync_create(..., gap_event, ...)` ——
   周期报文是挂在这个回调上的，漏传就一个 report 都收不到。
2. **同步成功后立刻 `ble_gap_disc_cancel()`**；且发现期扫描**必须限占空比**
   （`FOCSTEP_PA_SCAN_WINDOW_MS/INTERVAL_MS`）。默认扫描是 100% 占空比 = **实测 84mA**；
   对端不在场的 30s 后自动转慢扫（30ms/10s ≈ 0.3%）。
3. **醒来必须清 INT 锁存**（`wakeup_resume_from_light_sleep()`）：KTH5701 的 INT 是锁存型，
   不清就再睡 ⇒ 被同一次中断立刻再唤醒（唤醒风暴）。
4. **重复帧必须共用一个 `sequence`**（发送端 `s_seq` 的注释）：否则接收端把 N 帧都当新指令
   ⇒ 同一条 `OPEN` 执行 N 次。
5. **分层不许串**：承载层（`foc_link_protocol`）只搬字节、不定义业务命令；
   命令集在应用层（`foc_door_link`）；平台层 `pa_wake` **不解释** `cmd`/`arg`
   （只做"要不要打扰上层"的过滤，钩子由应用装）。加命令改应用层头文件，
   只有**帧格式**变了才动 `FOC_LINK_VERSION`。

### 13.6 上板验证（按顺序）

| # | 项 | 判据 |
| --- | --- | --- |
| 1 | 晶振真在跑 | 启动日志**不得**出现 `32.768kHz XTAL not detected`；`grep CONFIG_RTC_CLK_SRC_EXT_CRYS sdkconfig` 确认符号生效（写错会被 Kconfig **静默忽略**） |
| 2 | 同步 | `wl pa` → 日志 `已同步: per_adv_ival≈240ms`，`wl status` 的 T 实得 ≈480ms |
| 3 | 指令链 | 先用 `pa_send open` 验"事件→应用"（不经射频），再用发送端真发 |
| 4 | 重复帧幂等 | 发送端一条指令重复 5 次 → 接收端 `重复帧` 计数 +4，且**只执行一次** |
| 5 | 改 T | 发送端 `t 3000` → 接收端日志 terminate→create，T 实得=3120ms |
| 6 | 功耗三点 | 同会话差值口径：`wl off` / 480ms / 3120ms → 拟合 `f + 0.16/T` 得**本板真实 f** |
| 7 | 监听期真睡着 | `CONFIG_PM_PROFILING=y` 看 light sleep 占比应 >95%（低了就是有任务在轮询） |
| 8 | 本地唤醒 | 监听期手拉 → ≤`PA_TICK_MS` 内进接管，且**没有反复唤醒** |
| 9 | 失步恢复 | 发送端断电 → `失步` 计数增加并重扫，电流回落 |
| 10 | 体积 | `idf.py size`：BLE 加入后仍要装进 `TWO_OTA_LARGE` |

### 13.7 已知未做 / 风险

- **载荷未加密**（EAD 未做）：只能防误触发，不能防伪造（同 §11）。
- **ADI 未启用**（可再省 ~9%）：发送端 nonce 每包变化 ⇒ `filter_duplicates` 拿不到收益，
  要用得先改载荷语义（只在真事件时递增 DID）。
- `esp_pm_configure()` 运行时切换 `light_sleep_enable` 的副作用未实测。
- 发送端**改自己的 PA interval**（而不是让接收端改 skip）在"发送端也电池供电"时更省
  （skip 只摊薄接收端，发送端永远付 `C_TX/itvl`）—— 当前未做，留作优化。
- 多发送端共用同一 SID 时会锁到"先听到的那个"。
- **协议 v2 与 v1 不兼容**（加了 `arg`，18B→20B）：两端必须**同时重烧**。
  版本不符时接收端会打 `收到 vN 的包, 本端是 vM` 的警告（否则现场表现为"啥都没收到"）。
