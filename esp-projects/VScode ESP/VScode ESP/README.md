# ESP32-S3 摄像头巡线与超声波避障

2026-09-06 按用户要求恢复原版运动参数、巡线/避障控制和摄像头图像预览，不显示参数日志页。保留当前本车接线，LCD CS 仍为 GPIO0，不改回 GPIO1。原组找球代码仍在 `#if 0` 中，不参与运行。

## 本次回退范围

以基线提交 `9666379` 的 `main/main.c` 为准恢复主程序，包括 10MHz SPI 及轮询式同步传输、图像缩放与识别标注。撤销后来新增的参数屏任务、PWM 屏幕遥测和起步加力；删除不再使用的 `main/status_font.h`。串口 `line:`、`avoid:`、`us dist=` 等原版日志保留。

原版源码中 `vision_w`、`vision_h` 和 `last_vision_time` 的声明属于已停用的找球代码，但显示任务仍向它们赋值，导致编译失败。回退时仅删除这三处无效赋值。后续按实车测试调整速度与纠偏参数；本轮将避障前进和右移改为固定时间，并增加避障前进左右轮补偿。不改 `main/board_pins.h`，不回退本机 SDK、Flash/PSRAM 或其他工程的配置。

## 当前参数（原版基础小幅提速）

| 用途 | 参数 | 当前值 |
| --- | --- | --- |
| 巡线直行 | `BASE_SPEED` | 0.27 |
| 巡线纠偏 | `TURN_PULSE_SPEED` | 0.20 |
| 丢线搜索 | `SEARCH_TURN_SPEED` | 0.18 |
| 避障左移 | `AVOID_LEFT_SPEED` | 0.25 |
| 避障右移 | `AVOID_RIGHT_SPEED` | 0.32 |
| 避障前进 | `AVOID_FWD_SPEED` | 0.27 |
| 纠偏最长时间 / 观察停顿 | `TURN_PULSE_MS` / `OBSERVE_HOLD_MS` | 130ms / 100ms |
| 避障触发距离 / 左移脱离距离 | `AVOID_TRIGGER_CM` / `AVOID_LEFT_EXIT_CM` | 11cm / 25cm |
| 左移超时 / 右移固定时间 | `AVOID_LEFT_TIMEOUT_MS` / `AVOID_RIGHT_MS` | 3000ms / 1600ms |
| 避障前进固定时间 | `AVOID_FWD_MS` | 1500ms |
| 避障前进 D/A 补偿 | `AVOID_FWD_LEFT_COEF` / `AVOID_FWD_RIGHT_COEF` | 1.08 / 1.00 |

不使用单独起步加力。固定视觉参考点 0.53、检测带高度 0.79～0.94、纠偏触发阈值 0.33、退出死区 0.22、平滑系数 0.40。

## 当前运行流程

`app_main()` 初始化 LCD、超声波与电机，创建 `camera_display_task`、`line_follow_task` 和 `avoid_task`，再启动 USB 摄像头。解码图像送入原版黑线检测，再缩放、叠加标注并显示到 LCD。

原版巡线是“直行—原地纠偏—停顿观察”，不是连续 PID。普通巡线遇检测区域黑色占比达到 90% 时输出零速；退出全黑后可以继续动作。丢线时沿最后一次方向持续搜索，没有搜索转角限制。

避障连续两次有效测距小于 11cm 后左移；连续三次大于 25cm或左移超时后固定前进 1500ms，再固定右移 1600ms。右移期间不因 `line_found` 提前结束；结束时若当前找到线则武装后续全黑停车，否则直接交回巡线。前进偏左使用 D 轮 1.08、A 轮 1.00 的补偿，后续按实车小幅调整。

避障后的全黑锁定，需有线、黑色占比低于 90%、误差绝对值小于 0.20 并连续约 2 秒才解锁；锁定期间不测距。原版没有完整摄像头断流停车保护。先架空确认轮向，落地测试准备随时断电。

## 同参数不等于同车速

当前代码未读取编码器实现闭环控速。速度参数经轮速解算转成 PWM，不是 RPM；本轮直行 A/D 两轮约 17.3% PWM（原版约 14.7%），B 轮为零；纠偏三轮各约 12%（原版约 10%）。电机、减速比、轮型和安装方向、带载供电及机械阻力是需要对照实物排查的因素，不能假定另一组的参数适合本车。

先从串口确认实际输出：若 `vx=0`，说明当时不是直行；若 `vx=0.20`、M1/M3 约 -0.17/+0.17 且轮子不动，再核对架空/落地差异、带载 VM 电压和三轮实际方向。不要用提高全部参数替代这些检查，也不要长时间堵转。摄像头应固定，不使用的舵机断开供电，排除云台自行运动造成的识别偏差。

## 本车接线

所有可改脚号集中在 `main/board_pins.h`。这里是 GPIO 号，不是模组焊盘序号。

| 外设 | 本车 GPIO |
| --- | --- |
| LCD CS / SCK / MOSI / DC / RST | **0（恢复原接线）** / 13 / 14 / 21 / 38 |
| 超声 TRIG / ECHO | 18 / 11 |
| TB6612 STBY | 8 |
| M1 左前 = 本车 D：PWM / INA / INB | 16 / 7 / 15 |
| M1 编码器 A/B = E4A/E4B | 41 / 42 |
| M2 后轮 = 本车 B：PWM / INA / INB | 4 / 6 / 5 |
| M2 编码器 A/B = E2A/E2B | 17 / 3 |
| M3 右前 = 本车 A：PWM / INA / INB | 9 / 12 / 10 |
| M3 编码器 A/B = E1A/E1B | 39 / 40 |
| USB D- / D+ | 19 / 20，固定 |
| UART0 TX / RX | 43 / 44 |

**本轮 CS 接线：先断电，把 LCD CS 从 GPIO1 改回 GPIO0，并保持 GPIO1 给水平舵机。** 屏幕 CS 只能接 GPIO0，不能同时连接 GPIO1 或 GND。GPIO0 是 BOOT 启动绑带脚，正常上电/复位时不能被屏幕电路拉低；烧录或启动失败时先断开屏幕 CS/确认 GPIO0 电平。GPIO47/48 不作为本屏 CS 使用。

本次只是 VScode ESP 的临时测试接线。Excel 和 camera-claude 仍按 LCD CS=GPIO0、水平舵机=GPIO1；切回该方案前恢复接线。GPIO0 方案要求正常复位时 CS 不把 GPIO0 拉低。

GPIO13/14 是 SPI2 通过 GPIO Matrix 路由的时钟/MOSI，并非原生 IO_MUX 时钟/MOSI。当前 SPI 已随原版主程序恢复为 10MHz 请求频率，不需要挪到已被 AIN1/ECHO 占用的 GPIO12/11。该频率在当前屏幕及接线上是否稳定，仍须实测。

本组代码没有云台舵机驱动，GPIO1/2 均不配置，LCD CS 使用 GPIO0；不用舵机时断开其供电并固定摄像头角度，不要依赖本固件保持云台位置。编码器六个脚只配置成输入，原组没有读取计数或速度闭环。

轮子位置依据 `car-spin/README.md` 的 A右前、D左前、B后轮。保留 `motor_dir={1,1,-1}`：原算法直行给 M1负、M3正，映射后 D/A 的方向电平均为 IN1低、IN2高，与本仓库原接线说明一致；电机接头极性仍需架空实测。

## 本次必要适配

- 按上表替换 LCD、超声波、三个电机及其编码器引脚。
- 补齐 GPIO8 STBY：初始化时保持待机，方向脚清零、PWM占空比归零后拉高。
- 按本车 WROOM-2-N32R16V 设置 Octal Flash / OPI / 32MB / DTR 80MHz，保留 Octal PSRAM 80MHz 和 240MHz CPU；同时更新现有 `sdkconfig` 与 `sdkconfig.defaults`。
- 按本次要求接回巡线检测、巡线叠加、巡线和避障任务，停用找球。

原工程的 DIO/2MB 配置不能直接视作本车配置。原配置启用了 Flash 自动识别，在部分 IDF 环境可能自动切 OPI，并不表示 DIO 字样必然导致启动失败；现在显式指定实际模组配置，避免依赖自动识别。

## 构建和烧录

### 从仓库恢复环境

1. 拉取当前分支的最新提交，用 VS Code“文件 -> 从文件打开工作区”打开根目录的 `VScode-ESP.code-workspace`。左侧应只有 `VScode ESP` 一个工程；终端当前目录应是本 README 所在的内层目录。
2. 安装工作区推荐的 Espressif ESP-IDF 扩展和 Microsoft C/C++ 扩展，在 ESP-IDF 扩展中选择烧录电脑已安装的 **ESP-IDF v5.5.4** 及其 Python/工具链环境。本工程已按用户确认的实际环境调整，不需要为了原锁文件的 5.5.5 记录升级 SDK。
3. 通过 ESP-IDF 扩展打开终端，运行 `idf.py --version`，应显示 `ESP-IDF v5.5.4`。烧录与 Monitor 的默认串口均已设为 **COM6**，与当前开发板连接一致。

已提交根目录及本工程的 `.vscode` 配置和单工程工作区文件。工程内的 CMake、ESP-IDF（含 Windows 的 `idf.buildPathWin`）和 C/C++ `compileCommands` 全部使用 `build-local`；C/C++ 在首次构建后从生成的编译数据库读取真实工具链和头文件路径。配置不再写死原电脑的 `C:/Espressif`；`idf.port`、`idf.portWin` 和 `idf.monitorPort` 统一为 COM6。

| 环境项目 | 本仓库记录 |
| --- | --- |
| ESP-IDF / 串口 | 5.5.4 / COM6，按实际烧录电脑配置 |
| 芯片 | ESP32-S3，见 `sdkconfig` 和 IDE 的 `IDF_TARGET` |
| Flash / PSRAM | WROOM-2-N32R16V：32MB OPI Flash / 16MB Octal PSRAM |
| USB 组件 | `espressif/usb_stream` 1.5.2 |
| JPEG 组件 | `espressif/esp_jpeg` 1.3.1 |
| 间接依赖 | `espressif/cmake_utilities` 0.5.3 |
| 共享配置 | `sdkconfig`、`sdkconfig.defaults`、`main/idf_component.yml`、`dependencies.lock`、`.vscode/` |

以上组件版本及校验值已在原锁文件中提交，本次保留。已用官方组件管理器针对 IDF 5.5.4 重新解析依赖，锁文件仅更新 `idf` 版本。该检查确认依赖声明兼容，不代表已完成目标编译。

现有 `sdkconfig` 最初由 5.5.5 生成，保留原始文件头作为来源记录；首次使用 5.5.4 时执行下面的 `reconfigure`，由实际 SDK 更新配置。`managed_components` 由 IDF 按锁文件下载；SDK、Python 和交叉编译器由本机 ESP-IDF 安装器安装。

### 编译并确认实际固件

在上述 ESP-IDF 终端中构建。原组留下的 `build` 包含 `C:/VScode ESP/build` 等旧电脑绝对路径与旧引脚固件，不能直接使用。

```powershell
# 当前目录应包含本工程的 CMakeLists.txt、sdkconfig、main/。
idf.py -B build-local reconfigure
idf.py -B build-local build
idf.py -B build-local -p COM6 flash monitor
```

COM6 是当前开发板 USB 烧录接口实际枚举的串口，不需要外接 UART 模块；以后换电脑或接口导致串口号变化时，同时更新烧录和 Monitor 的端口。根目录的 CMake 配置也指向此工程，但 ESP-IDF 扩展应使用上面的单工程工作区，不在仓库根目录直接点击其烧录按钮。

本工程的 CMake 项目名是 `esp32s3-camera-line-follow`，构建产物应为 `build-local/esp32s3-camera-line-follow.bin`。若显示 `usb-uvc` 或日志出现 `camera_line: fps camera=...`、`Servo test disabled; continuing to USB Host`，说明运行的是 `camera-test`，应检查烧录目录或是否仍在运行旧固件。正确的本版启动日志见下方白屏排查。

构建后核对：`CONFIG_IDF_TARGET="esp32s3"`、`CONFIG_ESPTOOLPY_OCT_FLASH=y`、`CONFIG_ESPTOOLPY_FLASHMODE_OPI=y`、32MB 和 Octal PSRAM。OPI 时内部 `CONFIG_ESPTOOLPY_FLASHMODE="dout"` 是 IDF 对烧录镜像头的表示，不是错误配置。

摄像头用 GPIO19/20 的 USB PHY；持续日志使用 GPIO43/44 对应的 UART 口。屏幕恢复摄像头图像预览，运动参数与测距看串口 `line:` 和 `us dist=`，最后架空检查轮向后落地。

## 原版图像显示

摄像头 JPEG 解码、黑线检测、缩放、标注和 LCD 发送都在原版 `camera_display_task` 中运行。屏幕显示摄像头画面、蓝色检测窗口、青色固定参考线、绿色暗像素标记和黄色质心十字，不再显示状态/PWM/测距日志页。

绿色描点可能画到主检测窗之外，不等于所有绿色像素都参与误差计算。没有摄像头帧时不会显示 WAIT CAM 等参数文字；初始化清黑后等待图像。原版串口日志仍保留，便于分析运动状态。

## 白屏排查

当前 CS 保持 GPIO0，屏幕传输随原版恢复为 10MHz、轮询式同步传输。取消后来加入的独立参数屏和视觉任务每轮让步；此前为白屏/看门狗排查加入的这些改动不再保留，所以回退不保证白屏或调度问题消失。若重新出现看门狗报警、重启或白屏，先保存串口日志，不关闭看门狗掩盖异常。

改 CS 后必须重新编译并烧录本工程。打开 ESP-IDF Monitor，按一下开发板 RESET/EN（不要按住 BOOT），核对启动日志：

```text
LCD init: CS=0 SCK=13 MOSI=14 DC=21 RST=38
LCD init commands and black frame sent (no LCD readback)
```

第一行确认固件使用的新引脚；第二行只表示 ESP32 完成发送，本接线没有读回信号，不能证明屏幕接收成功。若没有第一行，先检查完整启动日志、是否反复重启以及实际烧录工程。若只有第一行，查看其后的 SPI 错误或复位日志。

正常初始化先填黑，收到并处理摄像头帧后才显示图像；未接摄像头时没有参数等待页。两行都有但仍白屏时，断电后核对 LCD RST/RES=GPIO38、DC/A0=GPIO21、SCK=GPIO13、SDI/MOSI=GPIO14、CS=GPIO0、VCC=3V3、GND=开发板 GND；屏幕 RST 不是开发板 EN。换一根短的 CS 线再试。GPIO0 的复用问题只是候选原因，换脚测试不能替代接线与启动日志核查。

## 验证范围

主程序与 `9666379` 基线相比，删除三处对停用找球变量的无效赋值以保证编译，并仅将直行/纠偏速度设为 0.20/0.12。搜索、避障、视觉参数、动作时长和图像显示不变。当前引脚配置保持不变，LCD CS=GPIO0；其余工程和本机环境配置不回退。原版图像显示、速度参数和控制流程已恢复，参数屏字体文件已移除。

本机使用 ESP-IDF v5.4.4 构建。编译成功不代表白屏、轮向、动力或避障已通过实车验证；新固件需重新烧录后测试。全部电气限制与 Excel 核查结论见仓库根目录 `引脚核查与复现说明.md`。
