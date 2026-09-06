# ESP32-S3 摄像头巡线与超声波避障

2026-09-06 按本车 `引脚对应表2.xlsx` 适配。当前只运行这组的巡线、避障和屏幕预览；原组找球检测、找球任务及找球叠加代码保留在 `#if 0` 中，不参与编译或运行，后续找球使用本车自己的程序。

## 当前运行流程

`app_main()` 初始化 LCD、超声波与电机，创建显示/视觉任务、`line_follow_task` 和 `avoid_task`，然后启动 USB 摄像头。解码图像送到原组 `detect_line_from_rgb565()`，屏幕绘制 `overlay_detected_line()`。

原组参数与算法保留：动态检测窗、短促转向、丢线搜索；超声距离连续两次小于 11cm 后左移，距离连续三次大于 25cm 或左移满 3s 后前进 1260ms，再右移找回线（最长 3s）。之后继续巡线。全黑停车/居中解锁也保留。

注意：右移超时的原代码只输出一次停止，随后回到正常状态，巡线任务仍可能继续动作；它不是永久停车。本次没有重写状态机。摄像头断流也没有完整的巡线/避障停车保护，首次试车应架空，急停使用电源开关。

## 本车接线

所有可改脚号集中在 `main/board_pins.h`。这里是 GPIO 号，不是模组焊盘序号。

| 外设 | 本车 GPIO |
| --- | --- |
| LCD CS / SCK / MOSI / DC / RST | **1（临时排查）** / 13 / 14 / 21 / 38 |
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

**白屏排查：先断电，拔掉 GPIO1 的水平舵机信号线，再把 LCD CS 从 GPIO0 移到 GPIO1。** GPIO1 是普通 GPIO，可避开 GPIO0 的 BOOT 复用；屏幕 CS 只能接 GPIO1，不能同时连 GPIO0、GND 或舵机信号。WROOM-2 的 GPIO47/48 是 1.8V 域，不要接回那里。

本次只是 VScode ESP 的临时测试接线。Excel 和 camera-claude 仍按 LCD CS=GPIO0、水平舵机=GPIO1；切回该方案前恢复接线。GPIO0 方案要求正常复位时 CS 不把 GPIO0 拉低。

GPIO13/14 是 SPI2 通过 GPIO Matrix 路由的时钟/MOSI，并非原生 IO_MUX 时钟/MOSI。当前 SPI 已降至 10MHz 供接线排查，不需要再挪到已被 AIN1/ECHO 占用的 GPIO12/11。

本组代码没有云台舵机驱动，GPIO1 临时给 LCD CS，GPIO2 不配置；试验前把摄像头固定到能看清地面的俯视位置。编码器六个脚只配置成输入，原组没有读取计数或速度闭环。

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
2. 安装工作区推荐的 Espressif ESP-IDF 扩展和 Microsoft C/C++ 扩展，在 ESP-IDF 扩展中选择本机安装的 **ESP-IDF v5.5.5** 及其 Python/工具链环境。这是本工程现有 `dependencies.lock` 记录的 SDK 版本，不使用其他工程的 5.4.4 环境覆盖本锁文件。
3. 通过 ESP-IDF 扩展打开终端，运行 `idf.py --version`，应显示 `ESP-IDF v5.5.5`。串口选择实际连接开发板的 COM 口，下面的 COM5 只是示例。

已提交根目录及本工程的 `.vscode` 配置和单工程工作区文件。工程内的 CMake、ESP-IDF（含 Windows 的 `idf.buildPathWin`）和 C/C++ `compileCommands` 全部使用 `build-local`；C/C++ 在首次构建后从生成的编译数据库读取真实工具链和头文件路径。配置不再写死原电脑的 `C:/Espressif` 或 COM5。

| 环境项目 | 本仓库记录 |
| --- | --- |
| ESP-IDF | 5.5.5，见现有 `dependencies.lock` 的 `idf` 项 |
| 芯片 | ESP32-S3，见 `sdkconfig` 和 IDE 的 `IDF_TARGET` |
| Flash / PSRAM | WROOM-2-N32R16V：32MB OPI Flash / 16MB Octal PSRAM |
| USB 组件 | `espressif/usb_stream` 1.5.2 |
| JPEG 组件 | `espressif/esp_jpeg` 1.3.1 |
| 间接依赖 | `espressif/cmake_utilities` 0.5.3 |
| 共享配置 | `sdkconfig`、`sdkconfig.defaults`、`main/idf_component.yml`、`dependencies.lock`、`.vscode/` |

以上组件版本及校验值已在原锁文件中提交，本次保留。`managed_components` 由 IDF 按锁文件下载；SDK、Python 和交叉编译器由本机 ESP-IDF 安装器安装。仓库保存复现配置，不把旧电脑的编译缓存当作环境。

### 编译并确认实际固件

在上述 ESP-IDF 终端中构建。原组留下的 `build` 包含 `C:/VScode ESP/build` 等旧电脑绝对路径与旧引脚固件，不能直接使用。

```powershell
# 当前目录应包含本工程的 CMakeLists.txt、sdkconfig、main/。
idf.py -B build-local reconfigure
idf.py -B build-local build
idf.py -B build-local -p COM5 flash monitor
```

把 COM5 换成开发板 USB 烧录接口实际枚举的串口，不需要外接 UART 模块。根目录的 CMake 配置也指向此工程，但 ESP-IDF 扩展应使用上面的单工程工作区，不在仓库根目录直接点击其烧录按钮。

本工程的 CMake 项目名是 `esp32s3-camera-line-follow`，构建产物应为 `build-local/esp32s3-camera-line-follow.bin`。若显示 `usb-uvc` 或日志出现 `camera_line: fps camera=...`、`Servo test disabled; continuing to USB Host`，说明运行的是 `camera-test`，应检查烧录目录或是否仍在运行旧固件。正确的本版启动日志见下方白屏排查。

构建后核对：`CONFIG_IDF_TARGET="esp32s3"`、`CONFIG_ESPTOOLPY_OCT_FLASH=y`、`CONFIG_ESPTOOLPY_FLASHMODE_OPI=y`、32MB 和 Octal PSRAM。OPI 时内部 `CONFIG_ESPTOOLPY_FLASHMODE="dout"` 是 IDF 对烧录镜像头的表示，不是错误配置。

摄像头用 GPIO19/20 的 USB PHY；持续日志使用 GPIO43/44 对应的 UART 口。屏幕先看能否显示画面及黑线标记，再看串口 `line:` 和 `us dist=`，最后架空检查轮向后落地。

## 白屏排查

改 CS 后必须重新编译并烧录本工程。打开 ESP-IDF Monitor，按一下开发板 RESET/EN（不要按住 BOOT），核对启动日志：

```text
LCD init: CS=1 SCK=13 MOSI=14 DC=21 RST=38
LCD init commands and black frame sent (no LCD readback)
```

第一行确认固件使用的新引脚；第二行只表示 ESP32 完成发送，本接线没有读回信号，不能证明屏幕接收成功。若没有第一行，先检查完整启动日志、是否反复重启以及实际烧录工程。若只有第一行，查看其后的 SPI 错误或复位日志。

正常初始化在启动摄像头前就会把屏幕填黑，因此未接摄像头也应变黑。两行都有但仍白屏时，断电后核对 LCD RST/RES=GPIO38、DC/A0=GPIO21、SCK=GPIO13、SDI/MOSI=GPIO14、CS=GPIO1、VCC=3V3、GND=开发板 GND；屏幕 RST 不是开发板 EN。换一根短的 CS 线再试。GPIO0 的复用问题只是候选原因，换脚测试不能替代接线与启动日志核查。

## 验证范围

已逐项核对引脚表与数据手册，检查重复占用，并用 C 预处理确认找球代码被排除、巡线和避障已启用。与收到的原文件比较，黑线检测、叠加、巡线、避障、横移与电机速度函数保留原算法和参数（避障日志中的 ECHO 脚号随接线更新）。

当前电脑未发现可调用的 ESP-IDF/交叉编译工具链，因此**未完成 ESP-IDF 编译、烧录或实车验证**。已有 `.bin` 不代表本次版本。全部电气限制与 Excel 核查结论见仓库根目录 `引脚核查与复现说明.md`。
