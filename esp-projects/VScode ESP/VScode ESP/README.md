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
| LCD CS / SCK / MOSI / DC / RST | **0** / 13 / 14 / 21 / 38 |
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

**LCD CS 实物必须从 GPIO47 移到 GPIO0。** WROOM-2 的 GPIO47/48 是 1.8V 域。GPIO0 是 BOOT 脚，正常复位时必须保持高电平；屏幕 CS 不得接下拉或主动拉低，可以用约 10kΩ 上拉到 3.3V。BOOT 下载时被主动拉低是正常现象。

GPIO13/14 是 SPI2 通过 GPIO Matrix 路由的时钟/MOSI，并非原生 IO_MUX 时钟/MOSI。保留原程序 26MHz，不需要为了“硬件 SPI”再挪到已被 AIN1/ECHO 占用的 GPIO12/11。

本组代码没有云台舵机驱动，GPIO1/2 不由它配置；试验前把摄像头固定到能看清地面的俯视位置。编码器六个脚只配置成输入，原组没有读取计数或速度闭环。

轮子位置依据 `car-spin/README.md` 的 A右前、D左前、B后轮。保留 `motor_dir={1,1,-1}`：原算法直行给 M1负、M3正，映射后 D/A 的方向电平均为 IN1低、IN2高，与本仓库原接线说明一致；电机接头极性仍需架空实测。

## 本次必要适配

- 按上表替换 LCD、超声波、三个电机及其编码器引脚。
- 补齐 GPIO8 STBY：初始化时保持待机，方向脚清零、PWM占空比归零后拉高。
- 按本车 WROOM-2-N32R16V 设置 Octal Flash / OPI / 32MB / DTR 80MHz，保留 Octal PSRAM 80MHz 和 240MHz CPU；同时更新现有 `sdkconfig` 与 `sdkconfig.defaults`。
- 按本次要求接回巡线检测、巡线叠加、巡线和避障任务，停用找球。

原工程的 DIO/2MB 配置不能直接视作本车配置。原配置启用了 Flash 自动识别，在部分 IDF 环境可能自动切 OPI，并不表示 DIO 字样必然导致启动失败；现在显式指定实际模组配置，避免依赖自动识别。

## 构建和烧录

在已经安装 ESP-IDF 的终端中，进入**本 README 所在的内层工程目录**。原组留下的 `build` 包含 `C:/VScode ESP/build` 等旧电脑绝对路径与旧引脚固件，不能直接使用。

```powershell
# 当前目录应包含本工程的 CMakeLists.txt、sdkconfig、main/。
idf.py -B build-local reconfigure
idf.py -B build-local build
idf.py -B build-local -p COM5 flash monitor
```

把 COM5 换成实际 **UART 桥** 的串口。若工作区从其他工程切过来，确认 VS Code IDF 扩展的项目目录和构建目录也指向此工程，勿在 `camera-test` 中编译。原组记录为 IDF 5.5.5，本仓库其他工程记录为 5.4.4；以实际构建结果为准，不替换原有 `dependencies.lock`。

构建后核对：`CONFIG_IDF_TARGET="esp32s3"`、`CONFIG_ESPTOOLPY_OCT_FLASH=y`、`CONFIG_ESPTOOLPY_FLASHMODE_OPI=y`、32MB 和 Octal PSRAM。OPI 时内部 `CONFIG_ESPTOOLPY_FLASHMODE="dout"` 是 IDF 对烧录镜像头的表示，不是错误配置。

摄像头用 GPIO19/20 的 USB PHY；持续日志使用 GPIO43/44 对应的 UART 口。屏幕先看能否显示画面及黑线标记，再看串口 `line:` 和 `us dist=`，最后架空检查轮向后落地。

## 验证范围

已逐项核对引脚表与数据手册，检查重复占用，并用 C 预处理确认找球代码被排除、巡线和避障已启用。与收到的原文件比较，黑线检测、叠加、巡线、避障、横移与电机速度函数保留原算法和参数（避障日志中的 ECHO 脚号随接线更新）。

当前电脑未发现可调用的 ESP-IDF/交叉编译工具链，因此**未完成 ESP-IDF 编译、烧录或实车验证**。已有 `.bin` 不代表本次版本。全部电气限制与 Excel 核查结论见仓库根目录 `引脚核查与复现说明.md`。
