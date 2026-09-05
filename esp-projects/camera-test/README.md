# ESP32-S3 USB 摄像头与云台测试

本工程基于 ESP-IDF 5.4.4 官方 USB Host UVC 示例，用于完成四项测试：

1. 判断摄像头 USB `D-`/`D+` 是否连接正确；
2. 从 UVC 摄像头采集 MJPEG 图像，并可选显示在 1.8 英寸 ST7735 屏幕上；
3. 默认从解码后的画面识别白底黑线，并驱动三轮 TB6612 小车巡线；
4. 可选通过 Wi-Fi 回传到电脑，实时预览并保存第一张有效照片。

当前实车默认配置为：巡线开启，TFT 调试预览开启，Wi-Fi streaming 与舵机测试关闭。TFT 由独立低优先级任务约每 400 ms 刷新一次，直接复用最近的控制 RGB565 帧并叠加巡线标记：绿点是各扫描行找到的线中心，红十字是最靠近车的种子行，黄点是判定为宽黑段（折角横条/终点横杆）的行。TFT 不参与控制帧回调；需要最高控制帧率时可显式关闭 `Enable TFT status page`，状态与阈值仍可从 UART0 日志查看。

关闭 `Enable camera black-line following` 后，本工程仍可作为单纯的摄像头解码/枚举测试使用。

## 接线

ESP32-S3 原生 USB OTG 的引脚固定如下：

| ESP32-S3 | 摄像头信号 | 说明 |
| --- | --- | --- |
| GPIO19 | USB D- | 固定的原生 USB D- |
| GPIO20 | USB D+ | 固定的原生 USB D+ |
| 5V | VBUS/5V | 摄像头 USB 供电，必须确认电源能力 |
| GND | GND | 摄像头与 ESP32 共地 |
| GPIO1 | PAN_SIG | 水平舵机 PWM |
| GPIO2 | TILT_SIG | 垂直舵机 PWM |

摄像头只接 D+/D- 而没有 5 V 和 GND，不会被枚举。MG90S 舵机不要直接由 ESP32 的 3V3 供电；应使用电流足够的 5 V 电源，并与 ESP32 共地。

### 三轮 TB6612（摄像头巡线模式）

巡线模式复用 `car-spin` 已校准的三轮电机接线。GPIO1/GPIO2 不再接红外探头，也不要同时启用舵机测试；它们在此模式中保持空闲。

| ESP32-S3 | TB6612 | 用途 |
| --- | --- | --- |
| GPIO9 | PWMA | Motor A PWM |
| GPIO12 / GPIO10 | AIN1 / AIN2 | Motor A 方向 |
| GPIO4 | PWMB | Motor B PWM |
| GPIO6 / GPIO5 | BIN1 / BIN2 | Motor B 方向 |
| GPIO16 | PWMD | Motor D PWM |
| GPIO7 / GPIO15 | DIN1 / DIN2 | Motor D 方向 |
| GPIO8 | STBY | 高电平使能 |

电机电源使用驱动板和电机额定电源，ESP32、TB6612、摄像头必须共地。第一次测试时把车轮架空；程序上电后先等待画面连续确认黑线，再拉高 `STBY`。

摄像头板上四针插座的方向很容易看反。按随附接口图的**元件面**观察，四针从左到右是：

```text
5V | D- | D+ | GND
```

从插座背面或线材焊接面观察时顺序会反过来：`GND | D+ | D- | 5V`。这里的方向只说明摄像头插座针序；ESP32-S3 仍必须连接为 `D- -> GPIO19`、`D+ -> GPIO20`，并且摄像头 `5V` 必须接稳定的 5 V、`GND` 必须与 ESP32 共地。

### ST7735 显示屏

本仓库已经按 `LQ_TFT18SPIV33`、常见 `ST7735`、`128x160` 分辨率适配。程序以横屏方式工作。控制链路把 JPEG 缩放到不超过 `160x120`（`320x240` 输入得 `160x120`，`480x320` 输入得 `120x80`）；TFT 约每 400 ms 直接显示这帧控制画面并叠加巡线标记，不再单独解码。`Example Configuration -> Enable TFT status page` 默认开启。

| ESP32-S3 | TFT | 说明 |
| --- | --- | --- |
| GPIO13 | SCK | SPI 时钟 |
| GPIO14 | SDI / MOSI | SPI 数据 |
| GPIO21 | D/C | 命令/数据选择 |
| GPIO47 | CS | 片选 |
| GPIO38 | RST | 硬件复位 |
| 3V3 | VCC | 屏幕电源，**不要接 5 V** |
| GND | GND | 必须和 ESP32、摄像头共地 |

如果屏幕板另有 `BLK`/`LED` 背光引脚，将它接到 `3V3`。摄像头仍必须接 **5 V**，不能把摄像头 VBUS 和屏幕 VCC 混接。TFT 预览是可选调试功能，默认开启但独立低频运行；比赛时可在 menuconfig 中关闭。

## 判断 D+/D- 是否接反

先按以下方式连接：

```text
摄像头 D- -> GPIO19
摄像头 D+ -> GPIO20
```

如果两根数据线没有标签，也不要根据颜色直接下结论。先把它们临时标为 `A`、`B`，按下面两种组合依次测试：

| 测试 | GPIO19（固定 D-） | GPIO20（固定 D+） |
| --- | --- | --- |
| 第一次 | A | B |
| 第二次 | B | A |

两次测试之间必须将整车和摄像头完全断电。程序每 5 秒提示一次未检测到摄像头；只要出现 `UVC device found`、VID/PID 或 USB 描述符，就说明当前组合正确。线色只能作为参考，常见 USB 线是白色 D-、绿色 D+，但非标准线束可能完全不同。

程序启动后，正确线序会出现类似日志：

```text
Waiting for USB UVC device connection ...
UVC device found
DEVICE CONFIGURATION (...)
Negotiation complete.
Streaming...
```

只要能够打印 VID/PID 和 USB 配置描述符，就能证明 D+/D- 线序正确。如果一直停在：

```text
Waiting for USB UVC device connection ...
```

请依次检查摄像头 5 V、GND 和数据线连续性。确认供电无误后，完全断电，再交换 GPIO19/20 上的两根数据线测试。不要带电交换 USB 数据线。

## 重要：USB 口复用

GPIO19/20 同时也是 ESP32-S3 原生 USB-Serial/JTAG 所使用的物理 USB 引脚。把它们用于 USB Host 摄像头后，不能再让电脑和摄像头同时连接到同一组 D+/D- 线上，否则会出现两个 USB Host 电气冲突。

- 烧录时可以先断开摄像头数据线，通过原生 USB 烧录；
- 烧录完成后断开电脑原生 USB 数据连接，再接摄像头并重新上电；
- 若要在摄像头运行时查看日志，应使用开发板的 USB-UART 端口，或外接 USB 转串口到 UART0：GPIO43 为 TX、GPIO44 为 RX，并共地；
- 启用 `Enable streaming` 时，也可以通过是否出现热点和能否显示图像做快速判断；默认巡线配置请看串口日志。

## 烧录

在 ESP-IDF 终端执行：

```powershell
cd <仓库根目录>\esp-projects\camera-test
idf.py set-target esp32s3
idf.py build
idf.py -p COM6 -b 115200 flash
```

如果实际串口不是 COM6，请替换端口号。

若普通 PowerShell 找不到 `idf.py`，请从 VS Code 打开 **ESP-IDF Terminal** 后再运行上述命令，或先执行你本机 ESP-IDF 安装目录中的 `export.ps1`。本工程要求 ESP-IDF 5.4.4。

首次构建会通过 ESP-IDF Component Manager 下载官方 `esp_jpeg` 解码组件；因此首次构建需要联网。该组件使用外部解码器并启用了默认 Huffman 表，能兼容许多 UVC 摄像头为节省 USB 带宽而省略 Huffman 表的 MJPEG 帧。

## 查看图像：屏幕和电脑

接好屏幕后，屏幕约以 2 Hz 显示 `state`、`armed/STBY`、三轮输出、横向/方向误差、转向命令、摄像头/控制 FPS、丢帧数和超声波距离。状态页由低优先级独立任务读取最近缓存，不参与控制帧回调，也没有额外 JPEG 解码或预览 framebuffer。

建议按以下顺序做首次功能验证：

1. 同时接好 TFT（仅在启用 preview 时使用）、摄像头的电源与共地，再接摄像头 `D- -> GPIO19`、`D+ -> GPIO20`。
2. 观察状态页参数是否约每 500 ms 更新；摄像头控制统计同时通过串口查看。
3. 摄像头巡线模式不需要电脑端播放器；需要取证时再连接播放器保存一张照片。

只有在 `Enable streaming` 开启时，程序才会先创建 Wi-Fi 热点，再初始化 USB Host；默认巡线配置关闭该选项，因此应通过串口日志判断固件是否运行到应用层。启用 streaming 时，即使 D+/D- 接反也应该能看到热点；热点不存在时先检查开发板供电或 UART0 日志。

复位后的前 3 秒是诊断窗口：启用 streaming 时热点应在这段时间内出现；3 秒后程序把 GPIO19/20 切换为 USB Host，COM6 正常消失，这不代表掉电或崩溃。

启用 streaming 后，电脑端连接方式如下：

1. 摄像头正确接到 GPIO19/20，并给摄像头提供 5 V 和 GND。
2. ESP32 启动后，电脑连接以下 Wi-Fi：

```text
SSID: ESP32-CAMERA-TEST
密码: camera123
```

3. 首次使用播放器时安装依赖：

```powershell
pip install opencv-python numpy
```

4. 在工程目录运行：

```powershell
py -3.13 player.py
```

播放器连接 `192.168.4.1:2222`，收到完整 JPEG 帧后显示窗口。按 `Esc` 退出。
第一张有效图像会自动保存到电脑上的工程目录 `camera_capture.jpg`。若只想生成这张证据图片而不打开窗口，运行：

```powershell
py -3.13 player.py --headless --frames 1 --save camera_capture.jpg
```

命令成功后会输出 `Frames received: 1; first frame saved: True`，并可直接打开该 JPEG 检查画面。

不要在 ESP-IDF 自带的 Python 环境中使用裸命令 `python player.py`；该环境通常未安装 OpenCV。本机已经在 Python 3.13 中安装并验证了 OpenCV，因此使用 `py -3.13` 可以明确选择正确的解释器。

ESP32-S3 是 USB Full Speed Host，自动协商依次尝试 MJPEG `320×240@30 FPS`、`480×320@25 FPS`、`480×320` 首个可用帧率、`640×480@15 FPS`，最后才尝试 `1280×720` 首个可用帧率。控制解码缩小到 `160×120` 或 `120×80`（串口会打印实际尺寸），TFT 直接显示控制帧。单帧 JPEG 缓冲上限为 256 KiB。如果可以枚举但无法协商视频格式，说明 D+/D- 已经接对，但该摄像头可能不是标准 UVC/MJPEG 设备，或者不支持这些分辨率。

## 摄像头巡黑线

`Enable camera black-line following` 默认开启。比赛模式的数据链路为：

```text
UVC MJPEG(320x240@30 或 480x320@25) -> 三槽位最新帧队列
       -> 控制解码(160x120 或 120x80) -> 自适应阈值 -> 局部巡线 -> PD -> TB6612
       -> 独立低优先级预览任务约每 400 ms 取一帧叠加标记后送 TFT
```

解码后保留原始 RGB565，控制链路只扫描底部种子和相邻搜索窗口，不生成整幅二值图，也不对全宽黑像素做主要重心定位。阈值从控制帧 ROI 直方图计算，并用整数帧间低通/变化限幅抑制光照抖动；每个被扫描像素只计算一次亮度并与阈值比较。TFT 直接复用控制帧，没有第二次解码或额外图像缓冲。

赛道跟踪从画面底部开始：有历史时只在上一帧 `seed_x` 附近找线，无历史时选择靠近画面中心的合理黑线段。后续扫描行只在上一中心点固定窗口内寻找连续线段，并检查线宽和相邻中心点的最大横向跳变。这样侧面或远处的大片黑块不会抢走当前赛道。

状态只保留 `NORMAL`、`CORNER`、`LOST`：

- `NORMAL` 用近场中心计算 `lateral_error`，用中远场中心序列计算 `heading_error`，控制只使用整数 PD（P、heading 前馈和 D），并保留 turn/slew 限幅。斜线始终是 `NORMAL`，`heading_error` 不会触发转弯状态。
- 只有在线末端附近检测到明确新支路，连续 2~3 帧确认后才保存 `pending_turn`；旧直线仍可见时继续沿旧线控制，旧线消失后才进入 `CORNER`。`CORNER` 低速强转，直到底部连续 2~3 帧得到新的中心线后回到 `NORMAL`。
- `LOST` 立即停车并锁存，不会因下一帧重新看见线而自动重启；固定避障已经完成时，首次 `LOST` 才交给 `FINDBALL`。

当前版本不依赖 T 型口检测，固定避障完成后以视觉 `LOST` 作为找球交接条件。上电后 `STBY` 保持低电平，黑线连续确认后才使能电机；watchdog、方向换向保护和终端停车逻辑保持有效。

需要实车调整的参数集中在 `main/camera_line_follow.c` 文件顶部，优先调整镜像方向、ROI/搜索窗口、合理线宽、PD 增益、转向/斜率限幅以及 CORNER/LOST 的确认和超时参数。一次只改一个参数，并先观察下面的低频统计。

巡线模块每秒输出一行摘要，不逐帧打印、不保存图片。摘要至少包含：

```text
camera_fps decoded_fps control_fps preview_fps frames_dropped frame_age_ms
line_us_avg line_us_max state armed candidate arm_frames threshold
seed_x valid_rows confidence lateral_error heading_error pending_turn
STBY motor[A,B,D]
```

其中 `camera_fps` 是 UVC 输入帧率，`processed_fps` 是实际完成解码/巡线的帧率，`control_fps` 是电机控制刷新率；`line_us_avg/max` 用 `esp_timer_get_time()` 统计单帧巡线耗时。比赛时通过 UART 观察这些统计，不依赖 TFT 或 Wi-Fi。

需要只测试摄像头而不让车动时，在 `idf.py menuconfig -> Example Configuration` 关闭 `Enable camera black-line following`，并把 TB6612 的 `STBY` 固定拉低。需要时可关闭 `Enable TFT status page`；不要同时开启舵机测试，舵机配置会与整车接线/LEDC 资源冲突。

### 视觉阈值校准

需要让摄像头只识别黑胶带时，把 `main/camera_line_follow.c` 顶部的
`LINE_VISION_CALIB_MODE` 临时改为 `1`，并打开 TFT 状态页。该模式锁住电机，
只运行解码和局部识别，每 500 ms 打印 `VISION_CALIB threshold/candidate/seed_x/rows`
并在 TFT 上画当前识别点。依次把白底、黑胶带、手和彩色物体放到镜头 ROI，
确认黑胶带能得到稳定 `candidate=1`，手和其它物体不能形成连续点列。实际黑像素还会经过
`LINE_ABSOLUTE_BLACK_MAX_LUMA`（默认 88）和颜色饱和度过滤；手仍被识别时先降到 72，
黑胶带在暗光下丢失才升到 96。校准完成后恢复为 `0`。

在 ESP-IDF Terminal 中烧录校准固件：

```text
cd esp-projects/camera-test
idf.py set-target esp32s3
idf.py build
idf.py -p COM6 flash monitor
```

## 舵机测试动作

摄像头巡线默认不执行舵机测试：舵机启动电流可能造成电源复位，且 GPIO1/GPIO2
在整车上可能已经接到其它功能。只有在舵机使用独立稳定的 5 V 电源、并关闭
`Enable camera black-line following` 后，才可在 `idf.py menuconfig` 的 `Example Configuration`
中显式启用 `Run servo test at startup`。

启用后，PAN 和 TILT 会执行一次：

```text
中位 -> 小角度运动 -> 反方向小角度运动 -> 回到中位
```

测试使用约 1.25 ms、1.5 ms、1.75 ms 的 50 Hz 舵机脉冲，避免首次测试直接碰到机械极限。
