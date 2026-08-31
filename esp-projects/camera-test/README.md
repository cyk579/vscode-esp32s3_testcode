# ESP32-S3 USB 摄像头与云台测试

本工程基于 ESP-IDF 5.4.4 官方 USB Host UVC 示例，用于完成四项测试：

1. 判断摄像头 USB `D-`/`D+` 是否连接正确；
2. 从 UVC 摄像头采集 MJPEG 图像，实时显示在 1.8 英寸 ST7735 屏幕上；
3. 可选通过 Wi-Fi 回传到电脑，实时预览并保存第一张有效照片；车载默认关闭回传以减少功耗。
4. 测试 GPIO1、GPIO2 对应的水平和垂直舵机信号。

该工程与 `car-spin` 相互独立，不控制电机。

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

摄像头板上四针插座的方向很容易看反。按随附接口图的**元件面**观察，四针从左到右是：

```text
5V | D- | D+ | GND
```

从插座背面或线材焊接面观察时顺序会反过来：`GND | D+ | D- | 5V`。这里的方向只说明摄像头插座针序；ESP32-S3 仍必须连接为 `D- -> GPIO19`、`D+ -> GPIO20`，并且摄像头 `5V` 必须接稳定的 5 V、`GND` 必须与 ESP32 共地。

### ST7735 显示屏

本仓库已经按 `LQ_TFT18SPIV33`、常见 `ST7735`、`128x160` 分辨率适配。程序以横屏方式工作，摄像头的 `320x240` 帧会在解码时缩小为 `160x120`，保留画面比例并在上下各留 4 像素黑边。

| ESP32-S3 | TFT | 说明 |
| --- | --- | --- |
| GPIO13 | SCK | SPI 时钟 |
| GPIO14 | SDI / MOSI | SPI 数据 |
| GPIO21 | D/C | 命令/数据选择 |
| GPIO47 | CS | 片选 |
| GPIO38 | RST | 硬件复位 |
| 3V3 | VCC | 屏幕电源，**不要接 5 V** |
| GND | GND | 必须和 ESP32、摄像头共地 |

如果屏幕板另有 `BLK`/`LED` 背光引脚，将它接到 `3V3`。摄像头仍必须接 **5 V**，不能把摄像头 VBUS 和屏幕 VCC 混接。

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
- 也可以不看串口日志，直接通过是否出现热点和能否显示图像判断。

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

接好屏幕后，摄像头一旦开始 `Streaming...`，屏幕会直接显示横向实时画面；不需要电脑连接 Wi-Fi 才能显示。显示任务只保留最新帧，避免 JPEG 解码和 SPI 刷新拖住 USB Host，因此屏幕帧率低于摄像头标称帧率是正常现象。

建议按以下顺序做首次功能验证：

1. 同时接好 TFT、摄像头的电源与共地，再接摄像头 `D- -> GPIO19`、`D+ -> GPIO20`。
2. 烧录后观察屏幕是否由黑屏切换到实时画面；这一步即可证明摄像头采集、MJPEG 解码和屏幕输出链路正常。
3. 再连接电脑端播放器保存一张照片，作为可传出的测试证据。

程序启动后会先创建 Wi-Fi 热点，再初始化 USB Host。只要热点出现，就说明固件已经运行到应用层；即使 D+/D- 接反，也应该能看到热点。若热点完全不存在，应先检查开发板供电或通过 UART0 查看启动日志。

复位后的前 15 秒是诊断窗口：COM6 暂时保持可用，热点应在这段时间内出现。15 秒后程序把 GPIO19/20 切换为 USB Host，COM6 正常消失；这不代表掉电或崩溃。

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

ESP32-S3 是 USB Full Speed Host，程序优先尝试 MJPEG `320×240@30 FPS`，随后尝试 `320×240` 的首个可用帧率，最后才尝试 `640×480@15 FPS`。前两种能直接缩放为屏幕预览尺寸；若退回到 640×480，程序会自动缩小到相同的 `160×120` 画面。如果可以枚举但无法协商视频格式，说明 D+/D- 已经接对，但该摄像头可能不是标准 UVC/MJPEG 设备，或者不支持这些分辨率。

## 舵机测试动作

摄像头预览模式默认不执行舵机测试：舵机启动电流可能造成电源复位，且 GPIO1/GPIO2
在整车上可能已经连接红外模块。只有在舵机使用独立稳定的 5 V 电源、并确认引脚没有冲突时，
才可在 `idf.py menuconfig` 的 `Example Configuration` 中显式启用 `Run servo test at startup`。

启用后，PAN 和 TILT 会执行一次：

```text
中位 -> 小角度运动 -> 反方向小角度运动 -> 回到中位
```

测试使用约 1.25 ms、1.5 ms、1.75 ms 的 50 Hz 舵机脉冲，避免首次测试直接碰到机械极限。
