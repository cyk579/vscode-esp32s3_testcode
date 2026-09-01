# ESP32-S3 USB 摄像头与云台测试

本工程基于 ESP-IDF 5.4.4 官方 USB Host UVC 示例，用于完成四项测试：

1. 判断摄像头 USB `D-`/`D+` 是否连接正确；
2. 从 UVC 摄像头采集 MJPEG 图像，实时显示在 1.8 英寸 ST7735 屏幕上；
3. 默认从解码后的画面识别白底黑线，并驱动三轮 TB6612 小车巡线；
4. 可选通过 Wi-Fi 回传到电脑，实时预览并保存第一张有效照片。

关闭 `Enable camera black-line following` 后，本工程仍可作为单纯的摄像头预览/枚举测试使用。

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

接好屏幕后，摄像头一旦开始 `Streaming...`，屏幕会直接显示横向实时画面；不需要电脑连接 Wi-Fi 才能显示。显示任务只保留最新帧，避免 JPEG 解码和 SPI 刷新拖住 USB Host，因此屏幕帧率低于摄像头标称帧率是正常现象。未接 TFT 时，解码和巡线回调仍会运行。

建议按以下顺序做首次功能验证：

1. 同时接好 TFT（如果使用）、摄像头的电源与共地，再接摄像头 `D- -> GPIO19`、`D+ -> GPIO20`。
2. 烧录后观察屏幕是否由黑屏切换到实时画面；这一步即可证明摄像头采集和 MJPEG 解码链路正常。
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

ESP32-S3 是 USB Full Speed Host，程序依次尝试 MJPEG `480×320@25 FPS`、`480×320` 的首个可用帧率、`320×240@30 FPS`、`640×480@15 FPS`，最后才尝试附件所示摄像头的 `1280×720` 首个可用帧率。高分辨率帧会在解码时缩小到适合 TFT 和巡线的低分辨率画面；单帧 JPEG 缓冲上限为 256 KiB。如果可以枚举但无法协商视频格式，说明 D+/D- 已经接对，但该摄像头可能不是标准 UVC/MJPEG 设备，或者不支持这些分辨率。

## 摄像头巡黑线

`Enable camera black-line following` 默认开启。当前算法不是用整幅图比较“左边黑像素多还是右边多”，而是先用自适应灰度阈值二值化，再把画面固定分成三个小区域：

| 区域 | 默认位置 | 唯一用途 |
| --- | ---: | --- |
| 远端区 | 高度 `35%~60%` | 看见未来支路时，连续 3 帧记住左弯或右弯；不控制电机 |
| 近端区 | 高度 `70%~87%` | 判断眼前线是否稳定、识别宽黑终点条 |
| 下方区 | 高度 `78%~87%` | 计算当前线中心供 PID 使用；当前直线消失后再确认转弯时机 |

弯道只有一个很小的状态机：远端支路相对眼前线明显偏向一侧时先保存方向；只要下方仍能看到保存方向前的那条直线，就继续按这条直线做 PID，远端弯线不参与输出；原直线消失后 Motor B 保持 `0` 并低速前探；直到下方黑像素重心连续 2 帧明显偏向已保存的一侧，才以固定转向量进入弯道。转过弯后，新直线回到近端中央并稳定 3 帧，清除记忆并恢复直线 PID。

这保留了[参考文章](https://blog.csdn.net/weixin_28285943/article/details/164210989)“控制优先看车前近处”的核心，但删掉了跨帧锚点、浮点前馈和盲目丢线旋转。实现只包含三个区域的整数黑像素重心和一个弯向记忆，适合低分辨率 RGB565 与单片机。直线 PID 最大只输出 8，每帧最多变化 2；中心死区内固定为 `A=-30、B=0、D=30`。相机应刚性朝下，车体中心线与镜头中心尽量重合，并让当前线落在画面下方区。

启动和停车条件如下：

- 复位后电机保持 `STBY=0`；第一帧到达后等待约 0.6 s，并连续确认 3 帧有效黑线才启动。
- 未记住弯向却丢线时只允许约 0.18 s 的低速直行，不再按旧误差原地搜索；约 0.9 s 仍无近端线就停车并重新确认。
- 已记住弯向但触发条件尚未满足时先低速直行，0.8 s 后仍不满足则停车观察；真正转弯超过 1.6 s 仍未找到新直线则停车复位巡线状态。
- 画面下方连续出现宽黑色终点条并确认 5 帧后停车。若赛道把宽线当作普通标记，可在源码中增大 `LINE_FINISH_CONFIRM_FRAMES` 或关闭该判断。

所有需要实车调节的值在 `main/camera_line_follow.c` 文件顶部：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `CAMERA_LINE_MIRROR_X` | `0` | 画面左右相反时改为 `1` |
| `LINE_ROI_TOP_PERCENT` / `LINE_ROI_BOTTOM_PERCENT` | `30` / `95` | 自适应灰度阈值的取样范围 |
| `LINE_FAR_TOP_PERCENT` / `LINE_FAR_BOTTOM_PERCENT` | `35` / `60` | 只负责提前记忆弯向；调整它不会直接改变 PID |
| `LINE_NEAR_TOP_PERCENT` / `LINE_NEAR_BOTTOM_PERCENT` | `70` / `90` | 近端线形和终点判断范围；底部会再跳过一条遮挡扫描线 |
| `LINE_LOWER_TOP_PERCENT` | `78` | PID 与转弯触发的最近区域；仍转早时增大，转得太晚时略减小 |
| `LINE_ROW_STEP` | `4` | 相邻扫描行的垂直间隔；数值越大越省算力，但过大会漏掉窄线 |
| `LINE_BOTTOM_SKIP_ROWS` | `1` | 从控制区底部跳过的遮挡行数；底部有车体阴影时可改为 `2` |
| `LINE_MIN_CONTRAST` | `32` | 低于此帧对比度时判定为无可靠线 |
| `LINE_BLACK_THRESHOLD_MIN/MAX` | `35` / `120` | 自适应黑色阈值上下限，降低灰色阴影误检 |
| `LINE_FAR_HINT_ERROR` / `LINE_FAR_CONFIRM_FRAMES` | `18` / `3` | 远端相对近端的最小偏移与记忆确认帧数 |
| `LINE_STRAIGHT_CORRIDOR_PERCENT` | `8` | 保存弯向后只在原直线左右各 8% 画宽内找线和做 PID；区域外支路不能参与微调 |
| `LINE_TURN_TRIGGER_ERROR` / `LINE_TURN_TRIGGER_FRAMES` | `25` / `2` | 原直线消失后，下方同向黑重心的转弯门槛与确认帧数 |
| `LINE_FORWARD_FAST/MEDIUM/SLOW/CRAWL` | `30/22/22/17` | 恢复红外巡线的直行、弯道和丢线恢复速度，避免低 PWM 失速 |
| `LINE_TURN_MAX` / `LINE_PID_TURN_MAX` | `19` / `8` | 红外最大转向量 / 直线 PID 最大微调量 |
| `LINE_ERROR_DEADBAND/MEDIUM/LARGE` | `18/35/60` | PID 死区和前进降速阈值；死区内 Motor B 保持 0 |
| `LINE_PID_KP/KI/KD` | `12/1/4` | 直线定点 PID；减弱 P、保留极小 I、增加 D 抑制来回摆动 |
| `LINE_PID_INTEGRAL_LIMIT` / `LINE_PID_SLEW_PER_FRAME` | `40` / `2` | 积分限幅和每帧输出变化上限 |
| `LINE_ERROR_FILTER_OLD/NEW` | `3/1` | 整数低通权重，降低摄像头延迟与帧间噪声造成的振荡 |

第一次地面测试建议先看日志状态：长直段应为 `LINE`，看见上方弯道后变为 `STRAIGHT-MEM`，但此时 `motor B` 仍应为 `0` 或很小的 PID 值；当前直线消失后先出现 `CORNER-WAIT`，下方黑线同向偏移连续两帧才进入 `TURN`。如果画面弯向与实车相反，只改 `CAMERA_LINE_MIRROR_X`；仍转早时只增大 `LINE_LOWER_TOP_PERCENT`，转晚时只减小它或略降 `LINE_TURN_TRIGGER_ERROR`。直线仍摆动时先降低 `LINE_FORWARD_FAST`，再小幅降低 `LINE_PID_KP`，每次只改一个参数。

需要只测试摄像头而不让车动时，在 `idf.py menuconfig -> Example Configuration` 关闭 `Enable camera black-line following`，并把 TB6612 的 `STBY` 固定拉低。不要同时开启舵机测试：舵机配置会与整车接线/LEDC 资源冲突。

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
