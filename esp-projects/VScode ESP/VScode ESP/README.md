# ESP32-S3 摄像头循迹小车（blink-led）

整理日期：2026-09-03。这是当前正在运行的完整版本，包含：

- 摄像头画面显示（ST7735 LCD + esp_jpeg 软解）
- 摄像头黑线循迹（动态检测窗 + 固定转向基准 + 误差平滑 + 短促转向脉冲）
- 超声波避障（左平移 → 前冲 → 右平移找回线 → 恢复循迹）
- 串口距离诊断与状态日志

## 环境

- 芯片：ESP32-S3（含 16MB PSRAM）
- ESP-IDF：v5.5.5，位置 `C:\Espressif\frameworks\esp-idf-v5.5.5`
- 工具链：`C:\Espressif`（IDF_TOOLS_PATH）
- Python 虚拟环境：`C:\Espressif\python_env\idf5.5_py3.11_env`
- 烧录串口：COM5（以实际为准）
- 编译目录：`build`

## 构建 / 烧录（PowerShell）

```powershell
$env:IDF_PYTHON_ENV_PATH='C:\Espressif\python_env\idf5.5_py3.11_env'
$env:IDF_TOOLS_PATH='C:\Espressif'
$env:PATH='C:\Espressif\python_env\idf5.5_py3.11_env\Scripts;' + $env:PATH
& 'C:\Espressif\frameworks\esp-idf-v5.5.5\export.ps1' *> $null
& 'C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe' 'C:\Espressif\frameworks\esp-idf-v5.5.5\tools\idf.py' -B build -p COM5 flash
```

首次构建会自动从组件仓库下载依赖（见 `dependencies.lock`）：
`espressif/usb_stream` 1.5.2、`espressif/esp_jpeg` 1.3.1、`espressif/cmake_utilities` 0.5.3。

## 硬件接线

### LCD ST7735（128x160，SPI）
| 屏幕 | GPIO |
| --- | --- |
| CS | GPIO2 |
| SCK | GPIO1 |
| SDI(MOSI) | GPIO45 |
| D/C | GPIO46 |
| RST | GPIO8 |
| VCC/GND | 3.3V/共地 |

### 摄像头（USB UVC）
- USB D+ → GPIO20，USB D- → GPIO19
- 独立 5V 供电，与主控共地

### 超声波
| 模块 | GPIO |
| --- | --- |
| Trig | GPIO14 |
| Echo | GPIO4 |

### 三个麦克纳姆轮电机
| 轮子 | 编码器 A/B | 方向 INA/INB | PWM |
| --- | --- | --- | --- |
| M1 左前 (+60°) | GPIO15/16 | GPIO17/18 | GPIO21 |
| M2 后轮 (180°) | GPIO9/10 | GPIO11/12 | GPIO13 |
| M3 右前 (-60°) | GPIO42/41 | GPIO40/39 | GPIO38（方向已软件反转） |

## 主要可调参数（均在 main/main.c 顶部）

- 循迹：`TURN_PULSE_MS`、`TURN_PULSE_SPEED`、`STEER_TRIGGER_ERR`、`STEER_DEADBAND`、`ERR_SMOOTH_K`
- 黑线阈值：检测函数内 `* 20u / 100u`（数值越大越容易识别，越小越严格）
- 避障：`AVOID_TRIGGER_CM`、`AVOID_LEFT_EXIT_CM`、`AVOID_FWD_MS`、`AVOID_LEFT_SPEED/RIGHT_SPEED`、`TRANSLATE_FRONT_COEF`
- 直行速度：`BASE_SPEED`

## 串口说明

正常运行时约每 0.3s 打印距离：
`us dist=xx.xcm (st=0 line=1 active=0)`
避障触发后打印 `avoid: trigger ... -> left translate` 等状态。
