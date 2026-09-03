# 超声波避障独立测试

这是从本地最新 `subject2-burst-b0ef2192` 工程抽出的最小 ESP-IDF 测试项目。分支中不包含摄像头、巡线、TFT、Wi-Fi 或红外代码。

## 行为

复位后立即使能 TB6612，使用 A/D 两轮直行。HC-SR04 连续采样，距离连续两次不大于 `10 cm` 后执行固定路线：

`停止 0.5 s -> 左移 2.5 s -> A/D 前进 2 s -> 右移 2.5 s -> 停止`

路线完成后保持停止；再次复位即可重复测试。

## 速度指令

所有指令都在 `main/main.c` 顶部，单位是 PWM 百分比，范围会限制在 `+/-40`。指令是电机方向脚应用极性之前的原始值：

```text
直行       A=-25  B=0    D=20
左移       A=-18  B=-25  D=-18
左移后直行 A=-25  B=0    D=20
右移       A=15   B=25   D=20
```

当前极性沿用已验证接线：`A=1, B=1, D=-1`。因此直行的 A 必须为负、D 必须为正；日志同时打印原始 `cmd` 和应用极性后的 `effective`，方便逐轮调速。

## 引脚

```text
Motor A: PWM=GPIO9,  IN1=GPIO12, IN2=GPIO10
Motor B: PWM=GPIO4,  IN1=GPIO6,  IN2=GPIO5
Motor D: PWM=GPIO16, IN1=GPIO7,  IN2=GPIO15
TB6612 STBY: GPIO8
HC-SR04 TRIG: GPIO18
HC-SR04 ECHO: GPIO11
```

HC-SR04 的 ECHO 若输出 5 V，必须先经分压再接 ESP32-S3 GPIO11，并共地。

## 编译烧录

在 ESP-IDF 终端中：

```powershell
cd esp-projects\ultrasonic-test
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

首次实机测试请架空车轮。串口每约 250 ms 输出当前阶段、阶段计时、距离、有效标志、确认次数和三个电机指令；调速只需修改 `main/main.c` 顶部的 `*_CMD` 常量。
