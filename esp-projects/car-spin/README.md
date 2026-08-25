# ESP32-S3 三轮小车四路红外巡线程序

本工程使用 ESP-IDF 控制三路电机 Motor A、Motor B、Motor D，并读取四路数字红外传感器，支持直道、连续弯道、直角弯和丢线搜索。程序每 10 ms 更新一次控制量，每 100 ms 从串口打印传感器与电机状态，方便现场调试。

## 引脚对应

### 四路红外模块

程序默认认为车头朝向红外模块，且 OUT1 到 OUT4 在车体上从左向右排列。

| 红外模块 | ESP32-S3 | 程序位置 |
| --- | --- | --- |
| OUT1 | GPIO41 | 最左 |
| OUT2 | GPIO42 | 中左 |
| OUT3 | GPIO2 | 中右 |
| OUT4 | GPIO1 | 最右 |

如果实际 OUT1 在车体右侧、OUT4 在左侧，把 `main/main.c` 中的：

```c
#define SENSOR_REVERSE_ORDER 0
```

改成 `1`。

### TB6612 四路驱动板

| ESP32-S3 | 驱动板信号 | 用途 |
| --- | --- | --- |
| GPIO4 | PWMB | Motor B PWM |
| GPIO6 | BIN1 | Motor B 方向 1 |
| GPIO5 | BIN2 | Motor B 方向 2 |
| GPIO8 | STBY | 驱动板总使能，高电平工作 |
| GPIO9 | PWMA | Motor A PWM |
| GPIO12 | AIN1 | Motor A 方向 1 |
| GPIO10 | AIN2 | Motor A 方向 2 |
| GPIO16 | PWMD | Motor D PWM |
| GPIO7 | DIN1 | Motor D 方向 1 |
| GPIO15 | DIN2 | Motor D 方向 2 |
| GPIO17 | ADC | 本巡线程序暂不使用 |

ESP32、红外模块、驱动板和电机电源必须共地。红外模块数字输出接 ESP32 时必须是 3.3 V 电平；若模块 OUT 会输出 5 V，需要电平转换，不能直接接 GPIO。

## 第一次调试：只看传感器

程序当前默认：

```c
#define SENSOR_ONLY_DEBUG 1
```

此模式下 STBY 保持低电平，所有电机不会运行。烧录后打开串口监视器，把黑线依次放在每个探头下方，日志示例：

```text
IR(L->R)=1000 mask=0x01
IR(L->R)=0100 mask=0x02
IR(L->R)=0010 mask=0x04
IR(L->R)=0001 mask=0x08
```

四位数从左到右对应 OUT1、OUT2、OUT3、OUT4；`1` 表示程序判断该路检测到黑线。本车模块实测为白底高电平、黑线低电平，因此程序默认：

```c
#define IR_ACTIVE_LEVEL 0
```

如果更换模块后逻辑相反（黑线输出高电平），再改为 `1`。也可调节红外板上的电位器，让每一路的指示灯在黑线与白底之间可靠翻转。

## 启用巡线

确认四个探头状态正确后，把：

```c
#define SENSOR_ONLY_DEBUG 1
```

改为 `0`，重新编译烧录。复位后程序等待 2 秒开始运动。

控制策略：

- 中间探头检测到线：以 `BASE_SPEED_PERCENT` 直行；
- 线逐渐偏向外侧：PD 控制器连续改变三轮速度，通过弯道；
- 只有最外侧探头检测到线：短时执行加强转向，用于直角弯；
- 完全丢线超过 1.2 秒：按照最后一次偏差方向原地搜索。

上电后如果四路都没有检测到线，程序保持电机 PWM 为 0；必须先让至少一个探头检测到线，巡线控制才会开始。

## 主要调参项

都在 `main/main.c` 顶部：

| 参数 | 默认值 | 调整方法 |
| --- | ---: | --- |
| `BASE_SPEED_PERCENT` | 28 | 直道速度；先低后高 |
| `MAX_SPEED_PERCENT` | 55 | 所有电机输出上限 |
| `MIN_CORNER_SPEED` | 18 | 大弯和直角弯的基础速度 |
| `KP` | 25 | 增大后纠偏更强；左右摆动明显则减小 |
| `KD` | 9 | 抑制快速摆动；响应迟缓时减小 |
| `CORNER_HOLD_MS` | 180 ms | 直角转不过去就增大，转过头就减小 |
| `LOST_LINE_TIMEOUT_MS` | 1200 ms | 丢线多久后转入原地搜索 |

串口日志中的 `err` 为线路偏差，负数表示偏左、正数表示偏右；`corr` 是转向修正量；`motor[A,B,D]` 是三个电机的有符号 PWM 百分比。

## 三轮方向校准

根据当前底盘描述，红外模块/车头位于 Motor A 与 Motor D 之间，Motor B 位于正对面。因此程序采用 120 度三全向轮模型：直行时 A 为负、D 为正、B 接近 0；转弯时再向 A/B/D 同时叠加同方向旋转量。第一次运动测试必须架空车轮。

如果某个电机的实际转向相反，修改对应项：

```c
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN 1
```

把相应的 `1` 改为 `-1`。如果传感器显示线在右边，但车辆实际向左修正，优先确认 OUT1/OUT4 的物理左右顺序；确认顺序无误后再校准电机方向。

## 编译、烧录和监视

请在 VS Code 的 ESP-IDF 终端中执行：

```powershell
cd C:\Electronic_Design\vscode-esp32s3_testcode\esp-projects\car-spin
idf.py set-target esp32s3
idf.py build
idf.py -p COM6 -b 115200 flash monitor
```

如果普通 PowerShell 找不到 `idf.py`，请先运行 ESP-IDF 环境脚本：

```powershell
& 'C:\esp\v5.4.4\esp-idf\export.ps1'
```

再运行上面的 `idf.py` 命令。串口监视器中按 `Ctrl+]` 退出。

## 建议调试顺序

1. 保持 `SENSOR_ONLY_DEBUG=1`，验证四路有效电平和左右顺序。
2. 架空轮子，设为 `0`，确认直行状态下 B/D 转向正确，A 基本不动。
3. 把小车放在低速直道上，先调 `KP`，再调 `KD`。
4. 测试缓弯，调整基础速度和 PD 参数。
5. 最后测试直角弯，只调整 `MIN_CORNER_SPEED` 和 `CORNER_HOLD_MS`。

开始地面测试时应预留断电空间；若传感器读数异常或小车转向与误差方向相反，立即断开电机电源，不要直接提高 PWM。
