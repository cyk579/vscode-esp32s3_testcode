# ESP32-S3 三轮小车四路红外巡线程序

本工程使用 ESP-IDF 控制三路电机 Motor A、Motor B、Motor D，并读取四路数字红外传感器，支持直道、连续弯道、直角弯和丢线搜索。程序每 10 ms 更新一次控制量，每 100 ms 从串口打印传感器与电机状态，方便现场调试。

## 引脚对应

### 四路红外模块

程序默认按车体实际方向解释传感器：本车 OUT1 在右侧、OUT4 在左侧，代码通过反向映射后再按左到右计算误差。

| 红外模块 | ESP32-S3 | 程序位置 |
| --- | --- | --- |
| OUT1 | GPIO41 | 最右（原始通道） |
| OUT2 | GPIO42 | 右中（原始通道） |
| OUT3 | GPIO2 | 左中（原始通道） |
| OUT4 | GPIO1 | 最左（原始通道） |

如果实际传感器左右方向与当前车体相反，把 `main/main.c` 中的：

```c
#define SENSOR_REVERSE_ORDER 0
```

在 `0` 和 `1` 之间切换。当前这台小车已经设置为 `1`。

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

## 第一次调试：检查传感器

先架空车轮并打开串口监视器，把黑线依次放在每个探头下方。程序上电后在首次检测到黑线前电机保持为 0，日志中的 `ACTIVE` 示例为：

```text
ACTIVE=1000
ACTIVE=0100
ACTIVE=0010
ACTIVE=0001
```

`ACTIVE` 从左到右对应车体实际的 OUT4、OUT3、OUT2、OUT1；`1` 表示检测到黑线。`RAW` 才按接口 OUT1、OUT2、OUT3、OUT4 排列。本车模块实测为白底高电平、黑线低电平，因此程序默认：

```c
#define IR_ACTIVE_LEVEL 0
```

如果更换模块后逻辑相反（黑线输出高电平），再改为 `1`。也可调节红外板上的电位器，让每一路的指示灯在黑线与白底之间可靠翻转。

## 启用巡线

程序复位后等待 2 秒。至少一个探头检测到黑线后才允许电机启动；全白时上电不会盲目运动。

控制策略：

- `1001`（OUT2/OUT3 在黑线上）：执行经过 A/D 配平的固定直行；
- 黑线偏向一侧：根据四路加权误差执行有限比例转向；
- 黑线越靠近最外侧，转向量越大；
- 完全丢线：立即沿最后一次偏差方向原地搜索，重新得到 `1001` 后直接恢复直行。

上电后如果四路都没有检测到线，程序保持电机 PWM 为 0；必须先让至少一个探头检测到线，巡线控制才会开始。

## 主要调参项

都在 `main/main.c` 顶部：

| 参数 | 默认值 | 调整方法 |
| --- | ---: | --- |
| `STRAIGHT_A_SPEED` | 18 | 直行时右前 Motor A 的输出幅值 |
| `STRAIGHT_D_SPEED` | 18 | 直行时左前 Motor D 的输出幅值，与 A 保持一致 |
| `CURVE_A_SPEED` | 16 | 转弯时 Motor A 的前进分量 |
| `CURVE_D_SPEED` | 16 | 转弯时 Motor D 的前进分量，与 A 保持一致 |
| `TURN_MAX` | 12 | 外侧检测或丢线搜索时的最大旋转分量 |
| `MAX_OUTPUT` | 28 | 任一电机最终输出上限 |
| `START_KICK_OUTPUT` | 20 | A/D 从停止起转时的短时增强输出 |
| `START_KICK_CYCLES` | 3 | 起转增强持续约 40 ms；Motor B 不使用起转增强 |
| `FILTER_SAMPLES` | 5 | 每周期多数采样次数 |
| `FILTER_STABLE_CYCLES` | 2 | 新方向连续出现两次才确认 |
| `TURN_DELAY_CYCLES` | 3 | 确认偏线后继续前进约 50-60 ms 再转向 |
| `PID_KP` | 4 | 比例增益，决定当前偏差的纠偏强度 |
| `PID_KI` | 1 | 积分增益，修正持续偏差，并带积分限幅 |
| `PID_KD` | 2 | 微分增益，抑制摆动，微分值经过平滑 |
| `PID_DEADBAND` | 3 | 小误差死区，避免对对称组合频繁修正 |
| `PID_INTEGRAL_LIMIT` | 24 | 泄漏积分的上下限，防止误差长期累积 |
| `PID_OUTPUT_STEP` | 2 | 每个控制周期允许转向输出变化的最大值 |

## 赛道适配说明

当前使用的 LQ_R4CHVB 模块为黑线低电平、白底高电平，因此 `IR_ACTIVE_LEVEL=0`。车体从左到右是 OUT4、OUT3、OUT2、OUT1，代码通过 `REVERSE_SENSOR_ORDER=1` 转换为内部左到右顺序。

控制器恢复为 11:22 版本的转弯流程：每周期进行 5 次多数采样，新方向连续出现两次后确认，再经过约 50-60 ms 的车头前探延迟交给 PID。偏差方向反转时先输出直行，不回放历史队列中的相反转向；`1001`、`1111` 等零误差状态立即直行。PID 继续使用误差低通、微分低通、泄漏积分、积分限幅、换向清历史和每周期 2% 的输出斜率限制。完全丢线时恢复为 11:22 版本的最后方向原地搜索，`1111` 不作为终点停车条件。

左转时，右前 Motor A 增加前进分量、左前 Motor D 减少前进分量；右转时相反。Motor B 保留原来的旋转方向，只提供转向分量。高层混控已经包含 A/D 的镜像安装方向，因此不再对 D 进行第二次反向。

串口日志中的 `RAW` 按 OUT1 到 OUT4 排列；OUT2/OUT3 压在线上时显示 `RAW=1001`。`ACTIVE` 按车体左到右排列，以 `1` 表示检测到黑线，此时为 `0110`。`err` 负数表示线在左侧、正数表示线在右侧；`motor[A,B,D]` 是三个电机的有符号 PWM 百分比。

## 三轮方向校准

根据当前底盘描述，OUT4 左侧是 Motor D，OUT1 右侧是 Motor A，Motor B 位于后方。高层混控直行时 A 为负、D 为正、B 为 0，A/D 的相反符号已经对应两只镜像安装的轮子，因此当前 `MOTOR_D_SIGN=1`，不再对 D 做第二次反向。转弯时 A/D 使用相反的差速修正，B 提供旋转分量。第一次运动测试必须架空车轮。

当日志稳定显示 `RAW=1001 ACTIVE=0110 err=0` 时，程序固定输出 `motor[A,B,D]=[-18,0,18]`。当前 `MOTOR_D_SIGN=1` 会直接把 `D=+18` 送给驱动器，与 `A=-18` 保持相反；日志显示的是混控层数值。两侧幅值保持相同，直线偏差交给 PID 在传感器离开中心后修正。

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

1. 架空轮子，让 OUT2/OUT3 对准黑线，确认日志为 `1001/0110`，A/D 反向旋转，B 不动。
2. 放在长直道上，只调整 `STRAIGHT_A_SPEED` 和 `STRAIGHT_D_SPEED`，先解决机械直行。
3. 测试缓弯，先调整 `PID_KP`，有持续偏差再调整 `PID_KI`，摆动明显再提高 `PID_KD`。
4. 测试 60/90 度弯；纠偏过强先降低 `PID_KP`，丢线转向不足再提高 `TURN_MAX`。

开始地面测试时应预留断电空间；若传感器读数异常或小车转向与误差方向相反，立即断开电机电源，不要直接提高 PWM。
