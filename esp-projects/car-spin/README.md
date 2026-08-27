# ESP32-S3 三轮小车四路红外巡线程序

本工程使用 ESP-IDF 控制三轮 120° 全向底盘的 Motor A、Motor B、Motor D，读取四路数字红外传感器，支持普通巡线、锐角锁向、丢线恢复和 T 型终点停车。连续控制采用加权比例控制与输出限斜率，只有需要记忆先后顺序的锐角和终点使用小型状态控制。每次控制后等待 10 ms（实际周期还包含本轮执行时间），每 100 ms 打印一次状态。

## 引脚对应

### 四路红外模块

程序里的 `sensors` 数组按车体从左到右排列。按本车的实际安装方向，Motor D 一侧为左，Motor A 一侧为右，四个探头依次是 OUT4、OUT3、OUT2、OUT1；这是装车方向，不是把模块正面照片直接朝向车头时的默认方向。

| 红外模块 | ESP32-S3 | 误差位 | 权重 |
| --- | --- | --- | ---: |
| OUT4 | GPIO1 | bit0（最左） | -3 |
| OUT3 | GPIO2 | bit1 | -1 |
| OUT2 | GPIO42 | bit2 | +1 |
| OUT1 | GPIO41 | bit3（最右） | +3 |

如果整车的左右方向装反了，把 `main/main.c` 里这一组引脚顺序颠倒：

```c
static const gpio_num_t sensors[4] = {
    GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_42, GPIO_NUM_41
};
```

### TB6612 四路驱动板（本车使用 A/B/D）

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

ESP32、红外模块、驱动板逻辑地和电机电源必须共地。LQ_R4CHVB 支持 3–5.5 V；本车应给模块供 3.3 V，使 OUT 高电平不超过 ESP32-S3 的 3.3 V。若模块由 5 V 供电，不能把 OUT 直接接 GPIO，应按模块资料加电平转换（或合适的串联限流电阻）。TB6612 的电机电源走驱动板的独立电源输入（图示范围 5.5–15 V），不要从 ESP32 的 3.3 V 口给电机供电。

## 第一次调试：检查传感器

先架空车轮并打开串口监视器，把黑线依次放在每个探头下方。程序上电后在首次检测到黑线前电机保持为 0，日志中的 `ACTIVE` 示例为：

```text
ACTIVE=1000
ACTIVE=0100
ACTIVE=0010
ACTIVE=0001
```

`ACTIVE` 从左到右对应车体实际的 OUT4、OUT3、OUT2、OUT1，`1` 表示检测到黑线。本车模块实测为白底高电平、黑线低电平，因此程序默认：

```c
#define IR_ACTIVE_LEVEL 0
```

如果更换模块后逻辑相反（黑线输出高电平），改为 `1`。也可调节红外板上的电位器，让每一路的指示灯在黑线与白底之间可靠翻转。

## 控制策略

程序复位后等待 2 秒。至少一个探头检测到黑线后才允许电机启动，全白时上电不会盲目运动。

- 误差 `error = 命中探头权重的平均值 × 10`，范围约 ±30。负数表示线在左侧，正数表示线在右侧。
- `error == 0`（`0110`、`1111`、`1001` 等对称情况）：按 `STRAIGHT_SPEED` 直行，转向量为 0。
- `error != 0`：降到 `CURVE_SPEED` 前进，目标转向量为 `-error × TURN_KP / 10`，并限幅到 `TURN_MAX`。实际转向量每周期最多变化 `TURN_SLEW_STEP`，避免四路数字量跳变直接传到电机。
- 短暂 `0000`：前两周期保持当前运动，避免单点漏读造成急停；持续 `0000` 才停止前进，并沿最后一次偏差方向搜索。
- 锐角：连续确认 `1110` 或 `0111` 后锁存左/右方向，先直行约 60 ms 补偿车头传感器与车轮的距离，再保持该方向转弯；至少转动约 80 ms 且连续回到 `0110` 后恢复普通巡线。锁向期间后续相反读数不会令车辆转反。
- T 型终点：连续约 50 ms 检测到 `1111` 才布防；其后约 300 ms 窗口内连续约 120 ms 检测到 `0000` 才进入永久停止。单独的 `0000` 或只有 `1111` 都不会停车。

四路数字探头给出的误差是量化值，而且没有电机速度或航向反馈；I 项容易保留旧方向，D 项会放大图样跳变，因此当前版本不加入 PID 的 I/D，而是在 P 输出端限斜率。串口日志的 `state` 为 `FOLLOW`、`ADVANCE`、`CORNER` 或 `STOPPED`；`finish=1` 表示已经识别到 T 型横杠、正在等待持续 `0000`。

## 主要调参项

都在 `main/main.c` 顶部：

| 参数 | 默认值 | 调整方法 |
| --- | ---: | --- |
| `STRAIGHT_SPEED` | 18 | 零误差时的前进幅值，先用它解决直道速度 |
| `CURVE_SPEED` | 16 | 有误差时的前进幅值，弯道冲出去就调低 |
| `TURN_KP` | 3 | 普通巡线比例增益（单位为 1/10）。纠偏不足调高，来回摆动调低 |
| `TURN_MAX` | 12 | 普通 P 转向的限幅 |
| `TURN_SLEW_STEP` | 3 | 每周期最大转向变化量；减小更平滑但响应更慢 |
| `RECOVERY_TURN` | 18 | 锐角锁向和持续丢线搜索的转向幅值 |
| `MAX_OUTPUT` | 28 | 任一电机的最终输出上限 |

状态时序也在文件顶部，以控制周期数表示：`LOST_CONFIRM_CYCLES=3`、`CORNER_CONFIRM_CYCLES=3`、`CORNER_ADVANCE_CYCLES=6`、`CORNER_MIN_TURN_CYCLES=8`、`FINISH_MARK_CYCLES=5`、`FINISH_BLANK_CYCLES=12`、`FINISH_WINDOW_CYCLES=30`。控制周期约 10 ms，因此数值乘以 10 即近似毫秒数；没有编码器时，前探“距离”只能用时间近似。

## 三轮方向校准

OUT4 一侧是 Motor D，OUT1 一侧是 Motor A，Motor B 在车尾，三轮相隔 120°。混控在 `drive()` 里，是唯一决定方向的地方：

```c
out[0] = clamp(-forward - turn, MAX_OUTPUT);  /* A 右前 */
out[1] = clamp(turn, MAX_OUTPUT);            /* B 后轮，只提供旋转分量 */
out[2] = clamp(-forward + turn, MAX_OUTPUT); /* D 左前，按实车极性取反 */
```

正 `turn` 为左转。这个 120° 切向轮布局下，直行时 A、D 都为负、B 为 0；D 的符号已经按本车实测电机极性校准。

`IN1`/`IN2` 宏始终保持与接线表一致。如果某个电机整体极性相反，应在 `drive()` 中反转该路完整表达式（前进和旋转两部分必须一起反转），或断电后互换该电机的两根输出线。代码在换向时会先撤掉该路 PWM，再切换方向脚；这只是电气保护，不是起转增强。如果日志显示线在右边但小车向左修正，先确认 OUT1/OUT4 的物理左右顺序，确认无误后再校准电机方向。

第一次运动测试必须架空车轮。稳定显示 `ACTIVE=0110 err=0` 时，输出应为 `motor[A,B,D]=[-18,0,-18]`，即 A/D 共同产生前进推力、B 不动。

## 编译、烧录和监视

请在 VS Code 的 ESP-IDF 终端中执行（把尖括号替换成实际路径和串口）：

```powershell
cd <本仓库路径>\esp-projects\car-spin
idf.py set-target esp32s3
idf.py build
idf.py -p <串口> -b 115200 flash monitor
```

如果普通 PowerShell 找不到 `idf.py`，请先运行 ESP-IDF 环境脚本：

```powershell
& '<ESP-IDF目录>\export.ps1'
```

再运行上面的 `idf.py` 命令。串口监视器中按 `Ctrl+]` 退出。

## 建议调试顺序

1. 先确认传感器探头距赛道约 3–10 mm（模块资料给出的可调范围），逐路把黑线放到探头下，确认 `ACTIVE` 位序为 OUT4→OUT1；必要时调电位器。
2. 架空轮子，让 OUT2/OUT3 对准黑线，确认日志为 `ACTIVE=0110 state=FOLLOW err=0`，A/D 共同产生前进推力，B 不动。
3. 放在长直道上，只调 `STRAIGHT_SPEED`，先解决机械直行；若 18% 不能可靠起转，先实测电机最低起转占空比，再决定是否恢复短时起转增强。
4. 测试缓弯，先调 `TURN_KP`；仍有顿挫再减小 `TURN_SLEW_STEP`，但转向跟不上时应增大它。
5. 测试左右锐角：`ADVANCE` 太早结束就增加 `CORNER_ADVANCE_CYCLES`，转向开始太晚则减小；锁向力度不足再提高 `RECOVERY_TURN`。
6. 最后单独测试 T 型终点。误停先增大 `FINISH_MARK_CYCLES` 或 `FINISH_BLANK_CYCLES`；到终点不停则反向调整，并核对日志是否实际出现 `1111→0000`。

开始地面测试时应预留断电空间；若传感器读数异常或小车转向与误差方向相反，立即断开电机电源，不要直接提高 PWM。

进入 `STOPPED` 后只有复位才能重新启动。当前终点判断明确依赖 `1111→0000` 序列；如果实车经过 T 型标记时没有产生这个序列，应依据串口日志调整阈值，而不是把任意一次 `0000` 直接改成停车。
