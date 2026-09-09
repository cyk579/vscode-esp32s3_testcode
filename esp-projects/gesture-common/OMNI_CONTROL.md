# 手机双摇杆全向控制

## 当前接口

手机横屏合并版为 app-debug-landscape.apk，版本 1.4-landscape-asr，首页显示 GESTURE / DRIVE · ASR。保留双摇杆、全屏和窗口失焦释放逻辑，并整合语音、音乐与手势扩展接口。14 字节运动协议与 OMNI-2 固件兼容，横屏运动控制无需重新烧录；车载音乐需要 subject3-ASR 固件和对应音频硬件。下文 app-debug-omni2.apk 是仍兼容的旧竖屏界面。

- 左摇杆上/下为前进/后退，左/右为横移；右摇杆横向为旋转，纵向保留、不输出第四自由度。
- 两杆可同时触摸以组合平移与旋转。任一杆松手或触摸取消，两个输入一起清零并停车；松开全部手指再重新触摸。
- 连接后先松开两杆回中至少 300ms，等待车辆就绪。急停、失联 250ms、IMU 无效与倾覆保护仍保留。
- 右上角徽章显示 BLE 连接状态；车辆就绪与故障是独立状态，不等于蓝牙断开。

## 三轮几何及符号

OMNI-2 不再根据观察照片时的左右关系猜测电机电气极性，而是沿用本仓库 car-spin/main/main.c 的 A/B/D 接头控制约定：直行 A负/B零/D正，左移三轮负，左旋 A负/B正/D负。
MOTOR_A_SIGN=+1、MOTOR_B_SIGN=+1、MOTOR_D_SIGN=-1 仍只在驱动层应用一次。

三轮等距切向布局的混控为：

    A = -√3/2 × forward - 0.5 × lateral - yaw
    B =                  -1 × lateral + yaw
    D = +√3/2 × forward - 0.5 × lateral - yaw

上一版的前進 A/D 同号、旋转 A/B 同号，不符合旧驱动约定；不能用那套矩阵自身的逆解测试来证明实车方向正确。
三个目标统一限幅到最大 30% PWM。斜坡使用三轮误差中的最大值计算共同插值比例，直行、横移、旋转从零起步时逐周期保持各自轮速指令比例；仍保留每轮至少 20ms 的换向零输出间隔。

满量程、稳定后串口请求值（尚未应用驱动层极性）应约为：

| 操作 | A | B | D |
| --- | ---: | ---: | ---: |
| 前进 | -26 | 0 | 26 |
| 后退 | 26 | 0 | -26 |
| 左移 | -15 | -30 | -15 |
| 右移 | 15 | 30 | 15 |
| 左旋 | -30 | 30 | -30 |
| 右旋 | 30 | -30 | 30 |

后轮 B 为横向驱动轮时，直行 B 不主动驱动是预期行为，不能把三轮都转当成直行正确的判断标准。

OMNI-2 取消驱动层 A/D<11%、B<13% 的逐轮裁零。旧逻辑下横移输入约在满杆的 55%～79% 之间会只输出 B，必然破坏混控比例。
现在所有轮子保留计算得到的 PWM，仅有 PWM 计数器的取整，不提高 30% 上限、不添加起转冲击。
这只消除了软件造成的比例失真，并不能消除真实电机静摩擦、负载差异和机械卡滞。若满幅命令下 A/D 仍不转，需架空检查实际轮向、供电与起转占空比，再决定标定或速度闭环，不能擅自提高单轮输出。

## 控制协议

小端、固定 14 字节；版本字段仍为 1，以长度严格拒绝历史 12 字节控制帧。

| 偏移 | 内容 |
| --- | --- |
| 0 | version=1 |
| 1 | VALID=1、HELD=2、ESTOP=4 |
| 2–3 | uint16 sequence |
| 4–5 | int16 forward，单位 0.01，满杆为 ±2500 |
| 6–7 | int16 lateral，单位同上，正值左移 |
| 8–9 | int16 yaw，单位同上，正值左旋 |
| 10–13 | uint32 uptime_ms |

12 字节状态通知不变，最后一字节为 IMU health。
当前串口版本标识为 omni2，保留 cmd=forward,lateral,yaw、seq、flags、age 和 IMU health。omni14 是旧混控固件，不应继续用于本轮验收。

## 验证步骤

1. 安装仓库根目录 app-debug-omni2.apk（版本 1.1-omni2，versionCode=2）。打开后必须看到 OMNI-2 · 三轴独立控制、平移摇杆和旋转摇杆两个并排控件；若仍显示“双轴虚拟摇杆”，不要继续用那版测试。
2. 架空车轮。按现有硬件限制，烧录后拔掉与摄像头 USB 冲突的 COM6 接线，使用正常运行供电并复位；COM11 用于日志。
3. 静止等待约 5 秒，确认 IMU 正常、fault=0；连接后松开两杆，等待就绪。
4. 分别短暂测试前、后、左移、右移、左旋、右旋，对照上表并观察实际轮子。
5. 测试双指同时操作、松开任意一杆、急停、蓝牙断开、切后台，均应撤销输出，不得自动恢复运动。
6. 架空方向与停车测试通过后，才在清空障碍物的地面低速测试。

主机测试覆盖协议、状态机、超时、换向、三轴回正、27 种三轴组合的逆解、比例限幅和正反对称性，以及三种纯运动起步过程中每个控制周期的比例与斜率限制。
编译与烧录校验不等于实车全向运动验收；实际轮向和带载运动仍需要观察。

## 本机可复现的构建与烧录

本次使用 C:/Espressif/frameworks/idf-extract/esp-idf-5.5.4，以及 C:/Espressif/tools/python/v5.4.4/venv/Scripts/python.exe。
Python 环境目录名不是 ESP-IDF 源码版本；不能误调用旧 PowerShell 配置中的 C:/esp/v5.4.4/esp-idf。
PATH 需包含 cmake/3.30.2/bin、ninja/1.12.1、xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin。

仓库根目录运行 gesture-common/test/run_tests.py 后，用上述 Python 运行 gesture-common/build.py car。
只在命令显示 BUILD AND CHECKS PASSED 后，从 gesture-car/build-local/verified/default 烧录：

    python -m esptool --chip esp32s3 --port COM6 --baud 460800 --before default_reset --after hard_reset write_flash @flash_args

PowerShell 中须将最后的参数写成单引号字符串 '@flash_args'，不要让 PowerShell 将其解释为变量展开。
必须使用生成的 flash_args 中地址、模式与三个镜像，不手工猜测偏移，也不使用旧 build 目录中的固件。
BUILD_ORIGIN.txt 和 source-manifest.json 保存构建来源；esptool 三个镜像均报告 Hash of data verified 才能记录烧录成功。

## 旧版启动排障记录（非 OMNI-2 运动验收）

本机实测普通 hard_reset 后 COM11 可能仍为 DOWNLOAD / waiting for download。
本次在该状态下通过同一 Python 执行以下命令后，COM11 出现 SPI_FAST_FLASH_BOOT 和新版 omni14 日志：

    python -m esptool --chip esp32s3 --port COM6 --before no_reset --after watchdog_reset read_mac

只有确认停在下载模式时才使用 no_reset；正常驾驶时不要运行复位命令。
本次启动确认 WHO_AM_I=0x68、校准 200 点、health=0、BLE 开始广播。
启动日志曾出现 acc≈(0.715,0.013,-0.724)，按当前安装映射计算 pitch≈-44.6°，超过 30° 保护阈值，故 fault=1。
这与 IMU 读取/融合失败不同：需水平架空车体并确认模块安装面，再观察故障是否清除，不可通过关闭保护来掩盖。

随后 COM11 持续收到 seq 递增的 14 字节控制，包含 cmd=0,0,2500、cmd=-444,-787,2496 等旋转及三轴组合输入，松手后 flags=1、cmd=0,0,0。
但 acc 的 X/Z 分量仍约为 +0.73/-0.71，state=4、fault=1、imu=0、pwm=0,0,0，尚未通过运动验收。
下一步需要用户水平架空整车并确认模块实际安装倾角；若车体已水平而模块固定倾斜，需要测定安装旋转后配置补偿，不能把任意上电姿态自动当作安全水平。
