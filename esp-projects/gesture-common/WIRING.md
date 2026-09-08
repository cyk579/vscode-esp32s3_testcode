# 待填接线表

所有数字均为 **GPIO 号**，不是芯片焊盘号或开发板排针序号。当前分配与 `gesture-car/main/board_pins.h` 及 `引脚对应表-创新实验.xlsx` 一致；接线前仍需按实际板卡丝印核对。

## 车端电机与显示

| 信号 | 宏 | 实测 GPIO |
| --- | --- | --- |
| 右前轮 A PWM | PIN_PWMA | GPIO9 |
| A IN1 / IN2 | PIN_AIN1 / PIN_AIN2 | GPIO12 / GPIO10 |
| 后轮 B PWM | PIN_PWMB | GPIO4 |
| B IN1 / IN2 | PIN_BIN1 / PIN_BIN2 | GPIO6 / GPIO5 |
| 左前轮 D PWM | PIN_PWMD | GPIO16 |
| D IN1 / IN2 | PIN_DIN1 / PIN_DIN2 | GPIO7 / GPIO15 |
| 驱动使能 | PIN_MOTOR_STBY | GPIO8 |
| TFT SCLK / MOSI（表格预留） | PIN_TFT_SCLK / PIN_TFT_MOSI | GPIO13 / GPIO14（当前代码未启用） |
| TFT DC / CS / RST（表格预留） | PIN_TFT_DC / PIN_TFT_CS / PIN_TFT_RST | GPIO21 / GPIO0 / GPIO38（当前代码未启用） |

## 车载 MPU6500

| 信号 | 宏或连接 | 实测 GPIO |
| --- | --- | --- |
| SDA | PIN_IMU_SDA | GPIO17（释放 E2A） |
| SCL | PIN_IMU_SCL | GPIO3（释放 E2B；启动/JTAG 复用，需核对具体模组复位电平与 eFuse） |
| VCC / GND | 按模块规格供电，3.3V 逻辑，共地 | 不适用 |
| AD0 | 接 GND，选择 I2C 地址 0x68 | GND |
| nCS（若模块引出） | 拉高到 3.3V，使用 I2C | 不适用 |
| INT | 本方案轮询数据就绪，不接 | 不适用 |

I2C SDA/SCL 需有到 3.3V 的上拉，优先核对模块已有电阻；固件也启用内部弱上拉。模块供电允许值与裸芯片逻辑电平不是同一件事。

## 手机使能

手机触摸按钮提供使能，不需要实体按键或第二块 ESP32-S3。

## 板级限制

- 默认硬件依据现有仓库的 WROOM-2-N32R16V：只选 0–21、38–46 内适合的 GPIO，UART0 的 43/44 留给日志和下载；47/48 属于该模组 1.8V 域，本实现不接受它们作为外设信号。
- 默认禁止 0、3、45、46 等启动配置脚。当前方案只因 MPU6500 SCL 使用 GPIO3 才显式允许启动脚参与校验；这不是对所有启动脚的通用放行，也不会修改 eFuse。GPIO3 还复用 JTAG 来源，实际启动行为取决于具体模组、电阻网络和 eFuse，必须先断开/限流外设做上电核验。
- 启用外设之间不能重复 GPIO。拆下未使用外设的冲突信号线；不能因软件未初始化旧功能就把相互连接的输出视为不存在。
- 当前固件使用 UART0 控制台，未使用原来的 USB 摄像头；若分配 GPIO19/20 给其他用途，就不能再把它们同时当 USB 数据口使用。
- 电机电源使用现有驱动板的合适供电，与主控共地。STBY 应在上电/复位时由硬件保持低电平。GPIO1/2 仅恢复摄像机水平/垂直舵机的物理接线，本创新实验固件不驱动舵机；GPIO17/3 改作 I2C 后不再接 E2A/E2B 编码器信号。
- 核实车端 S3 的 Flash/PSRAM 型号。默认配置适配现有 N32R16V；若实际模组不同，修改 `sdkconfig.defaults` 后重新配置。
