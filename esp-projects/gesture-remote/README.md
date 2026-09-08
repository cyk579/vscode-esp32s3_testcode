# ESP32-S3 实体体感手柄（方案 A）

需要第二块 ESP32-S3、MPU6500、常开按键和独立电源。车上使用 `gesture-car`，设置 `ENABLE_CAR_IMU=0`。

在 `main/board_pins.h` 填写 SDA、SCL 和使能按钮 GPIO；按钮接地时有效。传感器按 X 前、Y 左、Z 上安装，或填写轴映射。不要用 C3 固件，本工程目标为 S3。

默认存储配置同现有车端 N32R16V；新板模组型号不同必须单独调整。ESP-IDF 5.5.4 终端进入本目录：

```powershell
$env:PYTHONUTF8='1'
idf.py -B build-local build
idf.py -B build-local -p COM7 flash monitor
```

COM7 只是手柄串口示例，实际自行核对。组号在 `idf.py menuconfig → Gesture experiment` 设置，默认 1，必须与车端相同。

VS Code 可打开仓库根目录 `Gesture-Remote.code-workspace`，与车端工程分开烧录。

上电后把手柄放平约 5 秒，保持静止，看到 `valid=1` 后再拿起。连接车端后先松手回正，再按住按钮倾斜。前后左右符号可通过 `REMOTE_PITCH_SIGN`、`REMOTE_ROLL_SIGN` 调整。若 `WHO_AM_I` 不为 0x70，先核对传感器型号，不直接把身份检查关闭。

完整说明和试车步骤见 [共用说明](../gesture-common/README.md)。
