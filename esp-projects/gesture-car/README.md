# ESP32-S3 体感推球车端

完整方案、通信协议、参数解释和验收步骤见 [共用说明](../gesture-common/README.md)，接线见 [待填表](../gesture-common/WIRING.md)。

1. 按 [接线表](../gesture-common/WIRING.md) 核对 `main/board_pins.h` 中的电机引脚；当前保持 `ENABLE_CAR_IMU=1`，MPU6500 使用 GPIO17=SDA、GPIO3=SCL。GPIO1/2 恢复摄像机两个舵机信号，GPIO17/3 不再接 E2A/E2B 编码器。
2. 默认关闭 TFT。若打开 `ENABLE_TFT=1`，填写五个显示引脚，实物按 ST7735 160×128 横屏驱动；其他型号需另行适配。
3. 用 ESP-IDF 5.5.4 终端进入本目录，执行：

```powershell
$env:PYTHONUTF8='1'
idf.py -B build-local build
idf.py -B build-local -p COM6 flash monitor
```

COM6 仅作仓库现有串口示例，实际以设备管理器为准。构建目标由 `sdkconfig.defaults` 固定为 esp32s3，默认 Flash 配置为本车 32MB OPI。未使用 PSRAM，不需要为本实验配置大缓冲。

可从仓库根目录打开 `Gesture-Car.code-workspace`，防止 VS Code 烧录入口落到其他旧工程。中文路径下须启用上面的 Python UTF-8 环境变量，否则部分 Windows Python 环境的 kconfgen 会以 GBK 解码路径失败。

首次构建生成本机 `sdkconfig`。更改 `sdkconfig.defaults` 后，已有配置不自动被覆盖；用 `idf.py menuconfig` 同步修改，或保留备份后重新生成配置。不要烧写旧工程的二进制文件。

正确日志标签为 `car`、`imu`、`ble`。GPIO 留空时会打印 `CONFIG MISSING`，这是预期行为。完成 MPU 校准、BLE 连接和松手回正后，车端状态才会进入 READY；安卓端的 `ready` 只表示 GATT 通知订阅完成，不是车端 READY。
