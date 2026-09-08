# 验证记录

本文件记录代码检查和可重复构建结果。它不代表已经接线、烧录或在真实小车上完成验证。

## 已完成

- 分支：`subject3`
- 日期：2026-09-08
- 目标芯片：ESP32-S3，ESP-IDF 5.5.4（commit `735507283d5b2f9fb363a1901172dbd9e847945d`）
- 主机测试：`python esp-projects/gesture-common/test/run_tests.py`，通过协议编解码、状态机恢复/超时/急停、姿态超限、断连、换向、序号回绕、混控和 Fusion 姿态检查。
- 车端默认配置：`python esp-projects/gesture-common/build.py car`，当前源码应使用 GPIO17=SDA、GPIO3=SCL，GPIO1/2 仅保留为舵机物理接线。构建通过与实际接线、烧录和实车通过必须分开记录；产物目录中的 `source-manifest.json` 是确认固件来源的依据。
- 安卓端：`python esp-projects/gesture-common/build.py android`，APK、JUnit 单元测试和 lint 均通过；APK 在 `esp-projects/gesture-android/build-local/verified/default/app-debug.apk`。
- 车端显示变体：`python esp-projects/gesture-common/build.py car --car-imu 0 --tft 1` 构建通过，用于确认可选 TFT 编译路径；本次实际方案仍使用默认 `ENABLE_CAR_IMU=1`。
- 构建使用内容哈希 ASCII 快照，源码清单位于每个验证产物目录的 `source-manifest.json`，避免 Windows 中文路径工具链兼容性问题。

## 尚需实测

- `gesture-car/main/board_pins.h` 已按创新实验表配置电机和 MPU6500 GPIO；仍需根据实际 ESP32-S3 板卡丝印、排针和线束核对，改动后重新构建。
- 尚未烧录固件，未连接实际 MPU6500、电机驱动或 TFT，未验证 BLE 距离/时延和车轮方向。
- 安卓 APK 的权限、不同手机姿态传感器、后台切换和 BLE 行为必须在目标手机上验收；APK 编译与单元测试不能替代实机测试。

## 复现命令

在 ESP-IDF 5.5.4 终端中运行：

```powershell
$env:PYTHONUTF8='1'
python esp-projects/gesture-common/test/run_tests.py
python esp-projects/gesture-common/build.py car
python esp-projects/gesture-common/build.py car --car-imu 0 --tft 1  # 仅验证可选 TFT 编译
```

在 JDK 17、Android SDK Platform 35 环境中运行：

```powershell
python esp-projects/gesture-common/build.py android
```
