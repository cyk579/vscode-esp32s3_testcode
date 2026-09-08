# 安卓手机体感控制端

安卓应用通过 BLE 连接车端，使用屏幕双轴虚拟摇杆直接控制小车。小车只需现有的一块 ESP32-S3；MPU6500 接在车上，车端保留 `ENABLE_CAR_IMU=1`。

## 构建

使用 Android Studio 打开本目录，或准备 JDK 17、Android SDK Platform 35 后执行：

```powershell
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest
```

固定 AGP 8.7.3、Kotlin 2.0.21、Gradle 8.10.2；Gradle Wrapper 带发行包 SHA256 校验。安卓最低版本 8.0（API26），目标/编译版本 35。SDK 路径在 Android Studio 中配置，或使用本机 `ANDROID_HOME`，不要提交 `local.properties`。

APK 输出：`app/build/outputs/apk/debug/app-debug.apk`。开启手机开发者选项和 USB 调试后，可用 `adb install -r app/build/outputs/apk/debug/app-debug.apk` 安装，也可将 APK 传至手机手动安装。

## 使用

1. 车端上电，等待车载 MPU 校准完成。手机打开蓝牙和应用，点击扫描并授予所需权限。
2. 从扫描结果中选择自己的车。默认组号 1，需更改时同时修改 `Protocol.GROUP_ID` 和车端配置。
3. 连接成功后，触摸屏幕中央摇杆：向上推前进、向下推后退、向左/右推转向。松开摇杆自动回中并停车。
4. “急停”会锁定控制；点击“解除急停”后，重新触摸摇杆才能驾驶。
5. 切后台、锁屏或离开应用都会断开；重新打开后需要重新连接。应用不在后台继续驾驶。

应用优先选择 `TYPE_GAME_ROTATION_VECTOR`，缺失时使用 `TYPE_ROTATION_VECTOR`；若均不可用则禁止控制。传感器数据超过 100ms 不更新时，发送无效状态。发包每 50ms 一次，只有一条 GATT 写请求在途，写入超过 200ms 未确认就断开。

单元测试与 C 使用相同的 12 字节黄金帧，并检查手机前倾、左倾的符号及参考姿态归零。真实手机的 BLE 权限、时延、姿态手感和后台行为需要实机验收；APK 编译成功不能代替这些验证。
