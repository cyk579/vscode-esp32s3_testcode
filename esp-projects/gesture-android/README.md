# 安卓手机体感控制端

安卓应用使用手机内置姿态传感器，通过 BLE 连接车端。小车只需现有的一块 ESP32-S3；MPU6500 接在车上，车端保留 `ENABLE_CAR_IMU=1`。

## 构建

使用 Android Studio 打开本目录，或准备 JDK 17、Android SDK Platform 35 后执行：

```powershell
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest
```

固定 AGP 8.7.3、Kotlin 2.0.21、Gradle 8.10.2；Gradle Wrapper 带发行包 SHA256 校验。安卓最低版本 8.0（API26），目标/编译版本 35。SDK 路径在 Android Studio 中配置，或使用本机 `ANDROID_HOME`，不要提交 `local.properties`。

APK 输出：`app/build/outputs/apk/debug/app-debug.apk`。开启手机开发者选项和 USB 调试后，可用 `adb install -r app/build/outputs/apk/debug/app-debug.apk` 安装，也可将 APK 传至手机手动安装。

## 使用

1. 车端上电，等待车载 MPU 校准完成。手机打开蓝牙和应用，点击扫描，授予所需权限；旧版安卓还需系统定位开关可用。
2. 从扫描结果中选择自己的车。默认组号 1，需更改时同时修改 `Protocol.GROUP_ID` 和车端配置。不会自动连接其他组号的车。
3. 手机屏幕朝上、顶端朝前，接近水平放置，点击“水平校准”。看到车端“就绪”后，按住屏幕使能按钮，再倾斜手机。
4. 前倾前进、后倾后退、左倾左转、右倾右转。回正减速，松手撤销驱动；“急停”锁定至再次水平校准。
5. 切后台、锁屏或离开应用都会断开；重新打开后要重新连接、校准和使能。应用不在后台继续驾驶。

应用优先选择 `TYPE_GAME_ROTATION_VECTOR`，缺失时使用 `TYPE_ROTATION_VECTOR`；若均不可用则禁止控制。传感器数据超过 100ms 不更新时，发送无效状态。发包每 50ms 一次，只有一条 GATT 写请求在途，写入超过 200ms 未确认就断开。

单元测试与 C 使用相同的 12 字节黄金帧，并检查手机前倾、左倾的符号及参考姿态归零。真实手机的 BLE 权限、时延、姿态手感和后台行为需要实机验收；APK 编译成功不能代替这些验证。
