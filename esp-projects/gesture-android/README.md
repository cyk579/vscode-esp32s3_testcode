# 安卓手机体感控制端

安卓应用通过 BLE 连接车端。当前版本为 1.1-omni2（versionCode=2），首页显示 OMNI-2 · 三轴独立控制。使用两个并排摇杆：左杆二维平移，右杆横向旋转。小车保留车载 IMU 安全保护。

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
3. 连接成功后先松开两杆等待就绪。平移摇杆上推前进、下推后退、左右推横移；旋转摇杆左右推原地旋转。任意一杆松手时两个输入一起归零停车，全部松开后再重新触摸。
4. “急停”会锁定控制；点击“解除急停”后，重新触摸摇杆才能驾驶。
5. 切后台、锁屏或离开应用都会断开；重新打开后需要重新连接。应用不在后台继续驾驶。

当前虚拟摇杆控制不依赖手机姿态传感器。发包每 50ms 一次，只有一条 GATT 写请求在途，写入超过 200ms 未确认就断开。触摸过程中禁止父 ScrollView 抢走竖向滑动，绘制和触摸采用同一个半径。

单元测试与 C 使用相同的 14 字节黄金帧，并验证正上推只产生前进指令、左右平移与旋转独立、不同宽高比下的触摸范围和边缘限幅。真实手机的 BLE 权限、触摸手感、多指释放和后台行为仍需实机验收。
