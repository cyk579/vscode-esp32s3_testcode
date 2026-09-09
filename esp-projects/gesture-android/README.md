# 安卓全向小车控制端

一个 APP 提供双摇杆、手机中文语音和车载音乐控制，手势模块独立接入。源码版本 **1.3-asr-music（versionCode=4）**，首页标识 ASR-2。既有 APK 未更新，需重新构建。硬件、完整方案与验收见 [subject3-ASR](../subject3-ASR/README.md)，组员分工与接口见 [FEATURES.md](FEATURES.md)。

## 构建

Android Studio 打开本目录，或使用 JDK17、Android SDK Platform35：

```powershell
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest
```

沿用 AGP8.7.3、Kotlin2.0.21、Gradle8.10.2、minSdk26、targetSdk35。Wrapper 带发行包 SHA256 校验。SDK 路径在本机 Android Studio、`ANDROID_HOME` 或 `local.properties` 配置，勿照搬或提交其他电脑的绝对路径。

APK：`app/build/outputs/apk/debug/app-debug.apk`。新 APP 兼容旧 OMNI-2 车端的运动服务；音乐需要 `subject3-ASR` 固件、音乐分区和匹配的 APP 曲库。

## 使用

1. 手机打开蓝牙，扫描并连接自己的车（默认组号 1）。等 IMU 正常、车辆 READY。
2. 选择摇杆模式：左杆平移，右杆旋转；任一杆松开都停车，松手后重新触摸。
3. 选择语音模式，点击“说运动指令”，对手机说“前进、后退、左移、右移、左转、右转”。单次运动目标时长 0.8 秒。只执行最终整句匹配；点一次听一次，没有持续监听。
4. 音乐区可直接选曲，或点击“语音点歌”说“播放测试音”。只播放已导入曲库的歌曲，默认仅含测试音。导入方式见 [本地曲库](../subject3-ASR/music/README.md)。
5. 语音“停止”只停车，“停止音乐”才停止歌曲。运动模式切换、急停、BLE 断开和切后台不停止车端已开始的播放。
6. 急停后须手动解除，再重新启用运动。切后台、锁屏后 BLE 断开，返回需重新连接。

首次权限授权可能引起应用暂停；授权后重新连接、再点击。系统中文语音服务缺失或模型不可用时保持停车；可切换到系统语音服务重试，可能需要网络。歌曲外放对手机采音的影响需真机验证。

手势页目前明确显示“未接入”，没有模拟识别结果。组员实现 `GestureFeature` 并在 `GestureSlot` 注入后才能启用，其他功能不依赖这个实现。

## 验证边界

运动心跳每 50ms，单个 GATT 操作在途，200ms 无回调则断开；音乐短命令排在到期运动帧之后。APP 输入失效、断连、后台或模式变化会撤销运动请求，旧请求不恢复。

本机已运行 26 项 JVM 测试，并完成全部 Kotlin 源码的 API35 编译检查。完整 Gradle 打包、Android 运行界面、权限、语音服务、触摸/BLE 和实车音频仍需在目标设备验证。
