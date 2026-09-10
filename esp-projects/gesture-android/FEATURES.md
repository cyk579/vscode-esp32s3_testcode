# 同一 APP 的模块分工与手势接入

APP 只有一个 BLE 连接。横屏页面选择“双摇杆 / 语音 / 手势 / 音乐”；进入音乐页先停车且不执行运动语音，已开始的音乐在其他页面继续播放。UI 组装在 `MainActivity.kt`，运动状态、识别会话、播放命令和传输分别由下表模块管理。

路径均相对于 `app/src/main/java/cn/edu/gesturecar`。

| 文件 | 所有权与职责 | 依赖边界 |
| --- | --- | --- |
| `DriveController.kt` | 统一运动入口、所选来源、READY/急停/前后台、状态新鲜度、动作期限 | 只依赖纯协议和 `DriveArbiter`，不持有播放器或识别器 |
| `FeatureContract.kt` | `DriveRequest`、`DriveArbiter`、`MediaCommand`、`MediaSink`、`GestureFeature/Output` | 可独立 JVM 测试的基础接口 |
| `JoystickInput.kt`、`JoystickView.kt` | 触摸坐标转换与摇杆绘制；Activity 适配到 MANUAL 来源 | 不负责 GATT 或曲库 |
| `PhoneSpeechInput.kt` | Android 一轮语音服务、取消、超时与旧回调隔离 | 输出最终文本，不发运动包 |
| `VoiceCommand.kt`、`VoiceRouter.kt` | 完整语句映射及运动/音乐意图区分 | 纯 Kotlin；不会修改播放状态 |
| `MusicController.kt` | 目录匹配、音乐协议、状态反馈、目录指纹校验 | 注入发送函数；不依赖运动模块 |
| `BleCarTransport.kt` | 扫描、连接、状态订阅、串行 GATT、运动心跳优先 | 注入运动帧工厂并回传状态；不运行识别或解码 |
| `GestureSlot.kt` | 手势实现的唯一默认装配点 | 目前返回 null，组员替换并添加自己的识别文件 |
| `MainActivity.kt` | UI 与生命周期组装，统一调用上述接口 | 不在此加入手势模型或音频解码 |

## 手势组员怎样接入

1. 新建自己的识别实现，实现 `GestureFeature`。在 `GestureSlot.create(host: Activity)` 返回实现，可用 host 取得相机权限、Context 等；特征点/类别到运动的映射应保留为可独立测试的纯逻辑。
2. `start(output)` 启动输入源。算法可在工作线程处理手机图像、车端视频或自己的输入源，输入源选择属于手势模块。本版没有替组员实现摄像头权限、视频传输或模型。
3. 识别成功调用 `output.motion(forward, lateral, rotation, capturedAtMs)`。三轴范围均为 **-25f..25f**，零为停止；正方向是前进、左移、逆时针旋转。不要直接发送各个电机的 PWM。
4. `capturedAtMs` 必须使用 `SystemClock.elapsedRealtime()` 同一时基，记录**输入采集时刻**。不得在推理结束时重新打时间戳，掩盖陈旧输入；当前要求输入年龄不超过 150ms，运动期限为采集时间 +200ms。
5. 没有手、低置信度、输入源失效或识别异常时调用 `output.lost()`。APP 停车并结束本次会话，需要再次点击“启用手势”。正常输出零轴值可保持已启用的中立状态。
6. `stop()` 必须快速返回、可重复调用，并释放相机/推理任务。算法不能阻塞主线程；Activity 会把输出回调投递至主线程，用会话编号丢弃停止后的旧结果。

```kotlin
// Called by the detector after inference; capturedAtMs was recorded with the input.
output.motion(
    forward = mappedForward,
    lateral = mappedLateral,
    rotation = mappedRotation,
    capturedAtMs = capturedAtMs
)
// On loss of reliable input:
output.lost()
```

手势实现不持有 `BluetoothGatt`、`MusicController` 或 `PhoneSpeechInput`，不要另起 BLE 连接或发送定时器。权限弹窗可能使 Activity 暂停，授权后应重新连接、重新启用，不自动执行授权前的识别。

如果识别处理速度达不到 150ms，请测量采集/推理延迟并讨论新的控制频率与期限。不要通过伪造新时间戳或无限延长动作来掩盖延迟。

## 控制权与取消规则

生产路径先由 `DriveController.selected` 限定来源，不会让摇杆、语音、手势争抢电机。`DriveArbiter` 仍保留 MANUAL > VOICE > GESTURE 的优先级用于独立输入测试，但只保留当前请求，替换后丢弃旧请求，无后备队列。

只有新启用动作、连接有效、APP 前台、最近 500ms 内车端正常状态、READY 且中立 350ms 后才能起动。语音平移 3000ms、旋转 500ms 到期；摇杆/手势需要持续的新输入，期限为 200ms。超过期限即撤销，即使随后又有新帧也不能自动恢复本次动作，需重新触摸/启用。

顺序语音由 `VoiceRouter` 整句校验后产生最多 6 步的 `Sequence`，由 `DriveController` 执行，不使用多个延迟回调排队。每步到期撤销 HELD，等待至少 350ms 及 READY 后执行下一步；正常步骤间 WAIT_NEUTRAL 不视为故障，活动步骤中出现该状态则取消。等待就绪超过 2 秒、调度间隔超过 250ms 或任何安全取消都会丢弃剩余步骤。

页面切换、释放摇杆、手势丢失、急停、断连、后台、失去窗口焦点及横屏布局变化均撤销运动。所有来源共享急停与车端 IMU 保护。运动取消不调用 `MediaSink.Stop`。语音点歌开始采音前会停车，但不会停止音乐。

## 音乐协议

原运动服务 UUID、14 字节帧和 12 字节状态保持不变。新增音乐服务：

| 属性 | UUID |
| --- | --- |
| Primary service | `7f520001-1b15-4b85-9c13-8f08604a0001` |
| Command，WRITE with response | `7f520002-1b15-4b85-9c13-8f08604a0001` |
| Status，READ + NOTIFY | `7f520003-1b15-4b85-9c13-8f08604a0001` |

所有多字节字段为 little-endian，ATT 默认 MTU23 已可容纳。手机不会在 GATT 中传 WAV 数据。

| 命令偏移 | 字段 |
| --- | --- |
| 0 | 协议版本 1 |
| 1 | PLAY=1、PAUSE=2、RESUME=3、STOP=4 |
| 2..3 | uint16 序号，用于状态关联；当前没有重放/去重协议 |
| 4..5 | uint16 track ID，仅 PLAY 为非零，其他操作必须为 0 |
| 6..7 | 保留，必须为 0 |
| 8..11 | uint32 曲库 CRC32，PLAY 必须与车端一致 |

| 状态偏移 | 字段 |
| --- | --- |
| 0 | 协议版本 1 |
| 1 | IDLE=0、PLAYING=1、PAUSED=2、ERROR=3 |
| 2 | OK=0、NO_USB=1、NO_STORAGE=2、CATALOG_MISMATCH=3、NO_TRACK=4、BAD_WAV=5、IO_ERROR=6 |
| 3 | flags bit0 USB 格式就绪、bit1 曲库挂载就绪 |
| 4..5 | 最近处理的命令序号 |
| 6..7 | 当前/最近 track ID |
| 8..11 | 已提交音频的位置，单位秒，不是实际声学检测 |
| 12..15 | 车端曲库 CRC32 |

WRITE 成功只表示车端接收并排队，不保证喇叭已经播放；APP 等待状态显示结果。错误 PLAY 保持已有歌曲，状态 error 表示最近的命令或播放故障。运动故障只影响电机，播放器单独处理 USB/文件错误。

APP 始终只有一个 GATT 操作在途；每到 50ms 优先发运动帧，间隙处理音乐。手机音乐队列最多 4 条、2 秒过期、断连清空；GATT 操作 200ms 无回调则断开。车端音乐队列最多 8 条，GATT 回调无文件读取或 USB 等待。共用无线链路仍可能有调度延迟，车端独立超时停车负责兜底。

状态和音乐服务不构成认证机制，组号仅用于筛选自己的实验车，沿用仓库原有 BLE 访问方式。

## 独立测试矩阵

| 测试对象 | 所需依赖 | 应核对 |
| --- | --- | --- |
| 原摇杆 | 原 OMNI-2 或新固件；手势保持 null | 双杆、六方向、释放归零、急停/重连；无需语音服务或 USB |
| 语音映射 | JVM 测试直接传入文字 | 否定句拒绝、整句动作、歌名意图不发运动 |
| 语音实机 | 有中文服务的手机 + 任意兼容车端 | 权限、取消、平移 3s / 旋转 0.5s、组合步骤间停车、就绪失败不执行；无需手势 |
| 手势映射 | 组员自己的样本和纯函数测试 | 类别/坐标到三轴、低置信度 lost；无需 BLE、语音或音乐 |
| 手势控制接入 | 测试中实现 GestureFeature，提交有时间戳的样本 | 超期、丢失、模式切换、旧回调拒绝；不在发布 APP 假装识别成功 |
| 音乐 | 新固件 + 匹配曲库 + USB Speaker | 可只点按钮测试播放/暂停/继续/停止；无需语音或手势 |
| 联合验收 | 足够长的导入歌曲 + 实车 | 播放中运动、切模式、急停、BLE 断开，歌曲继续 |

纯逻辑测试位于 `app/src/test/java/cn/edu/gesturecar`，通过 `:app:testDebugUnitTest` 运行。BLE 调度、Android 权限/界面/音频服务、USB 供电和实际连续出声仍需要真机验收。主机测试不代替这些验证。
