package cn.edu.gesturecar

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothDevice
import android.content.pm.PackageManager
import android.graphics.Color
import android.graphics.Typeface
import android.os.*
import android.view.*
import android.widget.*
import android.graphics.drawable.GradientDrawable
import org.json.JSONObject

/** UI composition only. Motion, BLE scheduling, speech and playback have separate owners. */
@SuppressLint("MissingPermission", "SetTextI18n")
class MainActivity : Activity(), BleCarTransport.Listener {
    private val handler = Handler(Looper.getMainLooper())
    private val drive = DriveController()
    private lateinit var transport: BleCarTransport
    private lateinit var speech: PhoneSpeechInput
    private lateinit var music: MusicController
    private var gesture: GestureFeature? = null
    private var gestureEpoch = 0L
    private var foreground = false
    private var ready = false
    private var lastCarStatus = 0L
    private var forceSystemSpeech = false
    private var manualHeld = false
    private var forward = 0f; private var lateral = 0f; private var rotation = 0f
    private lateinit var connectionLabel: TextView
    private lateinit var carLabel: TextView
    private lateinit var voiceLabel: TextView
    private lateinit var musicLabel: TextView
    private lateinit var gestureLabel: TextView
    private lateinit var telemetry: TextView
    private lateinit var devices: LinearLayout
    private lateinit var joystick: JoystickView
    private lateinit var yawJoystick: JoystickView
    private lateinit var modes: RadioGroup
    private lateinit var manualPanel: LinearLayout
    private lateinit var voicePanel: LinearLayout
    private lateinit var gesturePanel: LinearLayout
    private fun now() = SystemClock.elapsedRealtime()
    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()
    private fun color(s: String) = Color.parseColor(s)
    private fun surface(fill: String) = GradientDrawable().apply { setColor(color(fill)); cornerRadius = dp(6).toFloat() }
    private fun label(value: String, size: Float = 14f, bold: Boolean = false) = TextView(this).apply {
        text = value; textSize = size; setTextColor(color("#263238"))
        if (bold) typeface = Typeface.DEFAULT_BOLD
        setPadding(0, dp(6), 0, dp(6))
    }
    private fun button(value: String, action: () -> Unit) = Button(this).apply {
        text = value; textSize = 14f; isAllCaps = false; setOnClickListener { action() }
    }
    private fun column() = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
    private fun row() = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
    private fun LinearLayout.equal(view: View) = addView(view, LinearLayout.LayoutParams(0, -2, 1f))

    override fun onCreate(state: Bundle?) {
        super.onCreate(state)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val root = column().apply { setPadding(dp(16), dp(8), dp(16), dp(12)); setBackgroundColor(color("#F4F6F5")) }
        setContentView(root)
        root.addView(label("GESTURE CAR", 22f, true))
        connectionLabel = label("未连接 · ASR-2", 13f); root.addView(connectionLabel)
        root.addView(row().apply {
            equal(button("扫描小车") { scan() })
            equal(button("断开") { transport.disconnect("已断开，电机停车；车端音乐继续") })
        })
        // Emergency control stays visible even when the content is scrolled.
        root.addView(row().apply {
            equal(button("急停") { stopInputs(); drive.emergency(true, now()); transport.requestDrive() }.apply { setTextColor(color("#B3261E")) })
            equal(button("解除急停") { stopInputs(); drive.emergency(false, now()); transport.requestDrive() })
        })
        val content = column()
        root.addView(ScrollView(this).apply { addView(content) }, LinearLayout.LayoutParams(-1, 0, 1f))
        devices = column(); content.addView(devices)
        modes = RadioGroup(this).apply { orientation = RadioGroup.HORIZONTAL }
        for ((index, name) in listOf("摇杆", "语音", "手势").withIndex()) {
            modes.addView(RadioButton(this).apply { id = 100 + index; text = name }, LinearLayout.LayoutParams(0, dp(48), 1f))
        }
        content.addView(modes)
        manualPanel = column(); voicePanel = column(); gesturePanel = column()
        content.addView(manualPanel); content.addView(voicePanel); content.addView(gesturePanel)
        buildManualPanel()
        voiceLabel = label("等待语音输入")
        speech = PhoneSpeechInput(this, { voiceLabel.text = it }, ::voiceResult)
        voicePanel.addView(button("说运动指令") { listen() })
        voicePanel.addView(CheckBox(this).apply {
            text = "系统语音服务（可能联网）"; textSize = 13f
            setOnCheckedChangeListener { _, checked -> speech.cancel(); drive.stop(now()); manualHeld = false; forceSystemSpeech = checked; transport.requestDrive() }
        })
        gestureLabel = label("手势识别未接入")
        gesturePanel.addView(gestureLabel)
        gesturePanel.addView(row().apply {
            equal(button("启用手势") { startGesture() })
            equal(button("停止手势") { stopInputs(); transport.requestDrive() })
        })
        telemetry = label("前进 0% · 左移 0% · 左旋 0%", 13f); content.addView(telemetry)
        carLabel = label("等待车辆状态", 13f); content.addView(carLabel)
        content.addView(voiceLabel)
        content.addView(label("车载音乐", 17f, true))
        musicLabel = label("等待音乐服务", 13f); content.addView(musicLabel)
        transport = BleCarTransport(this, drive::frame, this)
        music = loadMusic()
        val tracks = Spinner(this)
        tracks.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_dropdown_item, music.tracks.map { it.title })
        content.addView(tracks)
        content.addView(row().apply {
            equal(button("播放") { music.tracks.getOrNull(tracks.selectedItemPosition)?.let { music.submit(MediaCommand.PlayTitle(it.title)) } })
            equal(button("语音点歌") { listen() })
        })
        content.addView(row().apply {
            equal(button("暂停") { music.submit(MediaCommand.Pause) })
            equal(button("继续") { music.submit(MediaCommand.Resume) })
            equal(button("停止音乐") { music.submit(MediaCommand.Stop) })
        })
        modes.setOnCheckedChangeListener { _, id ->
            stopInputs()
            drive.select(when (id) { 101 -> ControlSource.VOICE; 102 -> ControlSource.GESTURE; else -> ControlSource.MANUAL }, now())
            manualPanel.visibility = if (id == 100) View.VISIBLE else View.GONE
            voicePanel.visibility = if (id == 101) View.VISIBLE else View.GONE
            gesturePanel.visibility = if (id == 102) View.VISIBLE else View.GONE
            transport.requestDrive()
        }
        modes.check(100)
    }
    private fun buildManualPanel() {
        val sticks = row().apply { isMotionEventSplittingEnabled = true }
        joystick = JoystickView(this).apply { listener = object : JoystickView.Listener {
            override fun onStart() { startManual() }
            override fun onMove(x: Float, y: Float) { val axes = JoystickInput.translation(x, y); forward = axes.first; lateral = axes.second }
            override fun onEnd() { stopInputs(); transport.requestDrive() }
        }; background = surface("#263D57") }
        yawJoystick = JoystickView(this).apply { rotationOnly = true; listener = object : JoystickView.Listener {
            override fun onStart() { startManual() }
            override fun onMove(x: Float, y: Float) { rotation = JoystickInput.rotation(x) }
            override fun onEnd() { stopInputs(); transport.requestDrive() }
        }; background = surface("#263D57") }
        sticks.addView(joystick, LinearLayout.LayoutParams(0, dp(180), 1f).apply { marginEnd = dp(6) })
        sticks.addView(yawJoystick, LinearLayout.LayoutParams(0, dp(180), 1f))
        manualPanel.addView(sticks)
    }
    private fun startManual() {
        speech.cancel()
        if (!manualHeld) manualHeld = drive.begin(ControlSource.MANUAL, now())
    }
    private fun stopGesture() {
        ++gestureEpoch
        val old = gesture; gesture = null
        try { old?.stop() } catch (_: RuntimeException) { }
        if (::gestureLabel.isInitialized) gestureLabel.text = "手势已停止"
    }
    private fun stopInputs() {
        if (::speech.isInitialized) speech.cancel()
        stopMotion()
    }
    private fun stopMotion() {
        stopGesture(); drive.stop(now()); manualHeld = false
        forward = 0f; lateral = 0f; rotation = 0f
        if (::joystick.isInitialized) joystick.reset()
        if (::yawJoystick.isInitialized) yawJoystick.reset()
    }
    private fun startGesture() {
        stopInputs()
        val feature = try { GestureSlot.create(this) } catch (_: RuntimeException) {
            gestureLabel.text = "手势模块初始化失败"; return
        }
        if (feature == null) { gestureLabel.text = "手势识别未接入"; return }
        val epoch = gestureEpoch
        handler.postDelayed({
            if (!foreground || epoch != gestureEpoch || drive.selected != ControlSource.GESTURE) return@postDelayed
            if (!drive.begin(ControlSource.GESTURE, now())) { gestureLabel.text = "车辆尚未就绪"; return@postDelayed }
            gesture = feature
            try { feature.start(object : GestureOutput {
                override fun motion(forward: Float, lateral: Float, rotation: Float, capturedAtMs: Long) { handler.post {
                    if (epoch == gestureEpoch && foreground && gesture === feature)
                        drive.submit(ControlSource.GESTURE, forward, lateral, rotation, capturedAtMs, now())
                } }
                override fun lost() { handler.post {
                    if (epoch == gestureEpoch) { stopInputs(); transport.requestDrive(); gestureLabel.text = "手势丢失，已停车" }
                } }
            }) } catch (_: RuntimeException) {
                stopMotion(); transport.requestDrive(); gestureLabel.text = "手势模块启动失败"; return@postDelayed
            }
            gestureLabel.text = "手势已启用"
        }, 400)
    }
    private fun listen() {
        stopInputs(); transport.requestDrive()
        if (!foreground || !ready) { voiceLabel.text = "请先连接小车"; musicLabel.text = "请先连接小车"; return }
        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(arrayOf(Manifest.permission.RECORD_AUDIO), 11); return
        }
        speech.start(forceSystemSpeech)
    }
    private fun voiceResult(text: String) {
        if (!foreground || !ready) return
        voiceLabel.text = "听到“$text”"
        when (val intent = VoiceRouter.route(text)) {
            is VoiceIntent.Play -> music.submit(MediaCommand.PlayTitle(intent.title))
            VoiceIntent.PauseMusic -> music.submit(MediaCommand.Pause)
            VoiceIntent.ResumeMusic -> music.submit(MediaCommand.Resume)
            VoiceIntent.StopMusic -> music.submit(MediaCommand.Stop)
            is VoiceIntent.Drive -> {
                if (intent.command == VoiceCommand.ESTOP) { stopInputs(); drive.emergency(true, now()) }
                else if (intent.command == VoiceCommand.STOP) stopInputs()
                else if (drive.selected != ControlSource.VOICE) voiceLabel.text = "运动语音只在语音模式启用"
                else if (drive.begin(ControlSource.VOICE, now())) {
                    val c = intent.command
                    drive.submit(ControlSource.VOICE, c.forward, c.lateral, c.rotation, now(), now())
                    voiceLabel.text = "听到“$text” · 0.8 秒"
                } else voiceLabel.text = "车辆尚未就绪，本次指令不执行"
                transport.requestDrive()
            }
            VoiceIntent.Unknown -> {
                val title = text.trim().trimEnd('。', '！', '!', '.', '？', '?').trim()
                if (music.resolve(title) != null) music.submit(MediaCommand.PlayTitle(title))
                else { voiceLabel.text = "未匹配运动指令或曲库歌名"; musicLabel.text = "未匹配曲库歌名，播放保持不变" }
            }
        }
    }
    private fun loadMusic(): MusicController = try {
        val json = JSONObject(assets.open("music_catalog.json").bufferedReader().use { it.readText() })
        val array = json.getJSONArray("tracks")
        val tracks = (0 until array.length()).map { i ->
            val item = array.getJSONObject(i); val aliases = item.getJSONArray("aliases")
            MusicTrack(item.getInt("id"), item.getString("title"), (0 until aliases.length()).map { aliases.getString(it) })
        }
        MusicController(tracks, json.getLong("catalog"), transport::enqueueMusic) { musicLabel.text = it }
    } catch (_: Exception) {
        musicLabel.text = "APP 曲库未生成"
        MusicController(emptyList(), 0, transport::enqueueMusic) { musicLabel.text = it }
    }
    private fun scan() {
        val required = if (Build.VERSION.SDK_INT >= 31) arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
                       else arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION)
        val missing = required.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) { requestPermissions(missing.toTypedArray(), 10); return }
        devices.removeAllViews(); transport.scan()
    }
    override fun connection(message: String, ready: Boolean, music: Boolean) {
        this.ready = ready; stopInputs(); drive.link(ready, now()); lastCarStatus = 0
        this.music.link(ready && music); connectionLabel.text = message
        if (!ready) musicLabel.text = "连接已断开，车端已开始的播放继续"
        else if (!music) musicLabel.text = "该车端没有音乐服务"
    }
    override fun device(device: BluetoothDevice) { devices.addView(button("连接 ${device.name ?: "GestureCar"} · ${device.address}") { transport.connect(device) }) }
    override fun carStatus(bytes: ByteArray) {
        lastCarStatus = now(); drive.updateStatus(bytes, lastCarStatus); carLabel.text = Protocol.statusText(bytes)
        if (bytes.size != 12 || bytes[0].toInt() != 1 || bytes[1].toInt() !in 2..3 ||
            bytes[2].toInt() != 0 || bytes[11].toInt() != 0) stopMotion()
    }
    override fun musicStatus(bytes: ByteArray) { music.acceptStatus(bytes) }
    private val ticker = object : Runnable {
        override fun run() {
            if (!foreground) return
            val t = now()
            if (ready && lastCarStatus != 0L && t - lastCarStatus > 500) { stopMotion(); lastCarStatus = 0 }
            if (manualHeld && (joystick.active || yawJoystick.active)) {
                if (!drive.submit(ControlSource.MANUAL, forward, lateral, rotation, t, t)) manualHeld = false
            }
            val output = drive.output(t)
            telemetry.text = "前进 ${((output?.forward ?: 0f) * 4).toInt()}% · 左移 ${((output?.lateral ?: 0f) * 4).toInt()}% · 左旋 ${((output?.rotation ?: 0f) * 4).toInt()}%"
            transport.tick(); handler.postDelayed(this, 25)
        }
    }
    override fun onResume() { super.onResume(); foreground = true; drive.foreground(true, now()); handler.post(ticker) }
    override fun onPause() {
        foreground = false; stopInputs(); drive.foreground(false, now()); transport.requestDrive()
        transport.disconnect("应用暂停，电机停车；车端音乐继续"); handler.removeCallbacks(ticker); super.onPause()
    }
}
