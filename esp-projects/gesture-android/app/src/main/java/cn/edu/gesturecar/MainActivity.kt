@file:Suppress("DEPRECATION")
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
    private var gestureMotionActive = false
    private var pendingGesturePermission = false
    private var deviceDialog: android.app.AlertDialog? = null
    private lateinit var gesturePreview: ImageView
    private var scanLabel: TextView? = null
    private var displayRotation = -1
    private lateinit var connectionBadge: TextView
    private lateinit var yawPanel: LinearLayout
    private lateinit var musicPanel: LinearLayout
    private var foreground = false
    private var ready = false
    private var lastCarStatus = 0L
    private var forceSystemSpeech = true
    private var awaitingSpeechActivity = false
    private var manualHeld = false
    private var forward = 0f; private var lateral = 0f; private var manualRotation = 0f
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

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: android.content.Intent?) {
        if (::speech.isInitialized && speech.acceptActivityResult(requestCode, resultCode, data)) {
            awaitingSpeechActivity = false
            return
        }
        super.onActivityResult(requestCode, resultCode, data)
    }
    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()
    private fun color(s: String) = Color.parseColor(s)
    private fun surface(fill: String) = GradientDrawable().apply { setColor(color(fill)); cornerRadius = dp(16).toFloat(); setStroke(dp(1), color("#243B56")) }
    private fun label(value: String, size: Float = 14f, bold: Boolean = false) = TextView(this).apply {
        text = value; textSize = size; setTextColor(color("#DCE9FA"))
        if (bold) typeface = Typeface.DEFAULT_BOLD
        setPadding(0, dp(6), 0, dp(6))
    }
    private fun button(value: String, action: () -> Unit) = Button(this).apply {
        text = value; textSize = 13f; isAllCaps = false; setTextColor(Color.WHITE)
        background = surface("#172A4A"); setOnClickListener { action() }
    }
    private fun column() = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
    private fun row() = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
    private fun LinearLayout.equal(view: View) = addView(view, LinearLayout.LayoutParams(0, -2, 1f))

    override fun onCreate(state: Bundle?) {
        super.onCreate(state)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        window.statusBarColor = color("#070E1A"); window.navigationBarColor = color("#070E1A")
        if (Build.VERSION.SDK_INT >= 28) window.attributes = window.attributes.apply {
            layoutInDisplayCutoutMode = WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES
        }
        if (Build.VERSION.SDK_INT >= 30) window.setDecorFitsSystemWindows(false)
        val root = column().apply {
            layoutDirection = View.LAYOUT_DIRECTION_LTR
            setPadding(dp(14), dp(8), dp(14), dp(8)); setBackgroundColor(color("#070E1A"))
        }
        setContentView(root)
        connectionLabel = label("等待连接 · 组号 " + Protocol.GROUP_ID, 11f).apply { maxLines = 1 }
        connectionBadge = label("● 未连接", 12f, true).apply {
            gravity = Gravity.CENTER; setPadding(dp(10), 0, dp(10), 0)
            background = surface("#301D2B"); setTextColor(color("#FFB4A8"))
        }
        root.addView(row().apply {
            gravity = Gravity.CENTER_VERTICAL
            addView(label(getString(R.string.app_name), 17f, true), LinearLayout.LayoutParams(0, -2, 1f))
            addView(connectionBadge, LinearLayout.LayoutParams(-2, dp(32)).apply { marginEnd = dp(8) })
            addView(button("连接车辆") { showConnections() }, LinearLayout.LayoutParams(dp(88), dp(40)))
            addView(button("断开") { transport.disconnect("已断开，电机停车；车端音乐继续") },
                LinearLayout.LayoutParams(dp(64), dp(40)).apply { marginStart = dp(8) })
        }, LinearLayout.LayoutParams(-1, dp(44)))
        modes = RadioGroup(this).apply { orientation = RadioGroup.HORIZONTAL }
        for ((index, name) in listOf("双摇杆", "语音", "手势", "音乐").withIndex()) {
            modes.addView(RadioButton(this).apply { id = 100 + index; text = name; textSize = 13f },
                LinearLayout.LayoutParams(0, dp(40), 1f))
        }
        root.addView(modes)
        devices = column()
        manualPanel = column(); voicePanel = column(); gesturePanel = column(); musicPanel = column()
        buildManualPanel()
        voiceLabel = label("左手平移 · 右手旋转 · 任一松手停车", 11f).apply {
            maxLines = 1; ellipsize = android.text.TextUtils.TruncateAt.END
        }
        speech = PhoneSpeechInput(this, { voiceLabel.text = it }, ::voiceResult)
        voicePanel.addView(label("PHONE / 中文语音控车", 18f, true))
        voicePanel.addView(label("前进、后退、左移、右移：3 秒；左转、右转：0.5 秒", 13f))
        voicePanel.addView(button("说运动指令") { listen() })
        voicePanel.addView(CheckBox(this).apply {
            text = "系统语音服务（可能联网）"; textSize = 13f
            setOnCheckedChangeListener { _, checked ->
                stopInputs(); forceSystemSpeech = checked; transport.requestDrive()
            }
        })
        gesturePanel.orientation = LinearLayout.HORIZONTAL
        gestureLabel = label("整只手入镜，张掌静止半秒定中", 12f).apply { maxLines = 6 }
        gesturePreview = GesturePreviewView(this).apply {
            contentDescription = "手势摄像头实时画面"
            scaleType = ImageView.ScaleType.FIT_CENTER
            setBackgroundColor(Color.BLACK)
        }
        gesturePanel.addView(gesturePreview, LinearLayout.LayoutParams(0, -1, 1f))
        gesturePanel.addView(column().apply {
            setPadding(dp(8), 0, 0, 0)
            addView(label("摄像头手势", 14f, true))
            addView(gestureLabel, LinearLayout.LayoutParams(-1, 0, 1f))
            addView(button("启用 / 重新定中") { startGesture() }, LinearLayout.LayoutParams(-1, dp(48)))
            addView(button("停止手势") { releaseControls() }, LinearLayout.LayoutParams(-1, dp(48)))
        }, LinearLayout.LayoutParams(dp(176), -1))
        musicLabel = label("等待音乐服务", 13f)
        transport = BleCarTransport(this, drive::frame, this)
        music = loadMusic()
        musicPanel.addView(musicLabel)
        val tracks = Spinner(this)
        tracks.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_dropdown_item, music.tracks.map { it.title })
        musicPanel.addView(tracks)
        musicPanel.addView(row().apply {
            equal(button("播放") { music.tracks.getOrNull(tracks.selectedItemPosition)?.let { music.submit(MediaCommand.PlayTitle(it.title)) } })
            equal(button("语音点歌") { listen() })
        })
        musicPanel.addView(row().apply {
            equal(button("暂停") { music.submit(MediaCommand.Pause) })
            equal(button("继续") { music.submit(MediaCommand.Resume) })
            equal(button("停止音乐") { music.submit(MediaCommand.Stop) })
        })
        val panels = FrameLayout(this).apply {
            addView(manualPanel, FrameLayout.LayoutParams(-1, -1))
            for (panel in listOf(voicePanel, gesturePanel, musicPanel)) {
                val container = if (panel === gesturePanel) FrameLayout(this@MainActivity).apply {
                    addView(panel, FrameLayout.LayoutParams(-1, -1))
                } else ScrollView(this@MainActivity).apply { addView(panel); tag = panel }
                addView(container, FrameLayout.LayoutParams(-1, -1))
            }
        }
        val center = column().apply {
            setPadding(dp(10), dp(6), dp(10), dp(6)); background = surface("#0C1626")
        }
        carLabel = label("等待车辆状态", 12f).apply {
            setAutoSizeTextTypeUniformWithConfiguration(9, 12, 1, android.util.TypedValue.COMPLEX_UNIT_SP)
        }
        telemetry = label("前进 0% · 横移 0% · 旋转 0%", 11f).apply {
            typeface = Typeface.MONOSPACE
            setAutoSizeTextTypeUniformWithConfiguration(9, 11, 1, android.util.TypedValue.COMPLEX_UNIT_SP)
        }
        center.addView(carLabel, LinearLayout.LayoutParams(-1, 0, 1f))
        center.addView(telemetry, LinearLayout.LayoutParams(-1, dp(48)))
        center.addView(button("急停 / STOP") {
            stopInputs(); drive.emergency(true, now()); transport.requestDrive()
            voiceLabel.text = "急停已锁定 · 解除后松开两杆再操作"
        }.apply { background = surface("#C63E50") }, LinearLayout.LayoutParams(-1, dp(48)))
        center.addView(button("解除急停") {
            stopInputs(); drive.emergency(false, now()); transport.requestDrive()
            voiceLabel.text = "急停已解除 · 松开两杆等待就绪"
        }, LinearLayout.LayoutParams(-1, dp(48)).apply { topMargin = dp(6) })
        root.addView(row().apply {
            isMotionEventSplittingEnabled = true
            addView(panels, LinearLayout.LayoutParams(0, -1, 1f))
            addView(center, LinearLayout.LayoutParams(dp(180), -1).apply { setMargins(dp(8), 0, dp(8), 0) })
            addView(yawPanel, LinearLayout.LayoutParams(0, -1, 1f))
        }, LinearLayout.LayoutParams(-1, 0, 1f).apply { topMargin = dp(4) })
        root.addView(row().apply {
            addView(connectionLabel, LinearLayout.LayoutParams(0, -1, 1f))
            addView(voiceLabel, LinearLayout.LayoutParams(0, -1, 1.5f))
        }, LinearLayout.LayoutParams(-1, dp(28)))
        modes.setOnCheckedChangeListener { _, id ->
            stopInputs()
            drive.select(when (id) { 101 -> ControlSource.VOICE; 102 -> ControlSource.GESTURE; else -> ControlSource.MANUAL }, now())
            manualPanel.visibility = if (id == 100) View.VISIBLE else View.GONE
            yawPanel.visibility = if (id == 100) View.VISIBLE else View.GONE
            for ((index, panel) in listOf(voicePanel, gesturePanel, musicPanel).withIndex()) {
                (panel.parent as View).visibility = if (id == 101 + index) View.VISIBLE else View.GONE
            }
            voiceLabel.text = when (id) {
                101 -> "点击说运动指令 · 平移 3 秒，旋转 0.5 秒"
                102 -> "张掌：上前下后、左右平移 · 握拳：旋转"
                103 -> "仅播放车端已导入曲库 · 不执行运动语音"
                else -> "左手平移 · 右手旋转 · 任一松手停车"
            }
            transport.requestDrive()
        }
        modes.check(modes.getChildAt(0).id)
        root.setOnApplyWindowInsetsListener { view, insets ->
            val safe = if (Build.VERSION.SDK_INT >= 30) {
                val edges = insets.getInsets(WindowInsets.Type.systemBars() or WindowInsets.Type.displayCutout())
                intArrayOf(edges.left, edges.top, edges.right, edges.bottom)
            } else if (Build.VERSION.SDK_INT >= 28) {
                val cutout = insets.displayCutout
                intArrayOf(maxOf(insets.systemWindowInsetLeft, cutout?.safeInsetLeft ?: 0),
                    maxOf(insets.systemWindowInsetTop, cutout?.safeInsetTop ?: 0),
                    maxOf(insets.systemWindowInsetRight, cutout?.safeInsetRight ?: 0),
                    maxOf(insets.systemWindowInsetBottom, cutout?.safeInsetBottom ?: 0))
            } else {
                intArrayOf(insets.systemWindowInsetLeft, insets.systemWindowInsetTop, insets.systemWindowInsetRight, insets.systemWindowInsetBottom)
            }
            val padding = intArrayOf(dp(14)+safe[0], dp(8)+safe[1], dp(14)+safe[2], dp(8)+safe[3])
            if (view.paddingLeft != padding[0] || view.paddingTop != padding[1] || view.paddingRight != padding[2] || view.paddingBottom != padding[3]) {
                releaseControls(); view.setPadding(padding[0], padding[1], padding[2], padding[3])
            }
            insets
        }
        root.addOnLayoutChangeListener { _, left, top, right, bottom, oldLeft, oldTop, oldRight, oldBottom ->
            if (oldRight > oldLeft && (right-left != oldRight-oldLeft || bottom-top != oldBottom-oldTop)) releaseControls()
        }
        root.requestApplyInsets(); enterImmersive()
    }
    private fun buildManualPanel() {
        joystick = JoystickView(this).apply {
            accentColor = color("#5CE4CC"); contentDescription = "左摇杆：前后及左右平移，松手停车"
            listener = object : JoystickView.Listener {
                override fun onStart() { startManual() }
                override fun onMove(x: Float, y: Float) { val axes = JoystickInput.translation(x, y); forward = axes.first; lateral = axes.second }
                override fun onEnd() { releaseControls() }
            }
        }
        yawJoystick = JoystickView(this).apply {
            rotationOnly = true; accentColor = color("#85ACFF"); contentDescription = "右摇杆：横向旋转，松手停车"
            listener = object : JoystickView.Listener {
                override fun onStart() { startManual() }
                override fun onMove(x: Float, y: Float) { manualRotation = JoystickInput.rotation(x) }
                override fun onEnd() { releaseControls() }
            }
        }
        manualPanel.addView(stickPanel("01 / MOVE · 平移", joystick, "#5CE4CC"), LinearLayout.LayoutParams(-1, -1))
        yawPanel = stickPanel("02 / ROTATE · 旋转", yawJoystick, "#85ACFF")
    }
    private fun stickPanel(title: String, stick: JoystickView, accent: String) = column().apply {
        setPadding(dp(10), dp(6), dp(10), dp(6)); background = surface("#0D1B2D")
        isMotionEventSplittingEnabled = true
        addView(label(title, 14f, true).apply { setTextColor(color(accent)) })
        addView(stick, LinearLayout.LayoutParams(-1, 0, 1f))
    }
    private fun releaseControls() {
        stopInputs()
        if (::transport.isInitialized) transport.requestDrive()
    }
    private fun enterImmersive() {
        if (Build.VERSION.SDK_INT >= 30) {
            window.insetsController?.apply {
                systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
                hide(WindowInsets.Type.systemBars())
            }
        } else {
            window.decorView.systemUiVisibility = View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or View.SYSTEM_UI_FLAG_FULLSCREEN or
                View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or View.SYSTEM_UI_FLAG_LAYOUT_STABLE or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
        }
    }
    private fun checkDisplayRotation() {
        val current = windowManager.defaultDisplay.rotation
        if (displayRotation != -1 && current != displayRotation) releaseControls()
        displayRotation = current
    }
    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) enterImmersive() else releaseControls()
    }
    private fun showConnections() {
        if (deviceDialog?.isShowing == true) return
        releaseControls()
        devices = column()
        scanLabel = label("扫描会断开当前连接，请选择自己的车辆", 12f)
        val container = column().apply {
            setPadding(dp(16), 0, dp(16), 0)
            addView(scanLabel)
            addView(ScrollView(this@MainActivity).apply { addView(devices) },
                LinearLayout.LayoutParams(-1, (resources.displayMetrics.heightPixels * .35f).toInt()))
        }
        val dialog = android.app.AlertDialog.Builder(this).setTitle("连接车辆 · 组号 " + Protocol.GROUP_ID)
            .setView(container).setPositiveButton("扫描", null).setNegativeButton("关闭", null).create()
        deviceDialog = dialog
        dialog.setOnShowListener {
            dialog.getButton(android.app.AlertDialog.BUTTON_POSITIVE).setOnClickListener { scan() }
            dialog.window?.setLayout(minOf(dp(520), resources.displayMetrics.widthPixels-dp(40)), -2)
        }
        dialog.setOnDismissListener {
            transport.stopScan(); deviceDialog = null; scanLabel = null
            releaseControls()
            if (foreground) enterImmersive()
        }
        dialog.show()
    }
    private fun startManual() {
        speech.cancel()
        if (!manualHeld && foreground && hasWindowFocus() && modes.checkedRadioButtonId == 100)
            manualHeld = drive.begin(ControlSource.MANUAL, now())
    }
    private fun stopGesture() {
        ++gestureEpoch
        gestureMotionActive = false
        val old = gesture; gesture = null
        try { old?.stop() } catch (_: RuntimeException) { }
        if (::gesturePreview.isInitialized) gesturePreview.setImageDrawable(null)
        if (::gestureLabel.isInitialized) gestureLabel.text = "手势已停止"
    }
    private fun stopInputs() {
        if (::speech.isInitialized) speech.cancel()
        stopMotion()
    }
    private fun stopMotion() {
        stopGesture(); drive.stop(now()); manualHeld = false
        forward = 0f; lateral = 0f; manualRotation = 0f
        if (::joystick.isInitialized) joystick.reset()
        if (::yawJoystick.isInitialized) yawJoystick.reset()
    }
    private fun startGesture() {
        stopInputs()
        transport.requestDrive()
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            pendingGesturePermission = true
            requestPermissions(arrayOf(Manifest.permission.CAMERA), 12)
            return
        }
        val feature = try { GestureSlot.create(this, gesturePreview) } catch (_: RuntimeException) {
            gestureLabel.text = "手势模块初始化失败"; return
        }
        val epoch = gestureEpoch
        handler.postDelayed({
            if (!foreground || epoch != gestureEpoch || drive.selected != ControlSource.GESTURE) return@postDelayed
            gesture = feature
            try { feature.start(object : GestureOutput {
                override fun motion(forward: Float, lateral: Float, rotation: Float, capturedAtMs: Long) { handler.post {
                    if (epoch == gestureEpoch && foreground && gesture === feature) {
                        if (!gestureMotionActive) gestureMotionActive = drive.begin(ControlSource.GESTURE, now())
                        if (gestureMotionActive) gestureMotionActive = drive.submit(ControlSource.GESTURE, forward, lateral, rotation, capturedAtMs, now())
                        if (!gestureMotionActive) gestureLabel.text = "识别中；等待车辆就绪或新鲜手势数据"
                    }
                } }
                override fun fist() { handler.post {
                    if (epoch == gestureEpoch && foreground && gesture === feature && gestureMotionActive) {
                        music.tracks.firstOrNull()?.let { music.submit(MediaCommand.PlayTitle(it.title)) }
                        gestureLabel.text = "握拳：原地旋转，已触发播放"
                    }
                } }
                override fun lost() { handler.post {
                    if (epoch == gestureEpoch && gesture === feature) {
                        if (gestureMotionActive) drive.stop(now())
                        gestureMotionActive = false
                        transport.requestDrive(); gestureLabel.text = "未识别到张开手掌或拳头，已停车；摄像头继续识别"
                    }
                } }
                override fun status(message: String) { handler.post {
                    if (epoch == gestureEpoch && gesture === feature) gestureLabel.text =
                        if (gestureMotionActive) message else "$message\n车辆尚未接受运动或正在定中（保持停车）"
                } }
                override fun error(message: String) { handler.post {
                    if (epoch == gestureEpoch && gesture === feature) {
                        stopInputs(); transport.requestDrive(); gestureLabel.text = message
                    }
                } }
            }) } catch (_: RuntimeException) {
                stopMotion(); transport.requestDrive(); gestureLabel.text = "手势模块启动失败"; return@postDelayed
            }
            gestureLabel.text = "手势已启用"
        }, 400)
    }
    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != 12 || !pendingGesturePermission) return
        pendingGesturePermission = false
        if (grantResults.firstOrNull() != PackageManager.PERMISSION_GRANTED) {
            gestureLabel.text = "摄像头权限被拒绝，请在系统设置中允许"
            return
        }
        handler.postDelayed({
            if (foreground && hasWindowFocus() && modes.checkedRadioButtonId == 102) startGesture()
        }, 400)
    }
    private fun listen() {
        stopInputs(); transport.requestDrive()
        if (!foreground || !ready) { voiceLabel.text = "请先连接小车"; musicLabel.text = "请先连接小车"; return }
        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(arrayOf(Manifest.permission.RECORD_AUDIO), 11); return
        }
        awaitingSpeechActivity = true
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
                    val command = intent.command
                    val duration = if (command.rotation != 0f) 500L else 3000L
                    drive.submit(ControlSource.VOICE, command.forward, command.lateral, command.rotation, now(), now(), duration)
                    voiceLabel.text = "听到“$text” · ${if (duration == 500L) "0.5" else "3"} 秒"
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
        if (missing.isNotEmpty()) {
            scanLabel?.text = "请授予蓝牙权限后重新点击扫描"
            requestPermissions(missing.toTypedArray(), 10); return
        }
        devices.removeAllViews(); transport.scan()
    }
    override fun connection(message: String, ready: Boolean, music: Boolean) {
        this.ready = ready; stopInputs(); drive.link(ready, now()); lastCarStatus = 0
        this.music.link(ready && music); connectionLabel.text = message; scanLabel?.text = message
        connectionBadge.text = if (ready) "● 已连接" else "● 未连接"
        connectionBadge.setTextColor(color(if (ready) "#8FF0C0" else "#FFB4A8"))
        connectionBadge.background = surface(if (ready) "#163B35" else "#301D2B")
        if (!ready) carLabel.text = "等待车辆状态"
        if (!ready) musicLabel.text = "连接已断开，车端已开始的播放继续"
        else if (!music) musicLabel.text = "该车端没有音乐服务"
    }
    override fun device(device: BluetoothDevice) { devices.addView(button("连接 ${device.name ?: "GestureCar"} · ${device.address}") { deviceDialog?.dismiss(); transport.connect(device) }) }
    override fun carStatus(bytes: ByteArray) {
        lastCarStatus = now(); drive.updateStatus(bytes, lastCarStatus); carLabel.text = Protocol.statusText(bytes)
        if (bytes.size != 12 || bytes[0].toInt() != 1 || bytes[1].toInt() !in 1..3 ||
            bytes[2].toInt() != 0 || bytes[11].toInt() != 0) stopMotion()
    }
    override fun musicStatus(bytes: ByteArray) { music.acceptStatus(bytes) }
    private val ticker = object : Runnable {
        override fun run() {
            if (!foreground) return
            checkDisplayRotation()
            val t = now()
            if (ready && lastCarStatus != 0L && t - lastCarStatus > 500) { stopMotion(); lastCarStatus = 0 }
            if (manualHeld && (joystick.active || yawJoystick.active)) {
                if (!drive.submit(ControlSource.MANUAL, forward, lateral, manualRotation, t, t)) manualHeld = false
            }
            val output = drive.output(t)
            telemetry.text = "前进 ${((output?.forward ?: 0f) * 4).toInt()}%\n横移 ${((output?.lateral ?: 0f) * 4).toInt()}%\n旋转 ${((output?.rotation ?: 0f) * 4).toInt()}%"
            transport.tick(); handler.postDelayed(this, 25)
        }
    }
    override fun onResume() { super.onResume(); foreground = true; drive.foreground(true, now()); handler.removeCallbacks(ticker); handler.post(ticker) }
    override fun onPause() {
        if (awaitingSpeechActivity) {
            super.onPause()
            return
        }
        foreground = false; deviceDialog?.dismiss(); stopInputs(); drive.foreground(false, now()); transport.requestDrive()
        transport.disconnect("应用暂停，电机停车；车端音乐继续"); handler.removeCallbacks(ticker); super.onPause()
    }
}
