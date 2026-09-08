@file:Suppress("DEPRECATION")
package cn.edu.gesturecar

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.*
import android.os.*
import android.view.*
import android.widget.*

@SuppressLint("MissingPermission", "SetTextI18n", "ClickableViewAccessibility")
class MainActivity : Activity(), SensorEventListener {
    private val handler = Handler(Looper.getMainLooper())
    private lateinit var sensors: SensorManager
    private var rotation: Sensor? = null
    private var sensorAvailable = false
    private var current = FloatArray(9)
    private var reference: FloatArray? = null
    private var sampleMs = 0L
    private var foreground = false
    private var held = false
    private var estop = false
    private var pitch = 0f
    private var roll = 0f
    private var seq = 0
    private var gatt: BluetoothGatt? = null
    private var control: BluetoothGattCharacteristic? = null
    private var ready = false
    private var pending = false
    private var pendingSince = 0L
    private var connectingSince = 0L
    private var scanner: BluetoothLeScanner? = null
    private var scanCallback: ScanCallback? = null
    private var scanDeadline = 0L
    private val found = mutableSetOf<String>()
    private lateinit var devices: LinearLayout
    private lateinit var connectionLabel: TextView
    private lateinit var anglesLabel: TextView
    private lateinit var carLabel: TextView
    private lateinit var holdButton: Button
    private fun now() = SystemClock.elapsedRealtime()
    private fun adapter(): BluetoothAdapter? = getSystemService(BluetoothManager::class.java)?.adapter

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val scroll = ScrollView(this)
        val page = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(24, 60, 24, 48) }
        scroll.addView(page); setContentView(scroll)
        fun label(text: String, size: Float = 18f) = TextView(this).apply { this.text = text; textSize = size; setPadding(0, 12, 0, 12); page.addView(this) }
        fun button(text: String, action: () -> Unit) = Button(this).apply { this.text = text; page.addView(this); setOnClickListener { action() } }
        label("体感推球遥控", 28f)
        label("屏幕向上、手机顶端朝前。校准后松手回正，看到车端就绪再按住使能。")
        connectionLabel = label("未连接 · 组号 ${Protocol.GROUP_ID}")
        button("扫描小车") { scan() }
        devices = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; page.addView(this) }
        button("断开连接") { disconnect("已断开") }
        anglesLabel = label("等待姿态传感器")
        button("水平校准 / 解除急停") {
            held = false
            if (sensorAvailable && now() - sampleMs < 100 && current[8] > 0.94f) {
                reference = current.copyOf(); pitch = 0f; roll = 0f; estop = false
                carLabel.text = "校准完成，请松手回正至少 0.3 秒"
            } else carLabel.text = "请将手机屏幕朝上、接近水平，等待读数后重试"
            sendLatest()
        }
        holdButton = button("按住使能 · 松手停车") {}
        holdButton.setOnTouchListener { _, e ->
            when (e.actionMasked) {
                MotionEvent.ACTION_DOWN -> { held = ready && reference != null && !estop; sendLatest() }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL, MotionEvent.ACTION_POINTER_DOWN -> { held = false; sendLatest() }
                MotionEvent.ACTION_MOVE -> if (e.x < 0 || e.y < 0 || e.x >= holdButton.width || e.y >= holdButton.height) { held = false; sendLatest() }
            }
            true
        }
        button("急停") { held = false; estop = true; sendLatest(); carLabel.text = "急停已锁定；回正后点击水平校准解除" }
        carLabel = label("请连接小车")
        sensors = getSystemService(SENSOR_SERVICE) as SensorManager
        rotation = sensors.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) ?: sensors.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
        if (rotation == null) anglesLabel.text = "手机没有可用的旋转矢量传感器，禁止控制"
    }
    private val ticker = object : Runnable {
        override fun run() {
            if (!foreground) return
            if (scanCallback != null && now() >= scanDeadline) stopScan()
            if (gatt != null && !ready && now() - connectingSince > 10000) disconnect("连接或服务发现超时，请重试")
            if (pending && now() - pendingSince >= 200) disconnect("控制写入超时，已断开")
            sendLatest()
            holdButton.isEnabled = ready && reference != null && !estop && sensorAvailable && now() - sampleMs < 100
            anglesLabel.text = "前后倾：%.1f°    左右倾：%.1f°\n%s".format(pitch, roll, if (now()-sampleMs<100 && sensorAvailable) "姿态数据正常" else "姿态数据无效")
            handler.postDelayed(this, 50)
        }
    }
    override fun onResume() {
        super.onResume(); foreground = true; held = false; reference = null; sampleMs = 0
        sensorAvailable = rotation?.let { sensors.registerListener(this, it, 10000) } ?: false
        handler.post(ticker)
    }
    override fun onPause() {
        foreground = false; held = false; sendLatest(); disconnect("应用暂停，已停车并断开")
        sensors.unregisterListener(this); sensorAvailable = false; handler.removeCallbacks(ticker)
        super.onPause()
    }
    override fun onSensorChanged(event: SensorEvent) {
        if (!foreground) return
        val next = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(next, event.values)
        if (next.any { !it.isFinite() }) { sensorAvailable = false; held = false; return }
        current = next; sampleMs = event.timestamp / 1000000L; sensorAvailable = true
        reference?.let { val angles = Tilt.relative(it, current); pitch = angles.first; roll = angles.second }
    }
    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) { }
    private fun permissions(): Boolean {
        val required = if (Build.VERSION.SDK_INT >= 31) arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
            else arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION)
        val missing = required.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) { requestPermissions(missing.toTypedArray(), 10); connectionLabel.text = "授权后再次点击扫描"; return false }
        return true
    }
    private fun scan() {
        if (!permissions()) return
        disconnect("扫描中…")
        val adapter = adapter()
        if (adapter == null) { connectionLabel.text = "设备不支持蓝牙"; return }
        if (!adapter.isEnabled) { startActivity(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)); return }
        devices.removeAllViews(); found.clear()
        val callback = object : ScanCallback() {
            override fun onScanResult(type: Int, result: ScanResult) {
                handler.post {
                    if (scanCallback !== this || !foreground) return@post
                    val group = result.scanRecord?.getManufacturerSpecificData(0xffff)
                    if (group == null || group.size != 2 || ((group[0].toInt() and 255) or ((group[1].toInt() and 255) shl 8)) != Protocol.GROUP_ID) return@post
                    if (found.add(result.device.address)) devices.addView(Button(this@MainActivity).apply {
                        text = "连接 ${result.device.name ?: "GestureCar"} · ${result.device.address}"
                        setOnClickListener { connect(result.device) }
                    })
                }
            }
            override fun onScanFailed(code: Int) { handler.post { if (scanCallback === this) { stopScan(); connectionLabel.text = "扫描失败：$code（旧版安卓请检查定位开关）" } } }
        }
        scanner = adapter.bluetoothLeScanner; scanCallback = callback; scanDeadline = now()+10000
        try {
            scanner?.startScan(listOf(ScanFilter.Builder().setServiceUuid(ParcelUuid(Protocol.SERVICE)).build()),
                ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), callback)
        } catch (e: RuntimeException) { stopScan(); connectionLabel.text = "无法扫描：${e.message}" }
    }
    private fun stopScan() {
        val callback = scanCallback; scanCallback = null
        try { if (callback != null) scanner?.stopScan(callback) } catch (_: RuntimeException) { }
        scanner = null
    }
    private fun connect(device: BluetoothDevice) {
        disconnect("正在连接 ${device.address}"); connectingSince = now()
        try { gatt = device.connectGatt(this, false, callbacks, BluetoothDevice.TRANSPORT_LE) }
        catch (e: RuntimeException) { disconnect("连接失败：${e.message}") }
    }
    private fun disconnect(message: String) {
        stopScan(); held = false; ready = false; pending = false; control = null
        val old = gatt; gatt = null
        try { old?.disconnect(); old?.close() } catch (_: RuntimeException) { }
        connectionLabel.text = message
    }
    private fun withGatt(g: BluetoothGatt, action: () -> Unit) { handler.post { if (gatt === g && foreground) action() } }
    private val callbacks = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, state: Int) = withGatt(g) {
            if (status != BluetoothGatt.GATT_SUCCESS || state == BluetoothProfile.STATE_DISCONNECTED) disconnect("蓝牙断开（$status），请重新连接")
            else if (state == BluetoothProfile.STATE_CONNECTED) {
                connectingSince = now(); g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                if (!g.discoverServices()) disconnect("服务发现启动失败")
            }
        }
        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) = withGatt(g) {
            val service = g.getService(Protocol.SERVICE)
            val c = service?.getCharacteristic(Protocol.CONTROL)
            val s = service?.getCharacteristic(Protocol.STATUS)
            val descriptor = s?.getDescriptor(Protocol.CCCD)
            if (status != BluetoothGatt.GATT_SUCCESS || c == null || s == null || descriptor == null ||
                (c.properties and BluetoothGattCharacteristic.PROPERTY_WRITE) == 0) { disconnect("车端服务不兼容"); return@withGatt }
            control = c
            if (!g.setCharacteristicNotification(s, true)) { disconnect("状态通知启用失败"); return@withGatt }
            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            if (!g.writeDescriptor(descriptor)) disconnect("订阅状态失败")
        }
        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) = withGatt(g) {
            if (status != BluetoothGatt.GATT_SUCCESS) disconnect("状态订阅失败：$status")
            else { ready = true; held = false; connectionLabel.text = "已连接；请水平校准并等待车端就绪" }
        }
        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) = withGatt(g) {
            if (c.uuid == Protocol.CONTROL) { pending = false; if (status != BluetoothGatt.GATT_SUCCESS) disconnect("控制写入失败：$status") }
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            val bytes = c.value?.copyOf() ?: return
            withGatt(g) { if (c.uuid == Protocol.STATUS) carLabel.text = Protocol.statusText(bytes) }
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            val bytes = value.copyOf()
            withGatt(g) { if (c.uuid == Protocol.STATUS) carLabel.text = Protocol.statusText(bytes) }
        }
    }
    private fun sendLatest() {
        val g = gatt ?: return; val c = control ?: return
        if (!ready || pending) return
        val valid = foreground && sensorAvailable && reference != null && now() - sampleMs < 100
        if (!valid) held = false
        val bytes = Protocol.encode(++seq, now(), pitch, roll, valid, held, estop)
        c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT; c.value = bytes
        pending = true; pendingSince = now()
        try { if (!g.writeCharacteristic(c)) disconnect("发送失败，已断开") }
        catch (e: RuntimeException) { disconnect("发送异常：${e.message}") }
    }
}
