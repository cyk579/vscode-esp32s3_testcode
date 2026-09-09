@file:Suppress("DEPRECATION")
package cn.edu.gesturecar

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Color
import android.graphics.Typeface
import android.os.*
import android.view.*
import android.widget.*
import android.graphics.drawable.GradientDrawable

@SuppressLint("MissingPermission", "SetTextI18n", "ClickableViewAccessibility")
class MainActivity : Activity() {
    private val handler = Handler(Looper.getMainLooper())
    private var foreground = false; private var held = false; private var estop = false
    private var pitch = 0f; private var roll = 0f; private var yaw = 0f; private var seq = 0
    private var gatt: BluetoothGatt? = null; private var control: BluetoothGattCharacteristic? = null
    private var ready = false; private var pending = false; private var pendingSince = 0L; private var connectingSince = 0L
    private var scanner: BluetoothLeScanner? = null; private var scanCallback: ScanCallback? = null; private var scanDeadline = 0L
    private val found = mutableSetOf<String>()
    private lateinit var devices: LinearLayout; private lateinit var connectionLabel: TextView; private lateinit var connectionBadge: TextView; private lateinit var telemetryLabel: TextView; private lateinit var carLabel: TextView; private lateinit var joystick: JoystickView; private lateinit var yawJoystick: JoystickView; private lateinit var unlockLabel: TextView
    private fun now() = SystemClock.elapsedRealtime()
    private fun adapter(): BluetoothAdapter? = getSystemService(BluetoothManager::class.java)?.adapter
    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()
    private fun c(v: String) = Color.parseColor(v)
    private fun bg(fill: Int, radius: Int = 16, stroke: Int? = null) = GradientDrawable().apply { setColor(fill); cornerRadius = dp(radius).toFloat(); if (stroke != null) setStroke(dp(1), stroke) }
    private fun label(value: String, size: Float, tint: Int, bold: Boolean = false) = TextView(this).apply { text = value; textSize = size; setTextColor(tint); typeface = if (bold) Typeface.DEFAULT_BOLD else Typeface.DEFAULT }
    private fun button(value: String, action: () -> Unit) = Button(this).apply { text = value; textSize = 14f; isAllCaps = false; setTextColor(Color.WHITE); background = bg(c("#172A4A"), 14, c("#2E4D78")); setOnClickListener { action() } }

    override fun onCreate(state: Bundle?) {
        super.onCreate(state); window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); window.statusBarColor = c("#0A1326"); window.navigationBarColor = c("#0A1326")
        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(c("#081224")); setPadding(dp(20), dp(22), dp(20), dp(14)) }
        setContentView(ScrollView(this).apply { addView(root) })
        val header = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        val title = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; layoutParams = LinearLayout.LayoutParams(0, -2, 1f) }
        title.addView(label("GESTURE CAR", 24f, Color.WHITE, true)); title.addView(label("OMNI-2 · 三轴独立控制", 13f, c("#8FA8C8"))); header.addView(title)
        connectionBadge = label("● 未连接", 13f, c("#FFB4A8"), true).apply { setPadding(dp(12), dp(8), dp(12), dp(8)); background = bg(c("#301D2B"), 20) }; header.addView(connectionBadge); root.addView(header)
        connectionLabel = label("等待连接 · 组号 " + Protocol.GROUP_ID, 13f, c("#9DB1CB")); root.addView(connectionLabel, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(10) })
        val actions = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }; val scan = button("扫描小车") { scan() }; val cut = button("断开") { disconnect("已断开") }; actions.addView(scan, LinearLayout.LayoutParams(0, dp(48), 1f).apply { marginEnd = dp(6) }); actions.addView(cut, LinearLayout.LayoutParams(0, dp(48), 1f).apply { marginStart = dp(6) }); root.addView(actions, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(14) })
        devices = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }; root.addView(devices)
        val sticks = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; isMotionEventSplittingEnabled = true }
        val card = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; gravity = Gravity.CENTER_HORIZONTAL; setPadding(dp(6), dp(14), dp(6), dp(14)); background = bg(c("#101F38"), 24, c("#203A5E")) }
        card.addView(label("平移摇杆", 17f, Color.WHITE, true))
        card.addView(label("前后 / 左右平移", 12f, c("#8FA8C8")))
        joystick = JoystickView(this).apply { listener = object : JoystickView.Listener {
            override fun onMove(x: Float, y: Float) { val axes = JoystickInput.translation(x, y); pitch = axes.first; roll = axes.second; updateTelemetry() }
            override fun onStart() { held = ready && !estop; unlockLabel.text = if (held) "控制中 · 任一摇杆松手即停车" else "请先连接并解除急停"; sendLatest() }
            override fun onEnd() { releaseControls() }
        }; background = bg(c("#0B172C"), 22) }
        card.addView(joystick, LinearLayout.LayoutParams(-1, dp(210)).apply { topMargin = dp(12) })
        yawJoystick = JoystickView(this).apply { rotationOnly = true; listener = object : JoystickView.Listener {
            override fun onMove(x: Float, y: Float) { yaw = JoystickInput.rotation(x); updateTelemetry() }
            override fun onStart() { held = ready && !estop; unlockLabel.text = if (held) "控制中 · 任一摇杆松手即停车" else "请先连接并解除急停"; sendLatest() }
            override fun onEnd() { releaseControls() }
        }; background = bg(c("#0B172C"), 22) }
        val yawCard = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; gravity = Gravity.CENTER_HORIZONTAL; setPadding(dp(6), dp(14), dp(6), dp(14)); background = bg(c("#101F38"), 24, c("#203A5E")) }
        yawCard.addView(label("旋转摇杆", 17f, Color.WHITE, true))
        yawCard.addView(label("横向旋转 / 纵向保留", 12f, c("#8FA8C8")))
        yawCard.addView(yawJoystick, LinearLayout.LayoutParams(-1, dp(210)).apply { topMargin = dp(12) })
        sticks.addView(card, LinearLayout.LayoutParams(0, -2, 1f).apply { marginEnd = dp(4) })
        sticks.addView(yawCard, LinearLayout.LayoutParams(0, -2, 1f).apply { marginStart = dp(4) })
        root.addView(sticks, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(18) })
        telemetryLabel = label("前进 0%    左移 0%    左旋 0%", 13f, c("#B9CBE1")); telemetryLabel.gravity = Gravity.CENTER
        root.addView(telemetryLabel, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(12) })
        unlockLabel = label("松开两杆等待就绪，再触摸控制", 13f, c("#6DE3FF")); unlockLabel.gravity = Gravity.CENTER
        root.addView(unlockLabel, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(8) })
        carLabel = label("车辆状态：等待连接", 14f, c("#C6D4E7")); carLabel.setPadding(dp(4), dp(14), dp(4), dp(4)); root.addView(carLabel)
        val controls = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }; val stop = button("急停") { resetControls(); estop = true; sendLatest(); unlockLabel.text = "急停已锁定 · 点击解除急停后再控制" }; val reset = button("解除急停") { estop = false; resetControls(); sendLatest(); unlockLabel.text = "急停已解除 · 触摸摇杆开始控制" }; controls.addView(stop, LinearLayout.LayoutParams(0, dp(48), 1f).apply { marginEnd = dp(6) }); controls.addView(reset, LinearLayout.LayoutParams(0, dp(48), 1f).apply { marginStart = dp(6) }); root.addView(controls, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(10) })
    }
    private fun resetControls() {
        held = false; pitch = 0f; roll = 0f; yaw = 0f
        if (::joystick.isInitialized) joystick.reset()
        if (::yawJoystick.isInitialized) yawJoystick.reset()
        updateTelemetry()
    }
    private fun releaseControls() {
        resetControls(); unlockLabel.text = "已停车 · 松开两杆后重新触摸"; sendLatest()
    }
    private val ticker = object : Runnable { override fun run() { if (!foreground) return; if (scanCallback != null && now() >= scanDeadline) stopScan(); if (gatt != null && !ready && now() - connectingSince > 10000) disconnect("连接或服务发现超时，请重试"); if (pending && now() - pendingSince >= 200) disconnect("控制写入超时，已断开"); sendLatest(); updateTelemetry(); handler.postDelayed(this, 50) } }
    private fun updateTelemetry() { if (::telemetryLabel.isInitialized) telemetryLabel.text = "前进 " + (pitch / 25f * 100).toInt() + "%    左移 " + (roll / 25f * 100).toInt() + "%    左旋 " + (yaw / 25f * 100).toInt() + "%" }
    override fun onResume() { super.onResume(); foreground = true; held = false; handler.post(ticker) }
    override fun onPause() { foreground = false; held = false; sendLatest(); disconnect("应用暂停，已停车并断开"); handler.removeCallbacks(ticker); super.onPause() }
    private fun permissions(): Boolean { val required = if (Build.VERSION.SDK_INT >= 31) arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT) else arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION); val missing = required.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }; if (missing.isNotEmpty()) { requestPermissions(missing.toTypedArray(), 10); connectionLabel.text = "请先授予蓝牙权限，再点击扫描"; return false }; return true }
    private fun scan() { if (!permissions()) return; disconnect("扫描中…"); val a = adapter(); if (a == null) { connectionLabel.text = "设备不支持蓝牙"; return }; if (!a.isEnabled) { startActivity(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)); return }; devices.removeAllViews(); found.clear(); val callback = object : ScanCallback() { override fun onScanResult(type: Int, result: ScanResult) { handler.post { if (scanCallback !== this || !foreground) return@post; val group = result.scanRecord?.getManufacturerSpecificData(0xffff); if (group == null || group.size != 2 || ((group[0].toInt() and 255) or ((group[1].toInt() and 255) shl 8)) != Protocol.GROUP_ID) return@post; if (found.add(result.device.address)) devices.addView(button("连接 " + (result.device.name ?: "GestureCar") + " · " + result.device.address) { connect(result.device) }, LinearLayout.LayoutParams(-1, dp(48)).apply { topMargin = dp(6) }) } }; override fun onScanFailed(code: Int) { handler.post { if (scanCallback === this) { stopScan(); connectionLabel.text = "扫描失败：" + code } } } }; scanner = a.bluetoothLeScanner; scanCallback = callback; scanDeadline = now() + 10000; try { scanner?.startScan(listOf(ScanFilter.Builder().setServiceUuid(ParcelUuid(Protocol.SERVICE)).build()), ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), callback) } catch (e: RuntimeException) { stopScan(); connectionLabel.text = "无法扫描：" + e.message }; connectionLabel.text = "正在扫描 GestureCar…" }
    private fun stopScan() { val callback = scanCallback; scanCallback = null; try { if (callback != null) scanner?.stopScan(callback) } catch (_: RuntimeException) {}; scanner = null }
    private fun connect(device: BluetoothDevice) { disconnect("正在连接 " + device.address); connectingSince = now(); try { gatt = device.connectGatt(this, false, callbacks, BluetoothDevice.TRANSPORT_LE) } catch (e: RuntimeException) { disconnect("连接失败：" + e.message) } }
    private fun disconnect(message: String) { stopScan(); resetControls(); ready = false; pending = false; control = null; val old = gatt; gatt = null; try { old?.disconnect(); old?.close() } catch (_: RuntimeException) {}; if (::connectionLabel.isInitialized) connectionLabel.text = message; if (::connectionBadge.isInitialized) { connectionBadge.text = "● 未连接"; connectionBadge.setTextColor(c("#FFB4A8")); connectionBadge.background = bg(c("#301D2B"), 20) }; if (::unlockLabel.isInitialized) unlockLabel.text = "连接后触摸摇杆开始控制" }
    private fun withGatt(g: BluetoothGatt, action: () -> Unit) { handler.post { if (gatt === g && foreground) action() } }
    private val callbacks = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, state: Int) = withGatt(g) { if (status != BluetoothGatt.GATT_SUCCESS || state == BluetoothProfile.STATE_DISCONNECTED) disconnect("蓝牙断开（" + status + "），请重新连接") else if (state == BluetoothProfile.STATE_CONNECTED) { connectionBadge.text = "● 已连接"; connectionBadge.setTextColor(c("#8FF0C0")); connectionBadge.background = bg(c("#163B35"), 20); connectionLabel.text = "已连接 · 正在发现服务"; connectingSince = now(); g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH); if (!g.discoverServices()) disconnect("服务发现启动失败") } }
        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) = withGatt(g) { val service = g.getService(Protocol.SERVICE); val chr = service?.getCharacteristic(Protocol.CONTROL); val stat = service?.getCharacteristic(Protocol.STATUS); val descriptor = stat?.getDescriptor(Protocol.CCCD); if (status != BluetoothGatt.GATT_SUCCESS || chr == null || stat == null || descriptor == null || (chr.properties and BluetoothGattCharacteristic.PROPERTY_WRITE) == 0) { disconnect("车端服务不兼容"); return@withGatt }; control = chr; if (!g.setCharacteristicNotification(stat, true)) { disconnect("状态通知启用失败"); return@withGatt }; descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE; if (!g.writeDescriptor(descriptor)) disconnect("订阅状态失败") }
        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) = withGatt(g) { if (status != BluetoothGatt.GATT_SUCCESS) disconnect("状态订阅失败：" + status) else { ready = true; held = false; connectionBadge.text = "● 已连接"; connectionBadge.setTextColor(c("#8FF0C0")); connectionBadge.background = bg(c("#163B35"), 20); connectionLabel.text = "已连接 · 等待车端就绪"; unlockLabel.text = "触摸摇杆开始控制" } }
        override fun onCharacteristicWrite(g: BluetoothGatt, chr: BluetoothGattCharacteristic, status: Int) = withGatt(g) { if (chr.uuid == Protocol.CONTROL) { pending = false; if (status != BluetoothGatt.GATT_SUCCESS) disconnect("控制写入失败：" + status) } }
        override fun onCharacteristicChanged(g: BluetoothGatt, chr: BluetoothGattCharacteristic) { val bytes = chr.value?.copyOf() ?: return; withGatt(g) { if (chr.uuid == Protocol.STATUS) updateCarStatus(bytes) } }
        override fun onCharacteristicChanged(g: BluetoothGatt, chr: BluetoothGattCharacteristic, value: ByteArray) { val bytes = value.copyOf(); withGatt(g) { if (chr.uuid == Protocol.STATUS) updateCarStatus(bytes) } }
    }
    private fun updateCarStatus(bytes: ByteArray) {
        carLabel.text = "车辆状态：" + Protocol.statusText(bytes)
        if (bytes.size != 12 || bytes[0].toInt() != 1) return
        connectionLabel.text = when (bytes[1].toInt() and 255) {
            2 -> "已连接 · 就绪，可触摸摇杆"
            3 -> "已连接 · 正在控制"
            4 -> "已连接 · 车辆已保护停车"
            else -> "已连接 · 松开双杆等待就绪"
        }
    }
    private fun sendLatest() { val g = gatt ?: return; val chr = control ?: return; if (!ready || pending) return; if (!foreground) held = false; val bytes = Protocol.encode(++seq, now(), pitch, roll, yaw, foreground, held, estop); chr.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT; chr.value = bytes; pending = true; pendingSince = now(); try { if (!g.writeCharacteristic(chr)) disconnect("发送失败，已断开") } catch (e: RuntimeException) { disconnect("发送异常：" + e.message) } }
}
