@file:Suppress("DEPRECATION")
package cn.edu.gesturecar

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import android.os.SystemClock
import java.util.ArrayDeque

/** One BLE connection, one in-flight GATT operation. Drive heartbeats take priority. */
@SuppressLint("MissingPermission")
class BleCarTransport(private val context: Context, private val frame: (Long) -> ByteArray,
                      private val listener: Listener) {
    interface Listener {
        fun connection(message: String, ready: Boolean, music: Boolean)
        fun device(device: BluetoothDevice)
        fun carStatus(bytes: ByteArray)
        fun musicStatus(bytes: ByteArray)
    }
    private val handler = Handler(Looper.getMainLooper())
    private var gatt: BluetoothGatt? = null
    private var drive: BluetoothGattCharacteristic? = null
    private var music: BluetoothGattCharacteristic? = null
    private var ready = false
    private var pending = false
    private var pendingSince = 0L
    private var connectingSince = 0L
    private var lastDrive = Long.MIN_VALUE
    private var scanner: BluetoothLeScanner? = null
    private var scanCallback: ScanCallback? = null
    private var scanDeadline = 0L
    private val found = mutableSetOf<String>()
    private val subscriptions = ArrayDeque<BluetoothGattDescriptor>()
    private val media = ArrayDeque<Pair<Long, ByteArray>>()
    private fun now() = SystemClock.elapsedRealtime()
    private fun adapter() = context.getSystemService(BluetoothManager::class.java)?.adapter

    fun scan(): Boolean {
        disconnect("扫描中")
        val a = adapter()
        if (a == null || !a.isEnabled) { listener.connection("请打开手机蓝牙", false, false); return false }
        found.clear()
        val callback = object : ScanCallback() {
            override fun onScanResult(type: Int, result: ScanResult) { handler.post {
                if (scanCallback !== this) return@post
                val group = result.scanRecord?.getManufacturerSpecificData(0xffff) ?: return@post
                if (group.size != 2 || ((group[0].toInt() and 255) or ((group[1].toInt() and 255) shl 8)) != Protocol.GROUP_ID) return@post
                if (found.add(result.device.address)) listener.device(result.device)
            } }
            override fun onScanFailed(code: Int) { handler.post { if (scanCallback === this) disconnect("扫描失败：$code") } }
        }
        scanner = a.bluetoothLeScanner; scanCallback = callback; scanDeadline = now() + 10000
        try {
            scanner?.startScan(listOf(ScanFilter.Builder().setServiceUuid(ParcelUuid(Protocol.SERVICE)).build()),
                ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), callback)
        } catch (e: RuntimeException) { disconnect("无法扫描：${e.message}"); return false }
        return true
    }
    fun stopScan() {
        val old = scanCallback; scanCallback = null
        try { if (old != null) scanner?.stopScan(old) } catch (_: RuntimeException) { }
        scanner = null
    }
    fun connect(device: BluetoothDevice) {
        disconnect("正在连接 ${device.address}"); connectingSince = now()
        try { gatt = device.connectGatt(context, false, callbacks, BluetoothDevice.TRANSPORT_LE) }
        catch (e: RuntimeException) { disconnect("连接失败：${e.message}") }
    }
    fun disconnect(message: String) {
        stopScan(); val old = gatt; gatt = null
        ready = false; pending = false; drive = null; music = null
        subscriptions.clear(); media.clear(); lastDrive = Long.MIN_VALUE
        try { old?.disconnect(); old?.close() } catch (_: RuntimeException) { }
        listener.connection(message, false, false)
    }
    fun enqueueMusic(bytes: ByteArray): Boolean {
        if (!ready || music == null || media.size >= 4) return false
        media.add(now() to bytes.copyOf()); pump(); return true
    }
    fun tick() {
        if (scanCallback != null && now() >= scanDeadline) stopScan()
        if (gatt != null && !ready && now() - connectingSince >= 10000) disconnect("连接或服务发现超时")
        if (pending && now() - pendingSince >= 200) disconnect("GATT 操作超时，已断开并停车")
        pump()
    }
    fun requestDrive() { lastDrive = Long.MIN_VALUE; pump() }
    private fun pump() {
        val g = gatt ?: return
        if (!ready || pending) return
        val current = now()
        while (media.isNotEmpty() && current - media.first.first > 2000) media.removeFirst()
        val chr: BluetoothGattCharacteristic
        val bytes: ByteArray
        if (lastDrive == Long.MIN_VALUE || current - lastDrive >= 50) {
            chr = drive ?: return; bytes = frame(current); lastDrive = current
        } else {
            chr = music ?: return
            if (media.isEmpty()) return
            bytes = media.removeFirst().second
        }
        chr.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT; chr.value = bytes
        pending = true; pendingSince = current
        try { if (!g.writeCharacteristic(chr)) disconnect("控制发送失败") }
        catch (e: RuntimeException) { disconnect("发送异常：${e.message}") }
    }
    private fun nextSubscription(g: BluetoothGatt) {
        if (subscriptions.isEmpty()) {
            pending = false; ready = true; listener.connection("已连接，等待车端就绪", true, music != null); pump(); return
        }
        val d = subscriptions.removeFirst(); d.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        pending = true; pendingSince = now()
        if (!g.writeDescriptor(d)) disconnect("状态订阅启动失败")
    }
    private fun withGatt(g: BluetoothGatt, action: () -> Unit) { handler.post { if (gatt === g) action() } }
    private fun notify(chr: BluetoothGattCharacteristic, bytes: ByteArray) {
        if (chr.uuid == Protocol.STATUS) listener.carStatus(bytes)
        else if (chr.uuid == MusicProtocol.STATUS) listener.musicStatus(bytes)
    }
    private val callbacks = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, state: Int) = withGatt(g) {
            if (status != BluetoothGatt.GATT_SUCCESS || state == BluetoothProfile.STATE_DISCONNECTED) disconnect("蓝牙断开：$status")
            else if (state == BluetoothProfile.STATE_CONNECTED) {
                connectingSince = now(); g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                // Android may retain the pre-music service table after a firmware update.
                try { g.javaClass.getMethod("refresh").invoke(g) } catch (_: Exception) { }
                if (!g.discoverServices()) disconnect("服务发现失败")
            }
        }
        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) = withGatt(g) {
            val service = g.getService(Protocol.SERVICE)
            drive = service?.getCharacteristic(Protocol.CONTROL)
            val driveStatus = service?.getCharacteristic(Protocol.STATUS)
            val mediaService = g.getService(MusicProtocol.SERVICE)
            val mediaWrite = mediaService?.getCharacteristic(MusicProtocol.COMMAND)
            val mediaStatus = mediaService?.getCharacteristic(MusicProtocol.STATUS)
            if (status != BluetoothGatt.GATT_SUCCESS || drive == null || driveStatus == null ||
                (drive!!.properties and BluetoothGattCharacteristic.PROPERTY_WRITE) == 0) { disconnect("车端运动服务不兼容"); return@withGatt }
            music = if (mediaWrite != null && mediaStatus != null &&
                (mediaWrite.properties and BluetoothGattCharacteristic.PROPERTY_WRITE) != 0) mediaWrite else null
            for (chr in listOfNotNull(driveStatus, if (music != null) mediaStatus else null)) {
                val descriptor = chr.getDescriptor(Protocol.CCCD)
                if (descriptor == null || !g.setCharacteristicNotification(chr, true)) { disconnect("状态通知不可用"); return@withGatt }
                subscriptions.add(descriptor)
            }
            nextSubscription(g)
        }
        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) = withGatt(g) {
            if (status != BluetoothGatt.GATT_SUCCESS) disconnect("状态订阅失败：$status") else nextSubscription(g)
        }
        override fun onCharacteristicWrite(g: BluetoothGatt, chr: BluetoothGattCharacteristic, status: Int) = withGatt(g) {
            pending = false
            if (status != BluetoothGatt.GATT_SUCCESS) {
                // A rejected music command must not be reported as a successful playback.
                if (chr.uuid == MusicProtocol.COMMAND) { media.clear(); listener.musicStatus(byteArrayOf()) }
                else { disconnect("运动写入失败：$status"); return@withGatt }
            }
            pump()
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, chr: BluetoothGattCharacteristic) {
            val bytes = chr.value?.copyOf() ?: return; withGatt(g) { notify(chr, bytes) }
        }
        override fun onCharacteristicChanged(g: BluetoothGatt, chr: BluetoothGattCharacteristic, value: ByteArray) {
            val bytes = value.copyOf(); withGatt(g) { notify(chr, bytes) }
        }
    }
}
