@file:Suppress("DEPRECATION")
package cn.edu.gesturecar

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.ImageFormat
import android.graphics.Matrix
import android.hardware.camera2.*
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import android.widget.ImageView
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizer
import java.util.concurrent.atomic.AtomicBoolean

class MediaPipeHandFeature(private val host: Activity, private val preview: ImageView) : GestureFeature {
    private val running = AtomicBoolean(false)
    private val previewPending = AtomicBoolean(false)
    private val main = Handler(Looper.getMainLooper())
    private var worker: Handler? = null
    private var reader: ImageReader? = null
    private var camera: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var recognizer: GestureRecognizer? = null
    private var output: GestureOutput? = null
    private var lastFrame = 0L
    private var fist = false
    private val tracker = HandTracker()
    @Volatile private var lastResult = 0L

    override fun start(output: GestureOutput) {
        if (!running.compareAndSet(false, true)) return
        this.output = output
        if (host.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            fail("请先授予摄像头权限", null)
            return
        }
        output.status("正在加载手势模型…")
        val thread = HandlerThread("gesture-camera").apply { start() }
        worker = Handler(thread.looper)
        val displayDegrees = when (host.windowManager.defaultDisplay.rotation) {
            Surface.ROTATION_90 -> 90
            Surface.ROTATION_180 -> 180
            Surface.ROTATION_270 -> 270
            else -> 0
        }
        lastResult = SystemClock.elapsedRealtime()
        main.post(watchdog)
        worker!!.post { guarded("手势初始化失败") { openCamera(displayDegrees) } }
    }

    private val watchdog = object : Runnable {
        override fun run() {
            if (!running.get()) return
            if (SystemClock.elapsedRealtime() - lastResult > 300) output?.lost()
            main.postDelayed(this, 100)
        }
    }

    @SuppressLint("MissingPermission")
    private fun openCamera(displayDegrees: Int) {
        if (!running.get()) return
        recognizer = GestureRecognizer.createFromOptions(host, GestureRecognizer.GestureRecognizerOptions.builder()
            .setBaseOptions(BaseOptions.builder().setModelAssetPath("gesture_recognizer.task").build())
            .setRunningMode(RunningMode.VIDEO).build())
        if (!running.get()) return
        val manager = host.getSystemService(CameraManager::class.java)
        val id = manager.cameraIdList.firstOrNull {
            manager.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_FRONT
        } ?: manager.cameraIdList.firstOrNull() ?: error("没有可用摄像头")
        val info = manager.getCameraCharacteristics(id)
        val front = info.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_FRONT
        val sensor = info.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 0
        val rotation = if (front) (sensor + displayDegrees) % 360 else (sensor - displayDegrees + 360) % 360
        val sizes = requireNotNull(info.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)).getOutputSizes(ImageFormat.YUV_420_888)
        val size = sizes.filter { it.width <= 640 && it.height <= 480 }.maxByOrNull { it.width * it.height }
            ?: sizes.minByOrNull { it.width * it.height } ?: error("摄像头不支持 YUV 图像")
        reader = ImageReader.newInstance(size.width, size.height, ImageFormat.YUV_420_888, 2).also { source ->
            source.setOnImageAvailableListener({ frames -> guarded("手势图像处理失败") { readFrame(frames, rotation, front) } }, worker)
        }
        manager.openCamera(id, object : CameraDevice.StateCallback() {
            override fun onOpened(device: CameraDevice) {
                if (!running.get()) { device.close(); return }
                camera = device
                guarded("摄像头会话失败") {
                    val target = requireNotNull(reader).surface
                    device.createCaptureSession(listOf(target), object : CameraCaptureSession.StateCallback() {
                        override fun onConfigured(value: CameraCaptureSession) {
                            if (!running.get()) { value.close(); return }
                            session = value
                            guarded("摄像头采集失败") {
                                value.setRepeatingRequest(device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                                    addTarget(target)
                                }.build(), null, worker)
                                output?.status("摄像头已启动；请将张开的手掌放入画面")
                                Log.i("GestureCamera", "Camera ready: $id ${size.width}x${size.height}, rotation=$rotation")
                            }
                        }
                        override fun onConfigureFailed(value: CameraCaptureSession) { value.close(); fail("摄像头配置失败", null) }
                    }, worker)
                }
            }
            override fun onDisconnected(device: CameraDevice) { device.close(); fail("摄像头已断开", null) }
            override fun onError(device: CameraDevice, error: Int) { device.close(); fail("摄像头错误 $error", null) }
        }, worker)
    }

    private fun readFrame(source: ImageReader, rotation: Int, front: Boolean) {
        if (!running.get()) return
        source.acquireLatestImage()?.use { frame ->
            val stamp = SystemClock.elapsedRealtime()
            if (stamp - lastFrame < 80) return
            lastFrame = stamp
            val planes = frame.planes
            val pixels = YuvRgb.convert(frame.width, frame.height,
                planes[0].buffer, planes[0].rowStride, planes[0].pixelStride,
                planes[1].buffer, planes[1].rowStride, planes[1].pixelStride,
                planes[2].buffer, planes[2].rowStride, planes[2].pixelStride)
            val raw = Bitmap.createBitmap(pixels, frame.width, frame.height, Bitmap.Config.ARGB_8888)
            val oriented = Bitmap.createBitmap(raw, 0, 0, raw.width, raw.height, Matrix().apply {
                postRotate(rotation.toFloat())
                if (front) postScale(-1f, 1f)
            }, true)
            val input = BitmapImageBuilder(oriented).build()
            try {
                val result = recognizer!!.recognizeForVideo(input, stamp)
                if (!running.get()) return
                lastResult = SystemClock.elapsedRealtime()
                val category = result.gestures().firstOrNull()?.firstOrNull()
                val landmarks = result.landmarks().firstOrNull()
                val points = landmarks?.map { it.x() to it.y() } ?: emptyList()
                val axes = tracker.update(category?.categoryName(), category?.score() ?: 0f, points, stamp, lastResult)
                if (previewPending.compareAndSet(false, true)) {
                    val display = oriented.copy(Bitmap.Config.ARGB_8888, false)
                    val origin = tracker.neutral
                    main.post {
                        previewPending.set(false)
                        if (!running.get()) display.recycle()
                        else if (preview is GesturePreviewView) preview.showFrame(display, points, origin)
                        else preview.setImageBitmap(display)
                    }
                }
                if (axes == null) {
                    fist = false; output?.lost()
                    output?.status(tracker.status)
                    return
                }
                output?.motion(axes.forward, axes.lateral, axes.yaw, stamp)
                if (axes.fist && !fist) output?.fist()
                fist = axes.fist
                output?.status(if (fist) "识别到握拳：旋转请求" else
                    "掌心已定中：前后 ${(axes.forward * 4).toInt()}% / 左右 ${(axes.lateral * 4).toInt()}%（正值=前/左）")
            } finally {
                input.close()
                if (!oriented.isRecycled) oriented.recycle()
                if (!raw.isRecycled) raw.recycle()
            }
        }
    }

    private fun guarded(message: String, action: () -> Unit) {
        try { if (running.get()) action() }
        catch (failure: Exception) { fail(message, failure) }
        catch (failure: LinkageError) { fail("手势原生库加载/运行失败", failure) }
    }

    private fun fail(message: String, failure: Throwable?) {
        if (!running.get()) return
        Log.e("GestureCamera", message, failure)
        output?.error(if (failure == null) message else "$message：${failure.message ?: failure.javaClass.simpleName}")
        stop()
    }

    override fun stop() {
        if (!running.getAndSet(false)) return
        main.removeCallbacks(watchdog)
        val handler = worker ?: return
        handler.post {
            for (close in listOf<() -> Unit>({ session?.close() }, { camera?.close() }, { reader?.close() }, { recognizer?.close() })) {
                try { close() } catch (failure: Exception) { Log.w("GestureCamera", "释放资源失败", failure) }
            }
            session = null; camera = null; reader = null; recognizer = null; output = null
            handler.looper.quitSafely()
        }
    }
}
