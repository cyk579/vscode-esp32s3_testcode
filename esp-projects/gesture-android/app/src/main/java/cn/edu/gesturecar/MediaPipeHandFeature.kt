package cn.edu.gesturecar

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.hardware.camera2.*
import com.google.mediapipe.framework.image.MediaImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizer
import java.util.concurrent.atomic.AtomicBoolean

class MediaPipeHandFeature(private val host: Activity) : GestureFeature {
    private val running = AtomicBoolean(false)
    private var output: GestureOutput? = null
    private var reader: ImageReader? = null
    private var camera: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var recognizer: GestureRecognizer? = null
    private var lastSeen = 0L
    private var fist = false

    override fun start(output: GestureOutput) {
        if (!running.compareAndSet(false, true)) return
        this.output = output
        if (host.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            running.set(false); output.lost(); host.requestPermissions(arrayOf(Manifest.permission.CAMERA), 12); return
        }
        thread = HandlerThread("gesture-camera").also { it.start() }; handler = Handler(thread!!.looper)
        val base = BaseOptions.builder().setModelAssetPath("gesture_recognizer.task").build()
        recognizer = GestureRecognizer.createFromOptions(host, GestureRecognizer.GestureRecognizerOptions.builder()
            .setBaseOptions(base).setRunningMode(RunningMode.LIVE_STREAM).setResultListener { result, _ ->
                val stamp = SystemClock.elapsedRealtime(); val category = result.gestures().firstOrNull()?.firstOrNull()?.categoryName()
                if (category == null) { if (stamp - lastSeen > 300) output.lost(); return@setResultListener }
                lastSeen = stamp
                val hand = result.landmarks().firstOrNull()?.firstOrNull()
                val x = hand?.x() ?: .5f; val y = hand?.y() ?: .5f
                val nowFist = category == "Closed_Fist"
                if (nowFist) { output.motion(0f, 0f, -25f, stamp); if (!fist) output.fist() }
                else output.motion(-(y-.5f)*25f, -(x-.5f)*25f, 0f, stamp)
                fist = nowFist
            }.setErrorListener { output.lost() }.build())
        val manager = host.getSystemService(CameraManager::class.java)
        val id = manager.cameraIdList.firstOrNull { manager.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_FRONT } ?: manager.cameraIdList.first()
        reader = ImageReader.newInstance(640, 480, ImageFormat.YUV_420_888, 2).also { source ->
            source.setOnImageAvailableListener({ source.acquireLatestImage()?.let { image ->
                if (running.get()) recognizer?.recognizeAsync(MediaImageBuilder(image).build(), SystemClock.elapsedRealtime())
                image.close()
            } }, handler)
        }
        manager.openCamera(id, object : CameraDevice.StateCallback() {
            override fun onOpened(device: CameraDevice) { camera=device; device.createCaptureSession(listOf(reader!!.surface), object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(value: CameraCaptureSession) { session=value; value.setRepeatingRequest(device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply { addTarget(reader!!.surface) }.build(), null, handler) }
                override fun onConfigureFailed(value: CameraCaptureSession) { output.lost() }
            }, handler) }
            override fun onDisconnected(device: CameraDevice) { device.close(); output.lost() }
            override fun onError(device: CameraDevice, error: Int) { device.close(); output.lost() }
        }, handler)
    }

    override fun stop() {
        if (!running.getAndSet(false)) return
        try { session?.stopRepeating() } catch (_: Exception) {}
        session?.close(); camera?.close(); reader?.close(); recognizer?.close(); thread?.quitSafely()
        session=null; camera=null; reader=null; recognizer=null; thread=null; handler=null; output=null; fist=false
    }
}
