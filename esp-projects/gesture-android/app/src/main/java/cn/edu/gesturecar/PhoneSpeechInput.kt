package cn.edu.gesturecar

import android.app.Activity
import android.content.Intent
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer

/** Owns one foreground recognition request. No audio is sent through BLE. */
class PhoneSpeechInput(
    private val activity: Activity,
    private val status: (String) -> Unit,
    private val result: (String) -> Unit
) {
    private val handler = Handler(Looper.getMainLooper())
    private var recognizer: SpeechRecognizer? = null
    private var generation = 0L
    private var timeout: Runnable? = null

    fun cancel() {
        ++generation // Invalidate callbacks before cancel()/destroy() can deliver them.
        timeout?.let { handler.removeCallbacks(it) }; timeout = null
        val old = recognizer; recognizer = null
        try { old?.cancel() } catch (_: RuntimeException) { }
        try { old?.destroy() } catch (_: RuntimeException) { }
    }

    fun start(forceSystemService: Boolean = false) {
        cancel()
        val session = generation
        try {
            val onDevice = !forceSystemService && Build.VERSION.SDK_INT >= 31 &&
                SpeechRecognizer.isOnDeviceRecognitionAvailable(activity)
            if (!onDevice && !SpeechRecognizer.isRecognitionAvailable(activity)) {
                status("手机未提供系统语音识别服务；可继续使用摇杆")
                return
            }
            val engine = if (onDevice) SpeechRecognizer.createOnDeviceSpeechRecognizer(activity)
                         else SpeechRecognizer.createSpeechRecognizer(activity)
            recognizer = engine
            engine.setRecognitionListener(object : RecognitionListener {
                private fun current() = session == generation && recognizer === engine
                override fun onReadyForSpeech(params: Bundle?) {
                    if (current()) status(if (onDevice) "请说指令 · 手机本地识别" else "请说指令 · 系统语音服务可能联网")
                }
                override fun onResults(results: Bundle?) {
                    if (!current()) return
                    val text = results?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)?.firstOrNull()
                    cancel()
                    if (text.isNullOrBlank()) status("没有识别到指令，保持停车") else result(text)
                }
                override fun onError(error: Int) {
                    if (!current()) return
                    cancel()
                    val reason = when (error) {
                        SpeechRecognizer.ERROR_NO_MATCH, SpeechRecognizer.ERROR_SPEECH_TIMEOUT -> "未听清，请重试"
                        SpeechRecognizer.ERROR_NETWORK, SpeechRecognizer.ERROR_NETWORK_TIMEOUT -> "语音服务网络不可用"
                        SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS -> "缺少麦克风权限"
                        SpeechRecognizer.ERROR_RECOGNIZER_BUSY -> "语音服务忙，请重试"
                        SpeechRecognizer.ERROR_LANGUAGE_NOT_SUPPORTED,
                        SpeechRecognizer.ERROR_LANGUAGE_UNAVAILABLE ->
                            if (onDevice) "本地中文模型不可用，可勾选系统语音服务后重试" else "系统语音服务未提供中文模型"
                        else -> "语音服务错误 $error"
                    }
                    status("$reason；保持停车")
                }
                override fun onBeginningOfSpeech() { }
                override fun onRmsChanged(rmsdB: Float) { }
                override fun onBufferReceived(buffer: ByteArray?) { }
                override fun onEndOfSpeech() { }
                override fun onPartialResults(partialResults: Bundle?) { }
                override fun onEvent(eventType: Int, params: Bundle?) { }
            })
            timeout = Runnable {
                if (session == generation) { cancel(); status("识别超时，保持停车") }
            }.also { handler.postDelayed(it, 8000) }
            status(if (onDevice) "正在启动手机本地识别…" else "正在启动系统语音服务（可能联网）…")
            engine.startListening(Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH).apply {
                putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                putExtra(RecognizerIntent.EXTRA_LANGUAGE, "zh-CN")
                putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 1)
                putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, false)
            })
        } catch (e: RuntimeException) {
            cancel()
            status("无法启动语音服务，保持停车：${e.message ?: "服务不可用"}")
        }
    }
}
