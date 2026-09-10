package cn.edu.gesturecar

import android.annotation.SuppressLint
import android.content.Context
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject

class PhoneSpeechInput(
    context: Context,
    private val status: (String) -> Unit,
    private val result: (String) -> Unit
) {
    private class Session {
        @Volatile var active = true
        @Volatile var finishInput = false
        var socket: WebSocket? = null
        val transcript = XfyunTranscript()
    }
    private val settings = XfyunSettings(context)
    private val handler = Handler(Looper.getMainLooper())
    private val worker = Executors.newSingleThreadExecutor()
    private val client = OkHttpClient.Builder().connectTimeout(8, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.MILLISECONDS).build()
    private var session: Session? = null
    private var timeout: Runnable? = null

    fun cancel() {
        val previous = session
        session = null
        previous?.active = false
        previous?.socket?.cancel()
        timeout?.let { handler.removeCallbacks(it) }
        timeout = null
    }
    fun close() {
        cancel()
        worker.shutdown()
        client.dispatcher.executorService.shutdown()
        client.connectionPool.evictAll()
    }
    fun finishInput() {
        session?.let {
            it.finishInput = true
            status("正在等待讯飞最终结果；保持停车")
        }
    }
    private fun current(request: Session) = session === request && request.active
    private fun fail(request: Session, message: String) {
        handler.post {
            if (current(request)) { cancel(); status("$message；保持停车") }
        }
    }
    fun start() {
        cancel()
        val credentials = settings.load()
        if (credentials == null) {
            status("请先填写“讯飞配置”；保持停车")
            return
        }
        val request = Session()
        session = request
        timeout = Runnable {
            if (current(request)) { cancel(); status("讯飞识别超时，请检查网络；保持停车") }
        }.also { handler.postDelayed(it, 20000) }
        status("正在连接科大讯飞；连接后再说话")
        try {
            request.socket = client.newWebSocket(Request.Builder().url(XfyunProtocol.signedUrl(credentials)).build(),
                object : WebSocketListener() {
                    override fun onOpen(webSocket: WebSocket, response: Response) {
                        handler.post {
                            if (current(request)) worker.execute { record(request, webSocket, credentials.appId) }
                            else webSocket.cancel()
                        }
                    }
                    override fun onMessage(webSocket: WebSocket, text: String) {
                        handler.post {
                            if (!current(request)) return@post
                            try {
                                val packet = JSONObject(text)
                                val code = packet.getInt("code")
                                if (code != 0) {
                                    cancel()
                                    status("讯飞错误 $code；请检查听写服务授权、额度与配置；保持停车")
                                    return@post
                                }
                                val complete = request.transcript.accept(packet.getJSONObject("data"))
                                val recognized = request.transcript.text
                                if (complete) {
                                    request.socket = null
                                    webSocket.close(1000, null)
                                    cancel()
                                    if (recognized.isBlank()) status("没有听清，请重试；保持停车") else result(recognized)
                                } else status("讯飞听到：$recognized（尚未执行）")
                            } catch (_: Exception) { fail(request, "讯飞返回数据无效") }
                        }
                    }
                    override fun onFailure(webSocket: WebSocket, error: Throwable, response: Response?) {
                        val reason = if (response?.code in listOf(401, 403))
                            "讯飞鉴权失败，请检查密钥、手机自动时间和服务 IP 白名单"
                        else "讯飞连接失败${response?.code?.let { "（HTTP $it）" } ?: ""}，请检查网络"
                        fail(request, reason)
                    }
                    override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                        webSocket.close(1000, null)
                        fail(request, "讯飞会话提前结束，未执行指令")
                    }
                })
        } catch (_: Exception) { fail(request, "无法启动讯飞识别") }
    }
    @SuppressLint("MissingPermission")
    private fun record(request: Session, socket: WebSocket, appId: String) {
        var recorder: AudioRecord? = null
        try {
            if (!request.active) return
            val minimum = AudioRecord.getMinBufferSize(16000, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
            check(minimum > 0)
            val audio = AudioRecord(MediaRecorder.AudioSource.VOICE_RECOGNITION, 16000,
                AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, maxOf(minimum, 1280 * 4))
            recorder = audio
            check(audio.state == AudioRecord.STATE_INITIALIZED)
            audio.startRecording()
            check(audio.recordingState == AudioRecord.RECORDSTATE_RECORDING)
            handler.post { if (current(request)) status("请说中文指令 · 科大讯飞在线识别（最长 12 秒）") }
            val buffer = ByteArray(1280)
            var first = true
            val started = SystemClock.elapsedRealtime()
            while (request.active && (first || (!request.finishInput && SystemClock.elapsedRealtime() - started < 12000))) {
                val count = audio.read(buffer, 0, buffer.size, AudioRecord.READ_BLOCKING)
                check(count > 0 && count % 2 == 0)
                if (!request.active) return
                check(socket.queueSize() < 64000)
                check(socket.send(XfyunProtocol.audioFrame(appId, if (first) 0 else 1, buffer.copyOf(count))))
                first = false
            }
            if (request.active) {
                check(socket.send(XfyunProtocol.audioFrame(appId, 2, byteArrayOf())))
                handler.post { if (current(request)) status("正在等待讯飞最终结果；保持停车") }
            }
        } catch (_: SecurityException) { fail(request, "缺少麦克风权限") }
        catch (_: Exception) { fail(request, "录音或发送失败，请检查麦克风占用和网络") }
        finally {
            try { recorder?.stop() } catch (_: RuntimeException) { }
            recorder?.release()
        }
    }
}
