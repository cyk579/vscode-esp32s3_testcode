package cn.edu.gesturecar

import java.net.URLEncoder
import java.text.SimpleDateFormat
import java.util.Base64
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import javax.crypto.Mac
import javax.crypto.spec.SecretKeySpec
import org.json.JSONObject

class XfyunCredentials(val appId: String, val apiKey: String, val apiSecret: String) {
    fun valid() = appId.matches(Regex("[A-Za-z0-9]{8}")) &&
        apiKey.matches(Regex("[A-Za-z0-9]{32}")) && apiSecret.matches(Regex("[A-Za-z0-9]{32}"))
}

object XfyunProtocol {
    private const val HOST = "iat-api.xfyun.cn"
    fun signedUrl(credentials: XfyunCredentials, now: Date = Date()): String {
        require(credentials.valid())
        val date = SimpleDateFormat("EEE, dd MMM yyyy HH:mm:ss 'GMT'", Locale.US).apply {
            timeZone = TimeZone.getTimeZone("GMT")
        }.format(now)
        val source = "host: $HOST\ndate: $date\nGET /v2/iat HTTP/1.1"
        val mac = Mac.getInstance("HmacSHA256").apply {
            init(SecretKeySpec(credentials.apiSecret.toByteArray(), "HmacSHA256"))
        }
        val signature = Base64.getEncoder().encodeToString(mac.doFinal(source.toByteArray()))
        val origin = "api_key=\"${credentials.apiKey}\", algorithm=\"hmac-sha256\", headers=\"host date request-line\", signature=\"$signature\""
        val authorization = Base64.getEncoder().encodeToString(origin.toByteArray())
        return "wss://$HOST/v2/iat?authorization=${URLEncoder.encode(authorization, "UTF-8")}" +
            "&date=${URLEncoder.encode(date, "UTF-8")}&host=$HOST"
    }
    fun audioFrame(appId: String, status: Int, audio: ByteArray): String {
        require(status in 0..2 && audio.size % 2 == 0)
        val packet = JSONObject()
        if (status == 0) {
            packet.put("common", JSONObject().put("app_id", appId))
            packet.put("business", JSONObject().put("language", "zh_cn").put("domain", "iat")
                .put("accent", "mandarin").put("dwa", "wpgs"))
        }
        packet.put("data", JSONObject().put("status", status).put("format", "audio/L16;rate=16000")
            .put("encoding", "raw").put("audio", Base64.getEncoder().encodeToString(audio)))
        return packet.toString()
    }
}

class XfyunTranscript {
    private val segments = sortedMapOf<Int, String>()
    var text: String = ""
        private set
    fun accept(data: JSONObject): Boolean {
        val result = data.optJSONObject("result")
        if (result != null) {
            if (result.optString("pgs") == "rpl") {
                val range = result.getJSONArray("rg")
                segments.keys.filter { it in range.getInt(0)..range.getInt(1) }.forEach { segments.remove(it) }
            }
            val words = result.getJSONArray("ws")
            segments[result.getInt("sn")] = buildString {
                for (index in 0 until words.length()) {
                    val candidates = words.getJSONObject(index).getJSONArray("cw")
                    if (candidates.length() > 0) append(candidates.getJSONObject(0).getString("w"))
                }
            }
            text = segments.values.joinToString("")
            require(text.length <= 1000)
        }
        return data.getInt("status") == 2
    }
}
