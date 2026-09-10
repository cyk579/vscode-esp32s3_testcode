package cn.edu.gesturecar

import org.junit.Assert.*
import org.junit.Test
import org.json.JSONObject
import java.net.URI
import java.net.URLDecoder
import java.util.Base64
import java.util.Date
import javax.crypto.Mac
import javax.crypto.spec.SecretKeySpec

class XfyunProtocolTest {
    @Test fun signatureUsesFixedTlsEndpointAndRfcDate() {
        val credentials = XfyunCredentials("12345678", "a".repeat(32), "b".repeat(32))
        val uri = URI(XfyunProtocol.signedUrl(credentials, Date(0)))
        assertEquals("wss", uri.scheme)
        assertEquals("iat-api.xfyun.cn", uri.host)
        assertEquals("/v2/iat", uri.path)
        val query = uri.rawQuery.split('&').associate {
            val parts = it.split('=', limit = 2)
            parts[0] to URLDecoder.decode(parts[1], "UTF-8")
        }
        assertEquals("Thu, 01 Jan 1970 00:00:00 GMT", query["date"])
        val mac = Mac.getInstance("HmacSHA256").apply {
            init(SecretKeySpec("b".repeat(32).toByteArray(), "HmacSHA256"))
        }
        val source = "host: iat-api.xfyun.cn\ndate: Thu, 01 Jan 1970 00:00:00 GMT\nGET /v2/iat HTTP/1.1"
        val expected = Base64.getEncoder().encodeToString(mac.doFinal(source.toByteArray()))
        val authorization = String(Base64.getDecoder().decode(query.getValue("authorization")))
        assertTrue(authorization.contains("signature=\"$expected\""))
        assertFalse(uri.toString().contains(credentials.apiSecret))
    }
    @Test fun firstMiddleAndLastAudioFrames() {
        val first = JSONObject(XfyunProtocol.audioFrame("12345678", 0, byteArrayOf(0, 1)))
        assertEquals("mandarin", first.getJSONObject("business").getString("accent"))
        assertEquals("wpgs", first.getJSONObject("business").getString("dwa"))
        assertEquals("AAE=", first.getJSONObject("data").getString("audio"))
        val middle = JSONObject(XfyunProtocol.audioFrame("12345678", 1, byteArrayOf(0, 0)))
        assertFalse(middle.has("business"))
        val last = JSONObject(XfyunProtocol.audioFrame("12345678", 2, byteArrayOf()))
        assertEquals(2, last.getJSONObject("data").getInt("status"))
        assertEquals("", last.getJSONObject("data").getString("audio"))
    }
    @Test fun replacesPartialSegmentsBeforeFinalResult() {
        val transcript = XfyunTranscript()
        assertFalse(transcript.accept(JSONObject("""{"status":1,"result":{"sn":1,"ws":[{"cw":[{"w":"前进"}]}]}}""")))
        assertFalse(transcript.accept(JSONObject("""{"status":1,"result":{"sn":2,"ws":[{"cw":[{"w":"一下"}]}]}}""")))
        assertEquals("前进一下", transcript.text)
        assertFalse(transcript.accept(JSONObject("""{"status":1,"result":{"sn":3,"pgs":"rpl","rg":[1,2],"ws":[{"cw":[{"w":"不要前进"}]}]}}""")))
        assertTrue(transcript.accept(JSONObject("""{"status":2}""")))
        assertEquals("不要前进", transcript.text)
        assertEquals(VoiceIntent.Unknown, VoiceRouter.route(transcript.text))
    }
    @Test fun acceptsConversationalCommandsButNotNegationOrAmbiguity() {
        val cases = mapOf("向前走一下" to VoiceCommand.FORWARD, "往前开" to VoiceCommand.FORWARD,
            "退回来一点" to VoiceCommand.BACKWARD, "往左挪一下" to VoiceCommand.LEFT,
            "向右挪" to VoiceCommand.RIGHT, "往右转" to VoiceCommand.TURN_RIGHT,
            "停下来" to VoiceCommand.STOP)
        for ((text, expected) in cases) assertEquals(expected, VoiceCommand.parse(text))
        for (text in listOf("不要向前走一下", "别往前开", "能不能前进", "往左", "前进十秒", "向前走一米")) {
            assertEquals(text, VoiceIntent.Unknown, VoiceRouter.route(text))
        }
        assertEquals(VoiceIntent.Sequence(listOf(VoiceCommand.FORWARD, VoiceCommand.LEFT, VoiceCommand.TURN_RIGHT)),
            VoiceRouter.route("先向前走一下，再往左挪一下，最后右转"))
    }
}
