package cn.edu.gesturecar

import androidx.test.platform.app.InstrumentationRegistry
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Assume.assumeTrue
import org.junit.Test

class XfyunDeviceTest {
    @Test fun provisionAndVerifyOnlineWithoutMotion() {
        assumeTrue(InstrumentationRegistry.getArguments().getString("xfyunProvision") == "true")
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val pending = File(context.filesDir, "xfyun-provision.json")
        val settings = XfyunSettings(context)
        if (pending.exists()) {
            val json = try { JSONObject(pending.readText()) } finally { check(pending.delete()) }
            val credentials = XfyunCredentials(json.getString("appId"), json.getString("apiKey"), json.getString("apiSecret"))
            assertTrue("Credential format invalid", credentials.valid())
            settings.save(credentials)
        }
        val credentials = settings.load()
        assertNotNull("Encrypted settings unavailable", credentials)
        val active = credentials!!
        val encrypted = context.getSharedPreferences("xfyun_private", 0).getString("encrypted", "")!!
        assertFalse(encrypted.contains(active.apiKey))
        assertFalse(encrypted.contains(active.apiSecret))
        val outcome = AtomicReference("No final response")
        val opened = CountDownLatch(1)
        val completed = CountDownLatch(1)
        val client = OkHttpClient.Builder().connectTimeout(10, TimeUnit.SECONDS).build()
        val socket = client.newWebSocket(Request.Builder().url(XfyunProtocol.signedUrl(active)).build(),
            object : WebSocketListener() {
                override fun onOpen(webSocket: WebSocket, response: Response) { opened.countDown() }
                override fun onMessage(webSocket: WebSocket, text: String) {
                    try {
                        val packet = JSONObject(text)
                        val code = packet.getInt("code")
                        if (code != 0) {
                            outcome.set("Service code $code")
                            completed.countDown()
                        } else if (packet.getJSONObject("data").getInt("status") == 2) {
                            outcome.set("OK")
                            completed.countDown()
                        }
                    } catch (_: Exception) {
                        outcome.set("Invalid response")
                        completed.countDown()
                    }
                }
                override fun onFailure(webSocket: WebSocket, error: Throwable, response: Response?) {
                    outcome.set("Connection failed HTTP ${response?.code ?: 0}")
                    opened.countDown()
                    completed.countDown()
                }
            })
        try {
            assertTrue("Connection timeout", opened.await(12, TimeUnit.SECONDS))
            for (index in 0 until 25) {
                if (completed.count == 0L) break
                assertTrue(socket.send(XfyunProtocol.audioFrame(active.appId, if (index == 0) 0 else 1, ByteArray(1280))))
                Thread.sleep(40)
            }
            if (completed.count != 0L) assertTrue(socket.send(XfyunProtocol.audioFrame(active.appId, 2, byteArrayOf())))
            assertTrue("Final response timeout", completed.await(12, TimeUnit.SECONDS))
            assertEquals("OK", outcome.get())
        } finally {
            socket.cancel()
            client.dispatcher.executorService.shutdown()
            client.connectionPool.evictAll()
        }
    }
}
