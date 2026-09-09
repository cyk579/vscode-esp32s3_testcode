package cn.edu.gesturecar
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.*
class ProtocolTest {
    @Test fun goldenPacket() {
        val bytes = Protocol.encode(65535, 0x12345678, -12.34f, 25f, 7.77f, true, true, false)
        assertArrayEquals(byteArrayOf(1,3,-1,-1,0x2e,-5,-60,9,0x09,0x03,0x78,0x56,0x34,0x12),bytes)
    }
    @Test fun invalidSensorNeverEncodesValid() {
        assertEquals(0, Protocol.encode(0,0,Float.NaN,0f,0f,true,false,false)[1].toInt())
    }
    @Test fun forwardAndLeftSigns() {
        val identity = floatArrayOf(1f,0f,0f,0f,1f,0f,0f,0f,1f)
        val c = cos(Math.PI/6).toFloat(); val s = sin(Math.PI/6).toFloat()
        val forward = floatArrayOf(1f,0f,0f,0f,c,s,0f,-s,c)
        val left = floatArrayOf(c,0f,-s,0f,1f,0f,s,0f,c)
        assertEquals(30f,Tilt.relative(identity,forward).first,0.01f)
        assertEquals(30f,Tilt.relative(identity,left).second,0.01f)
        assertEquals(0f,Tilt.relative(forward,forward).first,0.01f)
    }
    @Test fun invalidYawNeverEncodesMotion() {
        for (yaw in listOf(Float.NaN, Float.POSITIVE_INFINITY, 181f, -181f)) {
            val bytes = Protocol.encode(1, 10, 25f, 25f, yaw, true, true, false)
            assertEquals(0, bytes[1].toInt() and 1)
            assertTrue(bytes.sliceArray(4..9).all { it == 0.toByte() })
        }
    }
}
