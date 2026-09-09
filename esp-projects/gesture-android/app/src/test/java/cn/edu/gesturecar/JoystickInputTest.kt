package cn.edu.gesturecar

import java.nio.ByteBuffer
import java.nio.ByteOrder
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.hypot

class JoystickInputTest {
    @Test fun upwardTouchSendsOnlyForwardForEveryAspectRatio() {
        for ((width, height) in listOf(160 to 210, 800 to 220, 64 to 64, 220 to 800)) {
            val limit = JoystickInput.radius(width, height) * .62f
            val axes = JoystickInput.axes(width / 2f, height / 2f - limit, width, height)
            val command = JoystickInput.translation(axes.first, axes.second)
            val packet = ByteBuffer.wrap(Protocol.encode(1, 50, command.first, command.second, 0f, true, true, false)).order(ByteOrder.LITTLE_ENDIAN)
            assertEquals(2500, packet.getShort(4).toInt())
            assertEquals(0, packet.getShort(6).toInt())
            assertEquals(0, packet.getShort(8).toInt())
        }
    }

    @Test fun cardinalAxesStayIndependent() {
        val left = JoystickInput.translation(-1f, 0f)
        assertEquals(0f, left.first, 0f); assertEquals(25f, left.second, 0f)
        val back = JoystickInput.translation(0f, 1f)
        assertEquals(-25f, back.first, 0f); assertEquals(0f, back.second, 0f)
        assertEquals(25f, JoystickInput.rotation(-1f), 0f)
        assertEquals(-25f, JoystickInput.rotation(1f), 0f)
    }

    @Test fun draggingBeyondEdgePreservesDirectionAndLimit() {
        val axes = JoystickInput.axes(-200f, -200f, 200, 200)
        assertEquals(1f, hypot(axes.first, axes.second), .0001f)
        assertEquals(axes.first, axes.second, .0001f)
    }

    @Test fun centerAndInvalidTouchesAreNeutral() {
        assertEquals(0f to 0f, JoystickInput.axes(80f, 105f, 160, 210))
        assertEquals(0f to 0f, JoystickInput.axes(Float.NaN, 0f, 160, 210))
        assertEquals(0f to 0f, JoystickInput.axes(0f, 0f, 0, 0))
    }
}
