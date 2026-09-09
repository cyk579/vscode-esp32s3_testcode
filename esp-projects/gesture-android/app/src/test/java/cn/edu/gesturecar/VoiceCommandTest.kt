package cn.edu.gesturecar

import org.junit.Assert.*
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder

class VoiceCommandTest {
    @Test fun rejectsNegationAmbiguityAndChainedInstructions() {
        for (text in listOf("", "不要前进", "别后退", "前进然后右转", "前进停止", "旋转", "向左", "解除急停", "前进一米")) {
            assertNull(text, VoiceCommand.parse(text))
        }
    }
    @Test fun recognizesWholeChineseCommandsAndTerminalPunctuation() {
        assertEquals(VoiceCommand.FORWARD, VoiceCommand.parse(" 前进。 "))
        assertEquals(VoiceCommand.BACKWARD, VoiceCommand.parse("向后退"))
        assertEquals(VoiceCommand.STOP, VoiceCommand.parse("停下！"))
        assertEquals(VoiceCommand.ESTOP, VoiceCommand.parse("紧急停止"))
        assertEquals(VoiceCommand.TURN_LEFT, VoiceCommand.parse("逆时针旋转"))
        assertEquals(VoiceCommand.TURN_RIGHT, VoiceCommand.parse("顺时针旋转"))
    }
    @Test fun speechUsesExistingThreeAxisBleProtocol() {
        val cases = mapOf(
            "前进" to listOf(2500, 0, 0), "后退" to listOf(-2500, 0, 0),
            "左移" to listOf(0, 2500, 0), "右移" to listOf(0, -2500, 0),
            "左转" to listOf(0, 0, 2500), "右转" to listOf(0, 0, -2500)
        )
        for ((text, expected) in cases) {
            val cmd = VoiceCommand.parse(text)!!
            val packet = Protocol.encode(1, 100, cmd.forward, cmd.lateral, cmd.rotation, true, true, false)
            assertEquals(14, packet.size)
            assertEquals(3, packet[1].toInt())
            val decoded = ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN)
            assertEquals(text, expected, listOf(decoded.getShort(4).toInt(), decoded.getShort(6).toInt(), decoded.getShort(8).toInt()))
        }
    }
}
