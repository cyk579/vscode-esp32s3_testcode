package cn.edu.gesturecar

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.UUID
import kotlin.math.roundToInt

object Protocol {
    const val GROUP_ID = 1
    val SERVICE: UUID = UUID.fromString("7f510001-1b15-4b85-9c13-8f08604a0001")
    val CONTROL: UUID = UUID.fromString("7f510002-1b15-4b85-9c13-8f08604a0001")
    val STATUS: UUID = UUID.fromString("7f510003-1b15-4b85-9c13-8f08604a0001")
    val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    fun encode(sequence: Int, nowMs: Long, forward: Float, lateral: Float, yaw: Float, valid: Boolean, held: Boolean, estop: Boolean): ByteArray {
        val good = valid && forward.isFinite() && lateral.isFinite() && yaw.isFinite() && forward in -180f..180f && lateral in -180f..180f && yaw in -180f..180f
        val flags = (if (good) 1 else 0) or (if (held) 2 else 0) or (if (estop) 4 else 0)
        return ByteBuffer.allocate(14).order(ByteOrder.LITTLE_ENDIAN)
            .put(1.toByte()).put(flags.toByte()).putShort(sequence.toShort())
            .putShort(if (good) (forward * 100).roundToInt().toShort() else 0.toShort())
            .putShort(if (good) (lateral * 100).roundToInt().toShort() else 0.toShort())
            .putShort(if (good) (yaw * 100).roundToInt().toShort() else 0.toShort())
            .putInt(nowMs.toInt()).array()
    }
    fun statusText(bytes: ByteArray): String {
        if (bytes.size != 12 || bytes[0].toInt() != 1) return "状态格式不兼容"
        val names = arrayOf("未连接", "请松手并回正", "就绪，可以按住使能", "行驶", "已停车，请排除故障后回正")
        val state = bytes[1].toInt() and 255
        val reason = when (bytes[11].toInt() and 255) {
            0 -> "正常"
            1 -> "I2C 总线初始化失败"
            2 -> "找不到 MPU6500（检查地址/接线）"
            3 -> "WHO_AM_I 校验失败"
            4 -> "MPU6500 数据读取失败"
            5 -> "MPU6500 校准中"
            6 -> "姿态融合启动中/无效"
            else -> "未知"
        }
        return (names.getOrNull(state) ?: "未知状态") +
            "\n车端故障：" + (bytes[2].toInt() != 0) + "    输出：" + bytes[8] + " / " + bytes[9] + " / " + bytes[10] +
            "\nIMU：" + reason
    }

}
