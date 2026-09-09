package cn.edu.gesturecar

enum class VoiceCommand(val forward: Float, val lateral: Float, val rotation: Float) {
    FORWARD(25f, 0f, 0f), BACKWARD(-25f, 0f, 0f),
    LEFT(0f, 25f, 0f), RIGHT(0f, -25f, 0f),
    TURN_LEFT(0f, 0f, 25f), TURN_RIGHT(0f, 0f, -25f),
    STOP(0f, 0f, 0f), ESTOP(0f, 0f, 0f);

    companion object {
        // Match a whole utterance. Never execute substrings in negations or sentences.
        fun parse(text: String): VoiceCommand? = when (
            text.trim().trimEnd('。', '！', '!', '.', '？', '?').trim()
        ) {
            "前进", "向前", "向前走" -> FORWARD
            "后退", "向后", "向后退" -> BACKWARD
            "左移", "向左平移" -> LEFT
            "右移", "向右平移" -> RIGHT
            "左转", "左旋", "向左旋转", "逆时针旋转" -> TURN_LEFT
            "右转", "右旋", "向右旋转", "顺时针旋转" -> TURN_RIGHT
            "停止", "停车", "停下" -> STOP
            "急停", "紧急停止" -> ESTOP
            else -> null
        }
    }
}
