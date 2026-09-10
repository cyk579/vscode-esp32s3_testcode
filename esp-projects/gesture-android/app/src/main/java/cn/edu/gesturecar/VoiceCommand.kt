package cn.edu.gesturecar

enum class VoiceCommand(val forward: Float, val lateral: Float, val rotation: Float) {
    FORWARD(25f, 0f, 0f), BACKWARD(-25f, 0f, 0f),
    LEFT(0f, 25f, 0f), RIGHT(0f, -25f, 0f),
    TURN_LEFT(0f, 0f, 25f), TURN_RIGHT(0f, 0f, -25f),
    STOP(0f, 0f, 0f), ESTOP(0f, 0f, 0f);

    val durationMs: Long get() = when (this) {
        STOP, ESTOP -> 0L
        TURN_LEFT, TURN_RIGHT -> 500L
        else -> 3000L
    }

    val label: String get() = when (this) {
        FORWARD -> "前进"
        BACKWARD -> "后退"
        LEFT -> "左移"
        RIGHT -> "右移"
        TURN_LEFT -> "左转"
        TURN_RIGHT -> "右转"
        STOP -> "停车"
        ESTOP -> "急停"
    }

    companion object {
        // Match a whole utterance. Never execute substrings in negations or sentences.
        fun parse(text: String): VoiceCommand? = when (
            text.trim().trimEnd('。', '！', '!', '.', '？', '?').trim()
        ) {
            "前进", "向前", "向前走", "向前走一下", "往前走", "往前走一下", "往前开", "往前开一下", "前进一下", "请前进", "请向前走一下" -> FORWARD
            "后退", "向后", "向后退", "向后走", "往后退", "后退一下", "往后退一下", "退回来", "退回来一点", "倒退一下" -> BACKWARD
            "左移", "向左平移", "往左挪", "向左挪", "往左挪一下", "向左挪一下", "左移一下", "往左平移" -> LEFT
            "右移", "向右平移", "往右挪", "向右挪", "往右挪一下", "向右挪一下", "右移一下", "往右平移" -> RIGHT
            "左转", "左旋", "向左旋转", "逆时针旋转", "往左转", "向左转", "往左转一下", "左转一下" -> TURN_LEFT
            "右转", "右旋", "向右旋转", "顺时针旋转", "往右转", "向右转", "往右转一下", "右转一下" -> TURN_RIGHT
            "停止", "停车", "停下", "停下来", "停一下", "停止运动", "小车停下" -> STOP
            "急停", "紧急停止" -> ESTOP
            else -> null
        }
    }
}
