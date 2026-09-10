package cn.edu.gesturecar

import kotlin.math.abs
import kotlin.math.sign

class HandTracker {
    var status = "请将整只手放入画面"
        private set
    var neutral: Pair<Float, Float>? = null
        private set
    private var candidate: Pair<Float, Float>? = null
    private var candidateSince = 0L
    private var lastSample: Long? = null

    fun reset() { neutral = null; candidate = null; lastSample = null }

    fun update(category: String?, score: Float, points: List<Pair<Float, Float>>, capturedAt: Long, now: Long): HandAxes? {
        if (points.size != 21) return reject("未检测到完整手部；请张掌入镜")
        if (points.any { !it.first.isFinite() || !it.second.isFinite() }) return reject("手部坐标无效，保持停车")
        if (points.any { it.first !in 0f..1f || it.second !in 0f..1f })
            return reject("手指超出取景范围；请把手移远，完整入镜")
        if (now - capturedAt !in 0..150) return reject("识别图像超时，保持停车")
        if (!score.isFinite() || score < .6f) return reject("手势置信度不足；请正对镜头张开五指")
        val palm = listOf(0, 5, 9, 13, 17).map { points[it] }
        val center = palm.map { it.first }.average().toFloat() to palm.map { it.second }.average().toFloat()
        val valid = HandMotion.map(category, score, center.first, center.second, now - capturedAt)
        if (valid == null) return reject("请张开五指平移，或握拳旋转")
        if (lastSample?.let { capturedAt - it !in 1..300 } == true) reset()
        lastSample = capturedAt
        if (valid.fist) { candidate = null; status = "握拳旋转"; return valid }
        if (neutral == null) {
            val previous = candidate
            if (previous == null || abs(previous.first - center.first) > .025f || abs(previous.second - center.second) > .025f) {
                candidate = center; candidateSince = capturedAt
            } else if (capturedAt - candidateSince >= 400) neutral = previous
            if (neutral == null) {
                val progress = ((capturedAt - candidateSince) * 100 / 400).coerceIn(0, 99)
                status = "张掌定中 $progress%；保持手掌静止约半秒"
                return null
            }
        }
        val origin = requireNotNull(neutral)
        status = "掌心已定中；相对绿十字上下左右移动"
        return HandAxes(axis(origin.second - center.second), axis(origin.first - center.first), 0f, false)
    }

    private fun reject(message: String): HandAxes? {
        reset()
        status = message
        return null
    }

    private fun axis(displacement: Float): Float {
        val amount = abs(displacement)
        if (amount <= .025f) return 0f
        return sign(displacement) * (5f + 20f * ((amount - .025f) / .155f).coerceIn(0f, 1f))
    }
}
