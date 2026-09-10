package cn.edu.gesturecar

data class HandAxes(val forward: Float, val lateral: Float, val yaw: Float, val fist: Boolean)

object HandMotion {
    private fun axis(value: Float) = (value * 50f).coerceIn(-25f, 25f)
    fun map(category: String?, score: Float, horizontal: Float, vertical: Float, ageMs: Long): HandAxes? {
        if (!score.isFinite() || score < .6f || !horizontal.isFinite() || !vertical.isFinite() ||
            horizontal !in 0f..1f || vertical !in 0f..1f || ageMs !in 0..150) return null
        return when (category) {
            "Closed_Fist" -> HandAxes(0f, 0f, -25f, true)
            "Open_Palm" -> HandAxes(axis(.5f - vertical), axis(.5f - horizontal), 0f, false)
            else -> null
        }
    }
}
