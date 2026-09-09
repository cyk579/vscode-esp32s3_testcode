package cn.edu.gesturecar

import kotlin.math.hypot
import kotlin.math.max

object JoystickInput {
    fun radius(width: Int, height: Int): Float = (minOf(width, height) * .38f).coerceAtLeast(1f)

    fun axes(touchX: Float, touchY: Float, width: Int, height: Int): Pair<Float, Float> {
        if (width <= 0 || height <= 0 || !touchX.isFinite() || !touchY.isFinite()) return 0f to 0f
        val limit = radius(width, height) * .62f
        val horizontal = (touchX - width / 2f) / limit
        val vertical = (touchY - height / 2f) / limit
        val scale = max(1f, hypot(horizontal, vertical))
        return horizontal / scale to vertical / scale
    }

    fun translation(horizontal: Float, vertical: Float): Pair<Float, Float> = -vertical * 25f to -horizontal * 25f
    fun rotation(horizontal: Float): Float = -horizontal * 25f
}
