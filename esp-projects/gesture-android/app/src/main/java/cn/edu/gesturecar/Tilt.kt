package cn.edu.gesturecar

import kotlin.math.atan2

/** R = reference^T * current; phone top forward, screen up. All matrices row-major. */
object Tilt {
    fun relative(reference: FloatArray, current: FloatArray): Pair<Float, Float> {
        val z = FloatArray(3)
        for (column in 0..2) {
            for (k in 0..2) z[column] += reference[k * 3 + 2] * current[k * 3 + column]
        }
        val factor = (180.0 / Math.PI).toFloat()
        return Pair(atan2(-z[1], z[2]) * factor, atan2(z[0], z[2]) * factor)
    }
}
