package cn.edu.gesturecar

import android.content.Context
import android.graphics.Canvas
import android.graphics.Bitmap
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.widget.ImageView

class GesturePreviewView @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null
) : ImageView(context, attrs) {
    private val line = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(255, 210, 130); strokeWidth = 3f; style = Paint.Style.STROKE }
    private val point = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; style = Paint.Style.FILL }
    private var landmarks: List<Pair<Float, Float>> = emptyList()
    private var neutral: Pair<Float, Float>? = null
    private val marker = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(92, 228, 204); strokeWidth = 2f; style = Paint.Style.STROKE }
    private val edges = arrayOf(
        0 to 1, 1 to 2, 2 to 3, 3 to 4, 0 to 5, 5 to 6, 6 to 7, 7 to 8,
        5 to 9, 9 to 10, 10 to 11, 11 to 12, 9 to 13, 13 to 14, 14 to 15, 15 to 16,
        13 to 17, 17 to 18, 18 to 19, 19 to 20, 0 to 17
    )

    fun showFrame(bitmap: Bitmap, points: List<Pair<Float, Float>>, center: Pair<Float, Float>?) {
        landmarks = points.toList()
        neutral = center
        setImageBitmap(bitmap)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val picture = drawable ?: return
        fun project(horizontal: Float, vertical: Float): FloatArray {
            val coordinates = floatArrayOf(horizontal * picture.intrinsicWidth, vertical * picture.intrinsicHeight)
            imageMatrix.mapPoints(coordinates)
            coordinates[0] += paddingLeft
            coordinates[1] += paddingTop
            return coordinates
        }
        val saved = canvas.save()
        canvas.clipRect(paddingLeft, paddingTop, width - paddingRight, height - paddingBottom)
        if (landmarks.size == 21 && landmarks.all { it.first.isFinite() && it.second.isFinite() }) {
            val mapped = landmarks.map { project(it.first, it.second) }
            edges.forEach { (from, to) -> canvas.drawLine(mapped[from][0], mapped[from][1], mapped[to][0], mapped[to][1], line) }
            mapped.forEachIndexed { index, coordinates -> canvas.drawCircle(coordinates[0], coordinates[1], if (index == 0) 7f else 5f, point) }
        }
        neutral?.let { center ->
            val coordinates = project(center.first, center.second)
            canvas.drawCircle(coordinates[0], coordinates[1], 12f, marker)
            canvas.drawLine(coordinates[0] - 20f, coordinates[1], coordinates[0] + 20f, coordinates[1], marker)
            canvas.drawLine(coordinates[0], coordinates[1] - 20f, coordinates[0], coordinates[1] + 20f, marker)
        }
        canvas.restoreToCount(saved)
    }
}
