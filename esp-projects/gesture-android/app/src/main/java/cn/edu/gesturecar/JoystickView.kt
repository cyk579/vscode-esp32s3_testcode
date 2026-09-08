package cn.edu.gesturecar

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View
import kotlin.math.hypot

class JoystickView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null
) : View(context, attrs) {
    interface Listener {
        fun onMove(x: Float, y: Float)
        fun onStart()
        fun onEnd()
    }

    var listener: Listener? = null
    var active = false
        private set
    var axisX = 0f
        private set
    var axisY = 0f
        private set
    private val basePaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val ringPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 3f }
    private val knobPaint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { textAlign = Paint.Align.CENTER }
    private val knob = RectF()

    init { isClickable = true }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val cx = width / 2f
        val cy = height / 2f
        val radius = (minOf(width, height) * 0.38f).coerceAtLeast(80f)
        val knobRadius = radius * 0.28f
        basePaint.shader = android.graphics.LinearGradient(0f, 0f, 0f, height.toFloat(), Color.rgb(29, 44, 74), Color.rgb(12, 20, 39), android.graphics.Shader.TileMode.CLAMP)
        canvas.drawCircle(cx, cy, radius, basePaint)
        ringPaint.color = Color.argb(90, 130, 170, 220)
        canvas.drawCircle(cx, cy, radius, ringPaint)
        ringPaint.color = Color.argb(50, 150, 190, 235)
        canvas.drawCircle(cx, cy, radius * .66f, ringPaint)
        canvas.drawLine(cx - radius * .72f, cy, cx + radius * .72f, cy, ringPaint)
        canvas.drawLine(cx, cy - radius * .72f, cx, cy + radius * .72f, ringPaint)
        val maxOffset = radius * .62f
        val kx = cx + axisX * maxOffset
        val ky = cy + axisY * maxOffset
        knobPaint.shader = android.graphics.RadialGradient(kx - knobRadius * .3f, ky - knobRadius * .35f, knobRadius, Color.rgb(105, 222, 255), Color.rgb(27, 121, 198), android.graphics.Shader.TileMode.CLAMP)
        canvas.drawCircle(kx, ky, knobRadius, knobPaint)
        textPaint.textSize = radius * .15f
        textPaint.color = Color.argb(170, 220, 235, 255)
        canvas.drawText("前", cx, cy - radius * .78f, textPaint)
        canvas.drawText("后", cx, cy + radius * .91f, textPaint)
        canvas.drawText("左", cx - radius * .88f, cy + radius * .05f, textPaint)
        canvas.drawText("右", cx + radius * .88f, cy + radius * .05f, textPaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> { active = true; listener?.onStart(); update(event.x, event.y); return true }
            MotionEvent.ACTION_MOVE -> { update(event.x, event.y); return true }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> { active = false; axisX = 0f; axisY = 0f; listener?.onMove(0f, 0f); listener?.onEnd(); invalidate(); performClick(); return true }
        }
        return true
    }

    override fun performClick(): Boolean { super.performClick(); return true }

    private fun update(x: Float, y: Float) {
        val cx = width / 2f; val cy = height / 2f
        val limit = (minOf(width, height) * .38f * .62f).coerceAtLeast(50f)
        var dx = x - cx; var dy = y - cy
        val length = hypot(dx.toDouble(), dy.toDouble()).toFloat()
        if (length > limit) { dx *= limit / length; dy *= limit / length }
        axisX = dx / limit
        axisY = dy / limit
        listener?.onMove(axisX, axisY)
        invalidate()
    }
}
