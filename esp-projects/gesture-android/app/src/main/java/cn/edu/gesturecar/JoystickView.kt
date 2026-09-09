package cn.edu.gesturecar

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.MotionEvent
import android.view.View

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
    var rotationOnly = false
    var accentColor: Int = Color.rgb(92, 228, 204)
        set(value) { field = value; invalidate() }
    private var pointerId = MotionEvent.INVALID_POINTER_ID
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
        val radius = JoystickInput.radius(width, height)
        val knobRadius = radius * 0.28f
        basePaint.shader = android.graphics.LinearGradient(0f, 0f, 0f, height.toFloat(), Color.rgb(29, 44, 74), Color.rgb(12, 20, 39), android.graphics.Shader.TileMode.CLAMP)
        canvas.drawCircle(cx, cy, radius, basePaint)
        ringPaint.color = Color.argb(if (active) 220 else 110, Color.red(accentColor), Color.green(accentColor), Color.blue(accentColor))
        canvas.drawCircle(cx, cy, radius, ringPaint)
        ringPaint.color = Color.argb(50, 150, 190, 235)
        canvas.drawCircle(cx, cy, radius * .66f, ringPaint)
        canvas.drawLine(cx - radius * .72f, cy, cx + radius * .72f, cy, ringPaint)
        canvas.drawLine(cx, cy - radius * .72f, cx, cy + radius * .72f, ringPaint)
        val maxOffset = radius * .62f
        val kx = cx + axisX * maxOffset
        val ky = cy + axisY * maxOffset
        knobPaint.shader = android.graphics.RadialGradient(kx - knobRadius * .3f, ky - knobRadius * .35f, knobRadius, accentColor, Color.rgb(Color.red(accentColor)/3, Color.green(accentColor)/3, Color.blue(accentColor)/3), android.graphics.Shader.TileMode.CLAMP)
        canvas.drawCircle(kx, ky, knobRadius, knobPaint)
        textPaint.textSize = radius * (if (rotationOnly) .12f else .15f)
        textPaint.color = Color.argb(170, 220, 235, 255)
        canvas.drawText(if (rotationOnly) "" else "前", cx, cy - radius * .78f, textPaint)
        canvas.drawText(if (rotationOnly) "" else "后", cx, cy + radius * .91f, textPaint)
        canvas.drawText(if (rotationOnly) "左旋" else "左", cx - radius * .85f, cy + radius * .05f, textPaint)
        canvas.drawText(if (rotationOnly) "右旋" else "右", cx + radius * .85f, cy + radius * .05f, textPaint)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                pointerId = event.getPointerId(0); active = true
                listener?.onStart(); update(event.x, event.y); return true
            }
            MotionEvent.ACTION_MOVE -> {
                if (!active) return true
                val index = event.findPointerIndex(pointerId)
                if (index < 0) finishTouch() else update(event.getX(index), event.getY(index))
                return true
            }
            MotionEvent.ACTION_POINTER_UP -> {
                if (event.getPointerId(event.actionIndex) == pointerId) finishTouch()
                return true
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                finishTouch()
                if (event.actionMasked == MotionEvent.ACTION_UP) performClick()
                return true
            }
        }
        return true
    }

    override fun performClick(): Boolean { super.performClick(); return true }

    fun reset() {
        active = false; pointerId = MotionEvent.INVALID_POINTER_ID; axisX = 0f; axisY = 0f
        parent?.requestDisallowInterceptTouchEvent(false)
        invalidate()
    }

    private fun finishTouch() {
        val wasActive = active
        reset()
        if (wasActive) { listener?.onMove(0f, 0f); listener?.onEnd() }
    }

    private fun update(x: Float, y: Float) {
        val axes = JoystickInput.axes(x, y, width, height)
        axisX = axes.first
        axisY = axes.second
        listener?.onMove(axisX, axisY)
        invalidate()
    }
}
