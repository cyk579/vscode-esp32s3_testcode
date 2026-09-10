package cn.edu.gesturecar

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.assertEquals
import org.junit.Test
import android.view.View
import android.widget.ImageView

class GesturePreviewViewTest {
    @Test fun skeletonFollowsFittedImageWithoutSecondMirror() = InstrumentationRegistry.getInstrumentation().runOnMainSync {
        for ((width, height) in listOf(1000 to 300, 400 to 600)) {
            val view = GesturePreviewView(InstrumentationRegistry.getInstrumentation().targetContext)
            view.scaleType = ImageView.ScaleType.FIT_CENTER
            view.setPadding(10, 10, 10, 10)
            val source = Bitmap.createBitmap(400, 300, Bitmap.Config.ARGB_8888)
            source.eraseColor(Color.BLACK)
            view.showFrame(source, List(21) { .75f to .5f }, null)
            view.measure(View.MeasureSpec.makeMeasureSpec(width, View.MeasureSpec.EXACTLY), View.MeasureSpec.makeMeasureSpec(height, View.MeasureSpec.EXACTLY))
            view.layout(0, 0, width, height)
            val rendered = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
            view.draw(Canvas(rendered))
            val scale = minOf((width - 20) / 400f, (height - 20) / 300f)
            val horizontal = (width / 2f + 100f * scale).toInt()
            assertEquals(Color.WHITE, rendered.getPixel(horizontal, height / 2))
            assertEquals(Color.BLACK, rendered.getPixel(width - horizontal, height / 2))
            view.showFrame(source, emptyList(), null)
            view.draw(Canvas(rendered))
            assertEquals(Color.BLACK, rendered.getPixel(horizontal, height / 2))
        }
    }
}
