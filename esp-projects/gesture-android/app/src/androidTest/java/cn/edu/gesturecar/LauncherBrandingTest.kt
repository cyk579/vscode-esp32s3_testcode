package cn.edu.gesturecar

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.drawable.AdaptiveIconDrawable
import android.os.Build
import androidx.test.platform.app.InstrumentationRegistry
import java.io.File
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class LauncherBrandingTest {
    @Test fun launcherUsesKs21NameAndAdaptiveIcon() {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val info = context.applicationInfo
        assertEquals("KS-21创新", info.loadLabel(context.packageManager).toString())
        val icon = context.getDrawable(info.icon)
        assertTrue(icon is AdaptiveIconDrawable)
        val adaptive = icon as AdaptiveIconDrawable
        assertNotNull(adaptive.background)
        assertNotNull(adaptive.foreground)
        if (Build.VERSION.SDK_INT >= 33) assertNotNull(adaptive.monochrome)
        val preview = Bitmap.createBitmap(512, 512, Bitmap.Config.ARGB_8888)
        adaptive.setBounds(0, 0, 512, 512)
        adaptive.draw(Canvas(preview))
        File(context.getExternalFilesDir(null), "ks21-icon-preview.png").outputStream().use {
            assertTrue(preview.compress(Bitmap.CompressFormat.PNG, 100, it))
        }
        preview.recycle()
    }
}
