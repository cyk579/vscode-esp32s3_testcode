package cn.edu.gesturecar

import android.app.Activity
import android.widget.ImageView
object GestureSlot { fun create(host: Activity, preview: ImageView): GestureFeature = MediaPipeHandFeature(host, preview) }
