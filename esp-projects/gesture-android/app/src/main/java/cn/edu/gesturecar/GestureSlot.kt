package cn.edu.gesturecar

import android.app.Activity

object GestureSlot { fun create(host: Activity): GestureFeature = MediaPipeHandFeature(host) }
