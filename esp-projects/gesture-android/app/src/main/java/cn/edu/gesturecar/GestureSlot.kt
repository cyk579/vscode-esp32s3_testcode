package cn.edu.gesturecar

import android.app.Activity

/** The hand-recognition teammate replaces only this factory and supplies GestureFeature.
 * Return null until the detector is implemented; the UI must not claim recognition works. */
object GestureSlot {
    fun create(host: Activity): GestureFeature? = null
}
