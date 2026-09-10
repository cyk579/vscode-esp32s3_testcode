package cn.edu.gesturecar

import org.junit.Assert.assertEquals
import org.junit.Test

class HandMotionTest {
    @Test fun openPalmAxesStayWithinDriveRange() {
        val result = HandMotion.map("Open_Palm", 1f, 0f, 0f, 0)
        requireNotNull(result)
        assertEquals(25f, result.forward, 0f)
        assertEquals(25f, result.lateral, 0f)
    }
}
