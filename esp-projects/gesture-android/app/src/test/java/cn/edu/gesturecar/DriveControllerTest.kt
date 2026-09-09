package cn.edu.gesturecar
import org.junit.Assert.*
import org.junit.Test

class DriveControllerTest {
    private fun state(value: Int = 2) = ByteArray(12).also { it[0] = 1; it[1] = value.toByte() }
    private fun ready(source: ControlSource): DriveController = DriveController().apply {
        foreground(true, 0); link(true, 0); select(source, 0); updateStatus(state(), 400)
    }
    @Test fun voiceExpiresAndDoesNotResumeAfterFreshStatus() {
        val d = ready(ControlSource.VOICE)
        assertTrue(d.begin(ControlSource.VOICE, 400))
        assertTrue(d.submit(ControlSource.VOICE, 25f, 0f, 0f, 400, 400))
        d.updateStatus(state(3), 1000)
        assertNotNull(d.output(1199)); assertNull(d.output(1200))
        d.updateStatus(state(), 1600)
        assertFalse(d.submit(ControlSource.VOICE, 25f, 0f, 0f, 1600, 1600))
    }
    @Test fun staleGestureAndInactiveModesCannotDrive() {
        val d = ready(ControlSource.GESTURE)
        assertTrue(d.begin(ControlSource.GESTURE, 400))
        assertFalse(d.submit(ControlSource.VOICE, 25f, 0f, 0f, 400, 400))
        assertFalse(d.submit(ControlSource.GESTURE, 25f, 0f, 0f, 200, 400))
        assertTrue(d.submit(ControlSource.GESTURE, 25f, 0f, 0f, 400, 400))
        d.select(ControlSource.MANUAL, 410)
        assertNull(d.output(411))
        assertFalse(d.submit(ControlSource.GESTURE, 25f, 0f, 0f, 411, 411))
    }
    @Test fun estopStatusTimeoutAndBackgroundRevokeOutput() {
        val d = ready(ControlSource.MANUAL)
        assertTrue(d.begin(ControlSource.MANUAL, 400))
        d.submit(ControlSource.MANUAL, 25f, 0f, 0f, 400, 400)
        d.emergency(true, 450)
        assertEquals(4, d.frame(450)[1].toInt() and 4)
        assertNull(d.output(450))
        d.emergency(false, 500); d.updateStatus(state(), 900)
        assertFalse(d.submit(ControlSource.MANUAL, 25f, 0f, 0f, 900, 900))
        assertTrue(d.begin(ControlSource.MANUAL, 900))
        d.submit(ControlSource.MANUAL, 25f, 0f, 0f, 900, 900)
        assertNull(d.output(1501))
        d.foreground(false, 1502)
        assertEquals(0, d.frame(1502)[1].toInt() and 3)
    }
    @Test fun gattReadyDoesNotReplaceVehicleReady() {
        val d = DriveController()
        d.foreground(true, 0); d.link(true, 0)
        assertFalse(d.begin(ControlSource.MANUAL, 500))
        d.updateStatus(state(4), 500)
        assertFalse(d.begin(ControlSource.MANUAL, 1000))
    }
    @Test fun lateInputCannotRenewExpiredMotionBeforeTheNextOutputTick() {
        for (source in listOf(ControlSource.MANUAL, ControlSource.GESTURE)) {
            val d = ready(source)
            assertTrue(d.begin(source, 400))
            assertTrue(d.submit(source, 25f, 0f, 0f, 400, 400))
            // The last received car status can still say READY while input processing stalls.
            assertFalse(d.submit(source, 25f, 0f, 0f, 600, 600))
            assertNull(d.output(600))
            d.updateStatus(state(), 1000)
            assertFalse(d.submit(source, 25f, 0f, 0f, 1000, 1000))
            assertTrue(d.begin(source, 1000))
            assertTrue(d.submit(source, 25f, 0f, 0f, 1000, 1000))
        }
    }
    @Test fun carRearmingOrInvalidStatusRevokesAnActiveGesture() {
        for (status in listOf(0, 1, 4, 255)) {
            val d = ready(ControlSource.GESTURE)
            d.begin(ControlSource.GESTURE, 400)
            d.submit(ControlSource.GESTURE, 25f, 0f, 0f, 400, 400)
            d.updateStatus(state(status), 450)
            assertNull(d.output(450))
            d.updateStatus(state(), 1000)
            assertFalse(d.submit(ControlSource.GESTURE, 25f, 0f, 0f, 1000, 1000))
        }
    }
}
