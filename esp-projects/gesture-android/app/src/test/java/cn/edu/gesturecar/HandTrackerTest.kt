package cn.edu.gesturecar

import org.junit.Assert.*
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder

class HandTrackerTest {
    @Test fun clippedFingerExplainsWhyPalmStopsWhileFistWorks() {
        val tracker = calibrated()
        val clipped = points(.7f, .6f).toMutableList().apply { this[12] = .7f to -.01f }
        assertNull(tracker.update("Open_Palm", 1f, clipped, 600, 600))
        assertTrue(tracker.status.contains("超出取景范围"))
        assertNull(tracker.neutral)
        assertEquals(HandAxes(0f, 0f, -25f, true), tracker.update("Closed_Fist", 1f, points(.7f, .6f), 700, 700))
        assertNull(tracker.update("Open_Palm", 1f, points(.7f, .6f), 800, 800))
        assertTrue(tracker.status.contains("定中 0%"))
        for (stamp in 900L..1200L step 100) tracker.update("Open_Palm", 1f, points(.7f, .6f), stamp, stamp)
        assertEquals(HandAxes(25f, 0f, 0f, false), tracker.update("Open_Palm", 1f, points(.7f, .4f), 1300, 1300))
    }

    @Test fun rejectionReasonsAndCalibrationProgressAreDistinct() {
        val tracker = HandTracker()
        assertNull(tracker.update("Open_Palm", 1f, emptyList(), 0, 0))
        assertTrue(tracker.status.contains("未检测"))
        assertNull(tracker.update("Open_Palm", .5f, points(.5f, .5f), 0, 0))
        assertTrue(tracker.status.contains("置信度"))
        assertNull(tracker.update("Open_Palm", 1f, points(.5f, .5f), 0, 151))
        assertTrue(tracker.status.contains("超时"))
        assertNull(tracker.update("None", 1f, points(.5f, .5f), 0, 0))
        assertTrue(tracker.status.contains("张开五指"))
        tracker.update("Open_Palm", 1f, points(.5f, .5f), 200, 200)
        tracker.update("Open_Palm", 1f, points(.5f, .5f), 400, 400)
        assertTrue(tracker.status.contains("50%"))
        tracker.update("Open_Palm", 1f, points(.5f, .5f), 600, 600)
        assertTrue(tracker.status.contains("已定中"))
    }

    private fun points(horizontal: Float, vertical: Float) = List(21) { horizontal to vertical }
    private fun calibrated(): HandTracker = HandTracker().also { tracker ->
        for (stamp in 0L..500L step 100) tracker.update("Open_Palm", 1f, points(.7f, .6f), stamp, stamp)
    }

    @Test fun initialOffCenterPalmDoesNotCauseRightwardDrift() {
        val tracker = HandTracker()
        assertNull(tracker.update("Open_Palm", 1f, points(.7f, .6f), 0, 0))
        assertNull(tracker.update("Open_Palm", 1f, points(.7f, .6f), 100, 100))
        assertNull(tracker.update("Open_Palm", 1f, points(.7f, .6f), 200, 200))
        assertEquals(HandAxes(0f, 0f, 0f, false), tracker.update("Open_Palm", 1f, points(.7f, .6f), 400, 400))
    }

    @Test fun fourDirectionsReachFullScaleWithoutMovingHandToImageEdges() {
        for ((position, expected) in listOf(
            (.7f to .4f) to HandAxes(25f, 0f, 0f, false),
            (.7f to .8f) to HandAxes(-25f, 0f, 0f, false),
            (.5f to .6f) to HandAxes(0f, 25f, 0f, false),
            (.9f to .6f) to HandAxes(0f, -25f, 0f, false)
        )) assertEquals(expected, calibrated().update("Open_Palm", 1f, points(position.first, position.second), 600, 600))
    }

    @Test fun wristOffsetDoesNotBiasPalmCenter() {
        val tracker = calibrated()
        val landmarks = points(.7f, .55f).toMutableList()
        landmarks[0] = .7f to .8f
        assertEquals(HandAxes(0f, 0f, 0f, false), tracker.update("Open_Palm", 1f, landmarks, 600, 600))
    }

    @Test fun lossStaleFramesAndUnknownGesturesRequireNewNeutral() {
        for ((category, age) in listOf("Open_Palm" to 151L, "None" to 0L)) {
            val tracker = calibrated()
            assertNull(tracker.update(category, 1f, points(.7f, .6f), 600, 600 + age))
            assertNull(tracker.neutral)
            assertNull(tracker.update("Open_Palm", 1f, points(.5f, .6f), 800, 800))
        }
        val tracker = calibrated()
        assertNull(tracker.update("Open_Palm", 1f, points(.5f, .6f), 1000, 1000))
        assertNull(tracker.neutral)
    }

    @Test fun fistAndNeutralRemainIndependent() {
        val tracker = calibrated()
        assertEquals(HandAxes(0f, 0f, -25f, true), tracker.update("Closed_Fist", 1f, points(.7f, .6f), 600, 600))
        assertEquals(HandAxes(0f, 0f, 0f, false), tracker.update("Open_Palm", 1f, points(.71f, .61f), 700, 700))
    }

    @Test fun allFourDirectionsPassControllerAndUseCorrectBleFields() {
        for ((position, expected) in listOf(
            (.7f to .4f) to (2500 to 0), (.7f to .8f) to (-2500 to 0),
            (.5f to .6f) to (0 to 2500), (.9f to .6f) to (0 to -2500)
        )) {
            val axes = requireNotNull(calibrated().update("Open_Palm", 1f, points(position.first, position.second), 600, 600))
            val controller = DriveController()
            controller.foreground(true, 0); controller.link(true, 0); controller.select(ControlSource.GESTURE, 0)
            controller.updateStatus(ByteArray(12).also { it[0] = 1; it[1] = 2 }, 600)
            assertTrue(controller.begin(ControlSource.GESTURE, 600))
            assertTrue(controller.submit(ControlSource.GESTURE, axes.forward, axes.lateral, axes.yaw, 600, 600))
            val frame = ByteBuffer.wrap(controller.frame(600)).order(ByteOrder.LITTLE_ENDIAN)
            assertEquals(expected.first, frame.getShort(4).toInt())
            assertEquals(expected.second, frame.getShort(6).toInt())
            assertEquals(0, frame.getShort(8).toInt())
            controller.stop(601)
            assertEquals(0, controller.frame(601)[1].toInt() and 2)
        }
    }
}
