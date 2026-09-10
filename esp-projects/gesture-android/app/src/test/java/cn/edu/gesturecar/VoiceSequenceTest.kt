package cn.edu.gesturecar

import org.junit.Assert.*
import org.junit.Test

class VoiceSequenceTest {
    private fun state(value: Int = 2, fault: Boolean = false) = ByteArray(12).also {
        it[0] = 1; it[1] = value.toByte(); it[2] = if (fault) 1 else 0
    }

    private fun ready() = DriveController().apply {
        foreground(true, 0); link(true, 0); select(ControlSource.VOICE, 0)
        updateStatus(state(), 400)
    }

    @Test fun parsesCompleteOrderedUtterances() {
        val expected = VoiceIntent.Sequence(listOf(VoiceCommand.FORWARD, VoiceCommand.LEFT, VoiceCommand.TURN_RIGHT))
        for (text in listOf("先前进再左移再旋转", "前进，然后左移，接着右转。", "前进、左移、右转", "先前进，再左移，最后原地旋转")) {
            assertEquals(text, expected, VoiceRouter.route(text))
        }
        assertEquals(VoiceIntent.Sequence(listOf(VoiceCommand.BACKWARD, VoiceCommand.TURN_LEFT, VoiceCommand.STOP)),
            VoiceRouter.route("先后退再左转再停车"))
        assertEquals(VoiceIntent.Play("先前进再左移"), VoiceRouter.route("播放先前进再左移"))
        assertEquals(VoiceIntent.Drive(VoiceCommand.ESTOP), VoiceRouter.route("急停"))
    }

    @Test fun rejectsTheWholeUnsafeOrAmbiguousSequence() {
        for (text in listOf("不要前进再左移", "前进再不要左移", "前进再跳舞", "前进再", "再前进", "前进再再左移",
            "前进一米再左移", "前进同时左移", "前进再向左", "前进再急停再后退", "前进再停车再后退",
            "前进再前进再前进再前进再前进再前进再前进")) {
            assertEquals(text, VoiceIntent.Unknown, VoiceRouter.route(text))
        }
    }

    @Test fun translationThenRotationUseRequestedDurationsAndNeutralGap() {
        val controller = ready()
        assertTrue(controller.startVoiceSequence(listOf(VoiceCommand.FORWARD, VoiceCommand.TURN_LEFT), 400))
        for (stamp in 400L..3399L step 100) {
            controller.updateStatus(state(3), stamp)
            assertEquals(25f, controller.output(stamp)!!.forward, 0f)
        }
        assertNull(controller.output(3400))
        controller.updateStatus(state(1), 3500)
        assertNull(controller.output(3500))
        assertNull(controller.output(3749))
        controller.updateStatus(state(), 3750)
        assertEquals(25f, controller.output(3750)!!.rotation, 0f)
        for (stamp in 3850L..4249L step 100) {
            controller.updateStatus(state(3), stamp)
            assertEquals(25f, controller.output(stamp)!!.rotation, 0f)
        }
        assertNotNull(controller.output(4249))
        assertNull(controller.output(4250))
        assertFalse(controller.voiceSequenceActive)
        assertEquals("组合指令完成，已停车", controller.voiceSequenceStatus)
    }

    @Test fun interruptionsDiscardAllRemainingSteps() {
        val interrupts = listOf<(DriveController) -> Unit>(
            { it.stop(450) }, { it.emergency(true, 450) }, { it.link(false, 450) },
            { it.foreground(false, 450) }, { it.select(ControlSource.MANUAL, 450) },
            { it.updateStatus(state(4), 450) }, { it.updateStatus(state(2, true), 450) },
            { it.updateStatus(byteArrayOf(), 450) }
        )
        for (interrupt in interrupts) {
            val controller = ready()
            controller.startVoiceSequence(listOf(VoiceCommand.FORWARD, VoiceCommand.LEFT), 400)
            interrupt(controller)
            assertFalse(controller.voiceSequenceActive)
            assertNull(controller.output(450))
            controller.foreground(true, 1000); controller.link(true, 1000)
            controller.select(ControlSource.VOICE, 1000); controller.updateStatus(state(), 1400)
            assertNull(controller.output(1400))
        }
    }

    @Test fun statusExpiryAndSchedulerStallsNeverAdvanceToTheNextStep() {
        val controller = ready()
        controller.startVoiceSequence(listOf(VoiceCommand.FORWARD, VoiceCommand.LEFT), 400)
        controller.output(600); controller.output(800)
        assertNull(controller.output(901))
        assertFalse(controller.voiceSequenceActive)
        val stalled = ready()
        stalled.startVoiceSequence(listOf(VoiceCommand.TURN_LEFT, VoiceCommand.FORWARD), 400)
        stalled.updateStatus(state(), 2000)
        assertNull(stalled.output(2000))
        assertFalse(stalled.voiceSequenceActive)
    }

    @Test fun nextStepWaitsForReadyAndAbortsAfterTimeout() {
        val controller = ready()
        controller.startVoiceSequence(listOf(VoiceCommand.TURN_RIGHT, VoiceCommand.FORWARD), 400)
        for (stamp in 400L..3300L step 100) {
            controller.updateStatus(state(3), stamp)
            val request = controller.output(stamp)
            if (stamp >= 900) assertNull(request)
        }
        assertFalse(controller.voiceSequenceActive)
    }

    @Test fun invalidPlansNeverStartAndDurationsStayExplicit() {
        for (commands in listOf(emptyList(), listOf(VoiceCommand.FORWARD),
            listOf(VoiceCommand.ESTOP, VoiceCommand.FORWARD), List(7) { VoiceCommand.LEFT })) {
            val controller = ready()
            assertFalse(controller.startVoiceSequence(commands, 400))
            assertNull(controller.output(400))
        }
        for (command in listOf(VoiceCommand.FORWARD, VoiceCommand.BACKWARD, VoiceCommand.LEFT, VoiceCommand.RIGHT)) {
            assertEquals(3000L, command.durationMs)
        }
        assertEquals(500L, VoiceCommand.TURN_LEFT.durationMs)
        assertEquals(500L, VoiceCommand.TURN_RIGHT.durationMs)
    }
}
