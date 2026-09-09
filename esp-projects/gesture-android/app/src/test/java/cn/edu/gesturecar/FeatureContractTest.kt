package cn.edu.gesturecar

import org.junit.Assert.*
import org.junit.Test

class FeatureContractTest {
    @Test fun manualWinsOverVoiceAndGesture() {
        val arbiter = DriveArbiter()
        arbiter.submit(DriveRequest(ControlSource.GESTURE, 1f, 0f, 0f, 5000), 0)
        arbiter.submit(DriveRequest(ControlSource.VOICE, 0f, 1f, 0f, 5000), 0)
        arbiter.submit(DriveRequest(ControlSource.MANUAL, 0f, 0f, 1f, 5000), 0)
        assertEquals(ControlSource.MANUAL, arbiter.current(100)?.source)
    }
    @Test fun expiredAndInvalidRequestsCannotDrive() {
        val arbiter = DriveArbiter()
        assertFalse(arbiter.submit(DriveRequest(ControlSource.VOICE, 1f, 0f, 0f, 10), 10))
        assertFalse(arbiter.submit(DriveRequest(ControlSource.VOICE, Float.NaN, 0f, 0f, 100), 0))
        assertNull(arbiter.current(1000))
    }
    @Test fun movementDoesNotBecomeMediaStop() {
        assertEquals(VoiceIntent.Drive(VoiceCommand.FORWARD), VoiceRouter.route("前进"))
        assertEquals(VoiceIntent.Play("稻香"), VoiceRouter.route("播放稻香"))
        assertEquals(VoiceIntent.Play("稻香"), VoiceRouter.route("播放歌曲稻香。"))
        assertEquals(VoiceIntent.StopMusic, VoiceRouter.route("停止音乐"))
        assertEquals(VoiceIntent.PauseMusic, VoiceRouter.route("暂停播放"))
        assertNotEquals(VoiceIntent.StopMusic, VoiceRouter.route("前进"))
    }
    @Test fun mediaCommandsHaveIndependentIdentity() {
        val commands = mutableListOf<MediaCommand>()
        val sink = MediaSink { commands += it; true }
        sink.submit(MediaCommand.PlayTitle("稻香"))
        sink.submit(MediaCommand.Pause)
        sink.submit(MediaCommand.Resume)
        assertEquals(listOf(MediaCommand.PlayTitle("稻香"), MediaCommand.Pause, MediaCommand.Resume), commands)
    }
    @Test fun preemptedRequestsNeverResume() {
        val arbiter = DriveArbiter()
        arbiter.submit(DriveRequest(ControlSource.VOICE, 25f, 0f, 0f, 900), 0)
        arbiter.submit(DriveRequest(ControlSource.MANUAL, 0f, 25f, 0f, 500), 10)
        assertFalse(arbiter.submit(DriveRequest(ControlSource.GESTURE, 1f, 0f, 0f, 800), 20))
        arbiter.clear(ControlSource.MANUAL)
        assertNull(arbiter.current(30))
    }
    @Test fun rejectsOutOfRangeWithoutKeepingSameSourceMotion() {
        val arbiter = DriveArbiter()
        arbiter.submit(DriveRequest(ControlSource.GESTURE, 25f, 0f, 0f, 500), 0)
        assertFalse(arbiter.submit(DriveRequest(ControlSource.GESTURE, 2500f, 0f, 0f, 500), 1))
        assertNull(arbiter.current(2))
    }
}
