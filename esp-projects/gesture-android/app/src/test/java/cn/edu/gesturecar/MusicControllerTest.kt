package cn.edu.gesturecar
import org.junit.Assert.*
import org.junit.Test

class MusicControllerTest {
    private fun readyCatalog(catalog: Long): ByteArray = ByteArray(16).also {
        it[0] = 1; it[3] = 3
        for (i in 0..3) it[12+i] = (catalog shr (8*i)).toByte()
    }
    @Test fun goldenMusicPacketMatchesFirmware() {
        assertArrayEquals(byteArrayOf(1,1,0x34,0x12,2,0,0,0,0x78,0x56,0x34,0x12), MusicProtocol.encode(1,0x1234,2,0x12345678))
    }
    @Test fun unknownSongOrWrongCatalogLeavesCurrentPlaybackUntouched() {
        val sent = mutableListOf<ByteArray>()
        val m = MusicController(listOf(MusicTrack(1,"测试音",listOf("测试音乐"))), 5, { sent += it; true }, {})
        m.link(true); m.acceptStatus(readyCatalog(6))
        assertFalse(m.submit(MediaCommand.PlayTitle("测试音")))
        m.acceptStatus(readyCatalog(5))
        assertFalse(m.submit(MediaCommand.PlayTitle("不存在的歌")))
        assertTrue(sent.isEmpty())
        assertTrue(m.submit(MediaCommand.PlayTitle("测试音乐")))
        assertEquals(1, sent.single()[1].toInt())
    }
    @Test fun movementModeSwitchEmergencyAndDisconnectDoNotSendMusicStop() {
        val sent = mutableListOf<ByteArray>()
        val m = MusicController(listOf(MusicTrack(1,"测试音",emptyList())), 5, { sent += it; true }, {})
        m.link(true); m.acceptStatus(readyCatalog(5)); m.submit(MediaCommand.PlayTitle("测试音"))
        val d = DriveController()
        d.select(ControlSource.VOICE, 0); d.emergency(true, 1); d.link(false, 2)
        m.link(false)
        assertEquals(1, sent.size)
        assertEquals(VoiceIntent.Drive(VoiceCommand.FORWARD), VoiceRouter.route("前进"))
        assertEquals(VoiceIntent.ResumeMusic, VoiceRouter.route("继续播放"))
        assertEquals(VoiceIntent.Unknown, VoiceRouter.route("不要停止音乐"))
    }
}
