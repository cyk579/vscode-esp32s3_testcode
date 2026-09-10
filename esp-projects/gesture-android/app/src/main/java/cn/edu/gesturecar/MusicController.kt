package cn.edu.gesturecar

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.UUID

data class MusicTrack(val id: Int, val title: String, val aliases: List<String>,
                      val danceOnly: Boolean = false, val durationMs: Long = 0)
data class MusicStatus(val state: Int, val error: Int, val flags: Int, val sequence: Int,
                       val track: Int, val seconds: Long, val catalog: Long)
object MusicProtocol {
    val SERVICE: UUID = UUID.fromString("7f520001-1b15-4b85-9c13-8f08604a0001")
    val COMMAND: UUID = UUID.fromString("7f520002-1b15-4b85-9c13-8f08604a0001")
    val STATUS: UUID = UUID.fromString("7f520003-1b15-4b85-9c13-8f08604a0001")
    fun encode(op: Int, sequence: Int, track: Int, catalog: Long): ByteArray =
        ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN).put(1).put(op.toByte())
            .putShort(sequence.toShort()).putShort(track.toShort()).putShort(0).putInt(catalog.toInt()).array()
    fun decode(bytes: ByteArray): MusicStatus? {
        if (bytes.size != 16 || bytes[0].toInt() != 1) return null
        val b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
        return MusicStatus(bytes[1].toInt() and 255, bytes[2].toInt() and 255, bytes[3].toInt() and 255,
            b.getShort(4).toInt() and 65535, b.getShort(6).toInt() and 65535,
            b.getInt(8).toLong() and 0xffffffffL, b.getInt(12).toLong() and 0xffffffffL)
    }
}

/** Owns track resolution and media commands; no dependency on driving or speech recognition. */
class MusicController(val tracks: List<MusicTrack>, val catalog: Long,
                      private val send: (ByteArray) -> Boolean, private val report: (String) -> Unit) : MediaSink {
    private var sequence = 0
    var lastSentSequence = 0
        private set
    val selectableTracks: List<MusicTrack> get() = tracks.filterNot { it.danceOnly }
    val danceTrack: MusicTrack? get() = tracks.singleOrNull { it.danceOnly && it.durationMs in 1..180000 }
    fun danceAvailability(): String = when {
        !supported -> "车端未提供音乐服务"
        danceTrack == null -> "APK 未包含伴奏片段"
        status == null -> "正在等待车端音乐状态"
        status!!.catalog != catalog -> "手机与车端曲库编号不一致"
        status!!.flags and 3 != 3 -> "USB 扬声器或车端曲库尚未就绪"
        status!!.error != 0 -> "车端音乐错误 ${status!!.error}"
        else -> ""
    }
    private var supported = false
    private var status: MusicStatus? = null
    fun link(available: Boolean) { supported = available; status = null }
    fun resolve(title: String): MusicTrack? = selectableTracks.singleOrNull { it.title == title || title in it.aliases }
    fun acceptStatus(bytes: ByteArray) {
        status = MusicProtocol.decode(bytes)
        val s = status ?: return report("音乐状态格式不兼容")
        val title = tracks.find { it.id == s.track }?.title ?: "未选曲"
        val state = listOf("已停止", "播放中", "已暂停", "不可播放").getOrNull(s.state) ?: "未知状态"
        val error = listOf("", "USB 扬声器不可用", "曲库分区未就绪", "手机与车端曲库不一致", "车端缺少歌曲文件", "WAV 格式不匹配", "音频读取或传输失败").getOrNull(s.error) ?: "未知错误"
        report("$title · $state · ${s.seconds} 秒" + if (error.isNotEmpty()) "\n$error" else "")
    }
    override fun submit(command: MediaCommand): Boolean {
        return submitTrack(command, if (command is MediaCommand.PlayTitle) resolve(command.title) else null)
    }
    fun playDance(): Boolean {
        val track = danceTrack ?: return false
        return submitTrack(MediaCommand.PlayTitle(track.title), track)
    }
    private fun submitTrack(command: MediaCommand, track: MusicTrack?): Boolean {
        if (!supported) { report("车端不支持音乐服务，请使用 subject3-ASR 音乐固件"); return false }
        if (command is MediaCommand.PlayTitle) {
            if (track == null) { report("曲库中没有“${command.title}”，当前播放保持不变"); return false }
            val s = status
            if (s == null || s.catalog != catalog || (s.flags and 3) != 3) {
                report("等待扬声器和曲库就绪；手机与车端必须使用同一份曲库"); return false
            }
        }
        val op = when (command) { is MediaCommand.PlayTitle -> 1; MediaCommand.Pause -> 2; MediaCommand.Resume -> 3; MediaCommand.Stop -> 4 }
        sequence = (sequence + 1) and 65535
        val sent = send(MusicProtocol.encode(op, sequence, track?.id ?: 0, catalog))
        if (sent) lastSentSequence = sequence
        report(if (sent) "音乐指令已排队，等待车端状态" else "音乐通道忙，本次未发送")
        return sent
    }
}
