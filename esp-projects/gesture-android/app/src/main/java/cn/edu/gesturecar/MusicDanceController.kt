package cn.edu.gesturecar

class MusicDanceController(private val drive: DriveController) {
    var active = false
        private set
    var message = ""
        private set
    private var rotating = false
    private var track = 0
    private var sequence = 0
    private var catalog = 0L
    private var requestedAt = 0L
    private var statusAt = 0L
    private var tickAt = 0L
    private var progressAt = 0L
    private var seconds = -1L
    private var deadline = 0L
    private var durationMs = 0L

    fun start(music: MusicController, now: Long): Boolean {
        if (active) cancel(now)
        val song = music.danceTrack
        if (song == null) {
            drive.stop(now)
            message = "伴奏未启动：APK 未包含有效片段"
            return false
        }
        if (!drive.begin(ControlSource.VOICE, now)) {
            drive.stop(now)
            message = "伴奏未启动：请切换到语音模式，并等待车辆 READY"
            return false
        }
        if (!music.playDance()) {
            drive.stop(now)
            message = "伴奏未启动：${music.danceAvailability()}"
            return false
        }
        active = true; rotating = false
        track = song.id; durationMs = song.durationMs
        sequence = music.lastSentSequence; catalog = music.catalog
        requestedAt = now; tickAt = now; seconds = -1
        message = "等待片段播放确认，尚未旋转"
        return true
    }

    fun cancel(now: Long, reason: String = "伴奏旋转已取消，保持停车") {
        if (!active) return
        active = false; rotating = false
        drive.stop(now)
        message = reason
    }

    fun accept(status: MusicStatus?, now: Long) {
        if (!active) return
        if (status == null) { cancel(now, "音乐状态包无效，已停车"); return }
        if (status.catalog != catalog) { cancel(now, "曲库编号不一致，已停车"); return }
        if (status.error != 0) {
            val reason = when (status.error) {
                1 -> "USB 扬声器断开或未就绪"
                2 -> "车端音乐分区未挂载"
                3 -> "曲库编号不一致"
                4 -> "车端缺少片段文件 5.wav"
                5 -> "片段 WAV 格式不匹配"
                6 -> "车端音频读取或 USB 传输失败"
                else -> "未知音乐错误 ${status.error}"
            }
            cancel(now, "$reason，已停车"); return
        }
        if (status.flags and 3 != 3) { cancel(now, "USB 扬声器或曲库未就绪（flags=${status.flags}），已停车"); return }
        if (!rotating && status.sequence != sequence) return
        if (status.sequence != sequence || status.track != track || status.state != 1) {
            cancel(now, "伴奏结束或被中断，已停车"); return
        }
        if (!rotating) {
            if (status.seconds == 0L) return
            if (now - requestedAt !in 0..3000 || !drive.begin(ControlSource.VOICE, now)) {
                cancel(now); return
            }
            rotating = true; deadline = now + durationMs + 2000; progressAt = now
            message = "伴奏旋转中，音乐结束自动停车"
        }
        if (status.seconds < seconds) { cancel(now); return }
        if (status.seconds > seconds) { seconds = status.seconds; progressAt = now }
        statusAt = now
    }

    fun tick(now: Long) {
        if (!active) return
        if (drive.selected != ControlSource.VOICE || drive.emergency || now - tickAt !in 0..250) {
            cancel(now); return
        }
        tickAt = now
        if (!rotating) {
            if (now - requestedAt !in 0..3000) cancel(now, "播放确认超时，未启动旋转")
            return
        }
        if (now - statusAt !in 0..1500 || now - progressAt !in 0..2500 || now >= deadline) {
            cancel(now, "音乐状态超时，已停车"); return
        }
        if (!drive.submit(ControlSource.VOICE, 0f, 0f, -25f, now, now, 200L)) cancel(now)
    }
}
