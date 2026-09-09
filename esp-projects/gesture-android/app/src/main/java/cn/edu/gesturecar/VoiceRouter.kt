package cn.edu.gesturecar

sealed interface VoiceIntent {
    data class Drive(val command: VoiceCommand) : VoiceIntent
    data class Play(val title: String) : VoiceIntent
    data object PauseMusic : VoiceIntent
    data object ResumeMusic : VoiceIntent
    data object StopMusic : VoiceIntent
    data object Unknown : VoiceIntent
}

/** Routes final speech text without touching playback state when the intent is movement. */
object VoiceRouter {
    fun route(text: String): VoiceIntent {
        val normalized = text.trim().trimEnd('。', '！', '!', '.', '？', '?').trim()
        when (normalized) {
            "暂停音乐", "暂停播放" -> return VoiceIntent.PauseMusic
            "继续音乐", "继续播放" -> return VoiceIntent.ResumeMusic
            "停止音乐", "停止播放" -> return VoiceIntent.StopMusic
        }
        VoiceCommand.parse(text)?.let { return VoiceIntent.Drive(it) }
        val title = when {
            normalized.startsWith("播放歌曲") -> normalized.removePrefix("播放歌曲").trim()
            normalized.startsWith("播放") -> normalized.removePrefix("播放").trim()
            else -> ""
        }
        return if (title.isNotEmpty()) VoiceIntent.Play(title) else VoiceIntent.Unknown
    }
}
