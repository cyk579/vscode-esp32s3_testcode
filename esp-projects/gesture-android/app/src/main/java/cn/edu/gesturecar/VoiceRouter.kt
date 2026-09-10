package cn.edu.gesturecar

sealed interface VoiceIntent {
    data class Drive(val command: VoiceCommand) : VoiceIntent
    data class Sequence(val commands: List<VoiceCommand>) : VoiceIntent
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
        if (title.isNotEmpty()) return VoiceIntent.Play(title)
        val parts = normalized.removePrefix("先").split(Regex("\\s*(?:[，,、；;]\\s*)?(?:然后|接着|再)\\s*|\\s*[，,、；;]\\s*"))
        if (parts.size !in 2..6) return VoiceIntent.Unknown
        val commands = parts.map { part ->
            val action = part.trim().removePrefix("最后").trim()
            if (action == "旋转" || action == "原地旋转") VoiceCommand.TURN_RIGHT
            else VoiceCommand.parse(action) ?: return VoiceIntent.Unknown
        }
        if (commands.any { it == VoiceCommand.ESTOP }) return VoiceIntent.Unknown
        if (commands.dropLast(1).any { it == VoiceCommand.STOP }) return VoiceIntent.Unknown
        return VoiceIntent.Sequence(commands)
    }
}
