package cn.edu.gesturecar

/** Input sources are independent producers. The arbiter is the only movement owner. */
enum class ControlSource(val priority: Int) { GESTURE(1), VOICE(2), MANUAL(3) }

data class DriveRequest(
    val source: ControlSource,
    val forward: Float,
    val lateral: Float,
    val rotation: Float,
    val expiresAtMs: Long
)

/** Deterministic movement arbitration. Music commands never enter this class. */
class DriveArbiter {
    private var active: DriveRequest? = null

    fun submit(request: DriveRequest, nowMs: Long): Boolean {
        if (request.expiresAtMs <= nowMs || listOf(request.forward, request.lateral, request.rotation).any { !it.isFinite() || it !in -25f..25f }) {
            clear(request.source); return false
        }
        val current = current(nowMs)
        if (current != null && current.source.priority > request.source.priority) return false
        active = request // Replaced requests are discarded, never queued for later resumption.
        return true
    }

    fun current(nowMs: Long): DriveRequest? {
        if (active?.expiresAtMs?.let { it <= nowMs } == true) active = null
        return active
    }

    fun clear(source: ControlSource) { if (active?.source == source) active = null }
    fun clearAll() { active = null }
}

sealed interface MediaCommand {
    data class PlayTitle(val title: String) : MediaCommand
    data object Pause : MediaCommand
    data object Resume : MediaCommand
    data object Stop : MediaCommand
}

/** MediaFeature consumes this separately from DriveArbiter; a new drive request does not stop it. */
fun interface MediaSink { fun submit(command: MediaCommand): Boolean }

/** Gesture implementations never receive a BluetoothGatt or a music controller. */
interface GestureOutput {
    fun motion(forward: Float, lateral: Float, rotation: Float, capturedAtMs: Long)
    fun lost()
    fun fist() {}
}
interface GestureFeature {
    fun start(output: GestureOutput)
    fun stop()
}
