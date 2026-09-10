package cn.edu.gesturecar

/** Shared motion safety and source ownership. All calls occur on the main thread. */
class DriveController {
    private val arbiter = DriveArbiter()
    var selected = ControlSource.MANUAL
        private set
    var emergency = false
        private set
    private var connected = false
    private var foreground = false
    private var state = -1
    private var fault = true
    private var statusAt = Long.MIN_VALUE
    private var neutralUntil = 0L
    private var sequence = 0
    private var armed = false
    private var wasDriving = false

    fun select(source: ControlSource, now: Long) { selected = source; stop(now) }
    fun link(value: Boolean, now: Long) {
        connected = value; state = -1; fault = true; statusAt = Long.MIN_VALUE; stop(now)
    }
    fun foreground(value: Boolean, now: Long) { foreground = value; if (!value) stop(now) }
    fun emergency(value: Boolean, now: Long) { emergency = value; stop(now) }
    fun stop(now: Long) { arbiter.clearAll(); armed = false; wasDriving = false; neutralUntil = now + 350 }
    fun updateStatus(bytes: ByteArray, now: Long) {
        if (bytes.size != 12 || bytes[0].toInt() != 1) { fault = true; stop(now); return }
        state = bytes[1].toInt() and 255
        fault = bytes[2].toInt() != 0 || bytes[11].toInt() != 0
        statusAt = now
        if (fault || state !in 2..3) stop(now)
    }
    private fun available(now: Long) = foreground && connected && !emergency && !fault &&
        statusAt != Long.MIN_VALUE && now - statusAt in 0..500

    fun begin(source: ControlSource, now: Long): Boolean {
        if (source != selected || !available(now) || now < neutralUntil || state != 2) return false
        armed = true; return true
    }

    fun submit(source: ControlSource, forward: Float, lateral: Float, yaw: Float, capturedAt: Long, now: Long,
               durationMs: Long? = null): Boolean {
        val maxAge = if (source == ControlSource.GESTURE) 150L else 100L
        if (!armed || source != selected || !available(now) || now < neutralUntil || now - capturedAt !in 0..maxAge) return false
        val active = arbiter.current(now)
        if (wasDriving && active == null) { stop(now); return false }
        if (active == null && state != 2) return false
        val ttl = durationMs ?: if (source == ControlSource.VOICE) 800L else 200L
        if (ttl <= 0L) return false
        val accepted = arbiter.submit(DriveRequest(source, forward, lateral, yaw, capturedAt + ttl), now)
        if (accepted) wasDriving = true
        return accepted
    }
    fun output(now: Long): DriveRequest? {
        if (!available(now)) { if (armed) stop(now); return null }
        val request = arbiter.current(now)
        if (wasDriving && request == null) stop(now)
        return request
    }
    fun frame(now: Long): ByteArray {
        val request = output(now)
        return Protocol.encode(++sequence, now, request?.forward ?: 0f, request?.lateral ?: 0f,
            request?.rotation ?: 0f, foreground, request != null, emergency)
    }
}
