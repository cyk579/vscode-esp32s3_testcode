package cn.edu.gesturecar

import java.nio.ByteBuffer

object YuvRgb {
    fun convert(width: Int, height: Int,
        luma: ByteBuffer, lumaRow: Int, lumaPixel: Int,
        blue: ByteBuffer, blueRow: Int, bluePixel: Int,
        red: ByteBuffer, redRow: Int, redPixel: Int
    ): IntArray = IntArray(width * height) { index ->
        val row = index / width
        val column = index % width
        val luminance = (luma.get(luma.position() + row * lumaRow + column * lumaPixel).toInt() and 255) - 16
        val chromaBlue = (blue.get(blue.position() + row / 2 * blueRow + column / 2 * bluePixel).toInt() and 255) - 128
        val chromaRed = (red.get(red.position() + row / 2 * redRow + column / 2 * redPixel).toInt() and 255) - 128
        val scaled = 298 * luminance.coerceAtLeast(0)
        val redValue = ((scaled + 409 * chromaRed + 128) shr 8).coerceIn(0, 255)
        val greenValue = ((scaled - 100 * chromaBlue - 208 * chromaRed + 128) shr 8).coerceIn(0, 255)
        val blueValue = ((scaled + 516 * chromaBlue + 128) shr 8).coerceIn(0, 255)
        (255 shl 24) or (redValue shl 16) or (greenValue shl 8) or blueValue
    }
}
