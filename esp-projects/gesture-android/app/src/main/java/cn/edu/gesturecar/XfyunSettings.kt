package cn.edu.gesturecar

import android.content.Context
import android.security.keystore.KeyGenParameterSpec
import android.security.keystore.KeyProperties
import java.security.KeyStore
import java.util.Base64
import javax.crypto.Cipher
import javax.crypto.KeyGenerator
import javax.crypto.SecretKey
import javax.crypto.spec.GCMParameterSpec
import org.json.JSONObject

class XfyunSettings(context: Context) {
    private val preferences = context.getSharedPreferences("xfyun_private", Context.MODE_PRIVATE)
    private val alias = "gesturecar.xfyun.credentials"
    private fun key(): SecretKey {
        val store = KeyStore.getInstance("AndroidKeyStore").apply { load(null) }
        (store.getKey(alias, null) as? SecretKey)?.let { return it }
        return KeyGenerator.getInstance(KeyProperties.KEY_ALGORITHM_AES, "AndroidKeyStore").apply {
            init(KeyGenParameterSpec.Builder(alias, KeyProperties.PURPOSE_ENCRYPT or KeyProperties.PURPOSE_DECRYPT)
                .setBlockModes(KeyProperties.BLOCK_MODE_GCM)
                .setEncryptionPaddings(KeyProperties.ENCRYPTION_PADDING_NONE).build())
        }.generateKey()
    }
    fun load(): XfyunCredentials? = try {
        preferences.getString("encrypted", null)?.let { stored ->
            val bytes = Base64.getDecoder().decode(stored)
            val cipher = Cipher.getInstance("AES/GCM/NoPadding").apply {
                init(Cipher.DECRYPT_MODE, key(), GCMParameterSpec(128, bytes.copyOfRange(0, 12)))
            }
            val json = JSONObject(String(cipher.doFinal(bytes.copyOfRange(12, bytes.size)), Charsets.UTF_8))
            XfyunCredentials(json.getString("appId"), json.getString("apiKey"), json.getString("apiSecret"))
                .takeIf { it.valid() }
        }
    } catch (_: Exception) { null }
    fun save(credentials: XfyunCredentials) {
        require(credentials.valid())
        val json = JSONObject().put("appId", credentials.appId).put("apiKey", credentials.apiKey)
            .put("apiSecret", credentials.apiSecret).toString()
        val cipher = Cipher.getInstance("AES/GCM/NoPadding").apply { init(Cipher.ENCRYPT_MODE, key()) }
        val bytes = cipher.iv + cipher.doFinal(json.toByteArray(Charsets.UTF_8))
        check(preferences.edit().putString("encrypted", Base64.getEncoder().encodeToString(bytes)).commit())
    }
}
