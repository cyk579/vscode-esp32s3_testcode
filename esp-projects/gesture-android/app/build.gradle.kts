plugins { id("com.android.application"); id("org.jetbrains.kotlin.android") }
android {
    namespace = "cn.edu.gesturecar"
    compileSdk = 35
    defaultConfig {
        applicationId = "cn.edu.gesturecar"
        minSdk = 26
        targetSdk = 35
        versionCode = 4
        versionName = "1.3-asr-music"
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
}
dependencies { testImplementation("junit:junit:4.13.2") }
tasks.withType<Test>().configureEach {
    defaultCharacterEncoding = "UTF-8"
}
