plugins { id("com.android.application"); id("org.jetbrains.kotlin.android") }
android {
    namespace = "cn.edu.gesturecar"
    compileSdk = 35
    defaultConfig {
        applicationId = "cn.edu.gesturecar"
        minSdk = 26
        targetSdk = 35
        versionCode = 10
        versionName = "1.9-ks21"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
}
dependencies {
    implementation("androidx.annotation:annotation:1.8.2")
    implementation("com.google.mediapipe:tasks-vision:0.10.14")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test:runner:1.6.2")
    androidTestImplementation("junit:junit:4.13.2")
}
tasks.withType<Test>().configureEach {
    defaultCharacterEncoding = "UTF-8"
}
