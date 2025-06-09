plugins {
    id("java-library")
}
java {
    sourceCompatibility = JavaVersion.VERSION_11
    targetCompatibility = JavaVersion.VERSION_11
}
dependencies {
    implementation(files("libs\\opencv-4110.jar"))

    // https://mvnrepository.com/artifact/org.apache.commons/commons-lang3
    implementation(libs.commons.lang3)
    implementation(libs.commons.math3)
    implementation(libs.commons.geometry.euclidean)

}
