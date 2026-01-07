#include "TelemetryAPI.h"
#include "TelemetryManager.h"
#include "AndroidUploader.h"
#include <android/log.h>
#include <mutex>
#include "configReader.h"
#include "C3DRecorder.h"

#define LOG_TAG "telemetria"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// Minimal global state to keep eveerything as simple as posssible and smooth performance on the app it will be used
static JavaVM* g_vm = nullptr;
static jobject g_activity = nullptr;
static std::mutex g_mutex;

// singletons for the internal components managed through the api
static GestorTelemetria g_gestor;
static AndroidUploader g_uploader;
static C3DRecorder     g_c3d;
//Cached feautres flags to be exposed
static unsigned g_featureFlags = 0;

// Capture JavaVM when the library is loaded (this allow threads to attach later and obtain JNIEnv)
jint JNI_OnLoad(JavaVM* vm, void*) {
    // Captura JavaVM en carga de la libreria
    std::lock_guard<std::mutex> lock(g_mutex);
    g_vm = vm;
    return JNI_VERSION_1_6;
}

extern "C" {

void telemetry_set_java_context(JavaVM* vm, jobject activity) {
    std::lock_guard<std::mutex> lock(g_mutex);
    // if VM provided, override the global one
    if (vm) g_vm = vm;
    // If we already have a global Activity ref, delete it first.
    if (g_activity) {
        JNIEnv* env = nullptr;
        if (g_vm && g_vm->GetEnv((void**)&env, JNI_VERSION_1_6) == JNI_OK && env) {
            env->DeleteGlobalRef(g_activity);
        }
        g_activity = nullptr;
    }
    // Create new global ref from the passed Activity, if any.
    if (activity) {
        if (!g_vm) {
            LOGE("telemetry_set_java_context called without JavaVM");
            return;
        }
        JNIEnv* env = nullptr;
        if (g_vm->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK || !env) {
            LOGE("GetEnv failed");
            return;
        }
        g_activity = env->NewGlobalRef(activity);
    }
    LOGI("Java context set");
}

int telemetry_initialize(const TelemetryConfigPlain* cfg) {
    std::lock_guard<std::mutex> lock(g_mutex);
    // if no config provided, fail
    if (!cfg) return -1;
    // java context must have been set
    if (!g_vm || !g_activity) {
        LOGE("Java context not set");
        return -2;
    }
    // Contexto Java para el uploader
    g_uploader.setJavaContext(g_vm, g_activity);

    // Configure uploader with session and device info from the engine
    UploaderConfig ucfg;
    ucfg.sessionId   = cfg->sessionId ? cfg->sessionId : "";
    ucfg.deviceInfo = cfg->deviceInfo ? cfg->deviceInfo : "";

    // Load the rest of the configuration from initialConfig.json:
    // endpoint, apiKey, framesPerFile and feature flags.
    const bool cfgOk = configReader::setConfig(ucfg);
    if (!cfgOk) {
        LOGI("Config file not found or unreadable; using defaults.");
    }

    LOGI("config flags: hand=%d primary=%d secondary=%d grip=%d trigger=%d joystick=%d",
         (int)ucfg.handTracking, (int)ucfg.primaryButton, (int)ucfg.secondaryButton,
         (int)ucfg.grip, (int)ucfg.trigger, (int)ucfg.joystick);

    LOGI("config final: endpointUrl='%s', apiKey.len=%d, framesPerFile=%d, sessionId='%s', deviceInfo.len=%d",
         ucfg.endpointUrl.c_str(), (int)ucfg.apiKey.size(), ucfg.framesPerFile, ucfg.sessionId.c_str(), (int)ucfg.deviceInfo.size());

    //Initialize the HTTP uploader (JNI bridge to Java helper)
    if (!g_uploader.initialize(ucfg)) {
        LOGE("Uploader initialize failed");
        return -3;
    }
    // Initialize the telemetry manager (buffering + background uploads).
    if (!g_gestor.initialize(ucfg, &g_uploader)) {
        LOGE("Gestor initialize failed");
        return -4;
    }
    // Cache feature flags for the engine (telemetry_get_feature_flags).
    g_featureFlags = configReader::getFeatureFlagsBitmask(ucfg);

    LOGI("telemetry initialized");

    int frameRate = 60;
    configReader::getFrameRate(frameRate);
    // Check if frame rate is inside a safe [1, 240] range. If outside, fallback to 60
    if (frameRate < 1 || frameRate > 240) frameRate = 60;
    // Initialize C3D recorder. Even if it fails, HTTP telemetry will still work.
    if (!g_c3d.C3Dinitialize(ucfg, frameRate)) {
        LOGE("C3DRecorder initialize failed; continuing without C3D output");
        // No crash error bc we still have HTTP telemetry
    }
    return frameRate;
}

unsigned telemetry_get_feature_flags() {
    // Simple read of a 32-bit value
    return g_featureFlags;
}

void telemetry_record_frame(const VRFrameDataPlain* frame) {
    if (!frame) return;
    // Append the frame to the C3D buffer.
    g_c3d.C3DrecordFrame(*frame);
    // Append the frame to the JSON buffer.
    g_gestor.recordFrame(*frame);
}

void telemetry_force_upload() {
    // Ask GestorTelemetria to close the current buffer and enqueue it for upload, even if framesPerFile was not reached yet.
    g_gestor.flushAndUpload();
}

void telemetry_shutdown() {
    std::lock_guard<std::mutex> lock(g_mutex);
    // Finalize C3D file
    g_c3d.C3Dfinalize();
    // Flush JSON chunks and stop worker thread.
    g_gestor.shutdown();
    // Placeholder for any future uploader teardown.
    g_uploader.shutdown();
    // Release the global Activity reference (if any).
    if (g_activity) {
        JNIEnv* env = nullptr;
        if (g_vm && g_vm->GetEnv((void**)&env, JNI_VERSION_1_6) == JNI_OK && env) {
            env->DeleteGlobalRef(g_activity);
        }
        g_activity = nullptr;
    }
    LOGI("telemetry shutdown complete");
}

} // extern "C"
