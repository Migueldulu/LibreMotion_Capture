#include "AndroidUploader.h"
#include <android/log.h>

#define LOG_TAG "telemetria"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

//Para ver los errores de Java, se puede quitar el metodo entero pero los logs nunca estan de mas
static void logJavaException(JNIEnv* env, const char* where) {
    if (!env->ExceptionCheck()) return;
    jthrowable ex = env->ExceptionOccurred();
    env->ExceptionClear();

    jclass logCls = env->FindClass("android/util/Log");
    jmethodID getStack = env->GetStaticMethodID(logCls, "getStackTraceString",
                                                "(Ljava/lang/Throwable;)Ljava/lang/String;");
    jstring jstack = (jstring)env->CallStaticObjectMethod(logCls, getStack, ex);

    if (jstack) {
        const char* cstack = env->GetStringUTFChars(jstack, nullptr);
        __android_log_print(ANDROID_LOG_ERROR, "telemetria",
                            "Java exception at %s:\n%s", where, cstack);
        env->ReleaseStringUTFChars(jstack, cstack);
        env->DeleteLocalRef(jstack);
    } else {
        jclass thrCls = env->FindClass("java/lang/Throwable");
        jmethodID toString = env->GetMethodID(thrCls, "toString", "()Ljava/lang/String;");
        jstring jmsg = (jstring)env->CallObjectMethod(ex, toString);
        const char* cmsg = jmsg ? env->GetStringUTFChars(jmsg, nullptr) : "unknown";
        __android_log_print(ANDROID_LOG_ERROR, "telemetria",
                            "Java exception at %s: %s", where, cmsg);
        if (jmsg) { env->ReleaseStringUTFChars(jmsg, cmsg); env->DeleteLocalRef(jmsg); }
    }
}

void AndroidUploader::setJavaContext(JavaVM* vm, jobject activityGlobalRef) {
    vm_ = vm;
    activity_ = activityGlobalRef;
}

bool AndroidUploader::initialize(const UploaderConfig& cfg) {
    cfg_ = cfg;
    if (!vm_ || !activity_) {
        LOGE("AndroidUploader initialize without VM or Activity");
        return false;
    }
    return true;
}

void AndroidUploader::shutdown() {
    // Noop for now
}

bool AndroidUploader::uploadJson(const std::string& jsonBody) {
    if (!cfg_.enableCloudUpload) return true;
    if (cfg_.endpointUrl.empty() || cfg_.apiKey.empty()) {
        LOGE("Missing supabase config");
        return false;
    }
    // Build endpoint example. Adjust to your real REST path.
    std::string url = cfg_.endpointUrl;
    std::vector<std::pair<std::string,std::string>> headers = {
            {"Content-Type", "application/json"},
            {"apikey", cfg_.apiKey},
            {"Authorization", std::string("Bearer ") + cfg_.apiKey},
            {"Prefer", "return=representation"}
    };
    return callJavaMakeRequest("POST", url, jsonBody, headers);
}

bool AndroidUploader::callJavaMakeRequest(const std::string& method,
                                          const std::string& url,
                                          const std::string& body,
                                          const std::vector<std::pair<std::string,std::string>>& headers) {
    if (!vm_ || !activity_) return false;

    JNIEnv* env = nullptr;
    bool needDetach = false;
    if (vm_->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK || !env) {
        if (vm_->AttachCurrentThread(&env, nullptr) != JNI_OK || !env) {
            LOGE("AttachCurrentThread failed");
            return false;
        }
        needDetach = true;
    }

    jclass activityCls = env->GetObjectClass(activity_);
    if (!activityCls) {
        if (needDetach) vm_->DetachCurrentThread();
        LOGE("activity class not found");
        return false;
    }

    jmethodID getClassLoader = env->GetMethodID(activityCls, "getClassLoader", "()Ljava/lang/ClassLoader;");
    jobject loaderObj = env->CallObjectMethod(activity_, getClassLoader);
    jclass loaderCls = env->GetObjectClass(loaderObj);
    jmethodID loadClass = env->GetMethodID(loaderCls, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");

    jstring jName = env->NewStringUTF("io.github.migueldulu.telemetria.AyudanteHttp");
    jclass helperCls = (jclass)env->CallObjectMethod(loaderObj, loadClass, jName);
    env->DeleteLocalRef(jName);
    if (!helperCls) {
        if (needDetach) vm_->DetachCurrentThread();
        LOGE("AyudanteHttp class not found");
        return false;
    }

    // Signature: public static byte[] makeRequest(String method, String url, byte[] body, Map<String,String> headers)
    jmethodID makeReq = env->GetStaticMethodID(helperCls, "makeRequest",
                                               "(Ljava/lang/String;Ljava/lang/String;[BLjava/util/Map;)[B");
    if (!makeReq) {
        if (needDetach) vm_->DetachCurrentThread();
        LOGE("makeRequest method not found");
        return false;
    }

    jstring jMethod = env->NewStringUTF(method.c_str());
    jstring jUrl = env->NewStringUTF(url.c_str());

    jbyteArray jBody = nullptr;
    if (!body.empty()) {
        jBody = env->NewByteArray((jsize)body.size());
        env->SetByteArrayRegion(jBody, 0, (jsize)body.size(), reinterpret_cast<const jbyte*>(body.data()));
    }

    // Create HashMap headers
    jclass mapCls = env->FindClass("java/util/HashMap");
    jmethodID mapCtor = env->GetMethodID(mapCls, "<init>", "()V");
    jobject jMap = env->NewObject(mapCls, mapCtor);
    jmethodID mapPut = env->GetMethodID(mapCls, "put",
                                        "(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");
    for (const auto& kv : headers) {
        jstring k = env->NewStringUTF(kv.first.c_str());
        jstring v = env->NewStringUTF(kv.second.c_str());
        env->CallObjectMethod(jMap, mapPut, k, v);
        env->DeleteLocalRef(k);
        env->DeleteLocalRef(v);
    }

    jbyteArray jResp = (jbyteArray)env->CallStaticObjectMethod(helperCls, makeReq, jMethod, jUrl, jBody, jMap);
    logJavaException(env, "AndroidUploader.callJavaMakeRequest");

    if (jBody) env->DeleteLocalRef(jBody);
    env->DeleteLocalRef(jMethod);
    env->DeleteLocalRef(jUrl);
    env->DeleteLocalRef(jMap);

    if (jResp) {
        jsize len = env->GetArrayLength(jResp);
        std::string resp;
        resp.resize((size_t)len);
        env->GetByteArrayRegion(jResp, 0, len, reinterpret_cast<jbyte*>(&resp[0]));
        env->DeleteLocalRef(jResp);
        LOGI("HTTP response size: %d", (int)len);
        LOGI("HTTP response preview: %.200s", resp.c_str());
        if (resp.find("HTTP_STATUS:") != std::string::npos) {
            LOGE("Server returned error marker");
            if (needDetach) vm_->DetachCurrentThread();
            return false;
        }

    } else {
        LOGE("HTTP call returned null");
        if (needDetach) vm_->DetachCurrentThread();
        return false;
    }

    if (needDetach) vm_->DetachCurrentThread();
    return true;
}

static std::string urlJoin(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}

std::string AndroidUploader::makeSessionsUrl() const {
    // cfg_.endpointUrl debe ser algo como:
    // https://urldelproyecto.supabase.co/rest/v1/vr_movement_data
    // Cambiamos el ultimo segmento por vr_sessions y añadimos on_conflict
    const std::string marker = "/rest/v1/";
    auto pos = cfg_.endpointUrl.find(marker);
    if (pos == std::string::npos) return ""; // endpoint no esperado
    std::string base = cfg_.endpointUrl.substr(0, pos + marker.size()); // incluye .../rest/v1/
    std::string sessions = urlJoin(base, "vr_sessions");
    sessions += "?on_conflict=session_id";
    return sessions;
}

bool AndroidUploader::ensureSession() {
    if (cfg_.sessionId.empty()) {
        LOGE("ensureSession failed: empty sessionId");
        return false;
    }
    if (!cfg_.enableCloudUpload) return true;

    std::string url = makeSessionsUrl();
    if (url.empty()) {
        LOGE("ensureSession failed: could not build sessions url from endpoint");
        return false;
    }

    // la tabla de sesiones tambien envia el tiempo en el que se crea la sesion
    using namespace std::chrono;
    auto milisec = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::string tiempo = std::to_string(milisec);
    std::string infoDisp = cfg_.deviceInfo;
    std::string body = std::string("[{\"session_id\":\"") + cfg_.sessionId + "\",\"start_time\":\"" + tiempo + "\",\"device_info\":\"" + infoDisp + "\"}]";

    std::vector<std::pair<std::string,std::string>> headers = {
            {"Content-Type", "application/json"},
            {"apikey", cfg_.apiKey},
            {"Authorization", std::string("Bearer ") + cfg_.apiKey},
            {"Prefer", "resolution=merge-duplicates"}
    };

    LOGI("Ensuring session at URL: %s", url.c_str());
    bool ok = callJavaMakeRequest("POST", url, body, headers);
    if (!ok) {
        LOGE("ensureSession failed for session_id=%s", cfg_.sessionId.c_str());
    } else {
        LOGI("ensureSession ok for session_id=%s", cfg_.sessionId.c_str());
    }
    return ok;
}