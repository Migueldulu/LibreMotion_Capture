#include "AndroidUploader.h"
#include <android/log.h>

#define LOG_TAG "telemetria"
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)

// Helper to log full Java exceptions to Logcat. Can be deleted but useful for debugging.
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

// Store VM and Activity references for later use by JNI calls.
void AndroidUploader::setJavaContext(JavaVM* vm, jobject activityGlobalRef) {
    vm_ = vm;
    activity_ = activityGlobalRef;
}

// Initialize uploader with configuration.
// It just copies cfg into cfg_ and checks that we have both a VM and an Activity
bool AndroidUploader::initialize(const UploaderConfig& cfg) {
    cfg_ = cfg;
    if (!vm_ || !activity_) {
        LOGE("AndroidUploader initialize without VM or Activity");
        return false;
    }
    return true;
}

// Currently placeholder; kept for symmetry with other components lifecycle.
void AndroidUploader::shutdown() {
    // Noop for now
}

// Upload a JSON load to the configured endpoint.
// Builds headers (Content-Type, apikey, Authorization) and delegates the actual HTTP call to callJavaMakeRequest using POST.
bool AndroidUploader::uploadJson(const std::string& jsonBody) {
    // Must contain valid endpoint URL and API key
    if (cfg_.endpointUrl.empty() || cfg_.apiKey.empty()) {
        LOGE("Missing supabase config");
        return false;
    }
    std::string url = cfg_.endpointUrl;
    LOGI("uploadJson: url=%s body.size=%d", url.c_str(), (int)jsonBody.size());

    // Build the HTTP headers for typical backend:
    //   Content-Type: application/json
    //   apikey: <api key>
    //   Authorization: Bearer <api key>
    std::vector<std::pair<std::string,std::string>> headers = {
            {"Content-Type", "application/json"},
            {"apikey", cfg_.apiKey},
            {"Authorization", std::string("Bearer ") + cfg_.apiKey},
            {"Prefer", "return=representation"}
            // Here can be added more headers here if needed (e.g. Prefer: return=minimal)
    };
    return callJavaMakeRequest("POST", url, jsonBody, headers);
}

// Core JNI bridge that calls the Java static method:
//   byte[] AyudanteHttp.makeRequest(String method, String url, byte[] body, Map<String,String> headers)
bool AndroidUploader::callJavaMakeRequest(const std::string& method,
                                          const std::string& url,
                                          const std::string& body,
                                          const std::vector<std::pair<std::string,std::string>>& headers) {
    if (!vm_ || !activity_) return false;

    JNIEnv* env = nullptr;
    bool needDetach = false;
    // Try to get JNIEnv for the current thread; if not attached, attach it.
    if (vm_->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK || !env) {
        if (vm_->AttachCurrentThread(&env, nullptr) != JNI_OK || !env) {
            LOGE("AttachCurrentThread failed");
            return false;
        }
        needDetach = true;
    }

    //try to get the class from the activity
    jclass activityCls = env->GetObjectClass(activity_);
    if (!activityCls) {
        if (needDetach) vm_->DetachCurrentThread();
        LOGE("activity class not found");
        return false;
    }

    // loading the class
    jmethodID getClassLoader = env->GetMethodID(activityCls, "getClassLoader", "()Ljava/lang/ClassLoader;");
    jobject loaderObj = env->CallObjectMethod(activity_, getClassLoader);
    jclass loaderCls = env->GetObjectClass(loaderObj);
    jmethodID loadClass = env->GetMethodID(loaderCls, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
    // create java string with the class name and delete the local ref, exit if class not found
    jstring jName = env->NewStringUTF("io.github.migueldulu.telemetria.AyudanteHttp");
    jclass helperCls = (jclass)env->CallObjectMethod(loaderObj, loadClass, jName);
    env->DeleteLocalRef(jName);
    if (!helperCls) {
        if (needDetach) vm_->DetachCurrentThread();
        LOGE("AyudanteHttp class not found");
        return false;
    }

    // Signature: public static byte[] makeRequest(String method, String url, byte[] body, Map<String,String> headers) ([B = array de bytes)
    jmethodID makeReq = env->GetStaticMethodID(helperCls, "makeRequest",
                                               "(Ljava/lang/String;Ljava/lang/String;[BLjava/util/Map;)[B");
    if (!makeReq) {
        if (needDetach) vm_->DetachCurrentThread();
        LOGE("makeRequest method not found");
        return false;
    }

    jstring jMethod = env->NewStringUTF(method.c_str());
    jstring jUrl = env->NewStringUTF(url.c_str());

    // copies the string to the java array
    jbyteArray jBody = nullptr;
    if (!body.empty()) {
        jBody = env->NewByteArray((jsize)body.size());
        env->SetByteArrayRegion(jBody, 0, (jsize)body.size(), reinterpret_cast<const jbyte*>(body.data()));
    }

    // Create HashMap headers
    jclass mapCls = env->FindClass("java/util/HashMap");
    jmethodID mapCtor = env->GetMethodID(mapCls, "<init>", "()V");
    jobject jMap = env->NewObject(mapCls, mapCtor);
    //obtain method and for each header creates java string and adds it to the map
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

    //clean any local ref created
    if (jBody) env->DeleteLocalRef(jBody);
    env->DeleteLocalRef(jMethod);
    env->DeleteLocalRef(jUrl);
    env->DeleteLocalRef(jMap);

    // if theres response obtain its lenght and create a new string
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
    // Finally, detach thread if it was attached in this function
    if (needDetach) vm_->DetachCurrentThread();
    return true;
}

// Small helper to join URL parts, avoiding duplicated slashes.
// Example: urlJoin("https://example.com/api", "frames2") -> "https://example.com/api/frames2"
static std::string urlJoin(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}


