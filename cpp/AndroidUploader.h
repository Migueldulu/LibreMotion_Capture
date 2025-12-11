#pragma once
#include <jni.h>
#include <string>
#include <vector>
#include "TiposTelemetria.h"

// Uploader that calls the Java helper AyudanteHttp.makeRequest via JNI.
// This class does NOT perform HTTP by itself; it just bridges between
// C++ and Java, passing method, URL, body and headers to Java.
class AndroidUploader {
public:
    AndroidUploader() = default;
    ~AndroidUploader() = default;

    // Stores the Java VM pointer and the global Activity reference.
    // required later to obtain a JNIEnv* and to get the Activity class loader used to load the AyudanteHttp class.
    void setJavaContext(JavaVM* vm, jobject activityGlobalRef);

    //    // Initializes the uploader with the given configuration (endpoint URL, API key and other telemetry options)
    //    // Returns false if vm_ or activity_ were not provided before, or if basic configuration is missing.
    bool initialize(const UploaderConfig& cfg);

    // Right now its a placeholder to mirror the other components, but no need yet due to java/android automatic shutdown
    void shutdown();

    // Sends a JSON payload to the configured endpoint.
    // Returns true if the Java call was executed successfully, false if configuration is missing or if the JNI call fails.
    bool uploadJson(const std::string& jsonBody);

private:
    // Cached Java VM pointer, used to attach/detach threads and obtain JNIEnv*.
    JavaVM* vm_ = nullptr;
    jobject activity_ = nullptr; // global ref, not owned

    // Copy of the uploader configuration (endpoint URL, API key, flags).
    UploaderConfig cfg_;

    // Internal helper that performs the actual JNI call:
    // - method: HTTP method ("POST", "GET", etc.).
    // - url: full URL for the request.
    // - body: request body as a UTF-8 string (converted to byte[] in Java).
    // - headers: key-value pairs for HTTP headers.
    // This function attaches the thread to the JVM if necessary, calls AyudanteHttp.makeRequest, and detaches the thread if it was attached.
    bool callJavaMakeRequest(const std::string& method,
                             const std::string& url,
                             const std::string& body,
                             const std::vector<std::pair<std::string,std::string>>& headers);


};
