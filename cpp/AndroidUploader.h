#pragma once
#include <jni.h>
#include <string>
#include <vector>
#include "TiposTelemetria.h"

// Uploader that calls AyudanteHttp.makeRequest in Java via JNI
class AndroidUploader {
public:
    AndroidUploader() = default;
    ~AndroidUploader() = default;

    void setJavaContext(JavaVM* vm, jobject activityGlobalRef);
    bool initialize(const UploaderConfig& cfg);
    void shutdown();

    bool uploadJson(const std::string& jsonBody);

    // Este metodo es porque mi base de datos pide iniciar sesion antes de empezar a enviar datos.
    bool ensureSession();

private:
    JavaVM* vm_ = nullptr;
    jobject activity_ = nullptr; // global ref, not owned
    UploaderConfig cfg_;

    bool callJavaMakeRequest(const std::string& method,
                             const std::string& url,
                             const std::string& body,
                             const std::vector<std::pair<std::string,std::string>>& headers);

    // Construye la URL de vr_sessions a partir del endpoint de la tabla de frames
    std::string makeSessionsUrl() const;
};
