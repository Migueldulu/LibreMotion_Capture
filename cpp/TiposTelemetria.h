#pragma once
#include <string>

// Configuracion para uploader y gestor
struct UploaderConfig {
    std::string endpointUrl;
    std::string apiKey;
    std::string sessionId;
    std::string deviceInfo;
    bool enableCloudUpload = true;
    bool enableLocalBackup = true;
    int framesPerFile = 5400;
};

// Estructura plain para marshalling desde C#
struct TelemetryConfigPlain {
    const char* endpointUrl;   // UTF-8
    const char* apiKey;        // UTF-8
    const char* sessionId;     // UTF-8
    const char* deviceInfo;     // UTF-8
    int enableCloudUpload;     // 0 o 1
    int enableLocalBackup;     // 0 o 1
    int framesPerFile;         // por ejemplo 5400
};