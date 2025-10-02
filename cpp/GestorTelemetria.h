#pragma once
#include <vector>
#include <string>
#include <mutex>
#include "TiposTelemetria.h"
#include "TiposVR.h"

class AndroidUploader;

// Gestor basico de buffering y disparo de subida
class GestorTelemetria {
public:
    GestorTelemetria();
    ~GestorTelemetria();

    bool initialize(const UploaderConfig& cfg, AndroidUploader* uploader);
    void recordFrame(const VRFrameDataPlain& frame);
    void flushAndUpload();
    void shutdown();

private:
    std::mutex mtx_;
    std::vector<VRFrameDataPlain> buffer_;
    UploaderConfig cfg_;
    AndroidUploader* uploader_ = nullptr;
    int framesCount_ = 0;
    std::string sessionId_;

    void serializeAndSend(const std::vector<VRFrameDataPlain>& chunk);
};
