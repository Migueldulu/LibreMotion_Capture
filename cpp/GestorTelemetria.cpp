#include "GestorTelemetria.h"
#include "AndroidUploader.h"
#include <android/log.h>
#include <sstream>

#define LOG_TAG "telemetria"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

GestorTelemetria::GestorTelemetria() {}
GestorTelemetria::~GestorTelemetria() {}

bool GestorTelemetria::initialize(const UploaderConfig& cfg, AndroidUploader* uploader) {
    std::lock_guard<std::mutex> lock(mtx_);
    cfg_ = cfg;
    uploader_ = uploader;
    buffer_.clear();
    framesCount_ = 0;
    sessionId_ = cfg.sessionId;

    bool ok = true;
    if (uploader_) {
        // Crear o asegurar sesion antes de enviar frames
        ok = uploader_->ensureSession();
        if (!ok) {
            __android_log_print(ANDROID_LOG_ERROR, "telemetria",
                                "ensureSession failed, normalmente es por una clave erronea");
        }
    }
    return ok;
}

void GestorTelemetria::recordFrame(const VRFrameDataPlain& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    buffer_.push_back(frame);
    framesCount_++;
    if (cfg_.framesPerFile > 0 && framesCount_ >= cfg_.framesPerFile) {
        std::vector<VRFrameDataPlain> chunk;
        chunk.swap(buffer_);
        framesCount_ = 0;
        lock.~lock_guard();
        serializeAndSend(chunk);
    }
}

void GestorTelemetria::flushAndUpload() {
    std::vector<VRFrameDataPlain> chunk;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (buffer_.empty()) return;
        chunk.swap(buffer_);
        framesCount_ = 0;
    }
    serializeAndSend(chunk);
}

void GestorTelemetria::shutdown() {
    flushAndUpload();
}

static std::string toJsonFlat(const std::vector<VRFrameDataPlain>& frames, const std::string& sessionId) {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        os << "{";
        os << "\"session_id\":\"" << sessionId << "\",";
        os << "\"timestamp\":" << f.timestampSec << ",";
        os << "\"frame_data\":null,";

        os << "\"head_pos_x\":" << f.hmdPose.position[0] << ",";
        os << "\"head_pos_y\":" << f.hmdPose.position[1] << ",";
        os << "\"head_pos_z\":" << f.hmdPose.position[2] << ",";
        os << "\"head_rot_x\":" << f.hmdPose.rotation[0] << ",";
        os << "\"head_rot_y\":" << f.hmdPose.rotation[1] << ",";
        os << "\"head_rot_z\":" << f.hmdPose.rotation[2] << ",";
        os << "\"head_rot_w\":" << f.hmdPose.rotation[3] << ",";

        os << "\"left_tracked\":" << (f.leftCtrl.isActive ? "true" : "false") << ",";
        os << "\"left_pos_x\":" << f.leftCtrl.pose.position[0] << ",";
        os << "\"left_pos_y\":" << f.leftCtrl.pose.position[1] << ",";
        os << "\"left_pos_z\":" << f.leftCtrl.pose.position[2] << ",";
        os << "\"left_rot_x\":" << f.leftCtrl.pose.rotation[0] << ",";
        os << "\"left_rot_y\":" << f.leftCtrl.pose.rotation[1] << ",";
        os << "\"left_rot_z\":" << f.leftCtrl.pose.rotation[2] << ",";
        os << "\"left_rot_w\":" << f.leftCtrl.pose.rotation[3] << ",";
        os << "\"left_trigger\":" << f.leftCtrl.trigger << ",";

        os << "\"right_tracked\":" << (f.rightCtrl.isActive ? "true" : "false") << ",";
        os << "\"right_pos_x\":" << f.rightCtrl.pose.position[0] << ",";
        os << "\"right_pos_y\":" << f.rightCtrl.pose.position[1] << ",";
        os << "\"right_pos_z\":" << f.rightCtrl.pose.position[2] << ",";
        os << "\"right_rot_x\":" << f.rightCtrl.pose.rotation[0] << ",";
        os << "\"right_rot_y\":" << f.rightCtrl.pose.rotation[1] << ",";
        os << "\"right_rot_z\":" << f.rightCtrl.pose.rotation[2] << ",";
        os << "\"right_rot_w\":" << f.rightCtrl.pose.rotation[3] << ",";
        os << "\"right_trigger\":" << f.rightCtrl.trigger << ",";

        // ejemplo simple de mapeo de boton A desde bitmask
        bool buttonA = (f.rightCtrl.buttons & 0x1u) != 0u;
        os << "\"button_a\":" << (buttonA ? "true" : "false");

        os << "}";
        if (i + 1 < frames.size()) os << ",";
    }
    os << "]";
    return os.str();
}

void GestorTelemetria::serializeAndSend(const std::vector<VRFrameDataPlain>& chunk) {
    if (!uploader_ || !cfg_.enableCloudUpload) return;
    const std::string json = toJsonFlat(chunk, sessionId_);
    bool ok = uploader_->uploadJson(json);
    if (ok) {
        __android_log_print(ANDROID_LOG_INFO, "telemetria",
                            "Uploaded chunk, frames: %zu", chunk.size());
    } else {
        __android_log_print(ANDROID_LOG_ERROR, "telemetria",
                            "Upload FAILED, frames: %zu", chunk.size());
    }
}
