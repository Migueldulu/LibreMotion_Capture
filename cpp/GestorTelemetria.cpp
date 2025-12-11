#include "GestorTelemetria.h"
#include "AndroidUploader.h"
#include <android/log.h>
#include <sstream>
#include "configReader.h"
#include <chrono>
#include <deque>
#include <condition_variable>
#include <thread>

#define LOG_TAG "telemetria"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

// Bit masks used for controller button states. (see VRFrameDataPlain in TiposVR.h)
static constexpr unsigned BTN_PRIMARY   = 0x1u; // A/X
static constexpr unsigned BTN_SECONDARY = 0x2u; // B/Y
static constexpr unsigned BTN_JOYSTICK  = 0x4u; // Stick press

GestorTelemetria::GestorTelemetria() {}
// Destructor, is a safety fallback in case shutdown() was not called explicitly.
GestorTelemetria::~GestorTelemetria() {
    // Signal stop to the worker thread.
    {
        std::lock_guard<std::mutex> lk(qmtx_);
        if (!stopWorker_) {
            stopWorker_ = true;
            qcv_.notify_all();
        }
    }
    // Wait for the worker thread to finish if it was running.
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool GestorTelemetria::initialize(const UploaderConfig& cfg, AndroidUploader* uploader) {
        {
            // Initialize front buffer and configuration used by the producer.
            std::lock_guard <std::mutex> lock(mtx_);
            cfg_ = cfg;
            uploader_ = uploader;
            buffer_.clear();
            framesCount_ = 0;
            sessionId_ = cfg.sessionId;
            deviceInfo_ = cfg.deviceInfo;
            // Reserve space to minimize reallocations and reduce the risk of
            // losing frames due to allocations at high frequency.
            buffer_.reserve(cfg_.framesPerFile);
        }
    // --- Start background worker thread ---
    {
        std::lock_guard<std::mutex> lk(qmtx_);
        stopWorker_ = false;
        queue_.clear();
    }
    // Launch the worker thread which will run workerLoop().
    worker_ = std::thread(&GestorTelemetria::workerLoop, this);
    return true;
}

void GestorTelemetria::recordFrame(const VRFrameDataPlain& frame) {
    //when buffer reach end it will hold a complete chunk to enqueue AFTER releasing the mutex
    std::vector<VRFrameDataPlain> chunk;
    {
        std::lock_guard <std::mutex> lock(mtx_);
        buffer_.push_back(frame);
        framesCount_++;
        // When we reach framesPerFile, change the buffer into a new chunk.
        if (cfg_.framesPerFile > 0 && framesCount_ >= cfg_.framesPerFile) {
            chunk.swap(buffer_);
            framesCount_ = 0;
            buffer_.reserve(cfg_.framesPerFile);
        }
    }
    // If we produced a full chunk, enqueue it for asynchronous upload.
    if (!chunk.empty()) {
        std::unique_lock<std::mutex> lk(qmtx_);
        // Simple backpressure: if the queue is too large, drop the oldest chunk (as said before this should never happen).
        if (queue_.size() >= maxQueuedChunks_) {
            queue_.pop_front();
        }
        queue_.push_back(std::move(chunk));
        lk.unlock();
        // Wake up the worker thread so it can process the new chunk.
        qcv_.notify_one();
    }
}

void GestorTelemetria::flushAndUpload() {
    // Move any frames currently in buffer_ into a chunk and enqueue it,
    // even if we did not reach framesPerFile.
    std::vector<VRFrameDataPlain> chunk;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (buffer_.empty()) return;
        chunk.swap(buffer_);
        framesCount_ = 0;
        buffer_.reserve(cfg_.framesPerFile);
    }
    // Enqueue the partially-filled chunk as if it were a complete one.
    std::unique_lock<std::mutex> lk(qmtx_);
    if (queue_.size() >= maxQueuedChunks_) {
        queue_.pop_front(); // drop oldest chunk if queue is too large (again not happening BUT)
    }
    queue_.push_back(std::move(chunk));
    lk.unlock();
    qcv_.notify_one();
}

void GestorTelemetria::shutdown() {
    // First, flush remaining frames into the queue.
    flushAndUpload();
    // Then, signal the worker thread to stop and wake it up.
    {
        std::lock_guard<std::mutex> lk(qmtx_);
        stopWorker_ = true;
    }
    qcv_.notify_all();
    // Wait for the worker to finish any pending uploads.
    if (worker_.joinable()) worker_.join();
}

// Serialize a sequence of frames into the flat JSON format.
// The resulting JSON is an array of frame objects, each with head_pose, controllers (left/right) or optional hand joint data.
static std::string toJsonFlat(const std::vector<VRFrameDataPlain>& frames, const std::string& sessionId, const std::string& deviceInfo, const UploaderConfig& cfgFlags) {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        os << "{";
        os << "\"session_id\":\"" << sessionId << "\",";
        os << "\"timestamp\":" << f.timestampSec << ",";
        os << "\"device_info\":\"" << deviceInfo << "\",";

        // Estructura para head pose
        os << "\"head_pose\":{";
        os << "\"orientation\":[" << f.hmdPose.rotation[0] << "," << f.hmdPose.rotation[1] << "," << f.hmdPose.rotation[2] << "," << f.hmdPose.rotation[3] << "],";
        os << "\"position\":[" << f.hmdPose.position[0] << "," << f.hmdPose.position[1] << "," << f.hmdPose.position[2] << "]";
        os << "},";

        // Estructura para controladores
        os << "\"controllers\":{";
        os << "\"left\":{";
        os << "\"tracked\":" << (f.leftCtrl.isActive ? "true" : "false") << ",";
        os << "\"orientation\":[" << f.leftCtrl.pose.rotation[0] << "," << f.leftCtrl.pose.rotation[1] << "," << f.leftCtrl.pose.rotation[2] << "," << f.leftCtrl.pose.rotation[3] << "],";
        os << "\"position\":[" << f.leftCtrl.pose.position[0] << "," << f.leftCtrl.pose.position[1] << "," << f.leftCtrl.pose.position[2] << "]";

        // Campos condicionales del controlador izquierdo
        if (cfgFlags.trigger) {
            os << ",\"trigger\":" << f.leftCtrl.trigger;
        }
        if (cfgFlags.grip) {
            os << ",\"grip\":" << f.leftCtrl.grip;
        }
        if (cfgFlags.primaryButton) {
            bool lp = (f.leftCtrl.buttons & BTN_PRIMARY) != 0u;
            os << ",\"primary\":" << (lp ? "true":"false");
        }
        if (cfgFlags.secondaryButton) {
            bool ls = (f.leftCtrl.buttons & BTN_SECONDARY) != 0u;
            os << ",\"secondary\":" << (ls ? "true":"false");
        }
        if (cfgFlags.joystick) {
            os << ",\"joystick_x\":" << f.leftCtrl.stickX;
            os << ",\"joystick_y\":" << f.leftCtrl.stickY;
        }
        os << "},";  // cierra left

        // Controlador derecho
        os << "\"right\":{";
        os << "\"tracked\":" << (f.rightCtrl.isActive ? "true" : "false") << ",";
        os << "\"orientation\":[" << f.rightCtrl.pose.rotation[0] << "," << f.rightCtrl.pose.rotation[1] << "," << f.rightCtrl.pose.rotation[2] << "," << f.rightCtrl.pose.rotation[3] << "],";
        os << "\"position\":[" << f.rightCtrl.pose.position[0] << "," << f.rightCtrl.pose.position[1] << "," << f.rightCtrl.pose.position[2] << "]";

        // Campos condicionales del controlador derecho
        if (cfgFlags.trigger) {
            os << ",\"trigger\":" << f.rightCtrl.trigger;
        }
        if (cfgFlags.grip) {
            os << ",\"grip\":" << f.rightCtrl.grip;
        }
        if (cfgFlags.primaryButton) {
            bool rp = (f.rightCtrl.buttons & BTN_PRIMARY) != 0u;
            os << ",\"primary\":" << (rp ? "true":"false");
        }
        if (cfgFlags.secondaryButton) {
            bool rs = (f.rightCtrl.buttons & BTN_SECONDARY) != 0u;
            os << ",\"secondary\":" << (rs ? "true":"false");
        }
        if (cfgFlags.joystick) {
            os << ",\"joystick_x\":" << f.rightCtrl.stickX;
            os << ",\"joystick_y\":" << f.rightCtrl.stickY;
        }
        os << "}";  // cierra right
        os << "},";  // cierra controllers

        // Joints de las manos
        if (cfgFlags.handTracking) {
            os << "\"hands\":{";
            // Left joints
            os << "\"left\":{";
            os << "\"joint_count\":" << f.leftHandJointCount << ",";
            os << "\"joints\":[";
            for (int j = 0; j < f.leftHandJointCount && j < 30; ++j) {
                const auto& s = f.leftHandJoints[j];
                if (j) os << ",";
                os << "{"
                   << "\"id\":" << s.idIndex
                   << ",\"state\":" << s.state
                   << ",\"orientation\":[" << s.qx << "," << s.qy << "," << s.qz << "," << s.qw << "]"
                   << ",\"position\":[" << s.px << "," << s.py << "," << s.pz << "]"
                   << ",\"has_pose\":" << (s.hasPose ? "true":"false")
                   << "}";
            }
            os << "]";
            os << "},";

            // Right joints
            os << "\"right\":{";
            os << "\"joint_count\":" << f.rightHandJointCount << ",";
            os << "\"joints\":[";
            for (int j = 0; j < f.rightHandJointCount && j < 30; ++j) {
                const auto& s = f.rightHandJoints[j];
                if (j) os << ",";
                os << "{"
                   << "\"id\":" << s.idIndex
                   << ",\"state\":" << s.state
                   << ",\"orientation\":[" << s.qx << "," << s.qy << "," << s.qz << "," << s.qw << "]"
                   << ",\"position\":[" << s.px << "," << s.py << "," << s.pz << "]"
                   << ",\"has_pose\":" << (s.hasPose ? "true":"false")
                   << "}";
            }
            os << "]";
            os << "}"; // cierra right
            os << "}"; // cierra hand_joints
        }

        os << "}"; // cierra el objeto frame completo
        if (i + 1 < frames.size()) os << ",";
    }
    os << "]";
    return os.str();
}

void GestorTelemetria::serializeAndSend(const std::vector<VRFrameDataPlain>& chunk) {
    if (!uploader_) return;
    // Convert the chunk into JSON according to the configured feature flags
    const std::string json = toJsonFlat(chunk, sessionId_, deviceInfo_, cfg_);
    // Perform the HTTP upload via AndoidUploader.
    bool ok = uploader_->uploadJson(json);
    if (ok) {
        __android_log_print(ANDROID_LOG_INFO, "telemetria",
                            "Uploaded chunk, frames: %zu", chunk.size());
    } else {
        __android_log_print(ANDROID_LOG_ERROR, "telemetria",
                            "Upload FAILED, frames: %zu", chunk.size());
    }
}

void GestorTelemetria::workerLoop() {
    for (;;) {
        std::vector<VRFrameDataPlain> chunk;
        {
            // Wait until there is work to do or a stop signal.
            std::unique_lock<std::mutex> lk(qmtx_);
            qcv_.wait(lk, [&]{ return stopWorker_ || !queue_.empty(); });
            if (stopWorker_ && queue_.empty()) break; // Stop requested and no more chunks to process.
            chunk = std::move(queue_.front());
            queue_.pop_front();
        }
        auto t0 = std::chrono::steady_clock::now();

        // Serialize and upload outside of the queue lock.
        serializeAndSend(chunk);

        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        __android_log_print(ANDROID_LOG_INFO, "telemetria",
                            "worker: subido chunk de %zu frames en %lld ms",
                            chunk.size(), (long long)ms);
    }
}