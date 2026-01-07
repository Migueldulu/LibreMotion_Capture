#pragma once
#include <vector>
#include <string>
#include <mutex>
#include "TelemetryTypes.h"
#include "VRTypes.h"
#include <deque>
#include <condition_variable>
#include <thread>

class AndroidUploader;

// Basic manager for buffering VR frames and triggering uploads.
// It accumulates frames into an in-memory chunk and hands that chunk to a background worker thread
// that serializes it to JSON and sends it through the other class -> AndroidUploader.
class GestorTelemetria {
public:
    GestorTelemetria();
    // Needs Destructor to make sure that the background worker thread
    // is properly stopped and joined if it was still running.
    ~GestorTelemetria();

    // Initializes the manager with a configuration and an uploader.
    // - cfg: contains endpoint, API key, framesPerFile and feature flags.
    // - uploader: pointer to an AndroidUploader that will perform the HTTP POST.
    // This will also start the worker thread that consumes chunks.
    bool initialize(const UploaderConfig& cfg, AndroidUploader* uploader);

    // Records a one VR frame:
    // - The frame is appended to an internal buffer.
    // - When the number of frames reaches cfg_.framesPerFile, the buffer iis enqueued for asynchronous upload.
    void recordFrame(const VRFrameDataPlain& frame);

    // Forces the current in-memory buffer to be enqueued as a chunk,
    // even if it has fewer frames than cfg_.framesPerFile.
    // Useful when the app goes to background or the session is ending.
    void flushAndUpload();

    // Shuts down the manager:
    // - Flushes remaining frames in the buffer.
    // - Signals the worker thread to stop.
    // - Joins the worker thread to wait for completion.
    void shutdown();

private:
    // Mutex (mutex=mechanism to avoid multiple threads accesing the same resource) protecting the front buffer used by the producer thread(s).
    std::mutex mtx_;
    // Current batch of frames that will form the next JSON chunk.
    std::vector<VRFrameDataPlain> buffer_;
    // Copy of the uploader configuration (endpoint, flags, etc.).
    UploaderConfig cfg_;
    // Pointer to the uploader used to send JSON over HTTP.
    AndroidUploader* uploader_ = nullptr;
    //number of frames currently stored in the buffer
    int framesCount_ = 0;
    // Session and device identifier, repeated in each JSON frame object.
    std::string sessionId_;
    std::string deviceInfo_;

    // Serializes a completed chunk to JSON and sends it through the uploader.
    // This is invoked by the background worker thread.
    void serializeAndSend(const std::vector<VRFrameDataPlain>& chunk);

    // --- Asynchronous upload queue and worker thread --
    // Background worker that waits for chunks and uploads them.
    std::thread worker_;
    // Mutex protecting the chunk queue.
    std::mutex qmtx_;
    // Condition variable used to wake the worker when new chunks arrive.
    std::condition_variable qcv_;
    // Queue of chunks waiting to be serialized and uploaded.
    std::deque<std::vector<VRFrameDataPlain>> queue_;
    // Flag used to request the worker thread to stop.
    bool stopWorker_ = false;
    // Simple backpressure: maximum number of chunks kept in the queue.
    // If this limit is reached, the oldest chunk is dropped (it most likely wont get above 2-3 in queue, but in case some bug happens).
    size_t maxQueuedChunks_ = 4;

    // Worker loop: takes chunks from the queue, serializes and uploads them.
    void workerLoop();
};
