#pragma once
#include <string>
#include <vector>
#include <mutex>

#include "TiposTelemetria.h"
#include "TiposVR.h"

// Small helper that records one C3D file per VR session.
// It is called from TelemetriaAPI: initialize -> recordFrame (many times) -> finalize.

class C3DRecorder {
public:
    C3DRecorder();
    ~C3DRecorder();

    // Initializes the recorder for a specific session.
    // - cfg: telemetry configuration; used mainly for sessionId and for locating the directory where initialConfig.json is stored.
    // - frameRate: sampling frequency (Hz) that will be written into the C3D POINT:RATE and ANALOG:RATE parameters.
    // Returns:
    //   true  if initialization succeeded or if already initialized.
    //   false if the output path could not be built and C3D recording is disabled for this run.
    bool C3Dinitialize(const UploaderConfig& cfg, int frameRate);

    // Records a single frame of VR data (HMD, controllers, hands).
    // The frame is simply pushed into an internal vector and will be converted into C3D data later when finalize is called.
    void C3DrecordFrame(const VRFrameDataPlain& frame);

    // Later on, if larger time sessions are needed, the C3D will be split into multiple files.
    void C3Dflush();

    // Finalizes the recorder and writes the C3D file to disk. Calling it more than once is safe
    void C3Dfinalize();

    // Returns true if the recorder has been successfully initialized.
    bool C3DisInitialized() const;

private:
    // Mutex protecting internal state (file path, flags, frame buffer).
    mutable std::mutex mtx_;

    // Flags to track the lifecycle of the recorder.
    bool initialized_ = false;
    bool finalized_   = false;

    // Full path of the C3D file that will be written on finalize.
    std::string filePath_;
    int frameRate_ = 60;

    // We store all frames during the session and then write the C3D complete
    std::vector<VRFrameDataPlain> frames_;

    // Builds the output file path for the C3D, same path to initialConfig.json
    std::string buildOutputPath(const UploaderConfig& cfg);
    // In case sessionId name isnt safe to be a part of filename
    std::string sanitizeSessionId(const std::string& raw) const;

    // Construct points and analog channel names for c3d
    void buildPointNames(std::vector<std::string>& outPointNames) const;
    void buildAnalogNames(std::vector<std::string>& outAnalogNames) const;

    // The core method to write the C3D file from accumulated frames.
    void writeC3D();
};
