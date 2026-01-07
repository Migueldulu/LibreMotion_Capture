#pragma once
#include <string>
#include "TelemetryTypes.h"

// Reader for the initial configuration stored in:
//   /sdcard/Android/data/<package>/files/initialConfig.json
//
// This JSON file typically contains (tag names must be the same):
//   - "endpoint":     base URL for the REST API
//   - "apiKey":       API key for the backend
//   - "framesPerFile": number of frames per JSON chunk
//   - "frameRate":    sampling frequency in Hz
//   - "handTracking": boolean,
//   - "primaryButton":  boolean,
//   - "secondaryButton": boolean,
//   - "grip", "trigger", "joystick": booleans,


namespace configReader {
    // Builds the expected path for initialConfig.json based on /proc/self/cmdline.
    // On Android, the process name matches the package name, so we use:
    //   /sdcard/Android/data/<package>/files/initialConfig.json
    // Returns true on success and writes the full path into outPath.
    bool getExpectedConfigPath(std::string& outPath);

    // Reads a file completely into a string (opaque binary or text).
    // Returns true on success.
    bool readFileToString(const std::string& path, std::string& outText);

    // Reads initialConfig.json, logs its content and fills the endpointUrl, apiKey, framesPerFile and feature flags of outCfg.
    // It starts from built-in defaults and only overrides values found in the JSON file. Returns:
    //   true  if the file could be read and parsed (even if some keys are missing),
    //   false if the file could not be read at all, so defaults remain.
    bool setConfig(UploaderConfig& outCfg);

    // Reads only the "frameRate" field from initialConfig.json, with a default value if the file or the key is missing.
    // This is separated from setConfig() so that the TelemetriaAPI can return the frameRate to Unity/UE
    bool getFrameRate(int& outFrameRate);

    // bit0=handTracking, bit1=primary, bit2=secondary, bit3=grip, bit4=trigger, bit5=joystick
    // This is used by the public C API to expose feature flags as a single int.
    inline unsigned getFeatureFlagsBitmask(const UploaderConfig& cfg) {
        unsigned m = 0;
        if (cfg.handTracking)  m |= 1u << 0;
        if (cfg.primaryButton) m |= 1u << 1;
        if (cfg.secondaryButton)m |= 1u << 2;
        if (cfg.grip)          m |= 1u << 3;
        if (cfg.trigger)       m |= 1u << 4;
        if (cfg.joystick)      m |= 1u << 5;
        return m;
    }
}
extern "C" bool configReader_setConfig(UploaderConfig& outCfg);
extern "C" int  configReader_getFrameRate();