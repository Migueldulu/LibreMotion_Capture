#pragma once
#include <jni.h>
#include "VRTypes.h"
#include "TelemetryTypes.h"

// Export macro para Android (makes the function visible oustide de .so)
#ifndef TELEMETRIA_API
#define TELEMETRIA_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// This must be called before telemetry_initialize so that the uploader
// can dynamically load the classes from the AAR.
TELEMETRIA_API void telemetry_set_java_context(JavaVM* vm, jobject activity);

// Initializes the telemetry manager and HTTP uploader.
// - cfg: basic configuration coming from Unity/Unreal (sessionId, deviceInfo).
// Return:
//   > 0 : frame rate that the caller should use for sampling (1 to 240).
//   < 0 : negative error code.
TELEMETRIA_API int  telemetry_initialize(const TelemetryConfigPlain* cfg);

// Records one frame of VR telemetry data.
// This function is expected to be called frequently (e.g. every frame or at a fixed rate). It will:
//   - append the frame to the C3D recorder buffer;
//   - append the frame to the JSON buffer in GestorTelemetria.
TELEMETRIA_API void telemetry_record_frame(const VRFrameDataPlain* frame);

// Forces upload of any telemetry JSON chunks left.
// This can be called when the app is about to go to background to avoid losing any data
TELEMETRIA_API void telemetry_force_upload();

// Shuts down all telemetry components and releases resources.
// This will:
//   - finalize the C3D recorder and write the .c3d file if needed;
//   - flush and stop the JSON uploader worker;
//   - release the global Java Activity reference.
TELEMETRIA_API void telemetry_shutdown();

// Returns a bitmask of feature flags read from initialConfig.json.
// Bit layout:
//   bit 0: hand tracking enabled
//   bit 1: primary button fields enabled
//   bit 2: secondary button fields enabled
//   bit 3: grip value enabled
//   bit 4: trigger value enabled
//   bit 5: joystick axes enabled
// The engine can use this to know which fields are actually configured.
TELEMETRIA_API unsigned telemetry_get_feature_flags();

#ifdef __cplusplus
}
#endif
