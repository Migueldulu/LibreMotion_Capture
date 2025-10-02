#pragma once
#include <jni.h>
#include "TiposVR.h"
#include "TiposTelemetria.h"

// Export macro para Android
#ifndef TELEMETRIA_API
#define TELEMETRIA_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Contexto Java: activity es obligatorio para cargar clases del AAR.
// vm puede ser null si fue capturado en JNI_OnLoad.
TELEMETRIA_API void telemetry_set_java_context(JavaVM* vm, jobject activity);
// Inicializa el gestor y la capa de subida
// Devuelve 0 en exito, negativo en error.
TELEMETRIA_API int  telemetry_initialize(const TelemetryConfigPlain* cfg);
// Registra un frame de telemetria
TELEMETRIA_API void telemetry_record_frame(const VRFrameDataPlain* frame);
// Fuerza subida de datos pendientes
TELEMETRIA_API void telemetry_force_upload();
// Libera recursos
TELEMETRIA_API void telemetry_shutdown();

#ifdef __cplusplus
}
#endif
