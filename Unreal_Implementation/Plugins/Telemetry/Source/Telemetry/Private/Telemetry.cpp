// ============================================================================
// Telemetry.cpp
// Punto de entrada del modulo para el plugin Telemetry.
//
// Esta TU implementa el modulo. 
// ============================================================================

#include "Modules/ModuleManager.h"

class FTelemetryModule : public IModuleInterface
{
public:
    // Llamado al iniciar el modulo
    virtual void StartupModule() override {}
    // Llamado al cerrar el modulo
    virtual void ShutdownModule() override {}
};

IMPLEMENT_MODULE(FTelemetryModule, Telemetry)
