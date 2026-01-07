#pragma once

// Componente minimo: recoge HMD y manos y envia un VRFrameDataPlain a la libreria .so

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TelemetryComponent.generated.h"

struct ControllerStatePlain;

class USkinnedMeshComponent;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class TELEMETRY_API UTelemetryComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UTelemetryComponent();
    // Asegura flush/shutdown si destruyen el componente explícitamente
    virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    // ===== Helpers =====

    // Obtiene factor WorldToMeters del mundo (por defecto 100.0)
    float GetWorldToMeters() const;

    // Lee HMD; devuelve true si la lectura es valida
    bool ReadHmdPose(FVector& OutPosUU, FQuat& OutQuat) const;

    // Lee mano izquierda o derecha; devuelve true si la lectura es valida
    bool ReadHandPose(bool bLeft, FVector& OutPosUU, FQuat& OutQuat) const;

    // Convierte de unidades UE (uu) a metros
    static FVector ToMeters(const FVector& P_UU, float WorldToMeters);

    // Opcional: convertir a ejes de Unity. Desactivado por defecto.
    static FVector ToUnityPos(const FVector& Pm);
    static FQuat   ToUnityQuat(const FQuat& Q);

    // Envio en el mismo formato plano usado en Unity
    void SendFrame(double TimestampSec,
        const FVector& HmdPos_m, const FQuat& HmdQuat,
        bool bLeftValid, const FVector& LPos_m, const FQuat& LQuat,
        bool bRightValid, const FVector& RPos_m, const FQuat& RQuat);

	// Asegura que float es finito; si no, devuelve 0.f
    static float SafeFloat(float v) { return FMath::IsFinite(v) ? v : 0.f; }

    // Rellena un ControllerStatePlain con pose + inputs según flags
    void FillController(bool bLeft, ControllerStatePlain& OutCtrl) const;
    // NUEVO: cache de las manos y helpers para joints
    void CacheHandMeshes();
    void FillHandJointsFromMeshes(struct VRFrameDataPlain& OutFrame);
    bool FillHandJointsFromOpenXR(struct VRFrameDataPlain& OutFrame);

private:

    // ===== Flags cacheados (tras telemetry_initialize) =====
    bool bHand = false, bPrimary = false, bSecondary = false, bGrip = false, bTrigger = false, bJoystick = false;

    // NUEVO: punteros cacheados a las manos del VRPawn (owner del componente)
    USkinnedMeshComponent* LeftHandMesh = nullptr;
    USkinnedMeshComponent* RightHandMesh = nullptr;
    bool bHandMeshesCached = false;

    // Nombres de keys detectadas (cacheadas en BeginPlay)
    FName TriggerL, TriggerR;
    FName GripL, GripR;
    FName ThumbXL, ThumbYL, ThumbXR, ThumbYR;
    FName ThumbClickL, ThumbClickR;
    FName Btn1L, Btn1R, Btn2L, Btn2R;

    // Detecta y cachea los nombres de las keys VR
    void DetectVRInputKeys();
   

    // buffers C que viviran hasta EndPlay
    char* TelemetrySess = nullptr;
    char* TelemetryDev = nullptr;

    // Handles de delegados de ciclo de vida
    FDelegateHandle HandleWillEnterBackground;
    FDelegateHandle HandleWillTerminate;
    FDelegateHandle HandlePreExit;

    // Callbacks de ciclo de vida de la app
    void OnAppWillEnterBackground();

    // ---- Rate limiting ----
    int    FrameRateHz = 60;
    double PeriodSec = 1.0 / 60.0;
    double NextTick = 0.0;

    // Timestamp normalizado (primer envio = 0)
    double T0 = 0.0;
    bool   bZeroed = false;

    // Suscripcion para fin de frame (pose mas fresca)
    FDelegateHandle HandleEndFrame;
    void OnEndFrameTick();


};
