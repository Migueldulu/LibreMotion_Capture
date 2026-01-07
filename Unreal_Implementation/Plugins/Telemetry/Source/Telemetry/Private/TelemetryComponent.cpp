#include "TelemetryComponent.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#include "HeadMountedDisplayTypes.h"  //  FXRMotionControllerData
#include "GameFramework/WorldSettings.h"
#include "GameFramework/InputSettings.h"
#include "HAL/PlatformProcess.h"
#include "Misc/DateTime.h"
#include "Misc/CoreDelegates.h"
#include "IXRTrackingSystem.h"
#include "InputCoreTypes.h"
#include "XRMotionControllerBase.h"
#include "XRTrackingSystemBase.h"
#include "Engine/Engine.h"
#include "Misc/EngineVersionComparison.h" 
#include "Components/SkinnedMeshComponent.h"

#if PLATFORM_ANDROID
#include "Android/AndroidApplication.h"
#include "Android/AndroidJNI.h"
#endif

DEFINE_LOG_CATEGORY_STATIC(LogTelemetry, Log, All);

// simple time gate after BeginPlay
static double GFirstTickTime = 0.0;

// ============================================================================
// C ABI structs expected by the library
// ============================================================================

extern "C" {

    struct VRPosePlain
    {
        float position[3];
        float rotation[4];
    };

    struct ControllerStatePlain
    {
        VRPosePlain pose;
        uint32 buttons;
        float  trigger;
        float  grip;
        float  stickX;
        float  stickY;
        int    isActive;
    };

    struct FJointSamplePlain
    {
        int idIndex;
        int state;
        float px, py, pz;
        float qx, qy, qz, qw;
        uint8 hasPose;
    };

    struct VRFrameDataPlain
    {
        double timestampSec;
        VRPosePlain hmdPose;
        ControllerStatePlain leftCtrl;
        ControllerStatePlain rightCtrl;
        FJointSamplePlain leftHandJoints[30];
        int leftHandJointCount;
        FJointSamplePlain rightHandJoints[30];
        int rightHandJointCount;
    };

    struct TelemetryConfigPlain
    {
        const char* sessionId;
        const char* deviceInfo;
    };

#if PLATFORM_ANDROID
    int  telemetry_initialize(const TelemetryConfigPlain* cfg);
    void telemetry_set_java_context(void* vm, void* activity);
    void telemetry_record_frame(const VRFrameDataPlain* frame);
    void telemetry_force_upload();
    void telemetry_shutdown();
    unsigned telemetry_get_feature_flags();
#else
    static int  telemetry_initialize(const TelemetryConfigPlain*) { return 0; }
    static void telemetry_record_frame(const VRFrameDataPlain*) {}
    static void telemetry_force_upload() {}
    static void telemetry_shutdown() {}
    static unsigned telemetry_get_feature_flags() { return 0u; }
#endif
} // extern "C"

// Convertir de ejes de tracking UE (X fwd, Y right, Z up)
// a un sistema tipo "OpenXR/Unity": X right, Y up, Z forward.
static FORCEINLINE void ToOpenXRAxes(
    const FVector& InPos_m,
    const FQuat& InQuat,
    FVector& OutPos_m,
    FQuat& OutQuat)
{
    // Posicion: (x, y, -z)
    OutPos_m.X = InPos_m.Y;
    OutPos_m.Y = InPos_m.Z;
    OutPos_m.Z = -InPos_m.X;

    // --- Rotacion ---
    // Equivalente a:
    //  1) cambiar de base UE -> Unity
    //  2) Unity -> OpenXr
    //     q_old = (-q.x, -q.y, q.z, q.w)
  
    const float qx = InQuat.X;
    const float qy = InQuat.Y;
    const float qz = InQuat.Z;
    const float qw = InQuat.W;

    OutQuat.X = -qy;
    OutQuat.Y = -qz;
    OutQuat.Z = qx;
    OutQuat.W = qw;
}


// if you need Unity axes conversion, flip to 1 (kept disabled)
#ifndef TELEMETRY_USE_OPENXR_AXES
#define TELEMETRY_USE_OPENXR_AXES 1
#endif

// dup FString to UTF-8 char* (free with FMemory::Free)
static char* DupAnsi(const FString& S)
{
    FTCHARToUTF8 U(*S);
    char* P = (char*)FMemory::Malloc(U.Length() + 1);
    FMemory::Memcpy(P, U.Get(), U.Length());
    P[U.Length()] = '\0';
    return P;
}

// ============================================================================
// Runtime input mappings injection 
// ============================================================================

// list registered keys once
static bool KeyExists(const TCHAR* Name)
{
    static bool bInit = false;
    static TSet<FName> Registered;
    if (!bInit)
    {
        TArray<FKey> All;
        EKeys::GetAllKeys(All);
        for (const FKey& K : All) Registered.Add(K.GetFName());
        bInit = true;
    }
    return Registered.Contains(FName(Name));
}

static FKey FirstExistingKey(std::initializer_list<const TCHAR*> Names)
{
    for (const TCHAR* N : Names)
    {
        if (N && KeyExists(N)) return FKey(N);
    }
    return FKey(); // invalid if none
}

static void AddAxisIfMissing(UInputSettings* S, const FName AxisName, const FKey& Key, float Scale = 1.f)
{
    if (!S || !Key.IsValid()) return;
    const TArray<FInputAxisKeyMapping>& Existing = S->GetAxisMappings();
    for (const auto& M : Existing)
        if (M.AxisName == AxisName && M.Key == Key && FMath::IsNearlyEqual(M.Scale, Scale))
            return;
    S->AddAxisMapping(FInputAxisKeyMapping(AxisName, Key, Scale));
}

static void AddActionIfMissing(UInputSettings* S, const FName ActionName, const FKey& Key)
{
    if (!S || !Key.IsValid()) return;
    const TArray<FInputActionKeyMapping>& Existing = S->GetActionMappings();
    for (const auto& M : Existing)
        if (M.ActionName == ActionName && M.Key == Key)
            return;
    S->AddActionMapping(FInputActionKeyMapping(ActionName, Key));
}

static void InstallTelemetryRuntimeMappings()
{
    UInputSettings* S = GetMutableDefault<UInputSettings>();
    if (!S) return;

    // pick keys by runtime: OpenXR -> OculusXR -> Gamepad as last fallback
    const FKey LTrig = FirstExistingKey({ TEXT("OpenXR_Left_Trigger_Axis"),  TEXT("OculusTouch_Left_Trigger_Axis"),  TEXT("Gamepad_LeftTriggerAxis") });
    const FKey RTrig = FirstExistingKey({ TEXT("OpenXR_Right_Trigger_Axis"), TEXT("OculusTouch_Right_Trigger_Axis"), TEXT("Gamepad_RightTriggerAxis") });

    const FKey LGrip = FirstExistingKey({ TEXT("OpenXR_Left_Grip_Axis"),    TEXT("OculusTouch_Left_Grip_Axis") });
    const FKey RGrip = FirstExistingKey({ TEXT("OpenXR_Right_Grip_Axis"),   TEXT("OculusTouch_Right_Grip_Axis") });

    const FKey Lx = FirstExistingKey({ TEXT("OpenXR_Left_Thumbstick_X"),    TEXT("OculusTouch_Left_Thumbstick_X"),  TEXT("Gamepad_LeftThumbstick_X") });
    const FKey Ly = FirstExistingKey({ TEXT("OpenXR_Left_Thumbstick_Y"),    TEXT("OculusTouch_Left_Thumbstick_Y"),  TEXT("Gamepad_LeftThumbstick_Y") });
    const FKey Rx = FirstExistingKey({ TEXT("OpenXR_Right_Thumbstick_X"),   TEXT("OculusTouch_Right_Thumbstick_X"), TEXT("Gamepad_RightThumbstick_X") });
    const FKey Ry = FirstExistingKey({ TEXT("OpenXR_Right_Thumbstick_Y"),   TEXT("OculusTouch_Right_Thumbstick_Y"), TEXT("Gamepad_RightThumbstick_Y") });

    const FKey LStickClick = FirstExistingKey({ TEXT("OpenXR_Left_Thumbstick_Click"),  TEXT("OculusTouch_Left_Thumbstick_Click") });
    const FKey RStickClick = FirstExistingKey({ TEXT("OpenXR_Right_Thumbstick_Click"), TEXT("OculusTouch_Right_Thumbstick_Click") });

    const FKey Xbtn = FirstExistingKey({ TEXT("OpenXR_X_Click"), TEXT("OculusTouch_X_Click"), TEXT("Gamepad_FaceButton_Left") });
    const FKey Ybtn = FirstExistingKey({ TEXT("OpenXR_Y_Click"), TEXT("OculusTouch_Y_Click"), TEXT("Gamepad_FaceButton_Top") });
    const FKey Abtn = FirstExistingKey({ TEXT("OpenXR_A_Click"), TEXT("OculusTouch_A_Click"), TEXT("Gamepad_FaceButton_Bottom") });
    const FKey Bbtn = FirstExistingKey({ TEXT("OpenXR_B_Click"), TEXT("OculusTouch_B_Click"), TEXT("Gamepad_FaceButton_Right") });

    // AXIS
    AddAxisIfMissing(S, TEXT("Telemetry_LeftTrigger"), LTrig);
    AddAxisIfMissing(S, TEXT("Telemetry_RightTrigger"), RTrig);
    AddAxisIfMissing(S, TEXT("Telemetry_LeftGrip"), LGrip);
    AddAxisIfMissing(S, TEXT("Telemetry_RightGrip"), RGrip);
    AddAxisIfMissing(S, TEXT("Telemetry_LeftStickX"), Lx);
    AddAxisIfMissing(S, TEXT("Telemetry_LeftStickY"), Ly);
    AddAxisIfMissing(S, TEXT("Telemetry_RightStickX"), Rx);
    AddAxisIfMissing(S, TEXT("Telemetry_RightStickY"), Ry);

    // ACTIONS
    AddActionIfMissing(S, TEXT("Telemetry_LeftStickClick"), LStickClick);
    AddActionIfMissing(S, TEXT("Telemetry_RightStickClick"), RStickClick);
    AddActionIfMissing(S, TEXT("Telemetry_Primary_X"), Xbtn);
    AddActionIfMissing(S, TEXT("Telemetry_Secondary_Y"), Ybtn);
    AddActionIfMissing(S, TEXT("Telemetry_Primary_A"), Abtn);
    AddActionIfMissing(S, TEXT("Telemetry_Secondary_B"), Bbtn);

    // apply to current PlayerInput maps
    S->SaveKeyMappings();     // optional
    S->ForceRebuildKeymaps(); // required so GetInputAxisValue starts working now
}

// ============================================================================

UTelemetryComponent::UTelemetryComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
    bAutoActivate = true;
}

void UTelemetryComponent::BeginPlay()
{
    Super::BeginPlay();

    GFirstTickTime = FPlatformTime::Seconds();
    UHeadMountedDisplayFunctionLibrary::SetTrackingOrigin(EHMDTrackingOrigin::Stage);

    const FString Stamp = FDateTime::UtcNow().ToString(TEXT("%Y%m%d-%H%M%S"));
    const FString Rand6 = FGuid::NewGuid().ToString(EGuidFormats::Short).Left(6);
    const FString SessId = FString::Printf(TEXT("%s-%s-%s"), FApp::GetProjectName(), *Stamp, *Rand6);

#if PLATFORM_ANDROID
    const FString DevInfo = FString::Printf(TEXT("%s - %s"), *FAndroidMisc::GetDeviceModel(), FApp::GetProjectName());
#else
    const FString DevInfo = FString::Printf(TEXT("UE-Editor - %s"), FApp::GetProjectName());
#endif

    TelemetrySess = DupAnsi(SessId);
    TelemetryDev = DupAnsi(DevInfo);

    TelemetryConfigPlain Cfg{};
    Cfg.sessionId = TelemetrySess;
    Cfg.deviceInfo = TelemetryDev;

#if PLATFORM_ANDROID
    {
        JNIEnv* Env = FAndroidApplication::GetJavaEnv();
        JavaVM* Vm = nullptr;
        if (Env)
        {
            jint JniRc = Env->GetJavaVM(&Vm);
            UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] GetJavaVM rc=%d, Vm=%p, Env=%p"), (int)JniRc, Vm, Env);
        }
        else
        {
            UE_LOG(LogTelemetry, Error, TEXT("[Telemetry] Env is null"));
        }

        jobject Activity = FAndroidApplication::GetGameActivityThis();
        UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] Activity=%p"), Activity);

        telemetry_set_java_context((void*)Vm, (void*)Activity);
    }
#endif

    UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] BeginPlay"));

    const int Rc = telemetry_initialize(&Cfg);
    UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] init rc=%d sess=%s dev=%s"), Rc, *SessId, *DevInfo);

    int Effective = (Rc >= 1 && Rc <= 240) ? Rc : 60;
    FrameRateHz = Effective;
    PeriodSec = 1.0 / (double)FrameRateHz;
    NextTick = FPlatformTime::Seconds() + PeriodSec;

    const unsigned Flags = telemetry_get_feature_flags();
    bHand = (Flags & (1u << 0)) != 0;
    bPrimary = (Flags & (1u << 1)) != 0;
    bSecondary = (Flags & (1u << 2)) != 0;
    bGrip = (Flags & (1u << 3)) != 0;
    bTrigger = (Flags & (1u << 4)) != 0;
    bJoystick = (Flags & (1u << 5)) != 0;
    UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] flags Hand=%d Primary=%d Secondary=%d Grip=%d Trigger=%d Joystick=%d"),
        bHand, bPrimary, bSecondary, bGrip, bTrigger, bJoystick);

    // inject mappings in runtime so axis polling works without editor setup
    InstallTelemetryRuntimeMappings();

    // subscribe to end of frame tick
    HandleEndFrame = FCoreDelegates::OnEndFrame.AddUObject(this, &UTelemetryComponent::OnEndFrameTick);
    // subscribe lifecycle for background flush
    HandleWillEnterBackground = FCoreDelegates::ApplicationWillEnterBackgroundDelegate.AddUObject(
        this, &UTelemetryComponent::OnAppWillEnterBackground);
}

void UTelemetryComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] EndPlay enter"));

    if (HandleEndFrame.IsValid())
    {
        FCoreDelegates::OnEndFrame.Remove(HandleEndFrame);
        HandleEndFrame.Reset();
    }
    if (HandleWillEnterBackground.IsValid()) {
        FCoreDelegates::ApplicationWillEnterBackgroundDelegate.Remove(HandleWillEnterBackground);
        HandleWillEnterBackground.Reset();
    }

    telemetry_force_upload();
    telemetry_shutdown();

    FMemory::Free(TelemetrySess); TelemetrySess = nullptr;
    FMemory::Free(TelemetryDev);  TelemetryDev = nullptr;

    Super::EndPlay(EndPlayReason);
}

void UTelemetryComponent::OnComponentDestroyed(bool bDestroyingHierarchy)
{
    if (HandleEndFrame.IsValid())
    {
        FCoreDelegates::OnEndFrame.Remove(HandleEndFrame);
        HandleEndFrame.Reset();
    }
    telemetry_force_upload();
    telemetry_shutdown();

    FMemory::Free(TelemetrySess); TelemetrySess = nullptr;
    FMemory::Free(TelemetryDev);  TelemetryDev = nullptr;

    Super::OnComponentDestroyed(bDestroyingHierarchy);
}

void UTelemetryComponent::OnEndFrameTick()
{
    if (FPlatformTime::Seconds() - GFirstTickTime < 0.2)
        return;

    const double Now = FPlatformTime::Seconds();
    if (Now < NextTick)
        return;

    const double MaxLateness = 0.5;
    if (Now - NextTick > MaxLateness)
        NextTick = Now + PeriodSec;
    else
        NextTick += PeriodSec;

    double Ts = Now;
    if (!bZeroed) { T0 = Now; bZeroed = true; }
    Ts = FMath::Max(0.0, Now - T0);

    const float  W2M = GetWorldToMeters();

    // HMD
    FVector HPosUU; FQuat HQuat;
    bool bHValid = ReadHmdPose(HPosUU, HQuat);
    FVector HPos_m = bHValid ? ToMeters(HPosUU, W2M) : FVector::ZeroVector;
    if (!bHValid) HQuat = FQuat::Identity;

    // left
    FVector LPosUU; FQuat LQuat;
    const bool bLValid = ReadHandPose(true, LPosUU, LQuat);
    FVector LPos_m = bLValid ? ToMeters(LPosUU, W2M) : FVector::ZeroVector;
    if (!bLValid) { LQuat = FQuat::Identity; }

    // right
    FVector RPosUU; FQuat RQuat;
    const bool bRValid = ReadHandPose(false, RPosUU, RQuat);
    FVector RPos_m = bRValid ? ToMeters(RPosUU, W2M) : FVector::ZeroVector;
    if (!bRValid) { RQuat = FQuat::Identity; }

#if TELEMETRY_USE_OPENXR_AXES
    {
        FVector P; FQuat Q;
        P = HPos_m; Q = HQuat; ToOpenXRAxes(P, Q, HPos_m, HQuat);
        P = LPos_m; Q = LQuat; ToOpenXRAxes(P, Q, LPos_m, LQuat);
        P = RPos_m; Q = RQuat; ToOpenXRAxes(P, Q, RPos_m, RQuat);
     }
#endif

    SendFrame(Ts, HPos_m, HQuat, bLValid, LPos_m, LQuat, bRValid, RPos_m, RQuat);
}

// ============================================================================
// Poses
// ============================================================================

float UTelemetryComponent::GetWorldToMeters() const
{
    const UWorld* W = GetWorld();
    if (!W) return 100.0f;
    const AWorldSettings* WS = W->GetWorldSettings();
    return WS ? WS->WorldToMeters : 100.0f;
}

static FORCEINLINE void WorldToTrackingPose(UWorld* World, const FVector& InPosUU, const FQuat& InQuat, FVector& OutPosUU, FQuat& OutQuat)
{
    const FTransform TrackingToWorld = UHeadMountedDisplayFunctionLibrary::GetTrackingToWorldTransform(World);
    const FTransform WorldToTracking = TrackingToWorld.Inverse();

    const FTransform PoseWorld(InQuat, InPosUU, FVector::OneVector);
    const FTransform PoseTracking = PoseWorld * WorldToTracking;

    OutPosUU = PoseTracking.GetLocation();
    OutQuat = PoseTracking.GetRotation();
}

bool UTelemetryComponent::ReadHmdPose(FVector& OutPosUU, FQuat& OutQuat) const
{
    if (!UHeadMountedDisplayFunctionLibrary::IsHeadMountedDisplayEnabled())
        return false;

    UWorld* World = GetWorld();
    if (!World)
        return false;

    FRotator R;
    FVector PosTracking; // Ojo: ya es tracking space
    UHeadMountedDisplayFunctionLibrary::GetOrientationAndPosition(R, PosTracking);
    const FQuat QTracking = R.Quaternion();

    OutPosUU = PosTracking;
    OutQuat = QTracking;
    return true;
}

bool UTelemetryComponent::ReadHandPose(bool bLeft, FVector& OutPosUU, FQuat& OutQuat) const
{
    const EControllerHand Hand = bLeft ? EControllerHand::Left : EControllerHand::Right;

    FXRMotionControllerData Data;
    UHeadMountedDisplayFunctionLibrary::GetMotionControllerData(GetWorld(), Hand, Data);

    if (!Data.bValid)
        return false;

    // 1) usar grip si esta disponible
    if (!Data.GripPosition.IsNearlyZero() || Data.GripRotation.IsNormalized())
    {
        FVector P_tr; FQuat Q_tr;
        WorldToTrackingPose(GetWorld(), Data.GripPosition, Data.GripRotation, P_tr, Q_tr);
        OutPosUU = P_tr;
        OutQuat = Q_tr;
        return true;
    }

    // 2) fallback: Palm si hay hand joints
    if (Data.HandKeyPositions.Num() > 1 && Data.HandKeyRotations.Num() > 1)
    {
        const int PalmIndex = 1;
        FVector P_tr; FQuat Q_tr;
        WorldToTrackingPose(GetWorld(), Data.HandKeyPositions[PalmIndex], Data.HandKeyRotations[PalmIndex], P_tr, Q_tr);
        OutPosUU = P_tr;
        OutQuat = Q_tr;
        return true;
    }

    return false;
}

void UTelemetryComponent::FillController(bool bLeft, ControllerStatePlain& OutCtrl) const
{
    // reset estado
    OutCtrl.buttons = 0u;
    OutCtrl.trigger = 0.f;
    OutCtrl.grip = 0.f;
    OutCtrl.stickX = 0.f;
    OutCtrl.stickY = 0.f;
    OutCtrl.isActive = 0;

    UWorld* World = GetWorld();
    if (!World)
        return;

    APlayerController* PC = World->GetFirstPlayerController();
    if (!PC)
        return;

    // Elegimos keys segun la mano, con fallback OpenXR -> Oculus -> Gamepad
    FKey TrigKey, GripKey, StickXKey, StickYKey, StickClickKey;
    FKey PrimaryKey, SecondaryKey;

    if (bLeft)
    {
        TrigKey = FirstExistingKey({ TEXT("OpenXR_Left_Trigger_Axis"),
                                           TEXT("OculusTouch_Left_Trigger_Axis"),
                                           TEXT("Gamepad_LeftTriggerAxis") });

        GripKey = FirstExistingKey({ TEXT("OpenXR_Left_Grip_Axis"),
                                           TEXT("OculusTouch_Left_Grip_Axis") });

        StickXKey = FirstExistingKey({ TEXT("OpenXR_Left_Thumbstick_X"),
                                           TEXT("OculusTouch_Left_Thumbstick_X"),
                                           TEXT("Gamepad_LeftThumbstick_X") });

        StickYKey = FirstExistingKey({ TEXT("OpenXR_Left_Thumbstick_Y"),
                                           TEXT("OculusTouch_Left_Thumbstick_Y"),
                                           TEXT("Gamepad_LeftThumbstick_Y") });

        StickClickKey = FirstExistingKey({ TEXT("OpenXR_Left_Thumbstick_Click"),
                                           TEXT("OculusTouch_Left_Thumbstick_Click") });

        // X / Y en mando izquierdo
        PrimaryKey = FirstExistingKey({ TEXT("OpenXR_X_Click"),
                                           TEXT("OculusTouch_X_Click"),
                                           TEXT("Gamepad_FaceButton_Left") });

        SecondaryKey = FirstExistingKey({ TEXT("OpenXR_Y_Click"),
                                           TEXT("OculusTouch_Y_Click"),
                                           TEXT("Gamepad_FaceButton_Top") });
    }
    else
    {
        TrigKey = FirstExistingKey({ TEXT("OpenXR_Right_Trigger_Axis"),
                                           TEXT("OculusTouch_Right_Trigger_Axis"),
                                           TEXT("Gamepad_RightTriggerAxis") });

        GripKey = FirstExistingKey({ TEXT("OpenXR_Right_Grip_Axis"),
                                           TEXT("OculusTouch_Right_Grip_Axis") });

        StickXKey = FirstExistingKey({ TEXT("OpenXR_Right_Thumbstick_X"),
                                           TEXT("OculusTouch_Right_Thumbstick_X"),
                                           TEXT("Gamepad_RightThumbstick_X") });

        StickYKey = FirstExistingKey({ TEXT("OpenXR_Right_Thumbstick_Y"),
                                           TEXT("OculusTouch_Right_Thumbstick_Y"),
                                           TEXT("Gamepad_RightThumbstick_Y") });

        StickClickKey = FirstExistingKey({ TEXT("OpenXR_Right_Thumbstick_Click"),
                                           TEXT("OculusTouch_Right_Thumbstick_Click") });

        // A / B en mando derecho
        PrimaryKey = FirstExistingKey({ TEXT("OpenXR_A_Click"),
                                           TEXT("OculusTouch_A_Click"),
                                           TEXT("Gamepad_FaceButton_Bottom") });

        SecondaryKey = FirstExistingKey({ TEXT("OpenXR_B_Click"),
                                           TEXT("OculusTouch_B_Click"),
                                           TEXT("Gamepad_FaceButton_Right") });
    }

    // Leemos ejes analogicos
    const float TriggerVal = (bTrigger && TrigKey.IsValid())
        ? PC->GetInputAnalogKeyState(TrigKey)
        : 0.f;

    const float GripVal = (bGrip && GripKey.IsValid())
        ? PC->GetInputAnalogKeyState(GripKey)
        : 0.f;

    const float StickXVal = (bJoystick && StickXKey.IsValid())
        ? PC->GetInputAnalogKeyState(StickXKey)
        : 0.f;

    const float StickYVal = (bJoystick && StickYKey.IsValid())
        ? PC->GetInputAnalogKeyState(StickYKey)
        : 0.f;

    bool bAnyActivity = false;

    if (FMath::Abs(TriggerVal) > KINDA_SMALL_NUMBER ||
        FMath::Abs(GripVal) > KINDA_SMALL_NUMBER ||
        FMath::Abs(StickXVal) > KINDA_SMALL_NUMBER ||
        FMath::Abs(StickYVal) > KINDA_SMALL_NUMBER)
    {
        bAnyActivity = true;
    }

    // Boton primary (X/A)
    if (bPrimary && PrimaryKey.IsValid() && PC->IsInputKeyDown(PrimaryKey))
    {
        OutCtrl.buttons |= 0x1u;
        bAnyActivity = true;
    }

    // Boton secondary (Y/B)
    if (bSecondary && SecondaryKey.IsValid() && PC->IsInputKeyDown(SecondaryKey))
    {
        OutCtrl.buttons |= 0x2u;
        bAnyActivity = true;
    }

    if (!bAnyActivity)
    {
        // nada pulsado, no marcamos el mando como activo
        return;
    }

    OutCtrl.isActive = 1;
    OutCtrl.trigger = TriggerVal;
    OutCtrl.grip = GripVal;
    OutCtrl.stickX = StickXVal;
    OutCtrl.stickY = StickYVal;

    UE_LOG(LogTelemetry, Log,
        TEXT("[Telemetry][Ctrl] L=%d trig=%.2f grip=%.2f stick(%.2f,%.2f) btn=%u act=%d"),
        bLeft ? 1 : 0,
        OutCtrl.trigger,
        OutCtrl.grip,
        OutCtrl.stickX,
        OutCtrl.stickY,
        OutCtrl.buttons,
        OutCtrl.isActive);
}

// Mapea el indice de Unreal (EHandKeypoint / HandKeyPositions)
// al indice "XR/Unity" que espera la libreria (wrist y palm intercambiadas).
static FORCEINLINE int32 UEHandIndexToXRIndex(int32 UEIndex)
{
    switch (UEIndex)
    {
    case 0: // Palm en Unreal
        return 1; // Palm debe ir en 1
    case 1: // Wrist en Unreal
        return 0; // Wrist debe ir en 0
    default:
        return UEIndex; // el resto coincide
    }
}

bool UTelemetryComponent::FillHandJointsFromOpenXR(VRFrameDataPlain& OutFrame)
{
    OutFrame.leftHandJointCount = 0;
    OutFrame.rightHandJointCount = 0;

    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTelemetry, Log, TEXT("[Telemetry][Joints] no world in FillHandJointsFromOpenXR"));
        return false;
    }

    const float W2M = GetWorldToMeters();
    const int32 MaxJoints = 30; // arrays en VRFrameDataPlain

    auto FillOneSide = [&](EControllerHand Hand,
        FJointSamplePlain* OutArray,
        int32& OutCount,
        const TCHAR* Prefix)
        {
            OutCount = 0;

            FXRMotionControllerData Data;
            UHeadMountedDisplayFunctionLibrary::GetMotionControllerData(World, Hand, Data);

            if (!Data.bValid ||
                Data.HandKeyPositions.Num() == 0 ||
                Data.HandKeyRotations.Num() == 0)
            {
                UE_LOG(LogTelemetry, Log,
                    TEXT("[Telemetry][Joints] %s hand data not valid or no keypoints"), Prefix);
                return;
            }

            // En OpenXR deben ser 26, pero por seguridad limitamos
            const int32 Count = FMath::Min3(
                Data.HandKeyPositions.Num(),
                Data.HandKeyRotations.Num(),
                MaxJoints);

            for (int32 UEIndex = 0; UEIndex < Count; ++UEIndex)
            {
                FVector PosTr;
                FQuat   QTr;

                // HandKeyPositions / Rotations se devuelven en mundo,
                // los convertimos a tracking como el resto de poses
                WorldToTrackingPose(World,
                    Data.HandKeyPositions[UEIndex],
                    Data.HandKeyRotations[UEIndex],
                    PosTr, QTr);

                FVector Pos_m = ToMeters(PosTr, W2M);
                FQuat   Q = QTr;

#if TELEMETRY_USE_OPENXR_AXES
                {
                    FVector Ptmp = Pos_m;
                    FQuat   Qtmp = Q;
                    ToOpenXRAxes(Ptmp, Qtmp, Pos_m, Q);
                }
#endif
                // Remapeamos al indice "XR" que espera tu libreria (0=Wrist, 1=Palm)
                const int32 XRIndex = UEHandIndexToXRIndex(UEIndex);
                if (XRIndex < 0 || XRIndex >= MaxJoints)
                {
                    continue;
                }

                FJointSamplePlain& J = OutArray[XRIndex];
                J.idIndex = XRIndex;
                J.state = 1;
                J.hasPose = 1;

                J.px = Pos_m.X;
                J.py = Pos_m.Y;
                J.pz = Pos_m.Z;

                J.qx = Q.X;
                J.qy = Q.Y;
                J.qz = Q.Z;
                J.qw = Q.W;

                OutCount = FMath::Max(OutCount, XRIndex + 1);
            }
        };

    FillOneSide(EControllerHand::Left,
        OutFrame.leftHandJoints,
        OutFrame.leftHandJointCount,
        TEXT("L"));

    FillOneSide(EControllerHand::Right,
        OutFrame.rightHandJoints,
        OutFrame.rightHandJointCount,
        TEXT("R"));

    // Devolvemos true si al menos una mano tiene joints
    return (OutFrame.leftHandJointCount > 0 ||
        OutFrame.rightHandJointCount > 0);
}

// ============================================================================
// Send frame
// ============================================================================

void UTelemetryComponent::SendFrame(double TimestampSec,
    const FVector& HmdPos_m, const FQuat& HmdQuat,
    bool bLeftValid, const FVector& LPos_m, const FQuat& LQuat,
    bool bRightValid, const FVector& RPos_m, const FQuat& RQuat)
{

    VRFrameDataPlain F{};
    F.timestampSec = TimestampSec;

    ControllerStatePlain LCtrl{}, RCtrl{};
    FillController(true, LCtrl);
    FillController(false, RCtrl);

    // HMD
    F.hmdPose.position[0] = HmdPos_m.X; F.hmdPose.position[1] = HmdPos_m.Y; F.hmdPose.position[2] = HmdPos_m.Z;
    F.hmdPose.rotation[0] = HmdQuat.X;  F.hmdPose.rotation[1] = HmdQuat.Y;  F.hmdPose.rotation[2] = HmdQuat.Z;  F.hmdPose.rotation[3] = HmdQuat.W;

    // Left
    F.leftCtrl.isActive = bLeftValid ? 1 : 0;
    F.leftCtrl.pose.position[0] = LPos_m.X; F.leftCtrl.pose.position[1] = LPos_m.Y; F.leftCtrl.pose.position[2] = LPos_m.Z;
    F.leftCtrl.pose.rotation[0] = LQuat.X;  F.leftCtrl.pose.rotation[1] = LQuat.Y;  F.leftCtrl.pose.rotation[2] = LQuat.Z;  F.leftCtrl.pose.rotation[3] = LQuat.W;
    F.leftCtrl.trigger = LCtrl.trigger;
    F.leftCtrl.grip = LCtrl.grip;
    F.leftCtrl.buttons = LCtrl.buttons;
    F.leftCtrl.stickX = LCtrl.stickX;
    F.leftCtrl.stickY = LCtrl.stickY;

    // Right
    F.rightCtrl.isActive = bRightValid ? 1 : 0;
    F.rightCtrl.pose.position[0] = RPos_m.X; F.rightCtrl.pose.position[1] = RPos_m.Y; F.rightCtrl.pose.position[2] = RPos_m.Z;
    F.rightCtrl.pose.rotation[0] = RQuat.X;  F.rightCtrl.pose.rotation[1] = RQuat.Y;  F.rightCtrl.pose.rotation[2] = RQuat.Z;  F.rightCtrl.pose.rotation[3] = RQuat.W;
    F.rightCtrl.trigger = RCtrl.trigger;
    F.rightCtrl.grip = RCtrl.grip;
    F.rightCtrl.buttons = RCtrl.buttons;
    F.rightCtrl.stickX = RCtrl.stickX;
    F.rightCtrl.stickY = RCtrl.stickY;

    //joints

    if (bHand) // flag handtracking
    {
        FillHandJointsFromOpenXR(F);
    }

    UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] record_frame ts=%.3f left=%d right=%d"),
        F.timestampSec, F.leftCtrl.isActive, F.rightCtrl.isActive);
    telemetry_record_frame(&F);
}

// ============================================================================
// Helpers
// ============================================================================


FVector UTelemetryComponent::ToMeters(const FVector& P_UU, float WorldToMeters)
{
    return P_UU / WorldToMeters;
}

FVector UTelemetryComponent::ToUnityPos(const FVector& Pm)
{
    return FVector(Pm.Y, Pm.Z, Pm.X);
}

FQuat UTelemetryComponent::ToUnityQuat(const FQuat& Q)
{
    return Q;
}

// ============================================================================
// Lifecycle callback
// ============================================================================

void UTelemetryComponent::OnAppWillEnterBackground()
{
    UE_LOG(LogTelemetry, Log, TEXT("[Telemetry] OnAppWillEnterBackground"));
    telemetry_force_upload();
    telemetry_shutdown();

    // these frees were in your file; kept as is
    FMemory::Free(TelemetrySess); TelemetrySess = nullptr;
    FMemory::Free(TelemetryDev);  TelemetryDev = nullptr;
}
