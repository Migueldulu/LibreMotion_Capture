using System;
using System.Runtime.InteropServices;
using UnityEngine;

public static class TelemetriaAPI
{
    [DllImport("telemetria")]
    private static extern void telemetry_set_java_context(IntPtr vm, IntPtr activity);

    [DllImport("telemetria")]
    public static extern int telemetry_initialize(ref TelemetryConfigPlain cfg);

    [DllImport("telemetria")]
    public static extern void telemetry_record_frame(ref VRFrameDataPlain frame);

    [DllImport("telemetria")]
    public static extern void telemetry_force_upload();

    [DllImport("telemetria")]
    public static extern void telemetry_shutdown();
    // Devuelve un bitmask con los botones para enviar. LLamar despues de initialize 
    [DllImport("telemetria")]
    public static extern uint telemetry_get_feature_flags();

    public static void InyectarContextoJava()
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        using (var unityPlayer = new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
        using (var activity = unityPlayer.GetStatic<AndroidJavaObject>("currentActivity"))
        {
            telemetry_set_java_context(IntPtr.Zero, activity.GetRawObject());
        }
#endif
    }
}

[StructLayout(LayoutKind.Sequential)]
public struct TelemetryConfigPlain
{
    public IntPtr sessionId;
    public IntPtr deviceInfo;
}

[StructLayout(LayoutKind.Sequential)]
public struct VRPosePlain
{
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
    public float[] position;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
    public float[] rotation;
}

[StructLayout(LayoutKind.Sequential)]
public struct ControllerStatePlain
{
    public VRPosePlain pose;
    public uint buttons;
    public float trigger;
    public float grip;
    // Analog stick
    public float stickX;
    public float stickY;
    public int isActive;
}

public struct JointSamplePlain
{
    public int idIndex;    // indice XRHandJointID (XRHandJointIDUtility.ToIndex)
    public int state;      // trackingState raw (int)
    public float px, py, pz;
    public float qx, qy, qz, qw;
    public byte hasPose;
}


[StructLayout(LayoutKind.Sequential)]
public struct VRFrameDataPlain
{
    public double timestampSec;
    public VRPosePlain hmdPose;
    public ControllerStatePlain leftCtrl;
    public ControllerStatePlain rightCtrl;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 30)] // el marshall sirve para convertir a un array fijo
    public JointSamplePlain[] leftHandJoints;
    public int leftHandJointCount;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 30)]
    public JointSamplePlain[] rightHandJoints;
    public int rightHandJointCount;
}
