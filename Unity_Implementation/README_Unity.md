# Telemetria – Unity Integration Guide

This document explains how to integrate and use the **Telemetria** library in a **Unity** project using the files provided in this repository.

The integration targets **Android / VR (Meta Quest)** applications and relies on a native library packaged as an **AAR**, together with a minimal C# bootstrap script.

For test purposes, Unity's VR template can be used as a trial project.

---

## 1. Required files

The Unity integration is composed of the following files:

```
Assets/
├── Plugins/
│   ├── Android/
│   │   └── AARTelemetria-debug.aar  # Check main README
│   └── telemetria/
│       └── TelemetriaAPI.cs
└── Scripts/
    └── TelemetriaBootstrap.cs
```

All files must be copied **preserving the same folder structure** inside the Unity project.

---

## 2. Unity project configuration

### 2.1 Switch build platform to Android

1. Open the Unity project
2. Go to `File > Build Profiles`
3. Select **Android** or **Meta Quest**
4. Click **Switch Platform**

---

### 2.2 Player Settings (Android)

Open `Edit > Project Settings > Player`, select the **Android** tab and configure (it should be default configuration) :

#### Other Settings
- **Scripting Backend**: `IL2CPP`
- **Target Architectures**: `ARM64 (arm64-v8a)`
- **API Compatibility Level**: `.NET Standard 2.1`
- **Internet Access**: `Require`

---

## 3. XR configuration (VR projects)

If the application uses VR  (it should be default configuration if VR project):

1. Open `Edit > Project Settings > XR Plug-in Management`
2. Enable **XR Plug-in Management**
3. Enable **OpenXR** for Android
4. Activate the required OpenXR feature set for Meta Quest

Without XR properly configured, no HMD or controller tracking data will be available.

---

## 4. For Android plugin verification

1. Select `AARTelemetria-debug` (or the `.aar` file) in the Project window
2. In the Inspector:
    - Ensure **Android** is enabled
    - Ensure **ARM64** is selected (if applicable)
    - Disable “Any Platform” if platform-specific options are shown

---

## 5. Scene setup

### 5.1 Create a GameObject for Telemetria

1. Open the desired scene
2. In the Hierarchy window:
    - Right click → **Create Empty**

---

### 5.2 Add the Telemetria bootstrap script

1. Select the created empty GameObject
2. Click **Add Component**
3. Search for **TelemetriaBootstrap**
4. Add the component

This script is responsible for initializing and shutting down the Telemetria native library.

---

## 6. Configuration handling

Configuration of `TelemetriaBootstrap.cs` must be on:

- `initialConfig.json`

The file must be located at:

```
/sdcard/Android/data/<package_name>/files/
```



An example of the json is provided on this github main directory. Ensure it exists in the expected location before running the application.

---

## 7. Build and deploy

1. Connect the Meta Quest device via USB (Developer Mode enabled)
4. Click **Build And Run**

---

## 8. Verifying correct execution

Correct execution can be verified by:

- Unity Console logs during initialization
- Android logs using `adb logcat`
- Presence of generated output files (e.g. C3D files) if enabled
- Successful network transmission if HTTP export is configured

---

## 9. Supported platform

- **Android**
- Tested on **Meta Quest devices**

