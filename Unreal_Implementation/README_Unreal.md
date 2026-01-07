# Telemetria – Unreal Engine Integration Guide

This document explains how to integrate and use the **Telemetria** library in an **Unreal Engine** project using the files provided in this repository.

The integration targets **Android / VR (Meta Quest)** applications and relies on a native shared library (`.so`) packaged inside an Unreal Engine **Runtime Plugin**, together with a C++ component that interfaces with the engine's XR system.

For test purposes, Unreal Engine VR or XR templates can be used as a trial project.

---

## 1. Required files

The Unreal Engine integration is composed of the following files:

```
YourProject/
└── Plugins/
    └── Telemetria/
        ├── Telemetria.uplugin
        ├── README_Unreal.md
        ├── Source/
        │   └── Telemetria/
        │       ├── Telemetria.Build.cs
        │       ├── Public/
        │       │   ├── TelemetriaComponent.h
        │       │   └── TelemetriaAPI.h
        │       └── Private/
        │           ├── TelemetriaComponent.cpp
        │           └── TelemetriaAPI.cpp
        ├── ThirdParty/
        │   └── TelemetriaNative/
        │       ├── Include/
        │       │   └── telemetria.h
        │       └── Lib/
        │           └── Android/
        │               └── arm64-v8a/
        │                   └── libtelemetria.so
        └── Resources/
            └── Icon128.png
```

All files must be copied **preserving the same folder structure** inside the Unreal project.

---

## 2. Unreal Engine project configuration

### 2.1 Enable Android support

1. Open the Unreal Engine project
2. Go to `Edit > Plugins`
3. Ensure **Android** and **OpenXR** plugins are enabled
4. Restart the editor if required

---

### 2.2 Android project settings

Open `Edit > Project Settings` and configure:

#### Platforms > Android
- **Target SDK Version**: API Level 24 or higher
- **Supported Architectures**: `arm64-v8a`
- **Internet Access**: Enabled

---

## 3. XR configuration (VR projects)

1. Open `Edit > Project Settings > XR`
2. Ensure **OpenXR** is selected as the XR runtime
3. Enable required interaction profiles for Meta Quest
4. Verify HMD and controller tracking are detected correctly

Without XR properly configured, no HMD or controller tracking data will be available.

For both hand tracking and controller's tracking, they must be correctly configured for the device being used.
If you want to register the hand joints, OpenXR configuration must be used.

---

## 4. Plugin verification

1. Open `Edit > Plugins`
2. Locate **Telemetria**
3. Ensure the plugin is:
    - Enabled
    - Loaded as **Runtime**
4. Restart the editor if requested

---

## 5. Scene setup

### 5.1 Add Telemetria component

1. Open the desired level
2. Select the Pawn or Actor responsible for VR tracking (e.g. VR Pawn)
3. Click **Add Component**
4. Search for **TelemetriaComponent**
5. Add the component

This component is responsible for initializing, updating, and shutting down the Telemetria native library.

---

## 6. Configuration handling

Configuration of Telemetria is performed using an external JSON file:

- `initialConfig.json`

The file must be located at:

```
/sdcard/Android/data/<package_name>/files/
```

An example configuration file is provided in the root of the repository. Ensure it exists in the expected location before running the application.

---

## 7. Build and deploy

1. Connect the Meta Quest device via USB (Developer Mode enabled)
2. Select **Android** as the target platform
3. Package the project
4. Install and run the application on the device

---

## 8. Verifying correct execution

Correct execution can be verified by:

- Unreal Engine output logs during initialization
- Android logs using `adb logcat`
- Presence of generated output files (e.g. C3D files) if enabled
- Successful network transmission if HTTP export is configured

---

## 9. Supported platform

- **Android**
- Tested on **Meta Quest devices**
- Unreal Engine **5.3+**

