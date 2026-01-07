# LibreMotion_Capture

LibreMotion_Capture es una libreria nativa desarrollada en C++ para Android cuyo objetivo es capturar datos de movimiento de dispositivos de Realidad Virtual, incluyendo HMD, controladores y seguimiento de manos, desde aplicaciones desarrolladas en Unity y Unreal Engine.

La libreria forma parte del ecosistema LibreMotion y esta disenada para facilitar la adquisicion, normalizacion y persistencia de datos de movimiento con fines de analisis biomecanico, investigacion o telemetria.

---

## Contenido del repositorio

Este repositorio incluye:

- Nucleo nativo en C++ compilable como libreria compartida (.so) en la carpeta /cpp
- Wrapper Android empaquetado como AAR para su uso en Unity en /LibreriaTelemetria
- Scripts para el uso de la libreria en Unity
- Plug in para el uso de la libreria en Unreal Engine
- Ejemplo de JSON de configuración inicial `initialConfig.json`
- Scripts y configuracion necesarios para la compilacion

---

## Requisitos

- Android NDK
- CMake
- Ninja
- Gradle

Nota: el flujo de compilacion actual esta orientado a Android arm64-v8a con API 24.

---

## Compilacion de la libreria nativa (.so)

### 1. Configurar la ruta del NDK

En este ejemplo se utiliza el NDK incluido con Unity. Ajusta la ruta si tu version de Unity es distinta.

```powershell
$NDK = "C:/Program Files/Unity/Hub/Editor/6000.2.5f1/Editor/Data/PlaybackEngines/AndroidPlayer/NDK"
```

### 2. Generar los archivos de build con CMake

Desde la carpeta que contiene los archivos fuente (.cpp):

```powershell
cmake -S . -B build-arm64 -G "Ninja" ^
  -D CMAKE_BUILD_TYPE=Release ^
  -D CMAKE_TOOLCHAIN_FILE="%NDK%/build/cmake/android.toolchain.cmake" ^
  -D ANDROID_ABI=arm64-v8a ^
  -D ANDROID_PLATFORM=24 ^
  -D ANDROID_STL=c++_static ^
  -D ANDROID_ARM_NEON=ON
```

### 3. Compilar la libreria

```powershell
cmake --build build-arm64 --config Release
```

### 4. Resultado de la compilacion

Se generara el archivo:

```
build-arm64/libreriaTelemetria.so
```

Este archivo debe copiarse manualmente a la siguiente ruta para generar el AAR:

```
LibreriaTelemetria/AARTelemetria/src/main/jniLibs/arm64-v8a/
```

---

## Generacion del AAR (Android Library)

### 1. Archivos necesarios

Para generar correctamente el AAR, deben existir los siguientes archivos y rutas:

En `AARTelemetria/src/`:
- build.gradle
- AndroidManifest.xml

En `AARTelemetria/src/main/`:
- consumer-rules.pro

En `AARTelemetria/src/main/java/io/github/migueldulu/telemetria/`:
- AyudanteHttp.java

En la raiz del proyecto (`LibreriaTelemetria/`):
- settings.gradle
- build.gradle (vacio, solo necesario si Gradle produce errores)

### 2. Generar el wrapper de Gradle (si no existe)

Desde la carpeta `LibreriaTelemetria`:

```powershell
gradle wrapper --gradle-version 8.7 --distribution-type all
```

### 3. Compilar el AAR

```powershell
.\gradlew.bat :AARTelemetria:assembleDebug
```

### 4. Resultado

El archivo AAR se generara en:

```
AARTelemetria/build/outputs/aar/
```

---

## Licencia

Este proyecto forma parte del ecosistema LibreMotion y se distribuye como software open-source.
La licencia definitiva se anadira en futuras versiones.
