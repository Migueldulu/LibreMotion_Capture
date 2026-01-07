cmake_minimum_required(VERSION 3.22)
project(telemetria LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Android specific
if(ANDROID)
    add_definitions(-DANDROID)
endif()

# libreria que trabaja c3d
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libs" FORCE)
set(EZC3D_BUILD_SHARED OFF CACHE BOOL "Build ezc3d shared" FORCE)
set(EZC3D_BUILD_STATIC ON  CACHE BOOL "Build ezc3d static" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/ezc3d)

# Sources
add_library(telemetria SHARED
        TelemetryAPI.cpp
        TelemetryManager.cpp
        AndroidUploader.cpp
        configReader.cpp
        C3DRecorder.cpp
)

target_compile_options(telemetria PRIVATE
        -fno-exceptions
        -fno-rtti
)

# Include directories
target_include_directories(telemetria PRIVATE
        .
        ${CMAKE_SOURCE_DIR}/thirdparty/ezc3d/include
        ${CMAKE_BINARY_DIR}/thirdparty/ezc3d/include
)

# FORZAR libc++ estático y eliminar -static-libstdc++
target_link_libraries(telemetria
        ezc3d
        -lc++_static
        -lc++abi
        -landroid
        -llog
        -latomic
)

# Override del flag problemático - ELIMINA -static-libstdc++
target_link_options(telemetria PRIVATE
        "SHELL:-static-libstdc++"
        "SHELL:-Wl,-Bdynamic"
)

# Strip solo en Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_options(telemetria PRIVATE -s)
endif()