cmake_minimum_required(VERSION 3.22)

# Project
project(telemetria LANGUAGES C CXX)

# C++ standard and general flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type defaults
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Common compile options
add_compile_options(
        -fno-exceptions
        -fno-rtti
)

# Android specific defines
if(ANDROID)
    add_definitions(-DANDROID)
endif()

# Sources
set(TELEMETRIA_SOURCES
        TelemetriaAPI.cpp
        GestorTelemetria.cpp
        AndroidUploader.cpp
)

# Headers are added for IDEs and clarity
set(TELEMETRIA_HEADERS
        TelemetriaAPI.h
        GestorTelemetria.h
        AndroidUploader.h
        TiposVR.h
        TiposTelemetria.h
)

# Shared library
add_library(telemetria SHARED
        ${TELEMETRIA_SOURCES}
        ${TELEMETRIA_HEADERS}
)

# Include directories
target_include_directories(telemetria
        PRIVATE
        .
)

# Strip symbols in Release for smaller .so
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_link_options(telemetria PRIVATE -s)
    endif()
endif()

# Link android log
find_library(ANDROID_LOG_LIB log)
if(ANDROID_LOG_LIB)
    target_link_libraries(telemetria ${ANDROID_LOG_LIB})
endif()

# enlaza la STL compartida
target_link_libraries(telemetria
        ${ANDROID_LOG_LIB}
        c++_shared
)