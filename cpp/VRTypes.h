#pragma once

// Basic "plain" types for VR telemetry.
// These structs are designed to be simple C-compatible layouts that can be
// filled from Unity/Unreal and passed directly to the native telemetry API.

// Simple pose: position (meters) + orientation (quaternion).
struct VRPosePlain {
    float position[3];
    float rotation[4];
};

// Per-controller state (left or right).
struct ControllerStatePlain {
    VRPosePlain pose; //pose in tracking space
    unsigned int buttons; //button bitmask
    float trigger; //analog trigger value from 0 to 1
    float grip; //analog grip value from 0 to 1
    // Analog stick
    float stickX;
    float stickY;
    int isActive;
};

// Each hand joint (there are 26 per hand with OpenXr specification).
struct JointSamplePlain {
    int   idIndex;   // joint index (see C3DRecorder.cpp or OpenXrHandJointEXT docs)
    int   state;     // flags/estado por joint
    float px, py, pz; // position
    float qx, qy, qz, qw; // rotation
    unsigned char hasPose; // 0=false 1=true
    unsigned char _pad_[3]; // padding for a 4byte alignment
};

// Complete VR frame data, one instance is equal to one real sample
struct VRFrameDataPlain {
    double timestampSec; // real time when frame was captured
    VRPosePlain hmdPose; // head mounted display pose
    ControllerStatePlain leftCtrl; // left controller
    ControllerStatePlain rightCtrl; // right controller
    // left hand joints. 30 max in case we receive more joints bc of an Unreal Engine weird implementation (only overriding default Unreal-openxr lib)
    JointSamplePlain leftHandJoints[30];
    int leftHandJointCount; // will be used to know how many empty array cells we can ignore
    JointSamplePlain rightHandJoints[30]; // right hand joints
    int rightHandJointCount;
};