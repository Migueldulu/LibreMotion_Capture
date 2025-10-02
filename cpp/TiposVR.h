#pragma once

// Tipos basicos plain para VR

struct VRPosePlain {
    float position[3];
    float rotation[4];
};

struct ControllerStatePlain {
    VRPosePlain pose;
    unsigned int buttons;
    float trigger;
    float grip;
    int isActive;
};

struct VRFrameDataPlain {
    double timestampSec;
    VRPosePlain hmdPose;
    ControllerStatePlain leftCtrl;
    ControllerStatePlain rightCtrl;
};