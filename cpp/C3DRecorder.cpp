#include "C3DRecorder.h"
#include "configReader.h"

#include <android/log.h>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <ezc3d/ezc3d.h> // Libreria externa open source para tratamiento de archivos c3d
#include <ezc3d/Parameters.h>
#include <ezc3d/Data.h>

#define LOG_TAG "telemetria"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

struct Vector3 {
    float x, y, z;
};

C3DRecorder::C3DRecorder() {}
C3DRecorder::~C3DRecorder() {
    // Por seguridad: si alguien se olvida de llamar a finalize,
    if (initialized_ && !finalized_) {
        C3Dfinalize();
    }
}

bool C3DRecorder::C3DisInitialized() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return initialized_;
}

// Por si el nombre de sesion trae algun caracter no permitido
std::string C3DRecorder::sanitizeSessionId(const std::string& raw) const {
    std::string s = raw;
    // Reemplaza caracteres raros por '_'
    for (char& c : s) {
        if (!( (c >= '0' && c <= '9') ||
               (c >= 'A' && c <= 'Z') ||
               (c >= 'a' && c <= 'z') ||
               c == '-' || c == '_' )) {
            c = '_';
        }
    }
    if (s.empty()) s = "unknown";
    return s;
}

// Construye /sdcard/Android/data/<pkg>/files/session_<sessionId>.c3d
// reutilizando configReader::getExpectedConfigPath
std::string C3DRecorder::buildOutputPath(const UploaderConfig& cfg) {
    std::string cfgPath;
    if (!configReader::getExpectedConfigPath(cfgPath)) {
        LOGE("C3DRecorder: getExpectedConfigPath failed, cannot build C3D path");
        return "";
    }

    // Nos quedamos solo con el directorio de initialConfig.json
    size_t pos = cfgPath.find_last_of("/\\");
    std::string dir;
    if (pos == std::string::npos) {
        dir = "/sdcard";
    } else {
        dir = cfgPath.substr(0, pos);
    }

    std::string sid = sanitizeSessionId(cfg.sessionId);
    std::string path = dir + "/session_" + sid + ".c3d";
    return path;
}

bool C3DRecorder::C3Dinitialize(const UploaderConfig& cfg, int frameRate) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (initialized_) {
        LOGI("C3DRecorder already initialized, ignoring.");
        return true;
    }

    filePath_ = buildOutputPath(cfg);
    if (filePath_.empty()) {
        LOGE("C3DRecorder: empty file path, disabling C3D recording");
        initialized_ = false;
        return false;
    }

    frameRate_ = frameRate;
    if (frameRate_ <= 0) frameRate_ = 60;

    frames_.clear();
    finalized_ = false;
    initialized_ = true;

    LOGI("C3DRecorder initialized. Path='%s', frameRate=%d",
         filePath_.c_str(), frameRate_);

    // No abrimos nada todavia: ezc3d escribira en finalize.
    return true;
}

void C3DRecorder::C3DrecordFrame(const VRFrameDataPlain& frame) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!initialized_ || finalized_) {
        return;
    }
    frames_.push_back(frame);
}

// Streaming real mas adelante,
void C3DRecorder::C3Dflush() {
    // En esta primera version no hacemos flush parcial para evitar
    // complicar el layout del C3D. Se deja preparado para futuro.
}

void C3DRecorder::C3Dfinalize() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!initialized_ || finalized_) {
        return;
    }

    if (frames_.empty()) {
        LOGI("C3DRecorder: no frames to write, skipping C3D file.");
        finalized_ = true;
        return;
    }

    LOGI("C3DRecorder: writing C3D file '%s' with %zu frames",
         filePath_.c_str(), frames_.size());

// Sin try/catch porque estamos compilando con -fno-exceptions
    writeC3D();

    finalized_ = true;
}

//nombres OpenXRHandJoints -> cambiando palm y wrist de orden porque asi se reciben
// Cambiamos INDEX_PROXIMAl por FIN para que coincida con Gait Model (LFIN y RFIN)
static const char* kHandJointNames[26] = {
        "WRIST",
        "PALM",
        "THUMB_METACARPAL",
        "THUMB_PROXIMAL",
        "THUMB_DISTAL",
        "THUMB_TIP",
        "INDEX_METACARPAL",
        "FIN",
        "INDEX_INTERMEDIATE",
        "INDEX_DISTAL",
        "INDEX_TIP",
        "MIDDLE_METACARPAL",
        "MIDDLE_PROXIMAL",
        "MIDDLE_INTERMEDIATE",
        "MIDDLE_DISTAL",
        "MIDDLE_TIP",
        "RING_METACARPAL",
        "RING_PROXIMAL",
        "RING_INTERMEDIATE",
        "RING_DISTAL",
        "RING_TIP",
        "LITTLE_METACARPAL",
        "LITTLE_PROXIMAL",
        "LITTLE_INTERMEDIATE",
        "LITTLE_DISTAL",
        "LITTLE_TIP"
};

void C3DRecorder::buildPointNames(std::vector<std::string>& outPointNames) const {
    outPointNames.clear();
    outPointNames.reserve(3 + 26 + 26 + 4);

    //segun el modelo de Gait sera RFHD o LFWD
    outPointNames.emplace_back("HEAD");
    outPointNames.emplace_back("LFHD");
    outPointNames.emplace_back("RFHD");

    // Joints mano izquierda siguiendo el orden de XrHandJointEXT
    for (int i = 0; i < 26; ++i) {
        outPointNames.emplace_back(std::string("L") + kHandJointNames[i]);
    }

    // Joints mano derecha siguiendo el mismo orden
    for (int i = 0; i < 26; ++i) {
        outPointNames.emplace_back(std::string("R") + kHandJointNames[i]);
    }

    // Puntos extra tipo Plug-in Gait para la muñeca
    outPointNames.emplace_back("LWRA");
    outPointNames.emplace_back("LWRB");
    outPointNames.emplace_back("RWRA");
    outPointNames.emplace_back("RWRB");
}

void C3DRecorder::buildAnalogNames(std::vector<std::string>& outAnalogNames) const {
    outAnalogNames.clear();

    auto addQuat = [&](const std::string& prefix) {
        outAnalogNames.emplace_back(prefix + "_qx");
        outAnalogNames.emplace_back(prefix + "_qy");
        outAnalogNames.emplace_back(prefix + "_qz");
        outAnalogNames.emplace_back(prefix + "_qw");
    };

    addQuat("HMD");

    // Manos: L_*
    for (int i = 0; i < 26; ++i) {
        addQuat(std::string("L") + kHandJointNames[i]);
    }

    // Manos: R_*
    for (int i = 0; i < 26; ++i) {
        addQuat(std::string("R") + kHandJointNames[i]);
    }
    // Canal extra: timestamp en segundos
    outAnalogNames.emplace_back("RealTime");
}

//pequeno helper para calcular las posiciones exteriores de la cabeza
//primero rotamos el vector y luego sumamos 7.5cm (15/2) a cada lado
void calculateHeadWidth(const VRPosePlain& midPos,Vector3& pointRight,Vector3& pointLeft, float headWidth) {
    const float halfWidth = headWidth / 2.0f;

    // Extraer componentes
    float qx = midPos.rotation[0], qy =  midPos.rotation[1], qz =  midPos.rotation[2], qw =  midPos.rotation[3];

    // Vector lateral en el sistema local de la cabeza
    Vector3 vLocal;
    vLocal.x = halfWidth; vLocal.y = 0.0f; vLocal.z = 0.0f;

    // Parte vectorial del cuaternion
    Vector3 qv;
    qv.x = qx; qv.y = qy; qv.z = qz;

    // t = 2 * cross(qv, vLocal)
    Vector3 t;
    t.x = 2.0f * (qv.y * vLocal.z - qv.z * vLocal.y);
    t.y = 2.0f * (qv.z * vLocal.x - qv.x * vLocal.z);
    t.z = 2.0f * (qv.x * vLocal.y - qv.y * vLocal.x);

    // vWorld = vLocal + qw * t + cross(qv, t)
    Vector3 vWorld;
    vWorld.x = vLocal.x + qw * t.x + (qv.y * t.z - qv.z * t.y);
    vWorld.y = vLocal.y + qw * t.y + (qv.z * t.x - qv.x * t.z);
    vWorld.z = vLocal.z + qw * t.z + (qv.x * t.y - qv.y * t.x);

    Vector3 worldOffset = vWorld;

    // Aplicar a la posicion
    pointRight = {
            midPos.position[0] + worldOffset.x,
            midPos.position[1] + worldOffset.y,
            midPos.position[2] + worldOffset.z
    };

    pointLeft = {
            midPos.position[0] - worldOffset.x,
            midPos.position[1] - worldOffset.y,
            midPos.position[2] - worldOffset.z
    };
}
void calculateWristWidthFromJoint(const JointSamplePlain& joint, Vector3& pointRight, Vector3& pointLeft, float wristWidth) {
    const float halfWidth = wristWidth / 2.0f;

    float qx = joint.qx;
    float qy = joint.qy;
    float qz = joint.qz;
    float qw = joint.qw;

    // Vector lateral en el sistema local de la muneca
    Vector3 vLocal;
    vLocal.x = halfWidth; vLocal.y = 0.0f; vLocal.z = 0.0f;

    // Parte vectorial del cuaternion
    Vector3 qv;
    qv.x = qx; qv.y = qy; qv.z = qz;

    // t = 2 * cross(qv, vLocal)
    Vector3 t;
    t.x = 2.0f * (qv.y * vLocal.z - qv.z * vLocal.y);
    t.y = 2.0f * (qv.z * vLocal.x - qv.x * vLocal.z);
    t.z = 2.0f * (qv.x * vLocal.y - qv.y * vLocal.x);

    // vWorld = vLocal + qw * t + cross(qv, t)
    Vector3 vWorld;
    vWorld.x = vLocal.x + qw * t.x + (qv.y * t.z - qv.z * t.y);
    vWorld.y = vLocal.y + qw * t.y + (qv.z * t.x - qv.x * t.z);
    vWorld.z = vLocal.z + qw * t.z + (qv.x * t.y - qv.y * t.x);

    // Aplicar a la posicion del joint
    pointRight = {
            joint.px + vWorld.x,
            joint.py + vWorld.y,
            joint.pz + vWorld.z
    };

    pointLeft = {
            joint.px - vWorld.x,
            joint.py - vWorld.y,
            joint.pz - vWorld.z
    };
}

// Convierte metros a milimetros (solo posiciones)
inline void metersToMillimeters(float& x, float& y, float& z) {
    const float k = 1000.0f;
    x *= k;
    y *= k;
    z *= k;
}
inline void metersToMillimeters(Vector3& v) {
    metersToMillimeters(v.x, v.y, v.z);
}

// Sistema coordenadas OpenXR to gait: X_g = -Z_old (delante)  Y_g = -X_old (izq)  Z_g =  Y_old (arriba)
inline void openxrToGaitAxes(float& x, float& y, float& z) {
    const float ox = x;
    const float oy = y;
    const float oz = z;

    x = -oz;  // delantero +
    y = -ox;  // izquierda +
    z =  oy;  // arriba +
}
inline void openxrToGaitAxes(Vector3& v) {
    openxrToGaitAxes(v.x, v.y, v.z);
}

// Multiplica q_out = q_a * q_b (para cambiar sistema de cuaterniones)
inline void quatMul(const float ax, const float ay, const float az, const float aw,
                    const float bx, const float by, const float bz, const float bw,
                    float& rx, float& ry, float& rz, float& rw) {
    rx = aw*bx + ax*bw + ay*bz - az*by;
    ry = aw*by - ax*bz + ay*bw + az*bx;
    rz = aw*bz + ax*by - ay*bx + az*bw;
    rw = aw*bw - ax*bx - ay*by - az*bz;
}

// Aplica el cambio de ejes OpenXR -> gait al cuaternio (x,y,z,w)
inline void openxrToGaitAxesQuat(float& qx, float& qy, float& qz, float& qw) {
    // Cuaternion fijo que implementa la misma transformacion que openxrToGaitAxes en vectores
    const float ax =  0.5f;
    const float ay = -0.5f;
    const float az = -0.5f;
    const float aw =  0.5f;

    float rx, ry, rz, rw;
    quatMul(ax, ay, az, aw, qx, qy, qz, qw, rx, ry, rz, rw);

    qx = rx;
    qy = ry;
    qz = rz;
    qw = rw;
}

// --- Escritura real con ezc3d (version en memoria, simple) ---
void C3DRecorder::writeC3D() {
    // Creamos un C3D vacio
    ezc3d::c3d c3d;

    // Nombres de puntos y canales
    std::vector<std::string> pointNames;
    std::vector<std::string> analogNames;
    buildPointNames(pointNames);
    buildAnalogNames(analogNames);

    const size_t nbPoints  = pointNames.size();
    const size_t nbAnalogs = analogNames.size();
    const size_t nbFrames  = frames_.size();

    // Declarar los puntos y canales para que los parametros se ajusten
    for (const auto& name : pointNames) {
        c3d.point(name);
    }
    for (const auto& name : analogNames) {
        c3d.analog(name);
    }

    // Ajustar parametros basicos de RATE
    {
        // POINT:UNITS
        ezc3d::ParametersNS::GroupNS::Parameter pointUnits("UNITS");
        pointUnits.set(std::vector<std::string>{"mm"});
        c3d.parameter("POINT", pointUnits);
        // POINT:RATE
        ezc3d::ParametersNS::GroupNS::Parameter pointRate("RATE");
        pointRate.set(std::vector<double>{ static_cast<double>(frameRate_) });
        c3d.parameter("POINT", pointRate);

        // ANALOG:RATE = frameRate tambien (simple: 1 sample analog por frame)
        ezc3d::ParametersNS::GroupNS::Parameter analogRate("RATE");
        analogRate.set(std::vector<double>{ static_cast<double>(frameRate_) });
        c3d.parameter("ANALOG", analogRate);
    }

    // Dejar que ezc3d actualice header segun parametros y datos
    // (lo hace internamente cuando metemos frames).

    // Construir frames uno a uno
    for (size_t i = 0; i < nbFrames; ++i) {
        const VRFrameDataPlain& f = frames_[i];

        ezc3d::DataNS::Frame frame;

        // --- Puntos 3D ---
        {
            using ezc3d::DataNS::Points3dNS::Point;
            using ezc3d::DataNS::Points3dNS::Points;

            Points points(nbPoints);

            // Inicializamos todos como invalidos (residual = -1)
            for (size_t pi = 0; pi < nbPoints; ++pi) {
                Point p;
                p.x(0.0);
                p.y(0.0);
                p.z(0.0);
                p.residual(-1.0);
                points.point(p, static_cast<int>(pi));
            }

            // Indices coherentes con buildPointNames
            const size_t IDX_HEAD        = 0;
            const size_t IDX_LFHD        = 1;
            const size_t IDX_RFHD        = 2;
            const size_t IDX_L_HAND_BASE = 3;                        // 26 joints L
            const size_t IDX_R_HAND_BASE = IDX_L_HAND_BASE + 26;     // 26 joints R
            const int FIN_JOINT_INDEX = 7; // en kHandJointNames[7] = "FIN"
            const size_t IDX_LFIN = IDX_L_HAND_BASE + FIN_JOINT_INDEX;
            const size_t IDX_RFIN = IDX_R_HAND_BASE + FIN_JOINT_INDEX;
            const size_t IDX_LWRA        = IDX_R_HAND_BASE + 26;
            const size_t IDX_LWRB        = IDX_LWRA + 1;
            const size_t IDX_RWRA        = IDX_LWRA + 2;
            const size_t IDX_RWRB        = IDX_LWRA + 3;

            // HMD (HEAD)
            {
                float x = f.hmdPose.position[0];
                float y = f.hmdPose.position[1];
                float z = f.hmdPose.position[2];

                openxrToGaitAxes(x, y, z);
                metersToMillimeters(x, y, z);

                Point p;
                p.x(x);
                p.y(y);
                p.z(z);
                p.residual(0.0);
                points.point(p, static_cast<int>(IDX_HEAD));
            }

            // Cabeza: LFHD / RFHD
            Vector3 headRight, headLeft;
            calculateHeadWidth(f.hmdPose, headRight, headLeft, 0.15f);

            // LFHD = lado izquierdo
            openxrToGaitAxes(headLeft);
            metersToMillimeters(headLeft);
            {
                Point p;
                p.x(headLeft.x);
                p.y(headLeft.y);
                p.z(headLeft.z);
                p.residual(0.0);
                points.point(p, static_cast<int>(IDX_LFHD));
            }

            // RFHD = lado derecho
            openxrToGaitAxes(headRight);
            metersToMillimeters(headRight);
            {
                Point p;
                p.x(headRight.x);
                p.y(headRight.y);
                p.z(headRight.z);
                p.residual(0.0);
                points.point(p, static_cast<int>(IDX_RFHD));
            }

            // L_CTRL (siempre que este activo se supone que hay 0 joints pero metemos doble comprobacion)
            if (f.leftCtrl.isActive && f.leftHandJointCount==0) {
                float x = f.leftCtrl.pose.position[0];
                float y = f.leftCtrl.pose.position[1];
                float z = f.leftCtrl.pose.position[2];

                const bool isZero = (x == 0.0f && y == 0.0f && z == 0.0f);
                openxrToGaitAxes(x, y, z);
                metersToMillimeters(x, y, z);

                if (!isZero) {
                    Point p;
                    p.x(x);
                    p.y(y);
                    p.z(z);
                    p.residual(0.0);
                    points.point(p, static_cast<int>(IDX_LFIN));
                }
            }

            // R_CTRL (si esta activo)
            if (f.rightCtrl.isActive && f.rightHandJointCount==0) {
                float x = f.rightCtrl.pose.position[0];
                float y = f.rightCtrl.pose.position[1];
                float z = f.rightCtrl.pose.position[2];
                const bool isZero = (x == 0.0f && y == 0.0f && z == 0.0f);

                openxrToGaitAxes(x, y, z);
                metersToMillimeters(x, y, z);

                if (!isZero) {
                    Point p;
                    p.x(x);
                    p.y(y);
                    p.z(z);
                    p.residual(0.0);
                    points.point(p, static_cast<int>(IDX_RFIN));
                }
            }

            // Manos: joints L
            for (int j = 0; j < f.leftHandJointCount && j < 26; ++j) {
                const auto& s = f.leftHandJoints[j];
                float x = s.px;
                float y = s.py;
                float z = s.pz;

                const bool isZero = (x == 0.0f && y == 0.0f && z == 0.0f);
                openxrToGaitAxes(x, y, z);
                metersToMillimeters(x, y, z);

                if(!isZero) {
                    Point p;
                    p.x(x);
                    p.y(y);
                    p.z(z);
                    p.residual(s.hasPose ? 0.0 : -1.0);
                    points.point(p, static_cast<int>(IDX_L_HAND_BASE + j));
                }
            }

            // Manos: joints R
            for (int j = 0; j < f.rightHandJointCount && j < 26; ++j) {
                const auto& s = f.rightHandJoints[j];
                float x = s.px;
                float y = s.py;
                float z = s.pz;

                const bool isZero = (x == 0.0f && y == 0.0f && z == 0.0f);
                openxrToGaitAxes(x, y, z);
                metersToMillimeters(x, y, z);

                if(!isZero) {
                    Point p;
                    p.x(x);
                    p.y(y);
                    p.z(z);
                    p.residual(s.hasPose ? 0.0 : -1.0);
                    points.point(p, static_cast<int>(IDX_R_HAND_BASE + j));
                }
            }

            // Puntos extra de muneca tipo LWRA/LWRB/RWRA/RWRB
            // Mano izquierda: joint 0 = WRIST
            if (f.leftHandJointCount > 0) {
                const auto& wrist = f.leftHandJoints[0];
                if (wrist.hasPose) {
                    Vector3 wra, wrb;
                    calculateWristWidthFromJoint(wrist, wra, wrb, 0.06f);

                    const bool isZero = (wrist.px == 0.0f && wrist.py == 0.0f && wrist.pz == 0.0f);
                    openxrToGaitAxes(wra);
                    metersToMillimeters(wra);
                    openxrToGaitAxes(wrb);
                    metersToMillimeters(wrb);

                    if(!isZero){
                        Point p;
                        p.x(wra.x);
                        p.y(wra.y);
                        p.z(wra.z);
                        p.residual(0.0);
                        points.point(p, static_cast<int>(IDX_LWRA));
                    }
                    if(!isZero){
                        Point p;
                        p.x(wrb.x);
                        p.y(wrb.y);
                        p.z(wrb.z);
                        p.residual(0.0);
                        points.point(p, static_cast<int>(IDX_LWRB));
                    }
                }
            }

            // Mano derecha
            if (f.rightHandJointCount > 0) {
                const auto& wrist = f.rightHandJoints[0];
                if (wrist.hasPose) {
                    Vector3 wra, wrb;
                    calculateWristWidthFromJoint(wrist, wra, wrb, 0.06f);

                    const bool isZero = (wrist.px == 0.0f && wrist.py == 0.0f && wrist.pz == 0.0f);
                    openxrToGaitAxes(wra);
                    metersToMillimeters(wra);
                    openxrToGaitAxes(wrb);
                    metersToMillimeters(wrb);

                    // OJO: invertimos A/B para que RWRA/RWRB no queden cruzados
                    if(!isZero){
                        Point p;
                        p.x(wrb.x);
                        p.y(wrb.y);
                        p.z(wrb.z);
                        p.residual(0.0);
                        points.point(p, static_cast<int>(IDX_RWRA));
                    }
                    if(!isZero){
                        Point p;
                        p.x(wra.x);
                        p.y(wra.y);
                        p.z(wra.z);
                        p.residual(0.0);
                        points.point(p, static_cast<int>(IDX_RWRB));
                    }
                }
            }

            frame.points() = points;
        }

        // --- Analogicos (cuaterniones + flag) ---
        {
            using ezc3d::DataNS::AnalogsNS::Channel;
            using ezc3d::DataNS::AnalogsNS::SubFrame;
            using ezc3d::DataNS::AnalogsNS::Analogs;
            const size_t nbAnalogs = analogNames.size();
            // Solo 1 subframe por frame (ANALOG:RATE == POINT:RATE)
            SubFrame subframe;
            subframe.nbChannels(nbAnalogs);

            auto setChan = [&](size_t idx, float value) {
                Channel ch;
                ch.data(value);
                subframe.channel(ch, idx);
            };

            size_t idx = 0;

            // HMD_q*
            {
                float qx = f.hmdPose.rotation[0];
                float qy = f.hmdPose.rotation[1];
                float qz = f.hmdPose.rotation[2];
                float qw = f.hmdPose.rotation[3];

                openxrToGaitAxesQuat(qx, qy, qz, qw);

                setChan(idx++, qx);
                setChan(idx++, qy);
                setChan(idx++, qz);
                setChan(idx++, qw);
            }

            // L_CTRL_q*
            if (f.leftCtrl.isActive && f.leftHandJointCount==0){
                float qx = f.leftCtrl.pose.rotation[0];
                float qy = f.leftCtrl.pose.rotation[1];
                float qz = f.leftCtrl.pose.rotation[2];
                float qw = f.leftCtrl.pose.rotation[3];

                openxrToGaitAxesQuat(qx, qy, qz, qw);

                setChan(idx++, qx);
                setChan(idx++, qy);
                setChan(idx++, qz);
                setChan(idx++, qw);
            }

            // R_CTRL_q*
            if (f.rightCtrl.isActive && f.rightHandJointCount==0){
                float qx = f.rightCtrl.pose.rotation[0];
                float qy = f.rightCtrl.pose.rotation[1];
                float qz = f.rightCtrl.pose.rotation[2];
                float qw = f.rightCtrl.pose.rotation[3];

                openxrToGaitAxesQuat(qx, qy, qz, qw);

                setChan(idx++, qx);
                setChan(idx++, qy);
                setChan(idx++, qz);
                setChan(idx++, qw);
            }

            // L_Joints
            for (int j = 0; j < f.leftHandJointCount && j < 26; ++j) {
                const auto& s = f.leftHandJoints[j];
                float qx = s.qx, qy = s.qy, qz = s.qz, qw = s.qw;
                openxrToGaitAxesQuat(qx, qy, qz, qw);
                setChan(idx++, qx);
                setChan(idx++, qy);
                setChan(idx++, qz);
                setChan(idx++, qw);
            }
            // R_Joints
            for (int j = 0; j < f.rightHandJointCount && j < 26; ++j) {
                const auto& s = f.rightHandJoints[j];
                float qx = s.qx, qy = s.qy, qz = s.qz, qw = s.qw;
                openxrToGaitAxesQuat(qx, qy, qz, qw);
                setChan(idx++, qx);
                setChan(idx++, qy);
                setChan(idx++, qz);
                setChan(idx++, qw);
            }

            while (idx + 1 < nbAnalogs) { // dejamos el ultimo para RealTime
                setChan(idx++, 0.0f);
            }

            // Canal extra: tiempo real del frame en segundos
            if (nbAnalogs > 0) {
                const size_t timeIdx = nbAnalogs - 1; // siempre el ultimo
                setChan(timeIdx, static_cast<float>(f.timestampSec));
            }

            Analogs analogs;
            analogs.subframe(subframe, 0);
            frame.analogs() = analogs;
        }

        // Anadimos el frame al c3d
        c3d.frame(frame);
    }

    // Por ultimo, escribimos el archivo en disco
    c3d.write(filePath_);
    LOGI("C3DRecorder: file written OK: '%s'", filePath_.c_str());
}
