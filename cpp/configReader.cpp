#include "configReader.h"
#include <android/log.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <cerrno>
#include <cstring>

#define LOG_TAG "telemetria"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

// Default config in case JSON file is missing or they are not defined
static constexpr const char* kDefaultEndpoint = "http://localhost:1414";
static constexpr const char* kDefaultApiKey = "non_existant";
static constexpr int kDefaultFramesPerFile = 200;
static constexpr int kDefaultFrameRate = 60;

// The package name is obtained from /proc/self/cmdline. On Android, the process name
// is usually the same as the app package name.
static bool getPackageNameFromProc(std::string& outPkg) {
    std::ifstream f("/proc/self/cmdline", std::ios::in | std::ios::binary);
    if (!f) return false;
    std::string buf;
    // First token up to '\0'
    std::getline(f, buf, '\0');
    if (buf.empty()) return false;

    // Remove optional suffix after ':', e.g. "com.app:unity"
    size_t colon = buf.find(':');
    if (colon != std::string::npos) buf.resize(colon);

    outPkg = buf;
    return !outPkg.empty();
}

// Helpers for tiny JSON parsing. We do not use a full JSON library
// because initialConfig.json has a simple, predictable structure so we keep it simple (and smaller future aar).
namespace {
    // Looks for: "key" : "value" (with optional spaces) and extracts value.
    static bool extractJsonString(const std::string& text, const std::string& key, std::string& out) {
        const std::string q = "\"" + key + "\"";
        size_t p = text.find(q);
        if (p == std::string::npos) return false;

        p = text.find(':', p + q.size());
        if (p == std::string::npos) return false;

        // Skip colon and whitespace
        while (p < text.size() && (text[p] == ':' || text[p] == ' ' || text[p] == '\t' || text[p] == '\r' || text[p] == '\n')) ++p;
        if (p >= text.size() || text[p] != '"') return false;
        ++p; // now inside the string
        std::string val;
        while (p < text.size()) {
            char c = text[p++];
            if (c == '\\') {  // Minimal escape handling: copy next char as is
                if (p < text.size()) {
                    char n = text[p++];
                    val.push_back(n);
                }
            } else if (c == '"') {
                break;
            } else {
                val.push_back(c);
            }
        }
        out = val;
        return !out.empty();
    }

    // Looks for: "key" : <integer> and stores it in out.
    static bool extractJsonInt(const std::string& text, const std::string& key, int& out) {
        const std::string kq = "\"" + key + "\"";
        size_t p = text.find(kq);
        if (p == std::string::npos) return false;
        p = text.find(':', p + kq.size());
        if (p == std::string::npos) return false;

        // saltar espacios
        while (p < text.size() && (text[p] == ':' || std::isspace(static_cast<unsigned char>(text[p])))) ++p;
        if (p >= text.size()) return false;

        // leer numero
        bool neg = false;
        if (text[p] == '-') { neg = true; ++p; }
        if (p >= text.size() || !std::isdigit(static_cast<unsigned char>(text[p]))) return false;

        long val = 0;
        while (p < text.size() && std::isdigit(static_cast<unsigned char>(text[p]))) {
            val = val * 10 + (text[p] - '0');
            ++p;
        }
        if (neg) val = -val;
        out = static_cast<int>(val);
        return true;
    }

    // Looks for: "key" : true/false (lowercase)
    static bool extractJsonBool(const std::string& text, const std::string& key, bool& out) {
        const std::string kq = "\"" + key + "\"";
        size_t p = text.find(kq);
        if (p == std::string::npos) return false;
        p = text.find(':', p + kq.size());
        if (p == std::string::npos) return false;
        while (p < text.size() && (text[p] == ':' || std::isspace((unsigned char)text[p]))) ++p;
        if (p >= text.size()) return false;

        // aceptamos true/false (lowercase)
        if (text.compare(p, 4, "true") == 0)  { out = true;  return true; }
        if (text.compare(p, 5, "false") == 0) { out = false; return true; }
        return false;
    }
}


namespace configReader {

    // Build expected path: /sdcard/Android/data/<package>/files/initialConfig.json
    bool getExpectedConfigPath(std::string& outPath) {
        std::string pkg;
        if (!getPackageNameFromProc(pkg)) {
            LOGE("configReader: could not get package from /proc/self/cmdline");
            return false;
        }
        outPath = std::string("/sdcard/Android/data/") + pkg + "/files/initialConfig.json";
        return true;
    }

    // Single attempt to read the file fully into a string.
    static bool readFileToStringOnce(const std::string& path, std::string& outText) {
        std::ifstream in(path, std::ios::in | std::ios::binary);
        if (!in) {
            LOGE("configReader: open failed for %s (errno=%d: %s)",
                 path.c_str(), errno, strerror(errno));
            return false;
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        outText = ss.str();
        return true;
    }

    // Read a file to string with /sdcard → /storage/emulated/0 fallback.
    bool readFileToString(const std::string& path, std::string& outText) {
        // 1) Main attempt with the given path
        if (readFileToStringOnce(path, outText)) return true;
        // 2) Fallback: if path starts with /sdcard/, try /storage/emulated/0/
        const std::string sdcard = "/sdcard/";
        if (path.rfind(sdcard, 0) == 0) {
            std::string alt = std::string("/storage/emulated/0/") + path.substr(sdcard.size());
            LOGI("configReader: retrying with alternate path: %s", alt.c_str());
            if (readFileToStringOnce(alt, outText)) return true;
        }
        return false;
    }

    // Read initialConfig.json and fill UploaderConfig with endpoint/apiKey/framesPerFile/flags.
    bool setConfig(UploaderConfig& outCfg) {
        outCfg.endpointUrl   = kDefaultEndpoint;
        outCfg.apiKey        = kDefaultApiKey;
        outCfg.framesPerFile = kDefaultFramesPerFile;
        // Defaults de flags a false
        outCfg.handTracking  = false;
        outCfg.primaryButton = false;
        outCfg.secondaryButton = false;
        outCfg.grip          = false;
        outCfg.trigger       = false;
        outCfg.joystick      = false;

        std::string path, text;
        if (!getExpectedConfigPath(path)) {
            LOGE("configReader: failed to build expected config path; leaving defaults.");
            return false;
        }
        if (!readFileToString(path, text)) {
            LOGI("configReader: %s not found or unreadable. Using defaults.", path.c_str());
            return false;
        }

        // Log file content (truncated if too large bc of wrong file). For debugging.
        const size_t kMaxLog = 4096;
        if (text.size() <= kMaxLog) {
            LOGI("configReader: read %zu bytes from %s\n---BEGIN initialConfig.json---\n%.*s\n---END initialConfig.json---",
                 text.size(), path.c_str(), (int)text.size(), text.c_str());
        } else {
            LOGI("configReader: read %zu bytes from %s (truncated to %zu)\n---BEGIN initialConfig.json (TRUNCATED)---\n%.*s\n---END initialConfig.json---",
                 text.size(), path.c_str(), kMaxLog, (int)kMaxLog, text.c_str());
        }

        // Start from current values (defaults) and override if keys exist.
        std::string endpoint = outCfg.endpointUrl;
        std::string key = outCfg.apiKey;
        int framesPerFile = outCfg.framesPerFile;

        std::string tmp;
        if (extractJsonString(text, "endpoint", tmp) && !tmp.empty()) endpoint = tmp;
        if (extractJsonString(text, "apiKey", tmp) && !tmp.empty()) key = tmp;
        int vi;
        if (extractJsonInt(text, "framesPerFile", vi) && vi > 0) framesPerFile = vi;

        outCfg.endpointUrl   = endpoint;
        outCfg.apiKey        = key;
        outCfg.framesPerFile = framesPerFile;

        // Feature flags (if any are missing, they remain at default=false).
        bool vb;
        if (extractJsonBool(text, "handTracking", vb))  outCfg.handTracking  = vb;
        if (extractJsonBool(text, "primaryButton", vb)) outCfg.primaryButton = vb;
        if (extractJsonBool(text, "secondaryButton", vb)) outCfg.secondaryButton = vb;
        if (extractJsonBool(text, "grip", vb))          outCfg.grip          = vb;
        if (extractJsonBool(text, "trigger", vb))       outCfg.trigger       = vb;
        if (extractJsonBool(text, "joystick", vb))      outCfg.joystick      = vb;

        // Reading was done (even if some keys were missing).
        return true;
    }

    // Separated read of FramerRate because it will be returned to Unity/UnrealEngine
    bool getFrameRate(int& outFrameRate) {
        int result = kDefaultFrameRate;
        std::string path, text;
        if (getExpectedConfigPath(path) && readFileToString(path, text)) {
            int vi;
            if (extractJsonInt(text, "frameRate", vi)) {
                if (vi >= 1 && vi <= 240) {
                    result = vi;
                    LOGI("configReader: frameRate=%d", vi);
                } else {
                    LOGI("configReader: frameRate=%d fuera de rango [1,240]; usando default %d", vi, kDefaultFrameRate);
                }
            } else {
                LOGI("configReader: clave 'frameRate' no encontrada; usando default %d", kDefaultFrameRate);
            }
        } else {
            LOGI("configReader: initialConfig.json no encontrado; usando default %d", kDefaultFrameRate);
        }
        outFrameRate = result;
        return true;
    }
} // namespace configReader

extern "C" bool configReader_setConfig(UploaderConfig& outCfg) {
    return configReader::setConfig(outCfg);
}

extern "C" int configReader_getFrameRate() {
    int fr = 0;
    configReader::getFrameRate(fr);
    return fr;
}
