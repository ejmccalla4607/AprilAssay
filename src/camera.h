#pragma once

#include <vector>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <ostream>
#include <ctime>
#include <algorithm>

// ==========================
// Sensor Specifications
// ==========================
struct SensorSpec {
    const char* name;
    int         width;
    int         height;
    uint16_t    black_level;    // 10-bit black level
    // Intrinsics — calibrate per lens
    double fx, fy, cx, cy;
    // Distortion: k1, k2, p1, p2, k3
    double dist_k1, dist_k2, dist_p1, dist_p2, dist_k3;
    // V4L2 backend (RPi4) — verify with v4l2-ctl --list-ctrls on target hardware
    int     vblank_min;         // minimum vertical blanking lines
    int     hblank_min;         // minimum horizontal blanking pixels
    int64_t pixel_clock_hz;     // sensor pixel clock
    float   gain_reg_per_unit;  // analogue_gain register units per 1.0x gain
    int     gain_reg_min;
    int     gain_reg_max;
};

extern const SensorSpec SENSORS[2];
extern const SensorSpec* g_sensor;
extern int               g_exposure_us;
extern float             g_gain;
extern int               g_target_fps;

// ==========================
// Frame + Latest Frame Store
// ==========================
struct Frame {
    std::vector<uint8_t> gray;
    int    width;
    int    height;
    double ts_camera;
    double ts_captured;
    double ts_dequeued;
};

class LatestFrame {
    std::mutex mtx;
    Frame    frame;
    uint64_t push_id = 0;
    uint64_t get_id  = 0;
public:
    void push(Frame&& f) {
        std::lock_guard<std::mutex> lock(mtx);
        frame = std::move(f);
        push_id++;
    }
    bool get(Frame& out) {
        std::lock_guard<std::mutex> lock(mtx);
        if (get_id == push_id) return false;
        out    = frame;
        get_id = push_id;
        return true;
    }
};

// ==========================
// Shared runtime state
// ==========================
extern LatestFrame           latest_frame;
extern std::atomic<bool>     running;
extern std::atomic<uint64_t> camera_frame_count;
extern std::mutex            log_mtx;
extern std::ostream*         log_out;

inline double mono_now() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// ==========================
// Pixel unpacking: Y10P → uint8
// 4 pixels per 5 bytes (CSI-2 packed 10-bit).
// Bytes 0-3: upper 8 bits of each sample.
// Byte 4:    low 2 bits of all four: [p3[1:0] p2[1:0] p1[1:0] p0[1:0]].
// ==========================
inline uint8_t correct_pixel(uint16_t raw10, uint16_t black_level) {
    int v = (int)raw10 - black_level;
    if (v < 0) v = 0;
    return (uint8_t)(v * 255 / (1023 - black_level));
}

inline void unpack_to_u8(uint8_t* dst, const uint8_t* src,
                         int width, int height, unsigned stride,
                         uint16_t black_level) {
    for (int row = 0; row < height; row++) {
        const uint8_t* s = src + row * stride;
        uint8_t*       d = dst + row * width;
        for (int out = 0; out < width; out += 4, s += 5) {
            uint8_t low = s[4];
            d[out]   = correct_pixel((uint16_t)(s[0] << 2) | ((low     ) & 0x3), black_level);
            d[out+1] = correct_pixel((uint16_t)(s[1] << 2) | ((low >> 2) & 0x3), black_level);
            d[out+2] = correct_pixel((uint16_t)(s[2] << 2) | ((low >> 4) & 0x3), black_level);
            d[out+3] = correct_pixel((uint16_t)(s[3] << 2) | ((low >> 6) & 0x3), black_level);
        }
    }
}

void capture_thread();
