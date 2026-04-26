#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <cstdint>
#include <ostream>
#include <ctime>
#include <algorithm>
#include <pthread.h>

// Priority-inheriting mutex — prevents priority inversion between RT and non-RT threads.
// Satisfies BasicLockable so std::lock_guard/std::unique_lock work normally.
class PIMutex {
    pthread_mutex_t m_;
public:
    PIMutex() {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        pthread_mutex_init(&m_, &attr);
        pthread_mutexattr_destroy(&attr);
    }
    ~PIMutex()  { pthread_mutex_destroy(&m_); }
    void lock()   { pthread_mutex_lock(&m_); }
    void unlock() { pthread_mutex_unlock(&m_); }
    PIMutex(const PIMutex&)            = delete;
    PIMutex& operator=(const PIMutex&) = delete;
};

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
    float   gain_reg_per_unit;  // analogue_gain register units per 1.0x gain (unused when log_gain=true)
    int     gain_reg_min;
    int     gain_reg_max;
    bool    log_gain;           // true = register is 0.1 dB steps (e.g. IMX296)
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
    PIMutex                     mtx;
    std::condition_variable_any cv;
    std::shared_ptr<Frame>      frame;
    uint64_t push_id = 0;
    uint64_t get_id  = 0;
public:
    void push(std::shared_ptr<Frame> f) {
        {
            std::lock_guard<PIMutex> lock(mtx);
            frame = std::move(f);
            push_id++;
        }
        cv.notify_one();
    }
    // Blocks until a new frame is available; returns false on 50 ms timeout (for shutdown polling).
    bool wait_and_get(std::shared_ptr<Frame>& out) {
        std::unique_lock<PIMutex> lock(mtx);
        cv.wait_for(lock, std::chrono::milliseconds(50),
                    [&]{ return get_id != push_id; });
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
extern PIMutex               log_mtx;
extern std::ostream*         log_out;

struct LogLine {
    std::lock_guard<PIMutex> guard;
    LogLine(const char* tag) : guard(log_mtx) { *log_out << '[' << tag << "] "; }
    ~LogLine() { *log_out << '\n'; }
    template<class T> LogLine& operator<<(const T& v) { *log_out << v; return *this; }
};

inline double mono_now() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

inline void prefault_stack() {
    const size_t SZ = 256 * 1024;
    volatile char buf[SZ];
    for (size_t i = 0; i < SZ; i += 4096) buf[i] = 0;
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

#ifdef __ARM_NEON
#include <arm_neon.h>

// NEON path: processes 16 pixels (4 Y10P groups = 20 bytes) per iteration.
//
// The low 2 bits of each 10-bit sample are ignored. Their contribution to the
// 8-bit output is at most 3 * 255/(1023-black_level) < 1 count — below 1 LSB.
//
// Caller must ensure src buffer has ≥32 bytes of readable memory past the last
// valid row byte (a small pad on raw_staging is sufficient).
inline void unpack_to_u8(uint8_t* dst, const uint8_t* src,
                         int width, int height, unsigned stride,
                         uint16_t black_level) {
    const int     range     = 1023 - black_level;
    // Q15 fixed-point scale: vqrdmulhq_s16 computes round(2*a*b / 2^16).
    // scale_q15 = round(255 / range * 32768), capped at INT16_MAX.
    const int16_t scale_q15 = (int16_t)std::min((int)(255.0 * 32768.0 / range + 0.5), 32767);
    const int16_t bl        = (int16_t)black_level;

    // vqtbl2q_u8 gathers bytes from a 32-byte source by index.
    // These indices select the 16 high bytes from a 20-byte window of 4 Y10P groups,
    // skipping the low-bit bytes at positions 4, 9, 14, 19.
    alignas(16) static const uint8_t idx_arr[16] = {
        0, 1, 2, 3,  5, 6, 7, 8,  10, 11, 12, 13,  15, 16, 17, 18
    };
    const uint8x16_t idx    = vld1q_u8(idx_arr);
    const int16x8_t  bl_vec = vdupq_n_s16(bl);
    const int16x8_t  sc_vec = vdupq_n_s16(scale_q15);
    const int16x8_t  zero   = vdupq_n_s16(0);

    for (int row = 0; row < height; row++) {
        const uint8_t* s = src + (size_t)row * stride;
        uint8_t*       d = dst + (size_t)row * width;
        int x = 0;

        for (; x + 16 <= width; x += 16, s += 20, d += 16) {
            uint8x16x2_t raw2;
            raw2.val[0] = vld1q_u8(s);
            raw2.val[1] = vld1q_u8(s + 16);

            // Gather the 16 high bytes; raw10 ≈ high << 2
            uint8x16_t highs  = vqtbl2q_u8(raw2, idx);
            uint16x8_t raw_lo = vshll_n_u8(vget_low_u8(highs),  2);
            uint16x8_t raw_hi = vshll_n_u8(vget_high_u8(highs), 2);

            // Subtract black level, clamp ≥0, scale to 8-bit via Q15 multiply
            int16x8_t c_lo = vmaxq_s16(vsubq_s16(vreinterpretq_s16_u16(raw_lo), bl_vec), zero);
            int16x8_t c_hi = vmaxq_s16(vsubq_s16(vreinterpretq_s16_u16(raw_hi), bl_vec), zero);
            vst1q_u8(d, vcombine_u8(vqmovun_s16(vqrdmulhq_s16(c_lo, sc_vec)),
                                     vqmovun_s16(vqrdmulhq_s16(c_hi, sc_vec))));
        }

        // Scalar tail for widths not divisible by 16
        for (; x < width; x += 4, s += 5, d += 4) {
            uint8_t low = s[4];
            d[0] = correct_pixel((uint16_t)(s[0] << 2) | ((low     ) & 0x3), black_level);
            d[1] = correct_pixel((uint16_t)(s[1] << 2) | ((low >> 2) & 0x3), black_level);
            d[2] = correct_pixel((uint16_t)(s[2] << 2) | ((low >> 4) & 0x3), black_level);
            d[3] = correct_pixel((uint16_t)(s[3] << 2) | ((low >> 6) & 0x3), black_level);
        }
    }
}

#else

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

#endif

void capture_thread();
