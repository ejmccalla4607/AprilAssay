#include "camera.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <ctime>
#include <cmath>
#include <csignal>

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/image_u8.h"
}

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

static const double TAG_SIZE_M     = 0.165;
static const int    LOG_INTERVAL_MS = 2000;

// ==========================
// Sensor table
// ==========================
// Timing values derived from Linux kernel drivers; verify with
// v4l2-ctl --list-ctrls on target hardware before tuning.
const SensorSpec SENSORS[2] = {
    {
        // ov9281 driver: hts=1704, pixel_rate=160 MHz
        // fps_max = 160e6 / (1704 * (800 + vblank_min)) ≈ 116 fps at vblank_min=11
        "OV9281",
        1280, 800,
        64,     // 10-bit black level (4096 >> 6)
        600.0, 600.0, 640.0, 400.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        11,              // vblank_min
        424,             // hblank_min  (hts = 1704)
        160'000'000LL,   // pixel_clock_hz
        16.0f,           // gain_reg_per_unit: reg=16 → 1.0x
        16,              // gain_reg_min (1.0x)
        248,             // gain_reg_max (15.5x)
        false,           // linear gain
    },
    {
        // imx296: hblank=304 (read-only), pixel_rate=118.8 MHz (from v4l2-ctl)
        // fps_max = 118.8e6 / (1760 * (1088 + vblank_min)) ≈ 60 fps at vblank_min=30
        // Gain: 0.1 dB steps (reg = round(200 * log10(gain))); max reg=480 per driver.
        "IMX296",
        1456, 1088,
        60,     // 10-bit black level (3840 >> 6)
        900.0, 900.0, 728.0, 544.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        30,              // vblank_min (from v4l2-ctl min=30)
        304,             // hblank_min (from v4l2-ctl, read-only)
        118'800'000LL,   // pixel_clock_hz (from v4l2-ctl value=118800000)
        0.0f,            // gain_reg_per_unit (unused — log_gain=true)
        0,               // gain_reg_min (0 dB = 1.0x)
        480,             // gain_reg_max (from v4l2-ctl max=480)
        true,            // logarithmic: reg = round(200 * log10(gain))
    },
};

// ==========================
// Globals (declared extern in camera.h)
// ==========================
const SensorSpec* g_sensor      = &SENSORS[0];
int               g_exposure_us  = 333;
float             g_gain         = 1.0f;
int               g_target_fps   = 60;
const char*       g_snapshot_out   = nullptr;
int               g_nthreads       = 3;
float             g_quad_decimate  = 1.0f;

LatestFrame           latest_frame;
std::atomic<bool>     running{true};
std::atomic<uint64_t> camera_frame_count{0};
std::mutex            log_mtx;
std::ostream*         log_out = nullptr;

static void handle_signal(int) { running = false; }

// ==========================
// Logging setup
// ==========================
static void init_logging() {
#ifdef DEBUG_LOGGING
    log_out = &std::cout;
#else
    std::time_t t  = std::time(nullptr);
    std::tm*    tm = std::localtime(&t);
    char buf[64];
    std::strftime(buf, sizeof(buf), "vision_%Y%m%d_%H%M%S.log", tm);
    static std::ofstream file(buf, std::ios::out | std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open log file: " << buf << ", falling back to stdout\n";
        log_out = &std::cout;
    } else {
        std::cout << "Logging to: " << buf << "\n";
        log_out = &file;
    }
#endif
}

// ==========================
// Ring buffer for timing samples
// ==========================
struct LogSample {
    double capture_ms;
    double queue_ms;
    double detect_ms;
    double pose_ms;
    double total_ms;
    int detections_count;
    int frames_with_detections;
};

constexpr int LOG_SIZE = 1024;

class RingBuffer {
    LogSample        buf[LOG_SIZE];
    std::atomic<int> w{0}, r{0};
public:
    void push(const LogSample& s) {
        int idx = w.load(std::memory_order_relaxed);
        buf[idx % LOG_SIZE] = s;
        w.store(idx + 1, std::memory_order_release);
    }
    bool pop(LogSample& s) {
        int rr = r.load(std::memory_order_relaxed);
        int ww = w.load(std::memory_order_acquire);
        if (rr == ww) return false;
        s = buf[rr % LOG_SIZE];
        r.store(rr + 1, std::memory_order_release);
        return true;
    }
};

static RingBuffer log_buffer;

// ==========================
// Detection thread
// ==========================
void detection_thread() {
    apriltag_family_t*   tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();

    apriltag_detector_add_family(td, tf);
    td->quad_decimate = g_quad_decimate;
    td->nthreads      = g_nthreads;

    const SensorSpec& spec = *g_sensor;

    const cv::Mat cam_mat = (cv::Mat_<double>(3, 3)
        << spec.fx, 0,       spec.cx,
           0,       spec.fy, spec.cy,
           0,       0,       1.0);
    const cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1)
        << spec.dist_k1, spec.dist_k2,
           spec.dist_p1, spec.dist_p2,
           spec.dist_k3);

    static const std::vector<cv::Point3f> tag_obj_pts = {
        {-(float)TAG_SIZE_M / 2,  (float)TAG_SIZE_M / 2, 0.f},
        { (float)TAG_SIZE_M / 2,  (float)TAG_SIZE_M / 2, 0.f},
        { (float)TAG_SIZE_M / 2, -(float)TAG_SIZE_M / 2, 0.f},
        {-(float)TAG_SIZE_M / 2, -(float)TAG_SIZE_M / 2, 0.f},
    };

    Frame f;
    std::vector<cv::Point2f> img_pts(4);

    while (running) {
        if (!latest_frame.get(f)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        f.ts_dequeued = mono_now();

        image_u8_t img{
            .width  = f.width,
            .height = f.height,
            .stride = f.width,
            .buf    = f.gray.data()
        };

        zarray_t* detections   = apriltag_detector_detect(td, &img);
        double    ts_detect_done = mono_now();
        int       num          = zarray_size(detections);

        for (int i = 0; i < num; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            img_pts[0] = {(float)det->p[0][0], (float)det->p[0][1]};
            img_pts[1] = {(float)det->p[1][0], (float)det->p[1][1]};
            img_pts[2] = {(float)det->p[2][0], (float)det->p[2][1]};
            img_pts[3] = {(float)det->p[3][0], (float)det->p[3][1]};

            cv::Mat rvec, tvec;
            cv::solvePnP(tag_obj_pts, img_pts, cam_mat, dist_coeffs, rvec, tvec);

            cv::Mat R_mat;
            cv::Rodrigues(rvec, R_mat);
            double x     = tvec.at<double>(0);
            double y     = tvec.at<double>(2);
            double theta = std::atan2(R_mat.at<double>(0, 2), -R_mat.at<double>(2, 2));

            // TODO: publish pose over ethernet
            {
                std::lock_guard<std::mutex> lock(log_mtx);
                *log_out << "[DETECT]"
                         << " id="    << det->id
                         << " x="     << x
                         << " y="     << y
                         << " theta=" << theta
                         << " margin=" << det->decision_margin << "\n";
            }
        }

        double ts_pose_done = mono_now();
        apriltag_detections_destroy(detections);

        log_buffer.push({
            (f.ts_captured    - f.ts_camera)    * 1000.0,
            (f.ts_dequeued    - f.ts_captured)  * 1000.0,
            (ts_detect_done   - f.ts_dequeued)  * 1000.0,
            (ts_pose_done     - ts_detect_done) * 1000.0,
            (ts_pose_done     - f.ts_camera)    * 1000.0,
            num,
            num > 0 ? 1 : 0
        });
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

// ==========================
// Logging thread
// ==========================
void logging_thread() {
    LogSample s;
    double   capture_sum = 0, queue_sum = 0, detect_sum = 0,
             pose_sum    = 0, total_sum  = 0;
    int      detections_sum = 0;
    int      frames_with_detections_sum = 0;
    int      processed_frames_sum = 0;
    uint64_t last_camera_count = 0;

    const double interval_s = LOG_INTERVAL_MS / 1000.0;

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(LOG_INTERVAL_MS));

        while (log_buffer.pop(s)) {
            capture_sum += s.capture_ms;
            queue_sum   += s.queue_ms;
            detect_sum  += s.detect_ms;
            pose_sum    += s.pose_ms;
            total_sum   += s.total_ms;
            detections_sum += s.detections_count;
            frames_with_detections_sum += s.frames_with_detections;
            ++processed_frames_sum;
        }

        uint64_t current_camera_count = camera_frame_count.load(std::memory_order_relaxed);
        uint64_t cam_frames           = current_camera_count - last_camera_count;
        last_camera_count             = current_camera_count;

        double cam_fps    = cam_frames   / interval_s;
        double det_fps    = processed_frames_sum / interval_s;
        double detection_rate = processed_frames_sum > 0 ? (double)frames_with_detections_sum / processed_frames_sum : 0.0;

        {
            std::lock_guard<std::mutex> lock(log_mtx);
            *log_out << "Cam FPS: " << cam_fps << " | Det FPS: " << det_fps << " | Det Rate: " << detection_rate;
            if (detections_sum > 0) {
                double n = detections_sum;
                *log_out << " | Capture: "   << capture_sum / n << " ms"
                         << " | Queue: "     << queue_sum   / n << " ms"
                         << " | Detect: "    << detect_sum  / n << " ms"
                         << " | Pose: "      << pose_sum    / n << " ms"
                         << " | Total E2E: " << total_sum   / n << " ms";
            }
            *log_out << "\n";
            log_out->flush();
        }

        capture_sum = queue_sum = detect_sum = pose_sum = total_sum = 0;
        detections_sum = 0;
        frames_with_detections_sum = 0;
        processed_frames_sum = 0;
    }
}

// ==========================
// Main
// ==========================
static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [--camera ov9281|imx296] [--exposure <µs>] [--gain <x>] [--fps <n>]"
                                      " [--nthreads <n>] [--quad-decimate <f>] [--snapshot <file>]\n"
              << "  --camera        Sensor model                  (default: ov9281)\n"
              << "  --exposure      Exposure time in microseconds (default: 333)\n"
              << "  --gain          Analogue gain multiplier      (default: 1.0)\n"
              << "  --fps           Target frame rate             (default: 60)\n"
              << "  --nthreads      AprilTag detector threads     (default: 2)\n"
              << "  --quad-decimate AprilTag quad decimation      (default: 1.0)\n"
              << "  --snapshot      Capture one frame, save as PNG, and exit\n";
}

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--camera" && i + 1 < argc) {
            std::string model = argv[++i];
            if      (model == "ov9281") g_sensor = &SENSORS[0];
            else if (model == "imx296") g_sensor = &SENSORS[1];
            else {
                std::cerr << "Error: unknown camera model '" << model << "'\n";
                print_usage(argv[0]);
                return 1;
            }
        } else if (arg == "--exposure" && i + 1 < argc) {
            g_exposure_us = std::stoi(argv[++i]);
            if (g_exposure_us < 1) {
                std::cerr << "Error: --exposure must be >= 1 µs\n";
                return 1;
            }
        } else if (arg == "--gain" && i + 1 < argc) {
            g_gain = std::stof(argv[++i]);
            if (g_gain < 1.0f) {
                std::cerr << "Error: --gain must be >= 1.0\n";
                return 1;
            }
        } else if (arg == "--fps" && i + 1 < argc) {
            g_target_fps = std::stoi(argv[++i]);
            if (g_target_fps < 1) {
                std::cerr << "Error: --fps must be >= 1\n";
                return 1;
            }
        } else if (arg == "--nthreads" && i + 1 < argc) {
            g_nthreads = std::stoi(argv[++i]);
            if (g_nthreads < 1) {
                std::cerr << "Error: --nthreads must be >= 1\n";
                return 1;
            }
        } else if (arg == "--quad-decimate" && i + 1 < argc) {
            g_quad_decimate = std::stof(argv[++i]);
            if (g_quad_decimate < 1.0f) {
                std::cerr << "Error: --quad-decimate must be >= 1.0\n";
                return 1;
            }
        } else if (arg == "--snapshot" && i + 1 < argc) {
            g_snapshot_out = argv[++i];
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    init_logging();
    *log_out << "Camera: "         << g_sensor->name
             << " | Exposure: "   << g_exposure_us << " µs"
             << " | Gain: "       << g_gain << "x"
             << " | Target FPS: " << g_target_fps
             << " | nthreads: "   << g_nthreads
             << " | quad_decimate: " << g_quad_decimate << "\n";

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    std::thread t1(capture_thread);

    if (g_snapshot_out) {
        // Skip the first few frames — early deliveries can be black/garbage.
        Frame f;
        for (int skip = 0; skip < 5; skip++) {
            while (running && !latest_frame.get(f))
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (!f.gray.empty()) {
            cv::Mat img(f.height, f.width, CV_8UC1, f.gray.data());
            cv::imwrite(g_snapshot_out, img);
            *log_out << "Snapshot saved: " << g_snapshot_out << "\n";
        }
        running = false;
        t1.join();
        return 0;
    }

    std::thread t2(detection_thread);
    std::thread t3(logging_thread);

    while (running) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
