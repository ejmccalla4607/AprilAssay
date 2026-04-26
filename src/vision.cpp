#include "camera.h"

#include <pthread.h>
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
#include <filesystem>
#include <csignal>
#include <sstream>
#include <sys/mman.h>

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/image_u8.h"
}

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iomanip>

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
const char*       g_log_path       = nullptr;
int               g_nthreads       = 3;
float             g_quad_decimate  = 1.0f;

static int   g_condition_id    = 0;
static int   g_gantry_x        = 0;
static int   g_gantry_y        = 0;
static int   g_gantry_z        = 0;
static float g_lux_tag         = 0.0f;
static float g_supply_ma       = 0.0f;
static float g_shadow_coverage = 0.0f;
static float g_shadow_depth    = 0.0f;

LatestFrame           latest_frame;
std::atomic<bool>     running{true};
std::atomic<uint64_t> camera_frame_count{0};
PIMutex               log_mtx;
std::ostream*         log_out = nullptr;
static std::ostream*  log_tel = nullptr;

static void handle_signal(int) { running = false; }

// ==========================
// Logging setup
// ==========================
static std::ofstream debug_file;
static std::ofstream tel_file;

static const char* CSV_HEADER =
    "timestamp,frame_number,sensor,exposure_us,gain_db,gain_linear,"
    "condition_id,gantry_x_level,gantry_y_level,gantry_z_level,"
    "lux_tag,light_supply_current_ma,shadow_coverage_pct,shadow_depth_ratio,"
    "detected,tag_id,decision_margin,pixel_count_across_tag,"
    "white_mean_dn,white_median_dn,white_std_dn,white_min_dn,white_max_dn,"
    "black_mean_dn,black_median_dn,black_std_dn,black_min_dn,black_max_dn,"
    "contrast_ratio,saturated,glare_region_dn,"
    "pose_x_m,pose_y_m,pose_theta_rad,pose_error_mm\n";

static void init_logging() {
    if (!g_log_path) {
        log_out = &std::cout;
        log_tel = nullptr;
        return;
    }
    std::string base(g_log_path);
    auto parent = std::filesystem::path(base).parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent);

    debug_file.open(base + "_debug.log", std::ios::out | std::ios::app);
    if (!debug_file.is_open()) {
        std::cerr << "Failed to open debug log: " << base << "_debug.log, falling back to stdout\n";
        log_out = &std::cout;
    } else {
        log_out = &debug_file;
    }

    std::string tel_path = base + "_telemetry.csv";
    bool is_new = !std::filesystem::exists(tel_path) ||
                   std::filesystem::file_size(tel_path) == 0;
    tel_file.open(tel_path, std::ios::out | std::ios::app);
    if (!tel_file.is_open()) {
        std::cerr << "Failed to open telemetry log: " << tel_path << "\n";
        log_tel = nullptr;
    } else {
        log_tel = &tel_file;
        if (is_new) *log_tel << CSV_HEADER;
    }
}

// ==========================
// Ring buffers for async logging
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

struct TelemetryRecord {
    // Session inputs
    double   timestamp;
    uint64_t frame_number;
    int      exposure_us;
    float    gain_db, gain_linear;
    int      condition_id;
    int      gantry_x, gantry_y, gantry_z;
    float    lux_tag, supply_ma;
    float    shadow_coverage_pct, shadow_depth_ratio;
    // Per-frame outputs
    bool  detected;
    int   tag_id;
    float decision_margin;
    int   pixel_count_across_tag;
    float white_mean, white_median, white_std, white_min, white_max;
    float black_mean, black_median, black_std, black_min, black_max;
    float contrast_ratio;
    bool  saturated;
    float glare_region_dn;
    float pose_x_m, pose_y_m, pose_theta_rad, pose_error_mm;
};

template<typename T, int N>
class RingBuffer {
    T                buf[N];
    std::atomic<int> w{0}, r{0};
public:
    void push(const T& s) {
        int idx = w.load(std::memory_order_relaxed);
        buf[idx % N] = s;
        w.store(idx + 1, std::memory_order_release);
    }
    bool pop(T& s) {
        int rr = r.load(std::memory_order_relaxed);
        int ww = w.load(std::memory_order_acquire);
        if (rr == ww) return false;
        s = buf[rr % N];
        r.store(rr + 1, std::memory_order_release);
        return true;
    }
};

static RingBuffer<LogSample,       1024> log_buffer;
static RingBuffer<TelemetryRecord,  256> telemetry_buffer;

// ==========================
// Patch DN statistics helper
// ==========================
struct PatchStats { float mean, median, std_dev, min_val, max_val; };

// Computes pixel statistics for the annular region between two polygons scaled
// from the tag corner polygon. outer_scale > inner_scale defines a ring;
// e.g. white patch: outer=1.15/inner=1.00; black patch: outer=1.00/inner=0.85.
static PatchStats compute_patch_stats(
    const uint8_t* gray, int img_w, int img_h,
    const double corners[4][2],
    float outer_scale, float inner_scale)
{
    float cx = 0, cy = 0;
    for (int i = 0; i < 4; i++) { cx += (float)corners[i][0]; cy += (float)corners[i][1]; }
    cx /= 4; cy /= 4;

    // Scale corner polygon and track bounding box of outer ring
    std::vector<cv::Point> outer_pts(4), inner_pts(4);
    int x0 = img_w, y0 = img_h, x1 = 0, y1 = 0;
    for (int i = 0; i < 4; i++) {
        int ox = (int)std::round(cx + outer_scale * ((float)corners[i][0] - cx));
        int oy = (int)std::round(cy + outer_scale * ((float)corners[i][1] - cy));
        outer_pts[i] = {ox, oy};
        x0 = std::min(x0, ox); y0 = std::min(y0, oy);
        x1 = std::max(x1, ox); y1 = std::max(y1, oy);
        inner_pts[i] = {
            (int)std::round(cx + inner_scale * ((float)corners[i][0] - cx)),
            (int)std::round(cy + inner_scale * ((float)corners[i][1] - cy))
        };
    }
    x0 = std::max(0, x0); y0 = std::max(0, y0);
    x1 = std::min(img_w - 1, x1); y1 = std::min(img_h - 1, y1);
    int bw = x1 - x0 + 1, bh = y1 - y0 + 1;
    if (bw <= 0 || bh <= 0) return {0, 0, 0, 0, 0};

    // Offset polygon points to bounding-box origin for small mask allocation
    cv::Point offset(x0, y0);
    for (auto& pt : outer_pts) pt -= offset;
    for (auto& pt : inner_pts) pt -= offset;

    cv::Mat outer_mask(bh, bw, CV_8UC1, cv::Scalar(0));
    cv::Mat inner_mask(bh, bw, CV_8UC1, cv::Scalar(0));
    cv::fillConvexPoly(outer_mask, outer_pts, cv::Scalar(255));
    cv::fillConvexPoly(inner_mask, inner_pts, cv::Scalar(255));

    std::vector<float> px;
    px.reserve(512);
    for (int row = 0; row < bh; row++) {
        const uint8_t* om = outer_mask.ptr<uint8_t>(row);
        const uint8_t* im = inner_mask.ptr<uint8_t>(row);
        const uint8_t* gp = gray + (row + y0) * img_w + x0;
        for (int col = 0; col < bw; col++) {
            if (om[col] && !im[col])
                px.push_back((float)gp[col]);
        }
    }
    if (px.empty()) return {0, 0, 0, 0, 0};

    double sum = 0;
    float mn = 255, mx = 0;
    for (float v : px) { sum += v; mn = std::min(mn, v); mx = std::max(mx, v); }
    float mean = (float)(sum / px.size());

    double var = 0;
    for (float v : px) { double d = v - mean; var += d * d; }
    float std_dev = (float)std::sqrt(var / px.size());

    std::vector<float> s = px;
    std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
    float median = s[s.size() / 2];

    return {mean, median, std_dev, mn, mx};
}

// ==========================
// Detection thread
// ==========================
void detection_thread() {
    prefault_stack();
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

    std::shared_ptr<Frame> fp;
    std::vector<cv::Point2f> img_pts(4);
    cv::Mat rvec, tvec;

    while (running) {
        if (!latest_frame.wait_and_get(fp)) continue;

        fp->ts_dequeued = mono_now();
        Frame& f = *fp;

        image_u8_t img{
            .width  = f.width,
            .height = f.height,
            .stride = f.width,
            .buf    = f.gray.data()
        };

        zarray_t* detections     = apriltag_detector_detect(td, &img);
        double    ts_detect_done = mono_now();
        int       num            = zarray_size(detections);

        // Find the highest-margin detection (primary tag for telemetry)
        apriltag_detection_t* best = nullptr;
        for (int i = 0; i < num; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            if (!best || det->decision_margin > best->decision_margin)
                best = det;
        }

        // Pose estimation for all detections
        double pose_x = 0, pose_y = 0, pose_theta = 0;
        cv::Mat R_mat;
        for (int i = 0; i < num; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            img_pts[0] = {(float)det->p[0][0], (float)det->p[0][1]};
            img_pts[1] = {(float)det->p[1][0], (float)det->p[1][1]};
            img_pts[2] = {(float)det->p[2][0], (float)det->p[2][1]};
            img_pts[3] = {(float)det->p[3][0], (float)det->p[3][1]};

            cv::solvePnP(tag_obj_pts, img_pts, cam_mat, dist_coeffs, rvec, tvec,
                         false, cv::SOLVEPNP_IPPE_SQUARE);
            cv::Rodrigues(rvec, R_mat);

            if (det == best) {
                pose_x     = tvec.at<double>(0);
                pose_y     = tvec.at<double>(2);
                pose_theta = std::atan2(R_mat.at<double>(0, 2), -R_mat.at<double>(2, 2));
            }
            // TODO: publish pose over ethernet
        }

        double ts_pose_done = mono_now();

        // Build telemetry record — patch stats excluded from latency measurement
        TelemetryRecord rec{};
        rec.timestamp    = f.ts_camera;
        rec.frame_number = camera_frame_count.load(std::memory_order_relaxed);
        rec.exposure_us  = g_exposure_us;
        rec.gain_linear  = g_gain;
        rec.gain_db      = 20.0f * std::log10(std::max(g_gain, 1e-6f));
        rec.condition_id = g_condition_id;
        rec.gantry_x     = g_gantry_x;
        rec.gantry_y     = g_gantry_y;
        rec.gantry_z     = g_gantry_z;
        rec.lux_tag      = g_lux_tag;
        rec.supply_ma    = g_supply_ma;
        rec.shadow_coverage_pct = g_shadow_coverage;
        rec.shadow_depth_ratio  = g_shadow_depth;

        if (best) {
            rec.detected = true;
            rec.tag_id   = best->id;
            rec.decision_margin = best->decision_margin;

            float min_x = std::min({(float)best->p[0][0], (float)best->p[3][0]});
            float max_x = std::max({(float)best->p[1][0], (float)best->p[2][0]});
            rec.pixel_count_across_tag = (int)(max_x - min_x + 0.5f);

            auto ws = compute_patch_stats(f.gray.data(), f.width, f.height,
                                          best->p, 1.15f, 1.00f);
            auto bs = compute_patch_stats(f.gray.data(), f.width, f.height,
                                          best->p, 1.00f, 0.85f);
            rec.white_mean = ws.mean; rec.white_median = ws.median;
            rec.white_std  = ws.std_dev; rec.white_min = ws.min_val; rec.white_max = ws.max_val;
            rec.black_mean = bs.mean; rec.black_median = bs.median;
            rec.black_std  = bs.std_dev; rec.black_min = bs.min_val; rec.black_max = bs.max_val;
            rec.contrast_ratio = (bs.median > 0.0f) ? ws.median / bs.median : 0.0f;
            rec.saturated = (ws.max_val >= 255.0f);

            rec.pose_x_m       = (float)pose_x;
            rec.pose_y_m       = (float)pose_y;
            rec.pose_theta_rad = (float)pose_theta;
        } else {
            rec.detected = false;
            rec.tag_id   = -1;
        }
        telemetry_buffer.push(rec);

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
    double   capture_sum = 0, queue_sum = 0, detect_sum = 0,
             pose_sum    = 0, total_sum  = 0;
    double   capture_min = 1e9, capture_max = 0;
    double   queue_min   = 1e9, queue_max   = 0;
    double   detect_min  = 1e9, detect_max  = 0;
    double   pose_min    = 1e9, pose_max    = 0;
    double   total_min   = 1e9, total_max   = 0;
    int      frames_with_detections = 0;
    int      processed_frames = 0;

    double start_time = mono_now();

    // Drains both ring buffers without printing. Called periodically to prevent
    // overflow and at shutdown for a final flush before the summary is written.
    auto drain = [&]() {
        LogSample s;
        while (log_buffer.pop(s)) {
            capture_sum += s.capture_ms;
            queue_sum   += s.queue_ms;
            detect_sum  += s.detect_ms;
            pose_sum    += s.pose_ms;
            total_sum   += s.total_ms;
            capture_min = std::min(capture_min, s.capture_ms);
            capture_max = std::max(capture_max, s.capture_ms);
            queue_min   = std::min(queue_min,   s.queue_ms);
            queue_max   = std::max(queue_max,   s.queue_ms);
            detect_min  = std::min(detect_min,  s.detect_ms);
            detect_max  = std::max(detect_max,  s.detect_ms);
            pose_min    = std::min(pose_min,    s.pose_ms);
            pose_max    = std::max(pose_max,    s.pose_ms);
            total_min   = std::min(total_min,   s.total_ms);
            total_max   = std::max(total_max,   s.total_ms);
            frames_with_detections += s.frames_with_detections;
            ++processed_frames;
        }
        if (log_tel) {
            TelemetryRecord rec;
            while (telemetry_buffer.pop(rec)) {
                *log_tel << std::fixed << std::setprecision(6)
                         << rec.timestamp              << ","
                         << rec.frame_number           << ","
                         << g_sensor->name             << ","
                         << rec.exposure_us            << ","
                         << rec.gain_db                << ","
                         << rec.gain_linear            << ","
                         << rec.condition_id           << ","
                         << rec.gantry_x               << ","
                         << rec.gantry_y               << ","
                         << rec.gantry_z               << ","
                         << rec.lux_tag                << ","
                         << rec.supply_ma              << ","
                         << rec.shadow_coverage_pct    << ","
                         << rec.shadow_depth_ratio     << ","
                         << (rec.detected ? 1 : 0)     << ","
                         << rec.tag_id                 << ","
                         << rec.decision_margin        << ","
                         << rec.pixel_count_across_tag << ","
                         << rec.white_mean             << ","
                         << rec.white_median           << ","
                         << rec.white_std              << ","
                         << rec.white_min              << ","
                         << rec.white_max              << ","
                         << rec.black_mean             << ","
                         << rec.black_median           << ","
                         << rec.black_std              << ","
                         << rec.black_min              << ","
                         << rec.black_max              << ","
                         << rec.contrast_ratio         << ","
                         << (rec.saturated ? 1 : 0)    << ","
                         << rec.glare_region_dn        << ","
                         << rec.pose_x_m               << ","
                         << rec.pose_y_m               << ","
                         << rec.pose_theta_rad         << ","
                         << rec.pose_error_mm          << '\n';
            }
            log_tel->flush();
        }
    };

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(LOG_INTERVAL_MS));
        drain();
    }
    drain();  // final flush after running goes false

    double elapsed_s  = mono_now() - start_time;
    uint64_t cam_frames = camera_frame_count.load(std::memory_order_relaxed);
    double cam_fps    = elapsed_s > 0 ? cam_frames      / elapsed_s : 0;
    double det_fps    = elapsed_s > 0 ? processed_frames / elapsed_s : 0;
    double det_rate   = processed_frames > 0
                        ? (double)frames_with_detections / processed_frames : 0.0;

    std::lock_guard<PIMutex> lock(log_mtx);
    *log_out << "Duration: "    << std::fixed << std::setprecision(1) << elapsed_s << " s"
             << " | Cam frames: " << cam_frames
             << " | Det frames: " << processed_frames << "\n";
    *log_out << "Cam FPS: "    << cam_fps << " | Det FPS: " << det_fps << " | Det Rate: " << det_rate;
    if (processed_frames > 0) {
        double n = processed_frames;
        *log_out << " | Capture: "   << capture_sum / n << " ms [" << capture_min << "-" << capture_max << "]"
                 << " | Queue: "     << queue_sum   / n << " ms [" << queue_min   << "-" << queue_max   << "]"
                 << " | Detect: "    << detect_sum  / n << " ms [" << detect_min  << "-" << detect_max  << "]"
                 << " | Pose: "      << pose_sum    / n << " ms [" << pose_min    << "-" << pose_max    << "]"
                 << " | Total E2E: " << total_sum   / n << " ms [" << total_min   << "-" << total_max   << "]";
    }
    *log_out << "\n";
    log_out->flush();
}

// ==========================
// Helpers for RT setup
// ==========================
static cpu_set_t parse_cpu_list(const std::string& s) {
    cpu_set_t cs; CPU_ZERO(&cs);
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        auto dash = token.find('-');
        if (dash == std::string::npos) {
            CPU_SET(std::stoi(token), &cs);
        } else {
            int lo = std::stoi(token.substr(0, dash));
            int hi = std::stoi(token.substr(dash + 1));
            for (int k = lo; k <= hi; k++) CPU_SET(k, &cs);
        }
    }
    return cs;
}

static cpu_set_t read_isolated_cpus() {
    cpu_set_t cs; CPU_ZERO(&cs);
    std::ifstream f("/sys/devices/system/cpu/isolated");
    if (!f) return cs;
    std::string line;
    std::getline(f, line);
    if (!line.empty() && line.back() == '\n') line.pop_back();
    return parse_cpu_list(line);
}

// ==========================
// Main
// ==========================
static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [--camera ov9281|imx296] [--exposure <µs>] [--gain <x>] [--fps <n>]"
                                      " [--nthreads <n>] [--quad-decimate <f>] [--snapshot <file>] [--log [<base>]]\n"
                                      " [--condition <n>] [--gantry-x <n>] [--gantry-y <n>] [--gantry-z <n>]\n"
                                      " [--lux <f>] [--supply-ma <f>] [--shadow-coverage <f>] [--shadow-depth <f>]\n"
              << "  --camera           Sensor model                       (default: ov9281)\n"
              << "  --exposure         Exposure time in microseconds       (default: 333)\n"
              << "  --gain             Analogue gain multiplier            (default: 1.0)\n"
              << "  --fps              Target frame rate                   (default: 60)\n"
              << "  --nthreads         AprilTag detector threads           (default: 2)\n"
              << "  --quad-decimate    AprilTag quad decimation            (default: 1.0)\n"
              << "  --snapshot         Capture one frame, save as PNG, and exit\n"
              << "  --log [<base>]     Log base path (no extension); omit for logs/vision_YYYYMMDD_HHMMSS\n"
              << "                     Creates <base>_debug.log and <base>_telemetry.csv\n"
              << "                     Omitting --log entirely sends debug to stdout; telemetry suppressed\n"
              << "  --condition <n>    Test case ID 1/2/3              (default: 0=unknown)\n"
              << "  --gantry-x <n>     Gantry X level 1-3              (default: 0=unknown)\n"
              << "  --gantry-y <n>     Gantry Y level 1-3              (default: 0=unknown)\n"
              << "  --gantry-z <n>     Gantry Z level 1-3              (default: 0=unknown)\n"
              << "  --lux <f>          Lux at tag surface               (default: 0=unknown)\n"
              << "  --supply-ma <f>    COB supply current in mA         (default: 0=unknown)\n"
              << "  --shadow-coverage <f>  Shadow coverage pct (TC2)   (default: 0=N/A)\n"
              << "  --shadow-depth <f>     Shadow depth ratio (TC2)    (default: 0=N/A)\n";
}

int main(int argc, char* argv[]) {
    std::string log_path_buf;
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
        } else if (arg == "--log") {
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                g_log_path = argv[++i];
            } else {
                char tmp[64];
                std::time_t t = std::time(nullptr);
                std::strftime(tmp, sizeof(tmp), "logs/vision_%Y%m%d_%H%M%S", std::localtime(&t));
                log_path_buf = tmp;
                g_log_path = log_path_buf.c_str();
            }
        } else if (arg == "--condition" && i + 1 < argc) {
            g_condition_id = std::stoi(argv[++i]);
        } else if (arg == "--gantry-x" && i + 1 < argc) {
            g_gantry_x = std::stoi(argv[++i]);
        } else if (arg == "--gantry-y" && i + 1 < argc) {
            g_gantry_y = std::stoi(argv[++i]);
        } else if (arg == "--gantry-z" && i + 1 < argc) {
            g_gantry_z = std::stoi(argv[++i]);
        } else if (arg == "--lux" && i + 1 < argc) {
            g_lux_tag = std::stof(argv[++i]);
        } else if (arg == "--supply-ma" && i + 1 < argc) {
            g_supply_ma = std::stof(argv[++i]);
        } else if (arg == "--shadow-coverage" && i + 1 < argc) {
            g_shadow_coverage = std::stof(argv[++i]);
        } else if (arg == "--shadow-depth" && i + 1 < argc) {
            g_shadow_depth = std::stof(argv[++i]);
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    init_logging();

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        LogLine("MAIN") << "warning: mlockall failed (run as root)";

    if (g_log_path)
        LogLine("MAIN") << "debug log: " << g_log_path << "_debug.log"
                        << " | telemetry: " << g_log_path << "_telemetry.csv";

    LogLine("MAIN") << "Camera: "           << g_sensor->name
                    << " | Exposure: "     << g_exposure_us << " µs"
                    << " | Gain: "         << g_gain << "x"
                    << " | Target FPS: "   << g_target_fps
                    << " | nthreads: "     << g_nthreads
                    << " | quad_decimate: " << g_quad_decimate;

    if (g_condition_id || g_gantry_x || g_gantry_y || g_gantry_z ||
        g_lux_tag || g_supply_ma || g_shadow_coverage || g_shadow_depth)
        LogLine("MAIN") << "condition=" << g_condition_id
                        << " gantry=" << g_gantry_x << "/" << g_gantry_y << "/" << g_gantry_z
                        << " lux=" << g_lux_tag << " supply_ma=" << g_supply_ma
                        << " shadow_cov=" << g_shadow_coverage << " shadow_depth=" << g_shadow_depth;

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    std::thread t1(capture_thread);

    {
        sched_param sp{};
        sp.sched_priority = 10;
        if (pthread_setschedparam(t1.native_handle(), SCHED_FIFO, &sp) != 0)
            LogLine("MAIN") << "warning: SCHED_FIFO for capture thread failed (needs root or CAP_SYS_NICE)";
        // Pin capture thread to core 0 — cores 1,2,3 are isolated for the detection workers.
        // Capture is SCHED_FIFO 10 so it preempts any OS task on core 0 when a frame arrives.
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(0, &cpus);
        if (pthread_setaffinity_np(t1.native_handle(), sizeof(cpus), &cpus) != 0)
            LogLine("MAIN") << "warning: setaffinity for capture thread failed";
    }

    if (g_snapshot_out) {
        // Skip the first few frames — early deliveries can be black/garbage.
        std::shared_ptr<Frame> fp;
        for (int skip = 0; skip < 5; skip++) {
            while (running && !latest_frame.wait_and_get(fp));
        }

        if (fp && !fp->gray.empty()) {
            cv::Mat img(fp->height, fp->width, CV_8UC1, fp->gray.data());
            cv::imwrite(g_snapshot_out, img);
            LogLine("MAIN") << "Snapshot saved: " << g_snapshot_out;
        }
        running = false;
        t1.join();
        return 0;
    }

    std::thread t2(detection_thread);

    {
        // SCHED_FIFO below capture (10) so capture can always preempt for new frames.
        // apriltag worker threads inherit both scheduler policy and affinity.
        sched_param sp{};
        sp.sched_priority = 5;
        if (pthread_setschedparam(t2.native_handle(), SCHED_FIFO, &sp) != 0)
            LogLine("MAIN") << "warning: SCHED_FIFO for detection thread failed";
        cpu_set_t iso = read_isolated_cpus();
        CPU_CLR(0, &iso);  // don't share with capture thread's core
        if (CPU_COUNT(&iso) > 0) {
            if (pthread_setaffinity_np(t2.native_handle(), sizeof(iso), &iso) != 0)
                LogLine("MAIN") << "warning: setaffinity for detection thread failed";
        }
    }

    std::thread t3(logging_thread);

    while (running) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
