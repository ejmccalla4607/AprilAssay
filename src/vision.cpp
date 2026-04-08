#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <ctime>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>

extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/matd.h"
}

// Camera intrinsics — calibrate these for your lens
static const double CAM_FX     = 600.0;
static const double CAM_FY     = 600.0;
static const double CAM_CX     = 320.0;
static const double CAM_CY     = 240.0;
static const double TAG_SIZE_M = 0.165; // meters (FRC standard tag)

static const int CAM_WIDTH  = 640;
static const int CAM_HEIGHT = 480;

static const int LOG_INTERVAL_MS = 2000;

// ==========================
// Logging output
// ==========================
// In debug mode: log to stdout. Otherwise: log to timestamped file.
static std::ostream* log_out = nullptr;

static void init_logging() {
#ifdef DEBUG_LOGGING
    log_out = &std::cout;
#else
    // Build filename: vision_YYYYMMDD_HHMMSS.log
    std::time_t t = std::time(nullptr);
    std::tm* tm   = std::localtime(&t);
    char buf[64];
    std::strftime(buf, sizeof(buf), "vision_%Y%m%d_%H%M%S.log", tm);
    // Static so it lives for the duration of the program
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
// Frame + Latest Frame Store
// ==========================
struct Frame {
    std::vector<uint8_t> gray; // grayscale, one byte per pixel
    int width;
    int height;
    double ts_camera;    // V4L2 hardware timestamp (monotonic)
    double ts_captured;  // after YUYV→gray extraction, before push
    double ts_dequeued;  // when detection thread picks it up
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

    // Returns true only if a new frame has arrived since the last get()
    bool get(Frame& out) {
        std::lock_guard<std::mutex> lock(mtx);
        if (get_id == push_id) return false;
        out    = frame;
        get_id = push_id;
        return true;
    }
};

// ==========================
// Ring Buffer for Logging
// ==========================
struct LogSample {
    // All times in milliseconds
    double capture_ms;   // ts_captured  - ts_camera   (YUYV extract + push)
    double queue_ms;     // ts_dequeued  - ts_captured  (wait in LatestFrame)
    double detect_ms;    // after detect - ts_dequeued
    double pose_ms;      // after pose   - after detect
    double total_ms;     // ts_pose_done - ts_camera
    int    detections;
};

constexpr int LOG_SIZE = 1024;

class RingBuffer {
    LogSample       buf[LOG_SIZE];
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

// ==========================
LatestFrame           latest_frame;
RingBuffer            log_buffer;
std::atomic<bool>     running{true};
std::atomic<uint64_t> camera_frame_count{0};

static void handle_signal(int) { running = false; }

// Helper: current monotonic time in seconds
static inline double mono_now() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// ==========================
// Capture Thread (V4L2 YUYV)
// ==========================
void capture_thread() {
    int fd = open("/dev/video0", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("open /dev/video0");
        return;
    }

    // Request YUYV format
    v4l2_format fmt{};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_WIDTH;
    fmt.fmt.pix.height      = CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        running = false;
        close(fd);
        return;
    }

    // Request 30fps
    v4l2_streamparm parm{};
    parm.type                                  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = 30;
    ioctl(fd, VIDIOC_S_PARM, &parm);

    // Verify what the driver actually granted
    ioctl(fd, VIDIOC_G_PARM, &parm);
    *log_out << "Camera frame rate: "
             << parm.parm.capture.timeperframe.denominator << "/"
             << parm.parm.capture.timeperframe.numerator   << " fps\n";

    // Lock camera controls for consistent AprilTag detection
    struct { uint32_t id; int value; const char* name; } cam_ctrls[] = {
        { V4L2_CID_EXPOSURE_AUTO,          V4L2_EXPOSURE_MANUAL, "auto_exposure"           },
        { V4L2_CID_EXPOSURE_ABSOLUTE,      333,                  "exposure_time_absolute"  },
        { V4L2_CID_BACKLIGHT_COMPENSATION, 0,                    "backlight_compensation"  },
        { V4L2_CID_SHARPNESS,              15,                   "sharpness"               },
        { V4L2_CID_AUTO_WHITE_BALANCE,     0,                    "white_balance_automatic" },
    };

    for (auto& c : cam_ctrls) {
        v4l2_control ctrl{ c.id, c.value };
        if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0)
            perror(c.name);
    }

    // Give the camera time to apply control changes before streaming
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Request mmap buffers
    v4l2_requestbuffers req{};
    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        running = false;
        close(fd);
        return;
    }

    struct Buffer { void* ptr; size_t len; };
    std::vector<Buffer> buffers(req.count);

    for (unsigned i = 0; i < req.count; i++) {
        v4l2_buffer buf{};
        buf.type   = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ioctl(fd, VIDIOC_QUERYBUF, &buf);

        buffers[i].len = buf.length;
        buffers[i].ptr = mmap(nullptr, buf.length,
                              PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, buf.m.offset);
        if (buffers[i].ptr == MAP_FAILED) {
            perror("mmap");
            running = false;
            close(fd);
            return;
        }
        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    int type = req.type;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        running = false;
        close(fd);
        return;
    }

    const int pixels = CAM_WIDTH * CAM_HEIGHT;

    while (running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        timeval tv{0, 100000}; // 100ms timeout
        int ret = select(fd + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) continue;

        v4l2_buffer buf{};
        buf.type   = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) continue;

        // Timestamp immediately after dequeue — CLOCK_MONOTONIC, same
        // clock as all other pipeline timestamps. The V4L2 hardware
        // timestamp is not used because UVC cameras typically report
        // CLOCK_REALTIME which cannot be compared against CLOCK_MONOTONIC.
        double ts_camera = mono_now();

        // Extract Y channel from YUYV (Y is every even byte)
        Frame f;
        f.ts_camera  = ts_camera;
        f.width      = CAM_WIDTH;
        f.height     = CAM_HEIGHT;
        f.gray.resize(pixels);

        const uint8_t* src = (const uint8_t*)buffers[buf.index].ptr;
        for (int i = 0; i < pixels; i++) {
            f.gray[i] = src[i * 2];
        }

        f.ts_captured = mono_now();
        latest_frame.push(std::move(f));
        camera_frame_count.fetch_add(1, std::memory_order_relaxed);

        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (auto& b : buffers) munmap(b.ptr, b.len);
    close(fd);
}

// ==========================
// Detection Thread
// ==========================
void detection_thread() {
    apriltag_family_t*   tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();

    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0;
    td->nthreads      = 2;

    Frame f;

    while (running) {
        if (!latest_frame.get(f)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        f.ts_dequeued = mono_now();

        // --- Detection ---
        image_u8_t img{
            .width  = f.width,
            .height = f.height,
            .stride = f.width,
            .buf    = f.gray.data()
        };

        zarray_t* detections = apriltag_detector_detect(td, &img);
        double ts_detect_done = mono_now();

        int num = zarray_size(detections);

        // --- Pose estimation ---
        for (int i = 0; i < num; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            apriltag_detection_info_t info;
            info.det     = det;
            info.tagsize = TAG_SIZE_M;
            info.fx      = CAM_FX;
            info.fy      = CAM_FY;
            info.cx      = CAM_CX;
            info.cy      = CAM_CY;

            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);

            double tx = pose.t->data[0];
            double ty = pose.t->data[1];
            double tz = pose.t->data[2];

            matd_destroy(pose.R);
            matd_destroy(pose.t);

            // TODO: publish tx/ty/tz over ethernet
            (void)tx; (void)ty; (void)tz;
        }

        double ts_pose_done = mono_now();

        apriltag_detections_destroy(detections);

        // Push all stage latencies (ms) to log buffer
        log_buffer.push({
            (f.ts_captured   - f.ts_camera)    * 1000.0,  // capture
            (f.ts_dequeued   - f.ts_captured)  * 1000.0,  // queue wait
            (ts_detect_done  - f.ts_dequeued)  * 1000.0,  // detect
            (ts_pose_done    - ts_detect_done) * 1000.0,  // pose
            (ts_pose_done    - f.ts_camera)    * 1000.0,  // total end-to-end
            num
        });
    }

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

// ==========================
// Logging Thread
// ==========================
void logging_thread() {
    LogSample s;
    double capture_sum = 0, queue_sum = 0, detect_sum = 0,
           pose_sum = 0, total_sum = 0;
    int    detect_count = 0;
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
            detect_count++;
        }

        uint64_t current_camera_count = camera_frame_count.load(std::memory_order_relaxed);
        uint64_t cam_frames           = current_camera_count - last_camera_count;
        last_camera_count             = current_camera_count;

        double cam_fps    = cam_frames   / interval_s;
        double detect_fps = detect_count / interval_s;

        *log_out << "Cam FPS: " << cam_fps << " | Det FPS: " << detect_fps;
        if (detect_count > 0) {
            double n = detect_count;
            *log_out << " | Capture: "   << capture_sum / n << " ms"
                     << " | Queue: "     << queue_sum   / n << " ms"
                     << " | Detect: "    << detect_sum  / n << " ms"
                     << " | Pose: "      << pose_sum    / n << " ms"
                     << " | Total E2E: " << total_sum   / n << " ms";
        }
        *log_out << "\n";
        log_out->flush();

        capture_sum = queue_sum = detect_sum = pose_sum = total_sum = 0;
        detect_count = 0;
    }
}

// ==========================
// Main
// ==========================
int main() {
    init_logging();

    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    std::thread t1(capture_thread);
    std::thread t2(detection_thread);
    std::thread t3(logging_thread);

    while (running) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    t1.join();
    t2.join();
    t3.join();

    return 0;
}