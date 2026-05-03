#include "camera.h"

#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>

// Scan /sys/class/video4linux/<devname>/name to match candidate devices.
static std::string read_sysfs_name(const std::string& devname) {
    std::ifstream f("/sys/class/video4linux/" + devname + "/name");
    if (!f) return "";
    std::string s;
    std::getline(f, s);
    return s;
}

// Returns the /dev path for the first video device whose sysfs name
// contains `needle`, or "" if not found.
static std::string find_video_dev(const char* needle) {
    for (int i = 0; i < 64; i++) {
        std::string dev = "video" + std::to_string(i);
        if (read_sysfs_name(dev).find(needle) != std::string::npos)
            return "/dev/" + dev;
    }
    return "";
}

// Returns the /dev path for the first subdev whose sysfs name
// contains `needle`, or "" if not found.
static std::string find_subdev(const char* needle) {
    for (int i = 0; i < 64; i++) {
        std::string dev = "v4l-subdev" + std::to_string(i);
        if (read_sysfs_name(dev).find(needle) != std::string::npos)
            return "/dev/" + dev;
    }
    return "";
}

#ifdef RPI5
// Returns /dev/mediaX whose MEDIA_IOC_DEVICE_INFO driver field matches `driver`.
static std::string find_media_dev(const char* driver) {
    for (int i = 0; i < 16; i++) {
        std::string path = "/dev/media" + std::to_string(i);
        int fd = open(path.c_str(), O_RDWR);
        if (fd < 0) continue;
        struct media_device_info info = {};
        bool ok = ioctl(fd, MEDIA_IOC_DEVICE_INFO, &info) == 0
                  && strncmp(info.driver, driver, sizeof(info.driver)) == 0;
        close(fd);
        if (ok) return path;
    }
    return "";
}

// Pi 5 rp1-cfe requires explicit media pipeline configuration before streaming.
// Enables csi2:pad4 -> rp1-cfe-csi2_ch0:pad0 and sets Y10_1X10 on csi2 pads.
// Pad 4 is the CSI-2 channel-0 source on the rp1-cfe csi2 bridge (fixed in driver).
static bool configure_rp1_pipeline(const SensorSpec& spec) {
    std::string media_path = find_media_dev("rp1-cfe");
    if (media_path.empty()) {
        LogLine("CAPTURE") << "rp1-cfe media device not found";
        return false;
    }
    int mfd = open(media_path.c_str(), O_RDWR);
    if (mfd < 0) {
        LogLine("CAPTURE") << "open " << media_path << " failed";
        return false;
    }

    uint32_t csi2_id = 0, ch0_id = 0;
    {
        struct media_entity_desc ent = {};
        ent.id = MEDIA_ENT_ID_FLAG_NEXT;
        while (ioctl(mfd, MEDIA_IOC_ENUM_ENTITIES, &ent) == 0) {
            if (strcmp(ent.name, "csi2")              == 0) csi2_id = ent.id;
            if (strcmp(ent.name, "rp1-cfe-csi2_ch0") == 0) ch0_id  = ent.id;
            ent.id |= MEDIA_ENT_ID_FLAG_NEXT;
        }
    }
    if (!csi2_id || !ch0_id) {
        LogLine("CAPTURE") << "media entities missing (csi2=" << csi2_id
                           << " ch0=" << ch0_id << ")";
        close(mfd);
        return false;
    }

    struct media_link_desc link = {};
    link.source.entity = csi2_id; link.source.index = 4; link.source.flags = MEDIA_PAD_FL_SOURCE;
    link.sink.entity   = ch0_id;  link.sink.index   = 0; link.sink.flags   = MEDIA_PAD_FL_SINK;
    link.flags = MEDIA_LNK_FL_ENABLED;
    if (ioctl(mfd, MEDIA_IOC_SETUP_LINK, &link) < 0) {
        LogLine("CAPTURE") << "MEDIA_IOC_SETUP_LINK failed: " << strerror(errno);
        close(mfd);
        return false;
    }
    close(mfd);

    std::string csi2_dev = find_subdev("csi2");
    if (csi2_dev.empty()) {
        LogLine("CAPTURE") << "csi2 subdev not found";
        return false;
    }
    int csi2_fd = open(csi2_dev.c_str(), O_RDWR);
    if (csi2_fd < 0) {
        LogLine("CAPTURE") << "open csi2 subdev failed";
        return false;
    }
    for (int pad : {0, 4}) {
        struct v4l2_subdev_format sfmt = {};
        sfmt.which          = V4L2_SUBDEV_FORMAT_ACTIVE;
        sfmt.pad            = (uint32_t)pad;
        sfmt.format.width   = spec.width;
        sfmt.format.height  = spec.height;
        sfmt.format.code    = MEDIA_BUS_FMT_Y10_1X10;
        sfmt.format.field   = V4L2_FIELD_NONE;
        if (ioctl(csi2_fd, VIDIOC_SUBDEV_S_FMT, &sfmt) < 0)
            LogLine("CAPTURE") << "warning: csi2 pad" << pad << " format failed";
    }
    close(csi2_fd);

    LogLine("CAPTURE") << "rp1-cfe pipeline configured (" << media_path
                       << " csi2=" << csi2_id << " ch0=" << ch0_id << ")";
    return true;
}
#endif

static bool set_ctrl(int fd, uint32_t id, int32_t val, const char* label) {
    struct v4l2_control ctrl = {};
    ctrl.id    = id;
    ctrl.value = val;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        LogLine("CAPTURE") << "warning: failed to set " << label;
        return false;
    }
    return true;
}

void capture_thread() {
    prefault_stack();
    const SensorSpec& spec = *g_sensor;

    // ---- Device discovery ------------------------------------------------
    // Sensor name in sysfs is lowercase, e.g. "ov9281 10-0060"
    char sensor_needle[32];
    std::transform(spec.name, spec.name + strlen(spec.name),
                   sensor_needle, ::tolower);

#ifdef RPI5
    std::string video_path  = find_video_dev("rp1-cfe-csi2_ch0");
#else
    std::string video_path  = find_video_dev("unicam-image");
#endif
    std::string subdev_path = find_subdev(sensor_needle);

    if (video_path.empty() || subdev_path.empty()) {
        LogLine("CAPTURE") << "device discovery failed"
                           << " (video=" << video_path
                           << " subdev=" << subdev_path << ")";
        running = false;
        return;
    }

    LogLine("CAPTURE") << "video=" << video_path << " subdev=" << subdev_path;

    int subdev_fd = open(subdev_path.c_str(), O_RDWR);
    int video_fd  = open(video_path.c_str(),  O_RDWR);
    if (subdev_fd < 0 || video_fd < 0) {
        LogLine("CAPTURE") << "failed to open devices";
        running = false;
        if (subdev_fd >= 0) close(subdev_fd);
        if (video_fd  >= 0) close(video_fd);
        return;
    }

#ifdef RPI5
    if (!configure_rp1_pipeline(spec)) {
        running = false;
        close(subdev_fd); close(video_fd);
        return;
    }
#endif

    // ---- Sensor subdev format --------------------------------------------
    struct v4l2_subdev_format sfmt = {};
    sfmt.which            = V4L2_SUBDEV_FORMAT_ACTIVE;
    sfmt.pad              = 0;
    sfmt.format.width     = spec.width;
    sfmt.format.height    = spec.height;
    sfmt.format.code      = MEDIA_BUS_FMT_Y10_1X10;
    sfmt.format.field     = V4L2_FIELD_NONE;
    if (ioctl(subdev_fd, VIDIOC_SUBDEV_S_FMT, &sfmt) < 0) {
        LogLine("CAPTURE") << "VIDIOC_SUBDEV_S_FMT failed";
        running = false;
        close(subdev_fd); close(video_fd);
        return;
    }

    // ---- Video device format ---------------------------------------------
    struct v4l2_format fmt = {};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = spec.width;
    fmt.fmt.pix.height      = spec.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y10P;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) {
        LogLine("CAPTURE") << "VIDIOC_S_FMT failed";
        running = false;
        close(subdev_fd); close(video_fd);
        return;
    }
    unsigned stride     = fmt.fmt.pix.bytesperline;
    unsigned frame_size = fmt.fmt.pix.sizeimage;

    // ---- Sensor controls -------------------------------------------------
    // Compute vblank for the requested fps; clamp to vblank_min so we never
    // ask for a frame rate the sensor can't achieve.
    // Also pre-compute exposure lines here so we can expand vblank if needed:
    // when exposure > frame period the sensor silently ignores the exposure
    // register, producing a black frame.  Set vblank large enough that
    // exposure always fits within the frame, accepting a lower fps if needed.
    int64_t hts     = spec.width + spec.hblank_min;
    double  line_period_us = (double)hts / spec.pixel_clock_hz * 1e6;
    int     exposure_lines = std::max(1, (int)(g_exposure_us / line_period_us));

    int     vts_fps = (int)(spec.pixel_clock_hz / (hts * g_target_fps));
    // Sensor requires exposure <= vts - vblank_min (conservative safe margin).
    // If exposure demands a longer frame, expand vts and warn — fps will drop.
    int     vts     = std::max(vts_fps, exposure_lines + spec.vblank_min);
    if (vts > vts_fps)
        LogLine("CAPTURE") << "warning: exposure " << g_exposure_us / 1000.0
                           << " ms exceeds " << g_target_fps
                           << " fps frame period — actual fps will be reduced";
    int     vblank  = std::max(spec.vblank_min, vts - spec.height);
    double  actual_fps = (double)spec.pixel_clock_hz / (hts * (spec.height + vblank));

    set_ctrl(subdev_fd, V4L2_CID_HBLANK, spec.hblank_min, "HBLANK");
    set_ctrl(subdev_fd, V4L2_CID_VBLANK, vblank, "VBLANK");

    // Read back VBLANK to confirm it was accepted by the driver.
    {
        struct v4l2_control chk = {};
        chk.id = V4L2_CID_VBLANK;
        if (ioctl(subdev_fd, VIDIOC_G_CTRL, &chk) == 0) {
            double confirmed_fps = (double)spec.pixel_clock_hz
                                   / ((spec.width + spec.hblank_min)
                                      * (spec.height + chk.value));
            LogLine("CAPTURE") << "vblank readback=" << chk.value
                                << " confirmed_fps=" << confirmed_fps;
        }
    }

    set_ctrl(subdev_fd, V4L2_CID_EXPOSURE, exposure_lines, "EXPOSURE");
    {
        struct v4l2_control chk = {};
        chk.id = V4L2_CID_EXPOSURE;
        if (ioctl(subdev_fd, VIDIOC_G_CTRL, &chk) == 0)
            LogLine("CAPTURE") << "exposure readback=" << chk.value
                               << " lines (" << chk.value * line_period_us / 1000.0 << " ms)"
                               << (chk.value != exposure_lines ? " *** CLAMPED ***" : "");
    }

    int gain_reg = spec.log_gain
        ? std::clamp((int)std::round(200.0f * std::log10(g_gain)), spec.gain_reg_min, spec.gain_reg_max)
        : std::clamp((int)(g_gain * spec.gain_reg_per_unit),        spec.gain_reg_min, spec.gain_reg_max);
    set_ctrl(subdev_fd, V4L2_CID_ANALOGUE_GAIN, gain_reg, "ANALOGUE_GAIN");
    {
        struct v4l2_control chk = {};
        chk.id = V4L2_CID_ANALOGUE_GAIN;
        if (ioctl(subdev_fd, VIDIOC_G_CTRL, &chk) == 0)
            LogLine("CAPTURE") << "gain readback=" << chk.value
                               << (chk.value != gain_reg ? " *** CLAMPED ***" : "");
    }

    LogLine("CAPTURE") << spec.name
                       << " Y10P " << spec.width << "x" << spec.height
                       << " stride=" << stride
                       << " vblank=" << vblank
                       << " fps=" << actual_fps
                       << " exposure=" << exposure_lines << " lines"
                       << " gain_reg=" << gain_reg;

    // ---- Buffer allocation -----------------------------------------------
    const int n_bufs_requested = 2;
    struct v4l2_requestbuffers req = {};
    req.count  = n_bufs_requested;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) {
        LogLine("CAPTURE") << "VIDIOC_REQBUFS failed";
        running = false;
        close(subdev_fd); close(video_fd);
        return;
    }

    struct MappedBuf { uint8_t* ptr; size_t len; };
    std::vector<MappedBuf> mapped(req.count);
    for (uint32_t i = 0; i < req.count; i++) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ioctl(video_fd, VIDIOC_QUERYBUF, &buf);
        mapped[i].len = buf.length;
        mapped[i].ptr = static_cast<uint8_t*>(
            mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                 MAP_SHARED, video_fd, buf.m.offset));
        if (mapped[i].ptr == MAP_FAILED) {
            LogLine("CAPTURE") << "mmap failed for buffer " << i;
            running = false;
            close(subdev_fd); close(video_fd);
            return;
        }
        ioctl(video_fd, VIDIOC_QBUF, &buf);
    }

    // ---- Start streaming -------------------------------------------------
    enum v4l2_buf_type btype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMON, &btype) < 0) {
        LogLine("CAPTURE") << "VIDIOC_STREAMON failed";
        running = false;
        for (auto& m : mapped) munmap(m.ptr, m.len);
        close(subdev_fd); close(video_fd);
        return;
    }

    const int pixels = spec.width * spec.height;

    // Pre-allocate 2 Frames with gray buffers sized for this sensor.
    // Reusing them avoids a per-frame 1 MB heap allocation + zero-init in the hot path.
    // Pool size 2 is sufficient: frame period (16.67ms) >> detect time (4ms), so
    // the slot we're about to reuse has always been released by detection.
    std::shared_ptr<Frame> frame_pool[2];
    for (int pi = 0; pi < 2; pi++) {
        frame_pool[pi]         = std::make_shared<Frame>();
        frame_pool[pi]->width  = spec.width;
        frame_pool[pi]->height = spec.height;
        frame_pool[pi]->gray.resize(pixels);
    }
    int pool_idx = 0;

    // Global shutter: all pixels expose simultaneously, then rows are clocked out.
    // ts_hw (CSI-2 interrupt) fires at end of readout, so:
    //   ts_camera = ts_hw - readout_time - exposure_time/2
    // giving the center-of-exposure as the E2E latency reference.
    double line_period_s = (double)(spec.width + spec.hblank_min) / spec.pixel_clock_hz;
    double readout_s     = spec.height * line_period_s;
    double exposure_s    = g_exposure_us * 1e-6;
    double ts_correction = readout_s + exposure_s / 2.0;
    LogLine("CAPTURE") << "ts_correction=" << ts_correction * 1000.0
                       << " ms (readout=" << readout_s * 1000.0
                       << " ms + exposure/2=" << exposure_s / 2.0 * 1000.0 << " ms)";

    // Heap staging buffer — avoids byte-by-byte reads from uncached DMA memory.
    std::vector<uint8_t> raw_staging(frame_size + 32);  // +32 for NEON overread safety

    bool ts_source_logged = false;

    // ---- Capture loop ----------------------------------------------------
    while (running) {
        struct pollfd pfd = { video_fd, POLLIN, 0 };
        int r = poll(&pfd, 1, 100);
        if (r < 0 || !(pfd.revents & POLLIN)) continue;

        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(video_fd, VIDIOC_DQBUF, &buf) < 0) continue;

        double ts_dqbuf = mono_now();

        // Prefer the kernel's hardware timestamp (set at CSI-2 interrupt).
        // Fall back to monotonic dequeue time if the driver doesn't advertise it.
        bool hw_ts = (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK)
                         == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        double ts_hw = hw_ts
                       ? buf.timestamp.tv_sec + buf.timestamp.tv_usec * 1e-6
                       : ts_dqbuf;

        if (!ts_source_logged) {
            LogLine("CAPTURE") << "ts_camera source: "
                               << (hw_ts ? "V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC (hardware)"
                                         : "mono_now() fallback (MONOTONIC flag not set)");
            ts_source_logged = true;
        }

        // Copy from uncached DMA mapping to heap before unpacking.
        std::memcpy(raw_staging.data(), mapped[buf.index].ptr, frame_size);
        ioctl(video_fd, VIDIOC_QBUF, &buf);  // return buffer immediately after copy

        // Reuse pool slot — in steady state never spins since detection (4ms)
        // finishes well before the next frame arrives (16.67ms).
        while (frame_pool[pool_idx].use_count() > 1)
            std::this_thread::yield();

        auto& fp     = frame_pool[pool_idx];
        pool_idx    ^= 1;

        fp->ts_camera = ts_hw - ts_correction;
        unpack_to_u8(fp->gray.data(), raw_staging.data(),
                     spec.width, spec.height, stride, spec.black_level);

        fp->ts_captured = mono_now();
        latest_frame.push(fp);
        camera_frame_count.fetch_add(1, std::memory_order_relaxed);
    }

    // ---- Cleanup ---------------------------------------------------------
    ioctl(video_fd, VIDIOC_STREAMOFF, &btype);
    for (auto& m : mapped) munmap(m.ptr, m.len);
    close(subdev_fd);
    close(video_fd);
}
