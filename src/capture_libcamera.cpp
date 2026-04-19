#include "camera.h"

#include <libcamera/libcamera.h>
#include <sys/mman.h>
#include <condition_variable>
#include <vector>
#include <memory>

void capture_thread() {
    using namespace libcamera;

    const SensorSpec& spec = *g_sensor;

    CameraManager cm;
    if (cm.start() != 0) { running = false; return; }

    auto cameras = cm.cameras();
    if (cameras.empty()) {
        std::lock_guard<std::mutex> lk(log_mtx);
        *log_out << "[CAPTURE] no cameras found\n";
        running = false; cm.stop(); return;
    }

    std::shared_ptr<Camera> camera = cameras[0];
    if (camera->acquire() != 0) { running = false; cm.stop(); return; }

    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({ StreamRole::Raw });
    StreamConfiguration& sc = config->at(0);
    sc.pixelFormat = formats::R10_CSI2P;
    sc.size        = { (unsigned)spec.width, (unsigned)spec.height };
    sc.bufferCount = 4;

    CameraConfiguration::Status vstatus = config->validate();
    if (vstatus == CameraConfiguration::Invalid) {
        running = false; camera->release(); cm.stop(); return;
    }
    if (vstatus == CameraConfiguration::Adjusted) {
        std::lock_guard<std::mutex> lk(log_mtx);
        *log_out << "[CAPTURE] config adjusted to "
                 << sc.pixelFormat.toString() << " "
                 << sc.size.width << "x" << sc.size.height << "\n";
    }
    if (camera->configure(config.get()) != 0) {
        running = false; camera->release(); cm.stop(); return;
    }

    Stream*      stream = sc.stream();
    unsigned int stride = sc.stride;

    {
        std::lock_guard<std::mutex> lk(log_mtx);
        *log_out << "[CAPTURE] " << spec.name << " "
                 << sc.pixelFormat.toString()
                 << " " << sc.size.width << "x" << sc.size.height
                 << " stride=" << stride << "\n";
    }

    FrameBufferAllocator allocator(camera);
    allocator.allocate(stream);
    const auto& fbufs = allocator.buffers(stream);
    const int n_bufs = (int)fbufs.size();

    struct MappedBuf { uint8_t* ptr; size_t len; };
    std::vector<MappedBuf> mapped(n_bufs);
    for (int i = 0; i < n_bufs; i++) {
        const FrameBuffer::Plane& plane = fbufs[i]->planes()[0];
        mapped[i].len = plane.length;
        mapped[i].ptr = static_cast<uint8_t*>(
            mmap(nullptr, plane.length, PROT_READ, MAP_SHARED,
                 plane.fd.get(), plane.offset));
        if (mapped[i].ptr == MAP_FAILED) {
            running = false; camera->release(); cm.stop(); return;
        }
    }

    std::vector<std::unique_ptr<Request>> requests;
    for (auto& fb : fbufs) {
        auto req = camera->createRequest();
        req->addBuffer(stream, fb.get(), std::unique_ptr<Fence>{});
        requests.push_back(std::move(req));
    }

    struct CbCtx {} cb_ctx;
    std::mutex              cb_mtx;
    std::condition_variable cb_cv;
    std::vector<Request*>   completed;

    camera->requestCompleted.connect(&cb_ctx, [&](Request* req) {
        { std::lock_guard<std::mutex> lk(cb_mtx); completed.push_back(req); }
        cb_cv.notify_one();
    });

    // Both OV9281 and IMX296 are monochrome — AwbEnable is not in their control maps.
    ControlList ctrl(camera->controls());
    ctrl.set(controls::AeEnable,          false);
    ctrl.set(controls::ExposureTimeMode,  controls::ExposureTimeModeManual);
    ctrl.set(controls::ExposureTime,      g_exposure_us);
    ctrl.set(controls::AnalogueGainMode,  controls::AnalogueGainModeManual);
    ctrl.set(controls::AnalogueGain,      g_gain);
    ctrl.set(controls::FrameDurationLimits,
             {(int64_t)g_exposure_us, (int64_t)g_exposure_us});
    if (camera->start(&ctrl) != 0) {
        running = false; camera->release(); cm.stop(); return;
    }

    for (auto& req : requests) camera->queueRequest(req.get());

    const int pixels = spec.width * spec.height;

    while (running) {
        Request* req = nullptr;
        {
            std::unique_lock<std::mutex> lk(cb_mtx);
            cb_cv.wait_for(lk, std::chrono::milliseconds(100),
                           [&]{ return !completed.empty(); });
            if (completed.empty()) continue;
            req = completed.front();
            completed.erase(completed.begin());
        }
        if (req->status() == Request::RequestCancelled) continue;

        const auto& meta = req->metadata();
        int64_t sensor_ns = meta.get(controls::SensorTimestamp).value_or(0);
        double ts_camera  = sensor_ns ? sensor_ns * 1e-9 : mono_now();

        FrameBuffer* fb = req->findBuffer(stream);
        int idx = -1;
        for (int i = 0; i < n_bufs; i++)
            if (fbufs[i].get() == fb) { idx = i; break; }

        Frame f;
        f.ts_camera = ts_camera;
        f.width     = spec.width;
        f.height    = spec.height;
        f.gray.resize(pixels);

        if (idx >= 0)
            unpack_to_u8(f.gray.data(), mapped[idx].ptr,
                         spec.width, spec.height, stride, spec.black_level);

        f.ts_captured = mono_now();
        latest_frame.push(std::move(f));
        camera_frame_count.fetch_add(1, std::memory_order_relaxed);

        req->reuse(Request::ReuseBuffers);
        camera->queueRequest(req);
    }

    camera->stop();
    camera->requestCompleted.disconnect(&cb_ctx);
    for (auto& m : mapped) munmap(m.ptr, m.len);
    allocator.free(stream);
    camera->release();
    cm.stop();
}
