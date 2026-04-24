# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

AprilAssay is a latency-characterization pipeline for AprilTag detection in FRC robotics. It runs on a Raspberry Pi 4 with an OV9281 or IMX296 global-shutter CSI-2 camera, captures raw 10-bit frames via V4L2/unicam (bypassing the ISP entirely), detects AprilTag 36h11 tags, estimates 6-DOF pose, and logs per-stage latency. The primary goal is minimizing and measuring end-to-end latency from center-of-exposure to pose output.

## Build and run

```bash
# First-time setup (installs deps, builds apriltag, configures isolcpus, CPU governor)
./setup.sh

# Standard release build (logs to timestamped vision_YYYYMMDD_HHMMSS.log)
cmake -S . -B build && cmake --build build -j$(nproc)

# Debug build (logs to stdout — use this for development)
cmake -S . -B build -DDEBUG_LOGGING=1 && cmake --build build -j$(nproc)

# Run (must be root for SCHED_FIFO + unicam IRQ boost)
sudo ./run.sh --camera ov9281 --fps 60 --quad-decimate 2.0

# Snapshot mode — capture one frame to PNG and exit
sudo ./run.sh --camera ov9281 --snapshot frame.png
```

**Always use `run.sh` instead of `./build/vision` directly.** `run.sh` boosts the unicam kernel IRQ thread to SCHED_FIFO 49 before exec'ing the binary. Without this, the CSI-2 interrupt that wakes the capture thread runs at normal priority and adds wake-up jitter.

## Architecture

The pipeline has three threads plus main:

```
capture_thread  →  LatestFrame  →  detection_thread  →  log_buffer  →  logging_thread
   (core 0)       (shared_ptr       (cores 1,2,3)       (ring buf)     (any core)
                   handoff)
```

### capture_thread (`src/capture_v4l2.cpp`)

Opens `/dev/video0` (unicam-image) and the sensor subdev, configures V4L2 format/controls, and runs a poll+DQBUF loop. Per frame:
1. `poll()` blocks until the kernel signals a completed DMA buffer
2. `VIDIOC_DQBUF` — reads hardware timestamp from `buf.timestamp` (CSI-2 frame-end interrupt, `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC`)
3. `memcpy` from uncached DMA mapping → heap staging buffer (`raw_staging`) — necessary because uncached ARM reads are slow; copying to cached memory first makes the subsequent NEON unpack fast
4. `VIDIOC_QBUF` — returns buffer to kernel immediately after copy
5. NEON unpack (`unpack_to_u8` in `camera.h`) — Y10P → uint8, subtracting black level and scaling to [0,255]
6. Timestamps the frame and pushes a `shared_ptr<Frame>` to `LatestFrame`

**Frame pool**: Two `Frame` objects are pre-allocated at startup with their `gray` buffers already sized. The loop reuses them (checking `use_count() == 1` before reuse) to avoid per-frame 1 MB heap allocation and zero-initialization. In steady state the slot is always free since detection (3.5 ms) finishes well before the next frame period (16.67 ms at 60 fps).

**Timestamp correction**: Both sensors are global shutter. The hardware timestamp fires at end-of-readout (EOF). `ts_camera` is computed as:
```
ts_camera = ts_hw - readout_time - exposure_time/2
```
This gives the center-of-exposure as the latency reference (when the scene was actually captured). At 60 fps with 333 µs exposure, `ts_correction ≈ 8.69 ms`.

### LatestFrame (`src/camera.h`)

Lock-free-ish handoff between capture and detection. Holds one `shared_ptr<Frame>`. Detection blocks on a condition variable with a 50 ms timeout (for clean shutdown). Uses `push_id`/`get_id` counters so detection always gets the most recent frame — if detection falls behind, it skips frames rather than processing a queue.

Uses `PIMutex` (priority-inheriting pthread mutex) and `std::condition_variable_any` to prevent priority inversion between the SCHED_FIFO detection thread and any normal-priority thread that holds the lock.

### detection_thread (`src/vision.cpp`)

Calls `wait_and_get`, runs `apriltag_detector_detect`, then `cv::solvePnP` (IPPE_SQUARE — closed-form, non-iterative, fast) for each detected tag. Detection results are pushed to `detect_log_buffer` (a lock-free ring buffer) rather than written synchronously — the logging thread drains and writes them during each 2-second stats interval.

### logging_thread (`src/vision.cpp`)

Sleeps 2 seconds, drains both ring buffers (`log_buffer` for timing stats, `detect_log_buffer` for `[DETECT]` lines), and writes a single stats line:
```
Cam FPS: 60 | Det FPS: 60 | Det Rate: 0 | Capture: 20.3 ms [20.1-20.9] | Queue: 0.05 ms | Detect: 3.5 ms [3.4-4.4] | Pose: 0.0 ms | Total E2E: 23.8 ms [23.6-24.9]
```

## Key data structures

**`SensorSpec`** (`camera.h`) — per-sensor constants: resolution, black level, camera intrinsics (fx, fy, cx, cy, distortion), V4L2 timing (vblank_min, hblank_min, pixel_clock_hz), and gain register mapping. Defined in `SENSORS[]` at the top of `vision.cpp`. Calibrate intrinsics per lens — defaults are placeholders.

**`Frame`** — gray pixel buffer (uint8, width×height), plus three timestamps: `ts_camera` (center-of-exposure), `ts_captured` (after unpack done), `ts_dequeued` (when detection thread received it).

**`PIMutex`** (`camera.h`) — thin RAII wrapper around `pthread_mutex_t` with `PTHREAD_PRIO_INHERIT`. Used for `log_mtx` and inside `LatestFrame`. Required with PREEMPT_RT to prevent the normal-priority logging thread from blocking the SCHED_FIFO detection thread via priority inversion.

**`RingBuffer<T, N>`** (`vision.cpp`) — lock-free SPSC ring buffer. Two instances: `log_buffer` (timing, 1024 entries) and `detect_log_buffer` (detect events, 256 entries).

## RT / latency configuration

The system is tuned for deterministic low-latency operation. These are not defaults — they were deliberately configured:

### CPU isolation (`/boot/firmware/cmdline.txt`)
```
isolcpus=1,2,3 nohz_full=1,2,3
```
Cores 1, 2, 3 are removed from the kernel scheduler's housekeeping. Core 0 handles OS + main/logging threads + capture thread (capture is SCHED_FIFO 10 and preempts anything on core 0 when a frame arrives).

### CPU affinity (`vision.cpp` main)
- **Capture thread**: pinned to core 0, SCHED_FIFO priority 10
- **Detection thread**: pinned to `read_isolated_cpus() - {core 0}` = {1,2,3}, SCHED_FIFO priority 5
- **apriltag worker threads**: inherit the detection thread's affinity mask at creation — this is why `nthreads=3` (3 workers on 3 isolated cores) beats `nthreads=2`. Do **not** pin the detection thread to a single core or workers will all pile onto it.

### `run.sh` unicam IRQ boost
The unicam kernel IRQ thread (fires at CSI-2 frame-end) is boosted to SCHED_FIFO 49 — higher than the capture thread (10) so the wakeup is immediate.

### CPU governor
`/etc/rc.local` sets all cores to `performance` at boot. Without this, the `ondemand` governor scales the clock down between detection cycles, causing bimodal Detect latency (~4 ms vs ~8 ms).

### Kernel
Running `linux-image-rpi-v8-rt` (Raspberry Pi Foundation's PREEMPT_RT kernel). **Do not use `linux-image-rt-arm64`** (Debian upstream) — it lacks the unicam/CSI-2 drivers. The RT kernel reduced worst-case Detect latency from ~14 ms to ~5 ms by eliminating non-preemptible kernel sections.

### `mlockall` + stack prefault
`main()` calls `mlockall(MCL_CURRENT | MCL_FUTURE)` to prevent page faults during RT operation. Each RT thread (`capture_thread`, `detection_thread`) calls `prefault_stack()` at startup to touch 256 KB of stack pages, wiring them into RAM before the hot path starts.

## Latency budget (OV9281, 60 fps, quad_decimate=2.0, nthreads=3)

| Stage | Avg | [min–max] | Notes |
|-------|-----|-----------|-------|
| Capture | 20.3 ms | [20.1–20.9] | Dominated by ts_correction (8.69 ms) + DMA memcpy + NEON unpack |
| Queue | 0.05 ms | [0.03–0.10] | Condition variable handoff; previously ~1 ms with sleep-poll |
| Detect | 3.5 ms | [3.4–4.4] | quad_decimate=2.0; 3 workers on 3 isolated cores |
| Pose | <0.001 ms | | IPPE_SQUARE closed-form solver |
| **Total E2E** | **23.8 ms** | **[23.6–25.1]** | From center-of-exposure to pose complete |

The "Capture" value includes the 8.69 ms timestamp correction offset — actual processing time after DQBUF is ~11.6 ms (memcpy + unpack).

## Sensor table — what to verify when adding a sensor

Use `v4l2-ctl --list-ctrls` on target hardware to verify:
- `pixel_rate` → `pixel_clock_hz`
- `hblank` min value → `hblank_min` (IMX296's hblank is read-only)
- `vblank` min value → `vblank_min`
- Gain register range and whether it's linear or log (IMX296 uses 0.1 dB steps)

## Camera calibration

Intrinsics in `SENSORS[]` are placeholders. Calibrate with a checkerboard using `cv::calibrateCamera` and update `fx, fy, cx, cy, dist_k1..k3, dist_p1, dist_p2` per lens. `TAG_SIZE_M` (0.165 m) is the FRC standard tag side length — measure the physical printed tag, not the spec.
