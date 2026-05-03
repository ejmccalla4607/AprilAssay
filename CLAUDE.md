# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

AprilAssay characterizes camera sensor performance for FRC AprilTag detection. It has two interlocking goals:

1. **Latency pipeline** (current code): runs on a Raspberry Pi 5 with an OV9281 or IMX296 global-shutter CSI-2 camera, captures raw 10-bit frames via V4L2/rp1-cfe (bypassing the ISP entirely), detects AprilTag 36h11 tags, estimates 6-DOF pose, and logs per-stage latency. The primary goal is minimizing and measuring end-to-end latency from center-of-exposure to pose output.

2. **Sensor characterization** (planned, not yet implemented): a systematic sweep that quantifies how detection reliability degrades under the lighting failure modes documented at real FRC competitions — uneven venue spotlights (GCR 2025), sun glare through windows (Gotham 2025), robot-body shadowing (Team 6328 2025), and near-darkness venues (PCH District Championship 2023). The characterization compares OV9281 vs IMX296 across the full (exposure × gain) space under controlled illumination, shadow, and specular glare conditions, producing the first published quantitative data on which sensor holds up better and why. See `docs/AptilTag Sensor Characterization Plan.docx` for the full protocol.

## Characterization plan summary

The characterization targets three test cases (see `docs/` for full detail):

**Test Case 1 — Full sweep (primary)**: 54 manual fixture positions (3X × 3Y × 3Z × 2 illumination levels: ~500 lux "Well-Lit" and ~50 lux "Poorly-Lit"). At each position, automated inner loop over 40 (gain × exposure) combinations — gain at 0/6/12/18/24 dB, exposure at 25/50/100/200/400/800/1600/3200 µs (log-spaced). 300 frames per point. Output: detection rate, mean/std decision_margin, white/black patch DN, saturation flag.

**Test Case 2 — Shadow sweep**: tag partially or fully occluded (25/50/75/100% coverage) at shadow depths of 2:1, 5:1, 10:1 lux ratio. Camera exposed for the lit scene and locked; measures at what occlusion/depth combination each sensor loses the tag.

**Test Case 3 — Specular glare**: tag on standard AndyMark polycarbonate panel, focused COB light creates hotspot. Glare intensity swept to 25/50/75/100% of 8-bit saturation. Tests whether IMX296's larger full-well tolerates more glare before forcing a conservative exposure that darkens the rest of the tag below the detection floor.

**Why IMX296 is expected to win**: 10 dB more dynamic range (~70 dB vs ~60 dB), lower read noise (~2–3e vs ~4–7e), larger full well (~10,000e vs ~6,000–7,000e). At high gain (18–24 dB), read noise is amplified — OV9281's higher noise floor produces 2–3× more noise contamination than IMX296 at the same gain setting, narrowing the detection window faster from both ends.

**Platform note**: the codebase runs Pi 5 + V4L2/rp1-cfe with Y10P packed format. Do not add a libcamera dependency. Build with `-DRPI5=ON` (handled automatically by `setup.sh` and `rebuild.sh` on Pi 5 hardware).

**Key metric for FRC teams**: detection window width (range of exposures with >95% detection rate) at each gain level. A wider window means the sensor tolerates more venue-to-venue illumination variation without retuning. The characterization also validates a field workflow: measure lux at each tag with a $30 meter during calibration hour, look up the matching pre-characterized exposure/gain profile, load it — no subjective trial-and-error.

## Build and run

```bash
# First-time setup (installs deps, builds apriltag, configures isolcpus, CPU governor)
./setup.sh

# Build (Pi 5 — RPI5 flag set automatically by setup.sh/rebuild.sh)
cmake -DRPI5=ON -S . -B build && cmake --build build -j$(nproc)

# Run — debug to stdout, telemetry suppressed
sudo ./run.sh --camera imx296 --exposure 10 --gain 4 --fps 60 --quad-decimate 1.0

# Run with logging
sudo ./run.sh --camera imx296 --exposure 10 --gain 4 --fps 60 --quad-decimate 1.0 --log

# Snapshot mode — capture one frame to PNG and exit
sudo ./run.sh --camera imx296 --exposure 10 --gain 4 --snapshot frame.png

# Gain×exposure sweep (TC1 inner loop)
sudo ./sweep.sh imx296 5
python3 scripts/sweep_summary.py logs/sweep_YYYYMMDD_HHMMSS_telemetry.csv
```

**`--exposure` takes milliseconds** (e.g. `--exposure 1.5` = 1500 µs). Internally stored as `g_exposure_us` (µs).

**Always use `run.sh` instead of `./build/vision` directly.** `run.sh` boosts the rp1-cfe kernel IRQ thread to SCHED_FIFO 49 before exec'ing the binary. Without this, the CSI-2 interrupt that wakes the capture thread runs at normal priority and adds wake-up jitter.

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

Intrinsics in `SENSORS[]` are placeholders. The full calibration procedure is in Appendix B of `docs/AptilTag Sensor Characterization Plan.docx`. Summary:

- Use a ChArUco board (6×8, 30mm squares, DICT_4X4_50) — not a plain checkerboard. ChArUco works with partial board views, important for wide-angle lenses.
- Focus at 8–10 ft on a high-contrast target before calibrating. Do not focus at desk range.
- Collect 30–50 images with varied distance, tilt, and frame position. Frontal-only images produce poor principal-point estimates.
- Use `cv::aruco::calibrateCameraCharuco`. Target RMS reprojection error < 0.5 px; > 1.0 px means recollect.
- Update `fx, fy, cx, cy, dist_k1..k3, dist_p1, dist_p2` in `SENSORS[]` per lens configuration. Each lens + focus position requires its own calibration — do not reuse across configurations.
- `TAG_SIZE_M` (0.165 m) is the FRC standard tag side length — measure the physical printed tag, not the spec. A 1% size error produces a 1% range error.
- Wide-angle OV9281 configs (2.1mm, ~82° HFOV) have significant barrel distortion; calibration is especially critical and distortion coefficients will be non-trivial.
