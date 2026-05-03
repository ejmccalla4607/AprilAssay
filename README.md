# AprilAssay

<table><tr>
<td><img src="assets/logo.png" width="400" alt="AprilAssay logo"></td>
<td valign="top">AprilAssay characterizes camera sensor performance for AprilTag detection in the context of FRC robotics. It runs a live vision pipeline on a Raspberry Pi 4 or 5, capturing frames directly via V4L2, detecting AprilTag 36h11 tags, estimating 6-DOF pose, and producing two log streams: a per-frame telemetry CSV for sensor characterization and a human-readable debug log for pipeline diagnostics.</td>
</tr></table>

## What it produces

### Debug log (`<base>_debug.log` or stdout)

Human-readable pipeline diagnostics. A two-line summary is printed once at program exit (Ctrl-C), accumulating stats over the entire run:

```
Duration: 60.1 s | Cam frames: 3607 | Det frames: 3601
Cam FPS: 60.0 | Det FPS: 59.9 | Det Rate: 0.97 | Capture: 20.3 ms [20.1-20.9] | Queue: 0.05 ms [0.03-0.10] | Detect: 3.5 ms [3.4-4.4] | Pose: 0.0 ms [0.0-0.0] | Total E2E: 23.8 ms [23.6-24.9]
```

Each pipeline stage is timed independently:

| Stage | Description |
|---|---|
| Capture | DMA copy + NEON unpack from 10-bit packed to 8-bit gray; includes `ts_correction` offset to center-of-exposure |
| Queue | Wait time between capture complete and detection thread pickup |
| Detect | AprilTag detection (`apriltag_detector_detect`) |
| Pose | 6-DOF pose estimation (`cv::solvePnP`) |
| Total E2E | Center-of-exposure to pose complete |

### Telemetry CSV (`<base>_telemetry.csv`)

One row per captured frame. Written only when `--log` is specified. Contains the full per-frame measurement set defined in the characterization plan (Section 9, Tables 6–7):

**Session inputs** (set at startup via CLI or default 0):
`timestamp`, `frame_number`, `sensor`, `exposure_us`, `gain_db`, `gain_linear`, `condition_id`, `gantry_x/y/z_level`, `lux_tag`, `light_supply_current_ma`, `shadow_coverage_pct`, `shadow_depth_ratio`

**Per-frame outputs** (computed from the detected tag):
`detected`, `tag_id`, `decision_margin`, `pixel_count_across_tag`,
`white_mean/median/std/min/max_dn`, `black_mean/median/std/min/max_dn`,
`contrast_ratio`, `saturated`, `glare_region_dn`,
`pose_x_m`, `pose_y_m`, `pose_theta_rad`, `pose_error_mm`

White and black DN statistics come from pixel bands sampled just outside and just inside the tag corner polygon respectively — the same regions the AprilTag detector uses to distinguish foreground from background.

## Hardware

- Raspberry Pi 4 (V4L2/unicam) or Raspberry Pi 5 (V4L2/rp1-cfe)
- One of the following on CSI-2:
  - **OV9281** — global shutter monochrome, 1280×800, 10-bit
  - **IMX296** — global shutter monochrome, 1456×1088, 10-bit

Both sensors output raw 10-bit CSI-2 packed data (`Y10P`). Frames bypass the ISP entirely — no tone mapping, no noise reduction, no compression.

## Dependencies

- [AprilTag](https://github.com/AprilRobotics/apriltag)
- OpenCV 4 (`core`, `calib3d`, `imgcodecs`, `imgproc`)
- CMake ≥ 3.10
- C++17 compiler
- Linux kernel V4L2 headers (standard on Raspberry Pi OS)

Run the setup script once to install dependencies and build from scratch:

```bash
./setup.sh
```

## Building

```bash
./rebuild.sh
```

`rebuild.sh` auto-detects the target platform. Or manually:

```bash
# Pi 4
cmake -S . -B build && cmake --build build -j$(nproc)

# Pi 5
cmake -S . -B build -DRPI5=ON && cmake --build build -j$(nproc)
```

## Running

**Always use `run.sh`** — it boosts the camera IRQ kernel thread to SCHED_FIFO 49 before exec'ing the binary (auto-detects Pi 4 unicam or Pi 5 rp1-cfe). Running the binary directly causes CSI-2 wakeup jitter.

```bash
sudo ./run.sh [options]
```

### Core options

| Flag | Default | Description |
|---|---|---|
| `--camera ov9281\|imx296` | `ov9281` | Sensor model |
| `--exposure <µs>` | 333 | Exposure time in microseconds |
| `--gain <x>` | 1.0 | Analogue gain multiplier |
| `--fps <n>` | 60 | Target frame rate; clamped to sensor maximum |
| `--nthreads <n>` | 3 | AprilTag detector worker threads |
| `--quad-decimate <f>` | 1.0 | Downscale factor before quad detection; 1.0 = full resolution |
| `--snapshot <file>` | — | Capture one frame, save as PNG, and exit |

### Logging options

| Flag | Description |
|---|---|
| *(omit `--log`)* | Debug to stdout; telemetry suppressed |
| `--log` | Debug → `logs/vision_YYYYMMDD_HHMMSS_debug.log`; telemetry → `logs/vision_YYYYMMDD_HHMMSS_telemetry.csv` |
| `--log <base>` | Debug → `<base>_debug.log`; telemetry → `<base>_telemetry.csv` |

The `logs/` directory (or any parent of `<base>`) is created automatically.

### Characterization metadata options

These populate the session-input columns of the telemetry CSV. All default to 0 (unknown/N/A).

| Flag | Description |
|---|---|
| `--condition <1\|2\|3>` | Test case ID |
| `--gantry-x/y/z <1-3>` | Gantry position levels |
| `--lux <f>` | Lux measured at tag surface |
| `--supply-ma <f>` | COB LED supply current (mA) |
| `--shadow-coverage <f>` | Shadow coverage % (Test Case 2) |
| `--shadow-depth <f>` | Shadow depth ratio (Test Case 2) |

### Examples

```bash
# Development — debug to stdout, no telemetry
sudo ./run.sh --camera ov9281 --fps 60 --quad-decimate 2.0
sudo ./run.sh --camera imx296 --fps 60 --quad-decimate 2.0

# Single-point characterization run
sudo ./run.sh --camera ov9281 --fps 60 --exposure 400 --gain 4.0 --log ./logs/tc1_x2y2z1 --condition 1 --gantry-x 2 --gantry-y 2 --gantry-z 1 --lux 500 --supply-ma 1200
sudo ./run.sh --camera imx296 --fps 60 --exposure 400 --gain 4.0 --log ./logs/tc1_x2y2z1 --condition 1 --gantry-x 2 --gantry-y 2 --gantry-z 1 --lux 500 --supply-ma 1200

# Snapshot
sudo ./run.sh --camera ov9281 --snapshot frame.png
sudo ./run.sh --camera imx296 --snapshot frame.png
```

Stop with `Ctrl-C`.

## Configuration

Per-sensor parameters are defined in the `SENSORS[]` table at the top of [src/vision.cpp](src/vision.cpp). Each entry holds the sensor's resolution, black level, camera intrinsics, and V4L2 timing parameters:

```cpp
const SensorSpec SENSORS[2] = {
    {
        "OV9281",
        1280, 800,
        64,     // 10-bit black level
        600.0, 600.0, 640.0, 400.0,   // fx, fy, cx, cy — calibrate per lens
        0.0, 0.0, 0.0, 0.0, 0.0,      // k1, k2, p1, p2, k3
        11,             // vblank_min
        424,            // hblank_min  (hts = 1704)
        160'000'000LL,  // pixel_clock_hz
        16.0f, 16, 248, // gain_reg_per_unit, min, max
    },
    ...
};
```

Tag size is a separate constant (it's a property of the field, not the camera):

```cpp
static const double TAG_SIZE_M = 0.165;  // FRC standard tag, meters
```

## Pixel format

Both sensors are native 10-bit monochrome. Frames are captured as `Y10P` — CSI-2 packed 10-bit, 4 pixels per 5 bytes:

```
byte 0: pixel 0 [9:2]
byte 1: pixel 1 [9:2]
byte 2: pixel 2 [9:2]
byte 3: pixel 3 [9:2]
byte 4: [p3[1:0] | p2[1:0] | p1[1:0] | p0[1:0]]
```

`unpack_to_u8()` in [src/camera.h](src/camera.h) converts this to 8-bit for the AprilTag detector: the 10-bit sample is unpacked, the sensor black level is subtracted, the result is clamped to `[0, 1023 - black_level]`, then scaled to `[0, 255]`.

The DMA buffer is first copied to a heap-allocated staging buffer before unpacking. V4L2 DMA mappings on ARM are typically uncached; reading them byte-by-byte in the unpack loop would be very slow.

## Camera sensor parameters

Set via V4L2 subdev controls in `capture_thread()` in [src/capture_v4l2.cpp](src/capture_v4l2.cpp):

| Parameter | Control | Effect |
|---|---|---|
| Frame rate | `V4L2_CID_VBLANK` | Computed from `--fps` and pixel clock. Higher rates reduce queue latency but require shorter exposure to avoid sensor-extended frame times. |
| Horizontal blanking | `V4L2_CID_HBLANK` | Set to `hblank_min` from the sensor table. Determines horizontal total (`hts = width + hblank`), which feeds the fps calculation. |
| Exposure | `V4L2_CID_EXPOSURE` | In sensor lines. Converted from `--exposure` µs using `line_period = hts / pixel_clock`. Shorter exposure reduces motion blur; too short darkens the image and hurts quad detection. |
| Analogue gain | `V4L2_CID_ANALOGUE_GAIN` | Register value = `gain × gain_reg_per_unit`, clamped to `[gain_reg_min, gain_reg_max]`. Higher gain brightens dark scenes but increases noise. |

Both sensors are monochrome — AWB is not applicable.

## Camera intrinsic parameters

| Parameter | Description | Characterization notes |
|---|---|---|
| `fx` / `fy` | Focal length in pixels (x and y). Derived from physical focal length and pixel size. | Errors scale all translation estimates. Must be calibrated per lens — do not assume equal for non-square pixels. |
| `cx` / `cy` | Principal point: pixel coordinates of the optical axis. Ideally image center, but rarely exactly so. | Errors cause systematic pose bias that worsens off-axis. Calibrate rather than assume `width/2`, `height/2`. |
| `TAG_SIZE_M` | Physical side length of the tag in meters. FRC standard is 0.165 m. | Directly scales the translation vector. A 1% size error produces a 1% range error. Measure the printed tag, not the spec. |
| Distortion coefficients | Radial (`k1`, `k2`, `k3`) and tangential (`p1`, `p2`) lens distortion. Currently zero. | Uncorrected distortion degrades pose accuracy at the edges of the frame. Most impactful with wide-angle lenses. |

## AprilTag detector parameters

Set in `detection_thread()` in [src/vision.cpp](src/vision.cpp):

```cpp
td->quad_decimate = 1.0;
td->nthreads      = 3;
```

| Parameter | Value | Effect |
|---|---|---|
| `quad_decimate` | 1.0 | Downsamples the image by this factor before quad detection. Higher values are faster but reduce detection range and accuracy on small or distant tags. 1.0 disables decimation. |
| `nthreads` | 3 | Number of threads the detector uses internally. Set to match the 3 isolated cores (1, 2, 3). |

Other detector fields worth studying for characterization:

| Parameter | Default | Effect |
|---|---|---|
| `quad_sigma` | 0.0 | Gaussian blur (σ) applied before quad detection. Can help with noisy sensors; too high blurs tag edges. |
| `refine_edges` | 1 | Sub-pixel edge refinement on detected quads. Improves pose accuracy at a small compute cost. |
| `decode_sharpening` | 0.25 | Sharpening applied to tag interior before bit decoding. Higher values help low-contrast or blurry tags. |

Quad threshold parameters (`td->qtp`) control how pixel clusters are filtered into candidate quads:

| Parameter | Default | Effect |
|---|---|---|
| `min_cluster_pixels` | 5 | Minimum pixels in an edge cluster to be considered a quad side. Raise to filter noise; lower to catch small tags. |
| `max_nmaxima` | 10 | Maximum corner candidates evaluated per cluster. Higher improves accuracy at a compute cost. |
| `critical_rad` | ~10° | Lines forming an angle below this are treated as parallel and rejected. Affects which quad shapes are accepted. |
| `max_line_fit_mse` | 10.0 | Maximum mean squared error for a line fit. Lower values require cleaner edges and reject noisier detections. |
| `min_white_black_diff` | 5 | Minimum pixel contrast between light and dark tag regions. Lower detects faint tags; higher rejects false positives. |
| `deglitch` | 0 | Removes isolated pixels before clustering. Can help with sensor noise at a small compute cost. |
