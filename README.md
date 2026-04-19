# AprilAssay

<table><tr>
<td><img src="assets/logo.png" width="400" alt="AprilAssay logo"></td>
<td valign="top">AprilAssay characterizes camera performance for AprilTag detection in the context of FRC robotics. It runs a live vision pipeline on a Raspberry Pi 4, capturing frames directly via V4L2/unicam, detecting AprilTag 36h11 tags, estimating 6-DOF pose, and logging per-stage latency to measure end-to-end pipeline performance.</td>
</tr></table>

## What it measures

Each pipeline stage is timed independently and averaged over a 2-second reporting interval:

| Stage | Description |
|---|---|
| Capture | Unpack from DMA staging buffer to 8-bit gray |
| Queue | Wait time between capture and detection |
| Detect | AprilTag detection (`apriltag_detector_detect`) |
| Pose | 6-DOF pose estimation (`cv::solvePnP`) |
| Total E2E | Dequeue time to pose complete |

Latency stats are logged every 2 seconds to a timestamped file (`vision_YYYYMMDD_HHMMSS.log`) or stdout in debug mode. Per-frame detection results are logged immediately:

```
[DETECT] id=5 x=0.123 y=2.341 theta=0.157
```

`x` is lateral offset (meters, camera right), `y` is depth (meters, forward), `theta` is yaw of the tag about the vertical axis (radians, zero when facing the camera directly).

## Hardware

- Raspberry Pi 4
- One of the following on CSI-2:
  - **OV9281** — global shutter monochrome, 1280×800, 10-bit
  - **IMX296** — global shutter monochrome, 1456×1088, 10-bit

Both sensors output raw 10-bit CSI-2 packed data (`Y10P`). Frames bypass the ISP entirely — no tone mapping, no noise reduction, no compression.

## Dependencies

- [AprilTag](https://github.com/AprilRobotics/apriltag)
- OpenCV 4 (`core`, `calib3d`, `imgcodecs`)
- CMake ≥ 3.10
- C++17 compiler
- Linux kernel V4L2 headers (standard on Raspberry Pi OS)

Run the setup script once to install dependencies and build from scratch:

```bash
./setup.sh
```

## Building

After initial setup, use the rebuild script for subsequent builds:

```bash
# Release build (logs to timestamped file)
./rebuild.sh

# Debug build (logs to stdout)
./rebuild.sh DEBUG=1
```

## Running

```bash
./build/vision [--camera ov9281|imx296] [--exposure <µs>] [--gain <x>] [--fps <n>] [--snapshot <file>]
```

| Flag | Default | Description |
|---|---|---|
| `--camera` | `ov9281` | Sensor model — selects resolution, intrinsics, and timing parameters |
| `--exposure` | 333 µs | Exposure time in microseconds |
| `--gain` | 1.0× | Analogue gain multiplier |
| `--fps` | 60 | Target frame rate; clamped to sensor maximum |
| `--snapshot` | — | Capture one frame, save as PNG, and exit |

Examples:
```bash
./build/vision                                         # OV9281, defaults
./build/vision --camera imx296                         # IMX296, defaults
./build/vision --camera ov9281 --exposure 1000         # OV9281, 1 ms exposure
./build/vision --exposure 500 --gain 2                 # OV9281, 500 µs, 2x gain
./build/vision --fps 100                               # push toward sensor max (~116 fps)
./build/vision --snapshot frame.png                    # save a single frame and exit
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
td->nthreads      = 2;
```

| Parameter | Value | Effect |
|---|---|---|
| `quad_decimate` | 1.0 | Downsamples the image by this factor before quad detection. Higher values are faster but reduce detection range and accuracy on small or distant tags. 1.0 disables decimation. |
| `nthreads` | 2 | Number of threads the detector uses internally. Tune to match available cores without starving the capture thread. |

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
