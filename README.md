# AprilAssay

![AprilAssay logo](assets/logo.png)

AprilAssay characterizes camera performance for AprilTag detection in the context of FRC robotics. It runs a live vision pipeline on a Raspberry Pi 4, capturing frames via V4L2, detecting AprilTag 36h11 tags, estimating 6-DOF pose, and logging per-stage latency to measure end-to-end pipeline performance.

## What it measures

Each pipeline stage is timed independently and averaged over a 2-second reporting interval:

| Stage | Description |
|---|---|
| Capture | YUYV → grayscale extraction and frame push |
| Queue | Wait time between capture and detection |
| Detect | AprilTag detection (`apriltag_detector_detect`) |
| Pose | 6-DOF pose estimation (`estimate_tag_pose`) |
| Total E2E | Camera dequeue to pose complete |

Output is logged to a timestamped file (`vision_YYYYMMDD_HHMMSS.log`) or stdout in debug mode.

## Hardware

- Raspberry Pi 4
- USB camera with V4L2 support (tested at 640×480 YUYV 30fps)

## Dependencies

- [AprilTag](https://github.com/AprilRobotics/apriltag)
- CMake ≥ 3.10
- C++17 compiler

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
./build/vision
```

Stop with `Ctrl-C`.

## Configuration

Camera intrinsics and tag size are defined as constants at the top of [src/vision.cpp](src/vision.cpp):

```cpp
static const double CAM_FX     = 600.0;   // focal length x (pixels)
static const double CAM_FY     = 600.0;   // focal length y (pixels)
static const double CAM_CX     = 320.0;   // principal point x
static const double CAM_CY     = 240.0;   // principal point y
static const double TAG_SIZE_M = 0.165;   // tag size in meters (FRC standard)
```

## Camera sensor parameters

Set via V4L2 controls in `capture_thread()` in [src/vision.cpp](src/vision.cpp):

| Parameter | Current value | Effect |
|---|---|---|
| Resolution | 640×480 | Higher resolution extends detection range but increases capture and detection time. Must match `CAM_WIDTH`/`CAM_HEIGHT` constants. |
| Frame rate | 30 fps | Higher frame rates reduce queue latency but require shorter exposure times, which increases noise. |
| Exposure mode | Manual | Locks exposure so lighting changes don't alter detection reliability between runs. |
| Exposure time | 333 (33.3 ms) | In units of 100 µs. Shorter exposure reduces motion blur on fast-moving robots; too short increases noise and darkens the image. |
| Gain | not set | Amplifies the sensor signal. Higher gain brightens dark scenes but increases noise, which hurts quad detection. |
| Brightness | not set | Offset applied to pixel values. Can compensate for underexposed images, but gain and exposure are preferred. |
| Backlight compensation | 0 (disabled) | Prevents the camera from brightening the image to compensate for bright backgrounds, which can wash out tags. |
| Sharpness | 15 | In-camera edge enhancement. Moderate sharpness helps quad edge detection; too high introduces ringing artifacts. |
| Auto white balance | 0 (disabled) | Locks color processing so the grayscale conversion is consistent across lighting conditions. |

## Camera intrinsic parameters

| Parameter | Description | Characterization notes |
|---|---|---|
| `CAM_FX` / `CAM_FY` | Focal length in pixels (x and y). Derived from physical focal length and pixel size. | Errors scale all translation estimates. Must be calibrated per lens — do not assume equal for non-square pixels. |
| `CAM_CX` / `CAM_CY` | Principal point: pixel coordinates of the optical axis. Ideally image center, but rarely exactly so. | Errors cause systematic pose bias that worsens off-axis. Calibrate rather than assume `width/2`, `height/2`. |
| `TAG_SIZE_M` | Physical side length of the tag in meters. FRC standard is 0.165 m. | Directly scales the translation vector. A 1% size error produces a 1% range error. Measure the printed tag, not the spec. |
| Distortion coefficients | Radial (`k1`, `k2`, `k3`) and tangential (`p1`, `p2`) lens distortion. Not currently modeled. | Uncorrected distortion degrades pose accuracy at the edges of the frame. Most impactful with wide-angle lenses. |

## AprilTag detector parameters

Set in `detection_thread()` in [src/vision.cpp](src/vision.cpp):

```cpp
td->quad_decimate = 2.0;
td->nthreads      = 2;
```

| Parameter | Value | Effect |
|---|---|---|
| `quad_decimate` | 2.0 | Downsamples the image by this factor before quad detection. Higher values are faster but reduce detection range and accuracy on small or distant tags. 1.0 disables decimation. |
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
