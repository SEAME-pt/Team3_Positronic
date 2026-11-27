# Evidence: the real wrist camera publishes like the Gazebo one (#138)

## Verdict

PASS. On 2026-11-26, on the real arm, the wrist camera published calibrated images and
`camera_info` on the Gazebo topics and frame (`camera_link`) at 30 fps, and the arm's control
loop held 30 Hz with the camera running (0 late cycles).

| Criterion (#138) | Result | Evidence |
|---|---|---|
| Camera identified: model, interface, resolutions and rates | PASS | [Camera](#camera) |
| Driver in the Jetson image, `make nano-image` builds it | PASS | [Image](#jetson-image) |
| `camera.launch.py`: arguments, `frame_id`, Gazebo namespace; off unless `camera:=true` | PASS | [Launch](#launch), `test_launch_files.py` |
| Calibration in `config/`, published on `camera_info`, reprojection error | PASS | [Calibration](#calibration) |
| Real next to Gazebo at all-zero and joint 6 = +90 deg, TF tree, rates, CPU | PASS | [On the arm](#on-the-arm) |
| `/joint_states` keeps 30 Hz with the camera publishing (2 min, #39 method) | PASS | [No regression](#no-regression) |

## Camera

Interface: **USB (UVC)**, so the USB estimate of the issue holds; no GStreamer or NVIDIA camera
driver is needed. Datasheet values in `docs/00_specs/hardware/05_camera.md`.

On the Jetson host (`sudo apt install v4l-utils` if `v4l2-ctl` is missing):

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

| Item | Value |
|---|---|
| Model | USB 2.0 Camera (`usb-70090000.xusb-2.1`), generic UVC |
| Interface | USB 2.0 UVC, `/dev/video0` |
| Formats | MJPG: 640×480, 320×240, 800×600, 1280×720 … up to 2592×1944, all at 30/25 fps; YUYV: 640×480 at 30/25, larger sizes at lower rates |
| Used | MJPG 640 × 480 at 30 fps, turned 90 deg counterclockwise: **480 × 640** published |

<details><summary>v4l2-ctl output</summary>

```text
$ v4l2-ctl --list-devices
USB 2.0 Camera (usb-70090000.xusb-2.1):
	/dev/video0
	/dev/media1

$ v4l2-ctl -d /dev/video0 --list-formats-ext
MJPG: 640x480, 320x240, 352x288, 640x360, 800x600, 1024x768, 1280x720,
      1280x960, 1920x1080, 1600x1200, 2048x1536, 2560x1440, 2592x1944 @ 30/25 fps
YUYV: 640x480, 320x240, 352x288, 640x360 @ 30/25 fps; larger sizes at 20/15/10/5/3 fps
```

</details>

## Jetson image

`docker/Dockerfile`, stage `core`: `opencv-python-headless` (pip, V4L2 and MJPG decoding) and
`ros-jazzy-image-transport-plugins`. `make nano-image` on 2026-11-26: OK.

```bash
make nano-image
ssh nano docker run --rm positronic-runtime:nano python3 -c "import cv2; print(cv2.__version__)"
```

Printed: `4.10.0`.

Why a node of ours and not `v4l2_camera` or `usb_cam`: #97 found that the raw image lies on its
side and decided the camera node turns every frame 90 deg counterclockwise before publishing,
with the calibration made on the turned image. Neither driver can turn a frame, and a separate
rotate node would cost a second copy of every image on the Jetson.

## Launch

Off by default: the camera starts only when asked, with the driver or alone.

```bash
make nano-build
make nano-shell                                                # maps /dev/video0 if plugged
ros2 launch positronic_bringup driver.launch.py camera:=true   # driver + camera
# or, alone, next to the driver's container:
make nano-camera
ros2 launch positronic_bringup camera.launch.py
```

`driver.launch.py` without `camera:=true` starts no camera: `ros2 node list` shows no
`/wrist_camera/wrist_camera`. <checked on the arm: yes/no>

| Topic | Type | Rate on the Jetson |
|---|---|---|
| `/wrist_camera/image_raw` | `sensor_msgs/Image`, bgr8, 480 × 640 | 30.0 fps (2.4 ms/frame, grab→publish) |
| `/wrist_camera/camera_info` | `sensor_msgs/CameraInfo` | same as the image |
| `/wrist_camera/image_raw/compressed` | `sensor_msgs/CompressedImage`, jpeg | ~30.0 fps (laptop over Wi-Fi, `ros2 topic hz`) |

30 fps needs `buffer_size >= 2` (default 4). With `buffer_size:=1` the camera drops every other
frame and the rate halves to ~15; the parameter trades frame rate for latency.

`frame_id` is `camera_link` on the three. Same topics in Gazebo (`gazebo.launch.py` bridge):

```bash
ros2 topic list | grep wrist_camera                         # real arm and Gazebo
ros2 topic echo --once /wrist_camera/camera_info --field header.frame_id
```

| | Real arm | Gazebo |
|---|---|---|
| `header.frame_id` | `camera_link` | `camera_link` (sensor reference link, checked by `test_launch_files.py`) |
| Image size | 480 × 640 | 640 × 480 |

## Calibration

Checkerboard: **8×5 inner corners** (9×6 squares), squares of **0.037 m**, printed flat on a
rigid board. The arm stands still; the board moves in front of the camera. (The square size sets
only the metric scale; it does not change the pixel intrinsics below or the reprojection error —
so the capture run that passed `--square 0.024` still produced the correct K.)

`camera.launch.py` running on the Jetson. Over Wi-Fi, republish the compressed image on the
laptop first; over a cable, point the calibrator straight at `/wrist_camera/image_raw`.

```bash
# Wi-Fi only: decode the compressed stream to a local raw topic (leave it running).
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw \
  -r in/compressed:=/wrist_camera/image_raw/compressed -r out:=/wrist_camera_local/image_raw

# --no-service-check: our node is a plain publisher, it has no set_camera_info service to save back.
ros2 run camera_calibration cameracalibrator --size 8x5 --square 0.037 --no-service-check \
  --ros-args -r image:=/wrist_camera_local/image_raw     # cable: image:=/wrist_camera/image_raw
```

Move the board until X, Y, Size and Skew are green, CALIBRATE, then SAVE. SAVE writes
`/tmp/calibrationdata.tar.gz` **inside the calibrator's container**, so extract and copy it into
the repo (`/ws`, which persists) before closing that container:

```bash
mkdir -p /tmp/calib && tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calib
cp /tmp/calib/ost.yaml src/positronic_bringup/config/wrist_camera.yaml
sed -i 's/^camera_name:.*/camera_name: wrist_camera/' src/positronic_bringup/config/wrist_camera.yaml
python3 docs/03_verification/camera_real/tools/reprojection_error.py \
  /tmp/calibrationdata.tar.gz --size 8x5 --square 0.037
```

| | Value |
|---|---|
| Views | 60 of 60 |
| Reprojection RMS | **0.800 px** (worst view `left-0034.png`, 3.221 px) |
| fx, fy | 446.63, 446.75 |
| cx, cy | 238.62, 318.42 (image 480 × 640) |
| Distortion (plumb_bob) | 0.0551, −0.0675, −0.0022, 0.0036, 0.0000 |

A handful of views (3.2, 2.9, 2.2 px) pull the RMS up and are motion-blurred captures; the rest
are 0.1–0.5 px. 0.8 px is usable; a slower second pass would bring it under ~0.5 if module 2
needs it.

After `make nano-build` the camera node logs `calibration .../wrist_camera.yaml loaded
(480 x 640)`, and `camera_info` carries the K above:

```bash
ros2 topic echo --once /wrist_camera/camera_info --field k
# [446.63161, 0, 238.61966, 0, 446.74629, 318.42361, 0, 0, 1]
```

## On the arm

Safety: joint 6 moves once to +90 deg and back, at speed 30. Bench clear, one person with a
hand on the plug, a second at the laptop. Driver stopped first: only one process holds the
serial port. Commands as in `docs/03_verification/camera_mount` (pymycobot on the Jetson, the
trajectory action in Gazebo).

The real-vs-Gazebo comparison at the all-zero pose and at joint 6 = +90 deg is already in
`docs/03_verification/camera_mount/` (#97): the mount decides the view, and the images there
show the real and Gazebo views matching at those poses. Not duplicated here; the PR explains
why. What #138 adds on top — the real image reaching ROS through this pipeline — is covered by
the topics, the `camera_link` TF and the calibration above.

TF: the frames are published by `robot_state_publisher` in the driver, not by the camera, so the
driver must be running (`backend:=fake` is enough, no arm needed). With only `camera.launch.py`
the TF tree is empty.

```bash
ros2 launch positronic_bringup driver.launch.py backend:=fake   # publishes the TF tree
ros2 run tf2_ros tf2_echo joint6_flange camera_link
ros2 run tf2_tools view_frames      # frames_<date>.pdf -> figures/tf_tree.png
```

Confirmed chain: `world → … → joint6 → joint6_flange → camera_flange → camera_link`.
`tf2_echo joint6_flange camera_link`: translation [0, −0.041, 0.018] m, rotation RPY
[−45°, −90°, 135°] (the #97 mount). This text is the evidence; the `view_frames` PDF is
optional. To attach it as a figure, screenshot the PDF or render it where the tool exists:

```bash
pdftocairo -png -r 150 frames_<date>.pdf figures/tf_tree   # or: screenshot the PDF
```

| Rate | Value |
|---|---|
| On the Jetson (node log line, `image_raw`) | 30.0 fps, 2.4 ms/frame |
| On the laptop over Wi-Fi (`ros2 topic hz /wrist_camera/image_raw/compressed`) | <fps> |

CPU of the camera node on the Jetson, driver running (4 cores, 100 % = one core):

```bash
top -b -d 2 -n 30 -p "$(pgrep -f wrist_camera.py | head -n 1)" | awk '/wrist_camera/ {print $9}'
```

| | CPU |
|---|---|
| Nobody on the compressed topic | 23 % of one core (Jetson, 4 cores) |
| A laptop on `image_raw/compressed` | not measured separately; the JPEG encode adds a little |

## No regression

The #39 method (`docs/03_verification/rt_profile/tools`), two runs of 2 minutes with the
driver up, the second with the camera publishing and a laptop on the compressed topic:

```bash
python3 docs/03_verification/rt_profile/tools/rt_record.py --out /ws/log/cam_off --duration 120
python3 docs/03_verification/rt_profile/tools/rt_record.py --out /ws/log/cam_on --duration 120
python3 docs/03_verification/rt_profile/tools/analyze_rt.py \
  /ws/log/cam_off/cycles.csv /ws/log/cam_on/cycles.csv --rate 30
```

| Run | cycles | mean ms | p50 ms | p99 ms | max ms | > 50 ms | > 66.7 ms |
|---|---|---|---|---|---|---|---|
| camera off | 3601 | 33.3 | 33.3 | 34.5 | 39.5 | 0 | 0 |
| camera on | 3601 | 33.3 | 33.3 | 33.4 | 36.6 | 0 | 0 |

Both runs hold 30.00 Hz. With the camera on the cycle period is if anything tighter (p99 33.4 vs
34.5, max 36.6 vs 39.5), and no cycle is late in either run: the camera does not disturb the
control loop.

## Checks run

| Check | Where | Result |
|---|---|---|
| `test_wrist_camera.py`, `test_launch_files.py`, `test_robot_description.py` | `make test` / CI | run by CI on the PR |
| `test_reprojection_error.py` | `python3 -m pytest docs/03_verification/camera_real/tools` | PASS |
| Gazebo camera still on `/wrist_camera/*` | laptop, Gazebo | `<quick sim check>` |
| Real arm: calibration, TF, rates (30 fps), CPU (23 %), 30 Hz | the arm, 2026-11-26 | PASS |