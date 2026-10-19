<a id="top"></a>
<p align="right">
  <img src="../../assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# 00 · Specifications — Hardware — 05 Camera

<!-- nav:auto -->
<p align="left">
  <a href="../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="04_gripper.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="06_pinouts.md"><b>Forward &#8594;</b></a>
</p>



<p align="center">
  <img src="../../assets/hardware/camera_product.jpg" width="420" alt="myCobot USB camera module v2.0">
  <br><sub>The myCobot camera module v2.0.</sub>
</p>

The camera is the myCobot camera module v2.0 (`myCobot_cameraHolder_J6`): a USB 2.0 webcam on a LEGO
bracket. It mounts on the wrist (eye-in-hand) or on a fixed bracket.

---

## 1. Specification

| Item | Value |
| :--- | :--- |
| Model | myCobot camera module v2.0 (`myCobot_cameraHolder_J6`) |
| Interface | USB 2.0 High / Full Speed (Type-A into the base) |
| Lens | 1.7 mm focal length |
| Field of view | about 60° |
| Body | ABS, white, 83 × 64 × 16 mm |
| Mounting | LEGO connector |
| Lifespan | two years |

---

## 2. Device on the unit

| Item | Value |
| :--- | :--- |
| Video device | Realtek `0bda:5a63`, USB 2.0 UVC at `/dev/video0`; card "USB 2.0 Camera", serial `20220523` |
| Media node | `/dev/media1` (camera), `/dev/media0` (Tegra VI) |
| Audio | built-in microphone (USB audio interface) |
| MJPG | 30 fps up to 2592×1944 (**preferred**) |
| YUYV | 3 fps up to 2592×1944 |

---

## 3. Package contents

<p align="center">
  <img src="../../assets/hardware/camera_package.jpg" width="420" alt="Camera package contents">
  <br><sub>The camera module and its cable.</sub>
</p>

---

## 4. Fitting

**Mechanical**

1. Slot the LEGO connector of the `myCobot_cameraHolder_J6` bracket onto the tool flange at J6
   (eye-in-hand), or onto the fixed bracket.
2. Press it on until the pin seats.

**Electrical**

1. Route the camera's USB cable along the arm, clear of the joints.
2. Plug the Type-A connector into a USB port on the base.

<p align="center">
  <img src="../../assets/hardware/camera_lego_mount.jpg" width="380" alt="LEGO mount">
  <img src="../../assets/hardware/camera_flange_mount.jpg" width="380" alt="Flange mount">
  <br><sub>Left: the LEGO mount. Right: mounted on the arm's flange.</sub>
</p>


<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
