#!/usr/bin/env python3

# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Real wrist camera issue: the myCobot USB camera (UVC, MJPG 30 fps) must publish on the
#   topics of the Gazebo camera (/wrist_camera/image_raw, /wrist_camera/camera_info), with
#   frame_id = camera_link of the URDF. #97 found the raw image lies on its side and decided
#   the camera node turns every frame 90 deg counterclockwise before publishing, and that the
#   camera is calibrated on the turned images. v4l2_camera and usb_cam cannot turn a frame, so
#   write a small rclpy node with OpenCV: device, size, rate, rotation, frame_id and the
#   calibration yaml (camera_calibration's ost.yaml format) as parameters; image_raw (bgr8),
#   camera_info with every frame, and image_raw/compressed (jpeg, encoded only while someone
#   subscribes, for Wi-Fi). Read the camera in its own thread, reopen it if it fails, log the
#   achieved rate every few seconds, and keep the maths testable without a camera.

"""
Publish the wrist camera of the real arm on the topics of the Gazebo camera.

The camera is the myCobot USB camera (UVC, docs/00_specs/hardware/05_camera.md). The node opens
it with OpenCV (V4L2, MJPG), turns every frame by `rotation` (90 deg counterclockwise by
default: the raw image lies on its side, docs/03_verification/camera_mount) and publishes,
relative to its namespace (camera.launch.py puts it in /wrist_camera, as Gazebo):

  image_raw             sensor_msgs/Image, bgr8, the turned frame
  camera_info           sensor_msgs/CameraInfo, same stamp and frame_id, from calibration_file
  image_raw/compressed  sensor_msgs/CompressedImage, jpeg, encoded only while subscribed

Parameters:
  device            /dev/video0
  width, height     640, 480: the size asked of the camera, BEFORE the rotation. With a 90 deg
                    rotation the published image is height x width (480 x 640)
  fps               30
  rotation          none | cw90 | ccw90 | 180
  frame_id          camera_link, the camera link of positronic_real.urdf.xacro
  calibration_file  camera_calibration yaml (ost.yaml); empty or missing: camera_info carries
                    only the image size, and a warning says the camera is not calibrated
  jpeg_quality      80, of image_raw/compressed
  report_period     10.0 s between two log lines with the achieved rate

The calibration must be made on the published (turned) image, so its size is the published
size. A file of another size is refused with a warning rather than published wrong.
"""

import array
import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import yaml

# What each rotation parameter does to a frame. None: the frame is published as read.
ROTATIONS = {
    'none': None,
    'cw90': cv2.ROTATE_90_CLOCKWISE,
    'ccw90': cv2.ROTATE_90_COUNTERCLOCKWISE,
    '180': cv2.ROTATE_180,
}
# The format string compressed_image_transport writes, so its subscribers (RViz, rqt) decode it.
COMPRESSED_FORMAT = 'bgr8; jpeg compressed bgr8'
# Element count of each matrix of a calibration file. Distortion: any length (plumb_bob has 5,
# rational_polynomial 8).
MATRIX_SIZES = {'camera_matrix': 9, 'rectification_matrix': 9, 'projection_matrix': 12}
# Seconds between two attempts to open the camera, and after a failed read.
RETRY_S = 1.0
# Seconds between two identical warnings from the capture thread.
WARN_PERIOD_S = 5.0


class CalibrationError(ValueError):
    """A calibration file that cannot be used: missing key, wrong matrix size, bad yaml."""


def rotated_size(width, height, rotation):
    """Return (width, height) of a width x height frame after `rotation`."""
    if rotation not in ROTATIONS:
        raise ValueError(f'rotation must be one of {", ".join(ROTATIONS)}, not {rotation!r}')
    if rotation in ('cw90', 'ccw90'):
        return height, width
    return width, height


def rotate(frame, rotation):
    """Return `frame` turned by `rotation`; the same array when the rotation is none."""
    code = ROTATIONS[rotation]
    return frame if code is None else cv2.rotate(frame, code)


def matrix_data(document, key):
    """Return the data list of one matrix of a calibration document, checked for its size."""
    entry = document.get(key)
    if not isinstance(entry, dict) or not isinstance(entry.get('data'), list):
        raise CalibrationError(f'{key}: missing, or without a data list')
    data = [float(value) for value in entry['data']]
    expected = MATRIX_SIZES.get(key)
    if expected is not None and len(data) != expected:
        raise CalibrationError(f'{key}: {len(data)} values, expected {expected}')
    return data


def parse_calibration(text):
    """
    Return the calibration in `text`, the yaml camera_calibration saves (ost.yaml).

    The result is a dict: width, height, camera_name, distortion_model, d, k, r, p.
    Raises CalibrationError when something the CameraInfo needs is missing or malformed.
    """
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise CalibrationError(f'not valid yaml: {error}') from error
    if not isinstance(document, dict):
        raise CalibrationError('not a yaml mapping')
    try:
        width = int(document['image_width'])
        height = int(document['image_height'])
        model = str(document['distortion_model'])
    except (KeyError, TypeError, ValueError) as error:
        raise CalibrationError(f'image_width, image_height or distortion_model: {error}') \
            from error
    if width <= 0 or height <= 0:
        raise CalibrationError(f'image size {width} x {height} is not positive')
    return {
        'width': width,
        'height': height,
        'camera_name': str(document.get('camera_name', '')),
        'distortion_model': model,
        'd': matrix_data(document, 'distortion_coefficients'),
        'k': matrix_data(document, 'camera_matrix'),
        'r': matrix_data(document, 'rectification_matrix'),
        'p': matrix_data(document, 'projection_matrix'),
    }


def load_calibration(path):
    """Return the calibration in the file at `path` (see parse_calibration)."""
    with open(path, encoding='utf-8') as file:
        return parse_calibration(file.read())


def size_mismatch(calibration, width, height):
    """Return why `calibration` does not fit a width x height image, or None if it does."""
    if (calibration['width'], calibration['height']) == (width, height):
        return None
    return (f'calibrated for {calibration["width"]} x {calibration["height"]}, '
            f'the published image is {width} x {height}')


def camera_info(calibration, width, height, frame_id):
    """Return the CameraInfo of the published image; None calibration: the size alone."""
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = width
    info.height = height
    if calibration is not None:
        info.distortion_model = calibration['distortion_model']
        info.d = calibration['d']
        info.k = calibration['k']
        info.r = calibration['r']
        info.p = calibration['p']
    return info


def rate_line(frames, seconds, busy_s):
    """Return the log line of one report: rate, and time from grab to publish per frame."""
    if frames == 0:
        return f'no frame in the last {seconds:.0f} s'
    return (f'{frames / seconds:.1f} fps over {seconds:.0f} s, '
            f'{1000.0 * busy_s / frames:.1f} ms per frame from grab to publish')


def open_v4l2_capture(device, width, height, fps):
    """Open the camera with V4L2 and ask for MJPG at width x height and fps."""
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        return None
    # FOURCC first: the sizes and rates a UVC camera offers depend on the format. MJPG gives
    # 30 fps at every size, YUYV only 3 fps at full size (05_camera.md).
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    # One buffer: read() returns the newest frame, not one queued a few frames ago.
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


class WristCamera(Node):
    """Read the camera in a thread and publish image, camera_info and the jpeg."""

    def __init__(self, open_capture=open_v4l2_capture, **node_options):
        """Declare the parameters, load the calibration and create the publishers."""
        super().__init__('wrist_camera', **node_options)
        self.device = self.declare_parameter('device', '/dev/video0').value
        self.width = self.declare_parameter('width', 640).value
        self.height = self.declare_parameter('height', 480).value
        self.fps = self.declare_parameter('fps', 30).value
        self.rotation = self.declare_parameter('rotation', 'ccw90').value
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.calibration_file = self.declare_parameter('calibration_file', '').value
        self.jpeg_quality = self.declare_parameter('jpeg_quality', 80).value
        report_period = self.declare_parameter('report_period', 10.0).value

        self.open_capture = open_capture
        self.capture = None
        self.stopping = threading.Event()
        self.last_warning = {}  # message -> monotonic time it was last logged
        self.set_image_size(*rotated_size(self.width, self.height, self.rotation))

        # Default QoS (reliable, depth 5): reliable subscribers and best effort ones both
        # connect, as with the Gazebo bridge.
        self.image_pub = self.create_publisher(Image, 'image_raw', 5)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 5)
        self.compressed_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 5)

        self.stats_lock = threading.Lock()
        self.frames = 0
        self.busy_s = 0.0
        self.report_start = time.monotonic()
        self.create_timer(report_period, self.report)

    def set_image_size(self, width, height):
        """Set the published size and the CameraInfo that goes with it."""
        self.out_width, self.out_height = width, height
        calibration = self.read_calibration(width, height)
        self.info = camera_info(calibration, width, height, self.frame_id)

    def read_calibration(self, width, height):
        """Return the calibration for a width x height image, or None with a warning."""
        logger = self.get_logger()
        if not self.calibration_file:
            logger.warning('no calibration_file: camera_info carries only the image size')
            return None
        try:
            calibration = load_calibration(self.calibration_file)
        except (OSError, CalibrationError) as error:
            logger.warning(f'calibration {self.calibration_file} not used: {error}. '
                           'camera_info carries only the image size')
            return None
        mismatch = size_mismatch(calibration, width, height)
        if mismatch:
            logger.warning(f'calibration {self.calibration_file} not used: {mismatch}. '
                           'camera_info carries only the image size')
            return None
        logger.info(f'calibration {self.calibration_file} loaded ({width} x {height})')
        return calibration

    def warn_throttled(self, message):
        """Log `message` as a warning, at most once per WARN_PERIOD_S."""
        now = time.monotonic()
        if now - self.last_warning.get(message, -WARN_PERIOD_S) >= WARN_PERIOD_S:
            self.last_warning[message] = now
            self.get_logger().warning(message)

    def open_camera(self):
        """Open the camera; return True on success. Adapts to the size it actually gives."""
        capture = self.open_capture(self.device, self.width, self.height, self.fps)
        if capture is None:
            self.warn_throttled(f'cannot open {self.device}, retrying every {RETRY_S:.0f} s')
            return False
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = capture.get(cv2.CAP_PROP_FPS)
        if (width, height) != (self.width, self.height):
            self.get_logger().warning(f'asked {self.width} x {self.height}, the camera gives '
                                      f'{width} x {height}: publishing that')
            self.set_image_size(*rotated_size(width, height, self.rotation))
        self.get_logger().info(f'{self.device} open: {width} x {height} at {fps:.0f} fps, '
                               f'publishing {self.out_width} x {self.out_height} '
                               f'(rotation {self.rotation}) in frame {self.frame_id}')
        self.capture = capture
        return True

    def close_camera(self):
        """Release the camera, if open."""
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def publish_frame(self, frame, stamp):
        """Turn one frame as read from the camera and publish it with its camera_info."""
        frame = rotate(frame, self.rotation)
        height, width = frame.shape[:2]
        if (width, height) != (self.out_width, self.out_height):
            self.warn_throttled(f'frame of {width} x {height}, expected '
                                f'{self.out_width} x {self.out_height}: dropped')
            return False
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.width = width
        image.height = height
        image.encoding = 'bgr8'
        image.step = width * 3
        # An array.array, not bytes: rclpy then copies it in one go instead of checking every
        # one of the ~900 000 bytes in Python.
        image.data = array.array('B', frame.tobytes())
        self.image_pub.publish(image)

        self.info.header = image.header
        self.info_pub.publish(self.info)

        # Encoding costs CPU on the Jetson: only for someone listening (a laptop over Wi-Fi).
        if self.compressed_pub.get_subscription_count() > 0:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                compressed = CompressedImage()
                compressed.header = image.header
                compressed.format = COMPRESSED_FORMAT
                compressed.data = array.array('B', jpeg.tobytes())
                self.compressed_pub.publish(compressed)
        return True

    def step(self):
        """Read and publish one frame, opening the camera first if needed. False: retry later."""
        if self.capture is None and not self.open_camera():
            return False
        ok, frame = self.capture.read()
        # Stamped when read() returns, the closest OpenCV gets to the exposure time.
        stamp = self.get_clock().now().to_msg()
        if not ok or frame is None:
            self.warn_throttled(f'no frame from {self.device}: reopening it')
            self.close_camera()
            return False
        start = time.monotonic()
        published = self.publish_frame(frame, stamp)
        if published:
            with self.stats_lock:
                self.frames += 1
                self.busy_s += time.monotonic() - start
        return published

    def run(self):
        """Capture loop, for the camera thread. Ends when stop() is called."""
        while not self.stopping.is_set():
            if not self.step():
                self.stopping.wait(RETRY_S)
        self.close_camera()

    def stop(self):
        """Ask the capture loop to end."""
        self.stopping.set()

    def report(self):
        """Log the rate since the previous report, then start counting again."""
        now = time.monotonic()
        with self.stats_lock:
            frames, busy_s = self.frames, self.busy_s
            self.frames, self.busy_s = 0, 0.0
        seconds = max(now - self.report_start, 1e-9)
        self.report_start = now
        self.get_logger().info(rate_line(frames, seconds, busy_s))


def main():
    """Spin the node, with the capture loop in its own thread, until Ctrl+C."""
    rclpy.init()
    node = WristCamera()
    camera_thread = threading.Thread(target=node.run, name='wrist_camera', daemon=True)
    camera_thread.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.stop()
    camera_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
