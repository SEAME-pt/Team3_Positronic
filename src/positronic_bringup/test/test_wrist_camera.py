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
#   Write the pytest of wrist_camera.py, with no camera and no ROS graph: the rotation and the
#   size it gives; the calibration yaml (camera_calibration's ost.yaml) accepted, and refused
#   for each thing that can be wrong; the size check; the rate line; the CameraInfo with and
#   without a calibration. Then the node, with a fake camera and fake publishers: one frame
#   gives an upright bgr8 image and a camera_info with the same stamp and frame_id; the jpeg
#   is encoded only with a subscriber and decodes to the published image; a failed read
#   releases the camera and the next step reopens it; a camera that cannot open, or that gives
#   another size than asked, or a calibration of another size; the report and the run loop.
# Follow-up prompt (PR review):
#   Cover the drop path of publish_frame (a frame of another size than the published one: no
#   image, no camera_info, the camera kept), and the start-up check of width, height, fps,
#   buffer_size, report_period (> 0) and jpeg_quality (1 to 100).
# Follow-up prompt (PR review):
#   image_raw is filled and published only while subscribed, like the jpeg. The fake image_raw
#   publisher gets one subscriber by default; check that without one no raw image goes out,
#   while camera_info and the jpeg still do, with the same header, and the frame still counts.

"""Check wrist_camera.py without a camera: the maths, the calibration, and the node."""

import importlib.util
import os

import cv2
import numpy
import pytest
import rclpy
from rclpy.parameter import Parameter

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'scripts', 'wrist_camera.py')

# The yaml camera_calibration saves (ost.yaml), for the published 480 x 640 image.
CALIBRATION = """\
image_width: 480
image_height: 640
camera_name: wrist_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [550.0, 0.0, 240.0, 0.0, 551.0, 320.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.1, 0.01, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [540.0, 0.0, 240.0, 0.0, 0.0, 541.0, 320.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('wrist_camera', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


camera = load_script()


def test_rotated_size_swaps_for_a_quarter_turn_only():
    """Check width and height after each rotation, and that an unknown one is refused."""
    assert camera.rotated_size(640, 480, 'ccw90') == (480, 640)
    assert camera.rotated_size(640, 480, 'cw90') == (480, 640)
    assert camera.rotated_size(640, 480, 'none') == (640, 480)
    assert camera.rotated_size(640, 480, '180') == (640, 480)
    with pytest.raises(ValueError):
        camera.rotated_size(640, 480, '90')


def test_ccw90_puts_the_top_right_corner_top_left():
    """Check the direction of ccw90: what was on the right of the raw image goes up."""
    frame = numpy.zeros((2, 3, 3), dtype=numpy.uint8)
    frame[0, 2] = 255  # top right
    turned = camera.rotate(frame, 'ccw90')
    assert turned.shape == (3, 2, 3)
    assert turned[0, 0].tolist() == [255, 255, 255]
    assert int(turned.sum()) == 3 * 255


def test_rotation_none_returns_the_frame_itself():
    """Check that no rotation means no copy."""
    frame = numpy.zeros((2, 3, 3), dtype=numpy.uint8)
    assert camera.rotate(frame, 'none') is frame


def test_parse_calibration_reads_the_camera_calibration_file():
    """Check every value the CameraInfo needs."""
    calibration = camera.parse_calibration(CALIBRATION)
    assert (calibration['width'], calibration['height']) == (480, 640)
    assert calibration['camera_name'] == 'wrist_camera'
    assert calibration['distortion_model'] == 'plumb_bob'
    assert calibration['k'][0] == 550.0 and calibration['k'][4] == 551.0
    assert calibration['d'] == [-0.1, 0.01, 0.0, 0.0, 0.0]
    assert len(calibration['r']) == 9
    assert len(calibration['p']) == 12


def test_parse_calibration_accepts_any_number_of_distortion_coefficients():
    """Check the 8 coefficients of rational_polynomial are accepted."""
    text = CALIBRATION.replace('cols: 5\n  data: [-0.1, 0.01, 0.0, 0.0, 0.0]',
                               'cols: 8\n  data: [1, 2, 3, 4, 5, 6, 7, 8]')
    assert camera.parse_calibration(text)['d'] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]


@pytest.mark.parametrize('text', [
    '{not yaml',
    '- a list, not a mapping',
    CALIBRATION.replace('image_width: 480\n', ''),
    CALIBRATION.replace('image_height: 640', 'image_height: 0'),
    CALIBRATION.replace('image_width: 480', 'image_width: wide'),
    CALIBRATION.replace('distortion_model: plumb_bob\n', ''),
    CALIBRATION.replace('[550.0, 0.0, 240.0, 0.0, 551.0, 320.0, 0.0, 0.0, 1.0]',
                        '[550.0, 0.0, 240.0]'),
    CALIBRATION.replace('[540.0, 0.0, 240.0, 0.0, ', '['),
    CALIBRATION.replace('data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]', 'data: 1'),
])
def test_parse_calibration_refuses_a_broken_file(text):
    """Check each way a file can be wrong gives a CalibrationError, not a wrong CameraInfo."""
    with pytest.raises(camera.CalibrationError):
        camera.parse_calibration(text)


def test_load_calibration_reads_a_file(tmp_path):
    """Check the file is read, and that a missing one raises OSError."""
    path = tmp_path / 'wrist_camera.yaml'
    path.write_text(CALIBRATION)
    assert camera.load_calibration(str(path))['width'] == 480
    with pytest.raises(OSError):
        camera.load_calibration(str(tmp_path / 'missing.yaml'))


def test_size_mismatch():
    """Check a calibration fits only the image size it was made for."""
    calibration = camera.parse_calibration(CALIBRATION)
    assert camera.size_mismatch(calibration, 480, 640) is None
    assert camera.size_mismatch(calibration, 640, 480) == (
        'calibrated for 480 x 640, the published image is 640 x 480')


def test_rate_line():
    """Check the report line, and that a period without frames says so."""
    assert camera.rate_line(300, 10.0, 3.0) == (
        '30.0 fps over 10 s, 10.0 ms per frame from grab to publish')
    assert camera.rate_line(0, 10.0, 0.0) == 'no frame in the last 10 s'


def test_camera_info_without_calibration_has_the_size_only():
    """Check an uncalibrated CameraInfo: the size and the frame, K all zero."""
    info = camera.camera_info(None, 480, 640, 'camera_link')
    assert (info.width, info.height) == (480, 640)
    assert info.header.frame_id == 'camera_link'
    assert list(info.k) == [0.0] * 9


def test_camera_info_carries_the_calibration():
    """Check K, D, R, P and the distortion model reach the CameraInfo."""
    calibration = camera.parse_calibration(CALIBRATION)
    info = camera.camera_info(calibration, 480, 640, 'camera_link')
    assert info.distortion_model == 'plumb_bob'
    assert list(info.k) == calibration['k']
    assert list(info.d) == calibration['d']
    assert list(info.r) == calibration['r']
    assert list(info.p) == calibration['p']


# ---- the node, with a fake camera and fake publishers ----

class FakeCapture:
    """A camera that gives width x height frames, or fails the reads listed in `fail_reads`."""

    def __init__(self, width=640, height=480, fps=30.0, fail_reads=()):
        """Remember the size and which reads (counted from 0) fail."""
        self.width, self.height, self.fps = width, height, fps
        self.fail_reads = set(fail_reads)
        self.reads = 0
        self.released = False

    def get(self, prop):
        """Return the size or the rate, as OpenCV does."""
        return {cv2.CAP_PROP_FRAME_WIDTH: self.width, cv2.CAP_PROP_FRAME_HEIGHT: self.height,
                cv2.CAP_PROP_FPS: self.fps}[prop]

    def read(self):
        """Return a frame whose top right corner is white, or (False, None)."""
        index = self.reads
        self.reads += 1
        if index in self.fail_reads:
            return False, None
        frame = numpy.zeros((self.height, self.width, 3), dtype=numpy.uint8)
        frame[0, self.width - 1] = 255
        return True, frame

    def release(self):
        """Remember the release."""
        self.released = True


class FakeOpener:
    """open_capture for the node: hands out the given captures, None when there are no more."""

    def __init__(self, *captures):
        """Keep the captures to hand out, in order."""
        self.captures = list(captures)
        self.calls = []

    def __call__(self, device, width, height, fps, buffer_size):
        """Record the call and return the next capture."""
        self.calls.append((device, width, height, fps, buffer_size))
        return self.captures.pop(0) if self.captures else None


class FakePublisher:
    """Keeps what is published, with a fixed number of subscribers."""

    def __init__(self, subscribers=0):
        """Start with nothing published."""
        self.subscribers = subscribers
        self.sent = []

    def publish(self, message):
        """Keep the message."""
        self.sent.append(message)

    def get_subscription_count(self):
        """Return the fixed number of subscribers."""
        return self.subscribers


@pytest.fixture(scope='module')
def ros():
    """Bring rclpy up for the node tests of this module, and down at the end."""
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(opener, compressed_subscribers=0, image_subscribers=1, **parameters):
    """
    Return the node with fake publishers and the given parameter overrides.

    image_raw has one subscriber by default, so the tests see the raw image; the jpeg none.
    """
    overrides = [Parameter(name, value=value) for name, value in parameters.items()]
    node = camera.WristCamera(open_capture=opener, parameter_overrides=overrides)
    node.image_pub = FakePublisher(image_subscribers)
    node.info_pub = FakePublisher()
    node.compressed_pub = FakePublisher(compressed_subscribers)
    return node


def test_one_frame_gives_an_upright_image_and_its_camera_info(ros):
    """Check image size, encoding, data, frame_id, and camera_info with the same header."""
    opener = FakeOpener(FakeCapture())
    node = make_node(opener)
    try:
        assert node.step()
        assert opener.calls == [('/dev/video0', 640, 480, 30, 4)]
        image = node.image_pub.sent[0]
        assert (image.width, image.height) == (480, 640)
        assert image.encoding == 'bgr8'
        assert image.step == 480 * 3
        assert len(image.data) == 480 * 640 * 3
        assert list(image.data[:3]) == [255, 255, 255]  # top right raw is top left published
        assert image.header.frame_id == 'camera_link'
        info = node.info_pub.sent[0]
        assert info.header.stamp == image.header.stamp
        assert info.header.frame_id == 'camera_link'
        assert (info.width, info.height) == (480, 640)
        assert node.compressed_pub.sent == []  # nobody listening: no jpeg
    finally:
        node.destroy_node()


def test_jpeg_only_with_a_subscriber_and_it_decodes_to_the_image(ros):
    """Check image_raw/compressed: format string, header, and the decoded size."""
    node = make_node(FakeOpener(FakeCapture()), compressed_subscribers=1)
    try:
        assert node.step()
        compressed = node.compressed_pub.sent[0]
        assert compressed.format == 'bgr8; jpeg compressed bgr8'
        assert compressed.header.stamp == node.image_pub.sent[0].header.stamp
        decoded = cv2.imdecode(numpy.frombuffer(bytes(compressed.data), numpy.uint8),
                               cv2.IMREAD_COLOR)
        assert decoded.shape == (640, 480, 3)
    finally:
        node.destroy_node()


def test_raw_image_only_with_a_subscriber(ros):
    """Check no image_raw without a subscriber, but camera_info and the jpeg still go out."""
    node = make_node(FakeOpener(FakeCapture()), compressed_subscribers=1, image_subscribers=0)
    try:
        assert node.step()
        assert node.image_pub.sent == []
        assert len(node.info_pub.sent) == 1
        assert len(node.compressed_pub.sent) == 1
        info, compressed = node.info_pub.sent[0], node.compressed_pub.sent[0]
        assert info.header.stamp == compressed.header.stamp
        assert info.header.frame_id == compressed.header.frame_id == 'camera_link'
        assert node.frames == 1  # the frame was read and handled: it counts in the rate
    finally:
        node.destroy_node()


def test_failed_read_releases_the_camera_and_the_next_step_reopens_it(ros):
    """Check the recovery after a camera that stops answering."""
    first, second = FakeCapture(fail_reads=[0]), FakeCapture()
    opener = FakeOpener(first, second)
    node = make_node(opener)
    try:
        assert not node.step()
        assert first.released
        assert node.capture is None
        assert node.image_pub.sent == []
        assert node.step()
        assert len(opener.calls) == 2
        assert len(node.image_pub.sent) == 1
    finally:
        node.destroy_node()


def test_camera_that_does_not_open(ros):
    """Check a missing camera: no frame, no exception, the next step tries again."""
    opener = FakeOpener()
    node = make_node(opener)
    try:
        assert not node.step()
        assert not node.step()
        assert len(opener.calls) == 2
        assert node.image_pub.sent == []
    finally:
        node.destroy_node()


def test_camera_giving_another_size_is_published_at_that_size(ros):
    """Check the published size and camera_info follow what the camera really gives."""
    node = make_node(FakeOpener(FakeCapture(width=320, height=240)))
    try:
        assert node.step()
        image = node.image_pub.sent[0]
        assert (image.width, image.height) == (240, 320)
        assert (node.info_pub.sent[0].width, node.info_pub.sent[0].height) == (240, 320)
    finally:
        node.destroy_node()


def test_buffer_size_reaches_open_capture(ros):
    """Check the buffer_size parameter is passed through to the opener."""
    opener = FakeOpener(FakeCapture())
    node = make_node(opener, buffer_size=1)
    try:
        assert node.step()
        assert opener.calls[0][4] == 1
    finally:
        node.destroy_node()


def test_rotation_none_publishes_the_raw_frame(ros):
    """Check the rotation parameter reaches the image."""
    node = make_node(FakeOpener(FakeCapture()), rotation='none')
    try:
        assert node.step()
        assert (node.image_pub.sent[0].width, node.image_pub.sent[0].height) == (640, 480)
    finally:
        node.destroy_node()


def test_unknown_rotation_stops_the_node(ros):
    """Check a typo in rotation fails at start, not frame after frame."""
    with pytest.raises(ValueError):
        make_node(FakeOpener(), rotation='90')


@pytest.mark.parametrize('name, value', [
    ('width', 0), ('height', -1), ('fps', 0), ('buffer_size', 0), ('report_period', 0.0),
    ('jpeg_quality', 0), ('jpeg_quality', 101),
])
def test_parameter_out_of_range_stops_the_node(ros, name, value):
    """Check a bad launch argument fails at start, with the parameter's name."""
    with pytest.raises(ValueError, match=name):
        make_node(FakeOpener(), **{name: value})


def test_check_parameters_accepts_the_defaults():
    """Check the defaults of the node pass, jpeg_quality at both ends included."""
    camera.check_parameters(width=640, height=480, fps=30, buffer_size=4, report_period=10.0,
                            jpeg_quality=80)
    camera.check_parameters(jpeg_quality=1)
    camera.check_parameters(jpeg_quality=100)


class ShortFrameCapture(FakeCapture):
    """Reports 640 x 480 when opened, then delivers 320 x 240 frames."""

    def read(self):
        """Return a frame smaller than the size reported at open."""
        self.reads += 1
        return True, numpy.zeros((240, 320, 3), dtype=numpy.uint8)


def test_frame_of_another_size_is_dropped(ros):
    """Check a frame that does not match the published size publishes nothing, camera kept."""
    capture = ShortFrameCapture()
    node = make_node(FakeOpener(capture))
    try:
        assert not node.step()
        assert node.image_pub.sent == []
        assert node.info_pub.sent == []
        assert node.compressed_pub.sent == []
        assert node.frames == 0
        assert node.capture is capture  # not a camera failure: no reopen
        assert not capture.released
    finally:
        node.destroy_node()


def test_calibration_file_reaches_camera_info(ros, tmp_path):
    """Check a calibration of the published size is published."""
    path = tmp_path / 'wrist_camera.yaml'
    path.write_text(CALIBRATION)
    node = make_node(FakeOpener(FakeCapture()), calibration_file=str(path))
    try:
        assert node.step()
        assert list(node.info_pub.sent[0].k) == camera.parse_calibration(CALIBRATION)['k']
    finally:
        node.destroy_node()


OTHER_SIZE = CALIBRATION.replace('image_width: 480', 'image_width: 640')


@pytest.mark.parametrize('content', [None, OTHER_SIZE])
def test_calibration_missing_or_of_another_size_is_not_published(ros, tmp_path, content):
    """Check a missing file or a file for another size gives the size alone, K zero."""
    path = tmp_path / 'wrist_camera.yaml'
    if content is not None:
        path.write_text(content)
    node = make_node(FakeOpener(FakeCapture()), calibration_file=str(path))
    try:
        assert node.step()
        assert list(node.info_pub.sent[0].k) == [0.0] * 9
    finally:
        node.destroy_node()


def test_report_counts_the_frames_and_starts_again(ros):
    """Check report() empties the counters."""
    node = make_node(FakeOpener(FakeCapture()))
    try:
        assert node.step() and node.step()
        assert node.frames == 2
        node.report()
        assert node.frames == 0
        assert node.busy_s == 0.0
    finally:
        node.destroy_node()


def test_run_stops_when_asked_and_releases_the_camera(ros):
    """Check the capture loop: publishes until stop(), then releases the camera."""
    capture = FakeCapture()
    node = make_node(FakeOpener(capture))
    original_read = capture.read

    def read_then_stop():
        if capture.reads == 2:
            node.stop()
        return original_read()

    capture.read = read_then_stop
    try:
        node.run()
        assert len(node.image_pub.sent) == 3
        assert capture.released
    finally:
        node.destroy_node()
