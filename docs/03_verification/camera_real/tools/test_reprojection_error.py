# Copyright 2026 Positronic. MIT license, see LICENSE at the repository root.
#
# AI-generated (Claude, Anthropic).
# Prompt:
#   Unit tests of reprojection_error.py, no ROS 2 and no camera: render checkerboard views with
#   a known K (a flat board seen through a homography), save them with an ost.yaml as
#   camera_calibration does, in a folder and in a tar.gz, and check that the error is well
#   under a pixel with the right K, clearly larger with a wrong K, that a view without the
#   board is reported and not counted, and the size parsing.
#   python3 -m pytest docs/03_verification/camera_real/tools

"""Check reprojection_error.py on synthetic checkerboard views with a known camera."""

import argparse
import importlib.util
import io
import os
import tarfile

import cv2
import numpy
import pytest

TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'reprojection_error.py')

COLS, ROWS, SQUARE = 7, 5, 0.025  # inner corners and square side of the synthetic board
WIDTH, HEIGHT = 480, 640  # the published wrist camera image, portrait
K = numpy.array([[550.0, 0.0, 240.0], [0.0, 550.0, 320.0], [0.0, 0.0, 1.0]])
PIXELS_PER_SQUARE = 40
MARGIN = PIXELS_PER_SQUARE  # white border, one square wide: the detector needs it


def load_tool():
    """Import the tool as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('reprojection_error', TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tool = load_tool()


def board_raster():
    """Return the board as an image: (COLS+1) x (ROWS+1) squares and a white margin."""
    width = (COLS + 1) * PIXELS_PER_SQUARE + 2 * MARGIN
    height = (ROWS + 1) * PIXELS_PER_SQUARE + 2 * MARGIN
    image = numpy.full((height, width), 255, numpy.uint8)
    for row in range(ROWS + 1):
        for col in range(COLS + 1):
            if (row + col) % 2 == 0:
                top = MARGIN + row * PIXELS_PER_SQUARE
                left = MARGIN + col * PIXELS_PER_SQUARE
                image[top:top + PIXELS_PER_SQUARE, left:left + PIXELS_PER_SQUARE] = 0
    return image


def render_view(rvec, tvec):
    """Return the board seen by the camera K at pose (rvec, tvec), as a PNG."""
    rotation, _ = cv2.Rodrigues(numpy.array(rvec, numpy.float64))
    metres_per_pixel = SQUARE / PIXELS_PER_SQUARE
    # Board raster pixel -> board plane (metres); the first inner corner is the origin.
    origin = MARGIN + PIXELS_PER_SQUARE
    raster_to_plane = numpy.array([[metres_per_pixel, 0.0, -origin * metres_per_pixel],
                                   [0.0, metres_per_pixel, -origin * metres_per_pixel],
                                   [0.0, 0.0, 1.0]])
    plane_to_image = K @ numpy.column_stack([rotation[:, 0], rotation[:, 1], tvec])
    homography = plane_to_image @ raster_to_plane
    view = cv2.warpPerspective(board_raster(), homography, (WIDTH, HEIGHT),
                               flags=cv2.INTER_LINEAR, borderValue=200)
    ok, png = cv2.imencode('.png', view)
    assert ok
    return png.tobytes()


VIEWS = [
    ('left-0000.png', render_view([0.0, 0.0, 0.0], [-0.07, -0.05, 0.40])),
    ('left-0001.png', render_view([0.3, -0.2, 0.1], [-0.06, -0.06, 0.45])),
    ('left-0002.png', render_view([-0.25, 0.3, -0.2], [-0.08, -0.04, 0.42])),
]


def ost_yaml(k):
    """Return an ost.yaml with camera matrix `k` and no distortion."""
    return ('image_width: 480\nimage_height: 640\ncamera_name: wrist_camera\n'
            f'camera_matrix:\n  rows: 3\n  cols: 3\n  data: {k.reshape(-1).tolist()}\n'
            'distortion_model: plumb_bob\n'
            'distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [0, 0, 0, 0, 0]\n')


def test_parse_size():
    """Check COLSxROWS, upper or lower case x, and that a bad size is refused."""
    assert tool.parse_size('8x6') == (8, 6)
    assert tool.parse_size('8X6') == (8, 6)
    for bad in ('8', '8x', 'axb', '1x6'):
        with pytest.raises(argparse.ArgumentTypeError):
            tool.parse_size(bad)


def test_board_points_row_by_row():
    """Check the corner grid: x along the columns, square apart, z zero."""
    points = tool.board_points(3, 2, 0.5)
    assert points.shape == (6, 3)
    assert points[1].tolist() == [0.5, 0.0, 0.0]
    assert points[3].tolist() == [0.0, 0.5, 0.0]


def test_right_calibration_reprojects_well_under_a_pixel():
    """Check the error with the K the views were rendered with."""
    k, d = tool.intrinsics(ost_yaml(K))
    results, everything = tool.evaluate(VIEWS, k, d, COLS, ROWS, SQUARE)
    assert all(value is not None for _, value in results)
    assert len(everything) == len(VIEWS) * COLS * ROWS
    assert tool.rms(everything) < 0.3


def test_wrong_focal_length_shows_in_the_error():
    """Check a 10 % focal length error gives a clearly larger error."""
    wrong = K.copy()
    wrong[0, 0] *= 1.1
    k, d = tool.intrinsics(ost_yaml(wrong))
    _, everything = tool.evaluate(VIEWS, k, d, COLS, ROWS, SQUARE)
    assert tool.rms(everything) > 1.0


def test_view_without_the_board_is_reported_not_counted():
    """Check an empty view gives 'board not found' and adds no corner."""
    ok, blank = cv2.imencode('.png', numpy.full((HEIGHT, WIDTH), 128, numpy.uint8))
    assert ok
    k, d = tool.intrinsics(ost_yaml(K))
    results, everything = tool.evaluate(VIEWS[:1] + [('left-0009.png', blank.tobytes())],
                                        k, d, COLS, ROWS, SQUARE)
    assert results[1] == ('left-0009.png', None)
    assert len(everything) == COLS * ROWS
    lines = tool.report(results, everything)
    assert lines[1] == 'left-0009.png  board not found'
    assert lines[2] == f'views used 1 of 2, corners {COLS * ROWS}'
    assert lines[3].startswith('reprojection RMS ')


def test_report_without_any_view():
    """Check nothing measured is said, not hidden."""
    assert tool.report([('left-0000.png', None)], []) == [
        'left-0000.png  board not found', 'no view with the board: nothing measured']


def test_read_source_from_a_folder_and_from_a_tar(tmp_path):
    """Check both sources give the yaml and the views, other files ignored."""
    folder = tmp_path / 'calibrationdata'
    folder.mkdir()
    (folder / 'ost.yaml').write_text(ost_yaml(K))
    (folder / 'ost.txt').write_text('ignored')
    for name, data in VIEWS:
        (folder / name).write_bytes(data)
    calibration, views = tool.read_source(str(folder))
    assert calibration == ost_yaml(K)
    assert [name for name, _ in views] == [name for name, _ in VIEWS]

    archive = tmp_path / 'calibrationdata.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for path in sorted(folder.iterdir()):
            tar.add(str(path), arcname=path.name)
        info = tarfile.TarInfo('subdir')
        info.type = tarfile.DIRTYPE
        tar.addfile(info, io.BytesIO())
    calibration, views = tool.read_source(str(archive))
    assert calibration == ost_yaml(K)
    assert [name for name, _ in views] == [name for name, _ in VIEWS]
