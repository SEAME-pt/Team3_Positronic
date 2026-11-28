#!/usr/bin/env python3
# Copyright 2026 Positronic. MIT license, see LICENSE at the repository root.
#
# AI-generated (Claude, Anthropic).
# Prompt:
#   Real wrist camera issue: the README must give the reprojection error of the calibration.
#   Write a tool with no ROS 2, only OpenCV, numpy and yaml, that reads what camera_calibration
#   saves (/tmp/calibrationdata.tar.gz: ost.yaml and the left-*.png views it used, or the same
#   files unpacked in a folder), finds the checkerboard in every view, estimates the board pose
#   with the calibration's K and D (solvePnP), projects the corners back and prints the RMS
#   error per view and over all views, in pixels, plus the worst view. --size is the inner
#   corners as COLSxROWS, as cameracalibrator takes it.

"""
Reprojection error of a camera_calibration result, in pixels.

    python3 reprojection_error.py /tmp/calibrationdata.tar.gz --size 8x6 --square 0.025

The source is the tar.gz camera_calibration saves, or a folder with the same files: ost.yaml
(the calibration) and the left-*.png views. For every view where the board is found: board
pose from K and D (cv2.solvePnP), corners projected back (cv2.projectPoints), RMS of the
distance to the detected corners. Prints one line per view, then the RMS over all corners of
all views and the worst view. K and D stay fixed: this is the error of the saved calibration,
not of a new fit. --calibration reads K and D from another yaml.
"""

import argparse
import fnmatch
import math
import os
import sys
import tarfile

import cv2
import numpy
import yaml

VIEW_PATTERN = 'left-*.png'
CALIBRATION_NAME = 'ost.yaml'


def parse_size(text):
    """Return (cols, rows) of inner corners from 'COLSxROWS', as cameracalibrator takes it."""
    try:
        cols, rows = (int(part) for part in text.lower().split('x'))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f'size must be COLSxROWS, not {text!r}') from error
    if cols < 2 or rows < 2:
        raise argparse.ArgumentTypeError(f'size needs at least 2x2 inner corners, not {text!r}')
    return cols, rows


def board_points(cols, rows, square):
    """Return the inner corners of the board in its own plane, metres, row by row."""
    points = numpy.zeros((rows * cols, 3), numpy.float32)
    points[:, :2] = numpy.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return points


def intrinsics(text):
    """Return (K 3x3, D) from the yaml camera_calibration saves."""
    document = yaml.safe_load(text)
    k = numpy.array(document['camera_matrix']['data'], numpy.float64).reshape(3, 3)
    d = numpy.array(document['distortion_coefficients']['data'], numpy.float64)
    return k, d


def find_corners(gray, cols, rows):
    """Return the inner corners found in a grayscale view, refined to sub-pixel, or None."""
    found, corners = cv2.findChessboardCorners(
        gray, (cols, rows), flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)


def view_errors(corners, points, k, d):
    """Return the distance in pixels of every corner to its reprojection, K and D fixed."""
    ok, rvec, tvec = cv2.solvePnP(points, corners, k, d)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(points, rvec, tvec, k, d)
    return numpy.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)


def rms(values):
    """Return the root mean square of a sequence of numbers."""
    values = numpy.asarray(values, numpy.float64)
    return float(math.sqrt(float(numpy.mean(values ** 2))))


def read_source(path):
    """Return (calibration yaml text, [(view name, image bytes)]) from a tar.gz or a folder."""
    views = []
    calibration = None
    if os.path.isdir(path):
        for name in sorted(os.listdir(path)):
            full = os.path.join(path, name)
            if name == CALIBRATION_NAME:
                with open(full, encoding='utf-8') as file:
                    calibration = file.read()
            elif fnmatch.fnmatch(name, VIEW_PATTERN):
                with open(full, 'rb') as file:
                    views.append((name, file.read()))
    else:
        with tarfile.open(path) as archive:
            for member in sorted(archive.getmembers(), key=lambda m: m.name):
                name = os.path.basename(member.name)
                if not member.isfile():
                    continue
                if name == CALIBRATION_NAME:
                    calibration = archive.extractfile(member).read().decode('utf-8')
                elif fnmatch.fnmatch(name, VIEW_PATTERN):
                    views.append((name, archive.extractfile(member).read()))
    return calibration, views


def evaluate(views, k, d, cols, rows, square):
    """Return [(view name, RMS px or None when the board is not found)] and all the errors."""
    points = board_points(cols, rows, square)
    results, everything = [], []
    for name, data in views:
        gray = cv2.imdecode(numpy.frombuffer(data, numpy.uint8), cv2.IMREAD_GRAYSCALE)
        corners = None if gray is None else find_corners(gray, cols, rows)
        errors = None if corners is None else view_errors(corners, points, k, d)
        if errors is None:
            results.append((name, None))
            continue
        results.append((name, rms(errors)))
        everything.extend(errors.tolist())
    return results, everything


def report(results, everything):
    """Return the lines printed: one per view, then the total and the worst view."""
    lines = [f'{name}  {"board not found" if value is None else f"{value:.3f} px"}'
             for name, value in results]
    used = [(name, value) for name, value in results if value is not None]
    if not used:
        return lines + ['no view with the board: nothing measured']
    worst_name, worst = max(used, key=lambda item: item[1])
    return lines + [
        f'views used {len(used)} of {len(results)}, corners {len(everything)}',
        f'reprojection RMS {rms(everything):.3f} px, worst view {worst_name} {worst:.3f} px',
    ]


def main(argv=None):  # pragma: no cover
    """Read the source, print the errors; exit 1 when nothing could be measured."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('source', help='calibrationdata.tar.gz, or a folder with the same files')
    parser.add_argument('--size', type=parse_size, required=True,
                        help='inner corners, COLSxROWS (as cameracalibrator --size)')
    parser.add_argument('--square', type=float, required=True, help='square side, metres')
    parser.add_argument('--calibration', help='yaml with K and D (default: ost.yaml of source)')
    args = parser.parse_args(argv)

    calibration, views = read_source(args.source)
    if args.calibration:
        with open(args.calibration, encoding='utf-8') as file:
            calibration = file.read()
    if calibration is None:
        print(f'no {CALIBRATION_NAME} in {args.source}; give --calibration', file=sys.stderr)
        return 1
    k, d = intrinsics(calibration)
    results, everything = evaluate(views, k, d, *args.size, args.square)
    print('\n'.join(report(results, everything)))
    return 0 if everything else 1


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
