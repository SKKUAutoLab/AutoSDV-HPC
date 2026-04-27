# Copyright 2026 SKKU AutoSDV.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Core BEV perception helpers ported from BEV_Module/{1.py, svm.py}."""

import os

import cv2
import numpy as np
import yaml

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


# --------------------------------------------------------------------------- #
# YAML loaders
# --------------------------------------------------------------------------- #


def load_bev_calibration(yaml_path):
    """Load a per-camera BEV calibration YAML.

    Returns a dict with numpy arrays for camera_matrix, dist_coeffs,
    homography, src_points, dst_points; plus position, image_topic,
    resolution (W, H), bev_size (W, H).
    """
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(
            'BEV calibration YAML not found: {}'.format(yaml_path))

    with open(yaml_path, 'r') as fp:
        data = yaml.safe_load(fp)

    required = [
        'position', 'image_topic', 'resolution',
        'camera_matrix', 'dist_coeffs', 'homography',
        'bev_size', 'src_points', 'dst_points',
    ]
    missing = [k for k in required if k not in data]
    if missing:
        raise KeyError(
            'BEV calibration {} missing fields: {}'.format(yaml_path, missing))

    return {
        'position': str(data['position']),
        'image_topic': str(data['image_topic']),
        'resolution': tuple(int(v) for v in data['resolution']),
        'bev_size': tuple(int(v) for v in data['bev_size']),
        'camera_matrix': np.asarray(data['camera_matrix'], dtype=np.float64),
        'dist_coeffs': np.asarray(
            data['dist_coeffs'], dtype=np.float64).reshape(-1, 1),
        'homography': np.asarray(data['homography'], dtype=np.float64),
        'src_points': np.asarray(data['src_points'], dtype=np.float32),
        'dst_points': np.asarray(data['dst_points'], dtype=np.float32),
    }


def load_svm_layout(yaml_path):
    """Load the SVM canvas layout YAML.

    Returns a dict with output_size (W, H), rois (per-position dict
    of (x, y, w, h)), vehicle_box, overlay_order (list).
    """
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(
            'SVM layout YAML not found: {}'.format(yaml_path))

    with open(yaml_path, 'r') as fp:
        data = yaml.safe_load(fp)

    required = ['output_size', 'rois', 'overlay_order']
    missing = [k for k in required if k not in data]
    if missing:
        raise KeyError(
            'SVM layout {} missing fields: {}'.format(yaml_path, missing))

    rois = {}
    for pos, box in data['rois'].items():
        rois[pos] = (
            int(box['x']), int(box['y']),
            int(box['w']), int(box['h']))

    vehicle_box = None
    if 'vehicle_box' in data and data['vehicle_box']:
        vb = data['vehicle_box']
        vehicle_box = (
            int(vb['x']), int(vb['y']), int(vb['w']), int(vb['h']))

    return {
        'output_size': tuple(int(v) for v in data['output_size']),
        'rois': rois,
        'vehicle_box': vehicle_box,
        'overlay_order': [str(p) for p in data['overlay_order']],
    }


# --------------------------------------------------------------------------- #
# Combined fisheye undistort + perspective warp remap
# (ported from BEV_Module/1.py — get_combined_map)
# --------------------------------------------------------------------------- #


def get_combined_map(camera_matrix, dist_coeffs, src_points, dst_points,
                     bev_size, orig_size):
    """Build (map1, map2) for cv2.remap that does fisheye undistort + warp.

    Inputs map output BEV pixels back to distorted source pixels.

    Args:
        camera_matrix: (3,3) intrinsic K.
        dist_coeffs:   (4,1) fisheye [k1,k2,k3,k4].
        src_points:    (4,2) on the undistorted plane (float32).
        dst_points:    (4,2) on the BEV plane (float32).
        bev_size:      (W, H) of the BEV output.
        orig_size:     (W, H) of the original distorted image
                       (kept for signature parity; not used here).

    Returns:
        (map1, map2) both float32, shape (bev_h, bev_w).
    """
    del orig_size  # not used; kept for API parity with BEV_Module/1.py

    bev_w, bev_h = bev_size

    homography = cv2.getPerspectiveTransform(src_points, dst_points)
    homography_inv = np.linalg.inv(homography)

    grid_u, grid_v = np.meshgrid(np.arange(bev_w), np.arange(bev_h))
    bev_coords = np.vstack((
        grid_u.flatten(),
        grid_v.flatten(),
        np.ones(bev_w * bev_h),
    ))

    undist_coords_homo = homography_inv @ bev_coords
    z_inv = 1.0 / undist_coords_homo[2, :]
    u_undist = undist_coords_homo[0, :] * z_inv
    v_undist = undist_coords_homo[1, :] * z_inv

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    xc = (u_undist - cx) / fx
    yc = (v_undist - cy) / fy

    r = np.sqrt(xc * xc + yc * yc)
    theta = np.arctan(r)
    d = dist_coeffs.flatten()
    theta_d = theta * (1.0
                       + d[0] * theta ** 2
                       + d[1] * theta ** 4
                       + d[2] * theta ** 6
                       + d[3] * theta ** 8)

    scale = np.divide(theta_d, r, out=np.zeros_like(r), where=r != 0)
    x_dist = xc * scale
    y_dist = yc * scale

    map1 = (fx * x_dist + cx).reshape(bev_h, bev_w).astype(np.float32)
    map2 = (fy * y_dist + cy).reshape(bev_h, bev_w).astype(np.float32)
    return map1, map2


# --------------------------------------------------------------------------- #
# SVM canvas composition (v0: ROI overwrite via NumPy slicing)
# --------------------------------------------------------------------------- #


def composite_canvas(canvas, image, roi):
    """Place an image into canvas at roi=(x, y, w, h) with bounds-clipping.

    The image is resized to (w, h) when its shape differs from the ROI.
    Out-of-bounds portions of the ROI are silently clipped.
    Returns canvas (modified in place).
    """
    x, y, w, h = roi
    if w <= 0 or h <= 0:
        return canvas
    canvas_h, canvas_w = canvas.shape[:2]
    if x >= canvas_w or y >= canvas_h:
        return canvas

    if image.shape[1] != w or image.shape[0] != h:
        image = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)

    x0 = max(x, 0)
    y0 = max(y, 0)
    x1 = min(x + w, canvas_w)
    y1 = min(y + h, canvas_h)
    if x1 <= x0 or y1 <= y0:
        return canvas

    src_x0 = x0 - x
    src_y0 = y0 - y
    src_x1 = src_x0 + (x1 - x0)
    src_y1 = src_y0 + (y1 - y0)

    canvas[y0:y1, x0:x1] = image[src_y0:src_y1, src_x0:src_x1]
    return canvas


# --------------------------------------------------------------------------- #
# QoS helpers
# --------------------------------------------------------------------------- #


def parse_reliability(value):
    """Map a string ('BEST_EFFORT'|'RELIABLE') to a QoSReliabilityPolicy."""
    s = str(value).strip().upper()
    if s == 'RELIABLE':
        return QoSReliabilityPolicy.RELIABLE
    if s == 'BEST_EFFORT':
        return QoSReliabilityPolicy.BEST_EFFORT
    raise ValueError(
        'Unsupported reliability "{}". Use BEST_EFFORT or RELIABLE.'
        .format(value))


def make_image_qos(reliability_str, depth=1):
    """Standard image QoS profile keyed by reliability string."""
    return QoSProfile(
        reliability=parse_reliability(reliability_str),
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=int(depth),
        durability=QoSDurabilityPolicy.VOLATILE,
    )
