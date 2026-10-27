#!/usr/bin/env python3
"""Guardrail damage detection from the 3D LiDAR point cloud.

The right-hand guardrail is a continuous W-beam sitting at a roughly fixed
height next to the lane.  We take the points in a height band that contains
the beam (so the road surface is excluded), keep only the right-hand side,
estimate the rail's lateral position from the median ``y``, then histogram the
remaining points along the car's ``x`` (forward) axis.  A longitudinal void
that is long enough and bounded by rail on both sides means the beam is
discontinuous there -- i.e. the damaged section.

Everything here is pure numpy/math so it can be imported both by the ROS node
(if one is ever added) and by ``guardrail_gui.py``.
"""

import math

import numpy as np


def default_detection_config():
    """Default parameters, also used as ROS parameter defaults."""
    return {
        # Topic (used by the GUI; kept here so the config lives in one place).
        'scan_topic': '/scan/points',

        # Rail-height band in the LiDAR frame (x forward, y left, z up).
        # The LiDAR sits ~1.70 m above ground; the W-beam top is at ~0.755 m,
        # i.e. z in [-1.2, -0.9] m in the LiDAR frame.
        'z_min': -1.30,
        'z_max': -0.85,

        # Only look at the right-hand side of the vehicle.
        'y_min': -9.0,
        'y_max': -1.0,

        # Keep points within this lateral distance of the median rail y.
        'lateral_tol': 0.6,

        # Longitudinal histogram parameters [m].
        'bin_size': 0.25,
        'min_points_per_bin': 3,
        'min_gap': 2.5,        # a void >= this length counts as damage
        'edge_margin': 1.5,    # ignore voids this close to the visible ends

        'min_points_total': 120,
    }


def cloud_to_xyz(msg):
    """Convert a sensor_msgs/PointCloud2 to an (N, 3) float array.

    Returns None when the x/y/z fields are missing or not float32.
    """
    names = [f.name for f in msg.fields]
    if not {'x', 'y', 'z'}.issubset(set(names)):
        return None
    fields = {f.name: f for f in msg.fields}
    for key in ('x', 'y', 'z'):
        if fields[key].datatype != 7:  # sensor_msgs.msg.PointField.FLOAT32
            return None

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    buf = buf.reshape(msg.height, msg.width, msg.point_step)

    def column(key):
        off = fields[key].offset
        raw = np.ascontiguousarray(buf[:, :, off:off + 4])
        return raw.view(np.float32).reshape(-1)

    return np.column_stack([column('x'), column('y'), column('z')])


def detect_rail_gap(points, cfg):
    """Detect a longitudinal gap in the right-hand guardrail.

    ``points`` is an (N, 3) float array in the LiDAR frame.

    Returns None when there is not enough data to decide, otherwise a dict::

        {'found': bool,          # damage (discontinuity) detected
         'x': float,             # gap centre along the car x axis [m]
         'length': float,        # gap length [m]
         'rail_y': float,        # lateral position of the rail [m]
         'n': int}               # number of rail points used
    """
    if points is None or len(points) < cfg['min_points_total']:
        return None

    x = points[:, 0].astype(np.float64)
    y = points[:, 1].astype(np.float64)
    z = points[:, 2].astype(np.float64)

    sel = ((z >= cfg['z_min']) & (z <= cfg['z_max']) &
           (y >= cfg['y_min']) & (y <= cfg['y_max']))
    if int(sel.sum()) < cfg['min_points_total']:
        return None

    rail_y = float(np.median(y[sel]))

    xs = x[sel]
    ys = y[sel]
    near = np.abs(ys - rail_y) <= cfg['lateral_tol']
    xs = xs[near]
    if xs.size < cfg['min_points_total']:
        return None

    x_min = float(xs.min())
    x_max = float(xs.max())
    result_base = {'rail_y': rail_y, 'n': int(xs.size)}

    # Too short an extent to confidently call a gap.
    if x_max - x_min < cfg['min_gap'] + 2.0 * cfg['edge_margin']:
        result_base.update({'found': False, 'x': 0.0, 'length': 0.0})
        return result_base

    bin_size = cfg['bin_size']
    lo = math.floor(x_min / bin_size)
    hi = math.ceil(x_max / bin_size)
    edges = np.arange(lo, hi + 1) * bin_size
    hist, _ = np.histogram(xs, bins=edges)
    occupied = hist >= cfg['min_points_per_bin']

    best_len = 0.0
    best_x = 0.0
    i = 0
    m = occupied.size
    while i < m:
        if not occupied[i]:
            j = i
            while j < m and not occupied[j]:
                j += 1
            start_x = edges[i]
            end_x = edges[j]
            run_len = end_x - start_x
            if (start_x > x_min + cfg['edge_margin'] and
                    end_x < x_max - cfg['edge_margin'] and
                    run_len >= cfg['min_gap'] and
                    run_len > best_len):
                best_len = run_len
                best_x = 0.5 * (start_x + end_x)
            i = j
        else:
            i += 1

    result_base.update({'found': best_len >= cfg['min_gap'],
                        'x': float(best_x),
                        'length': float(best_len)})
    return result_base


def make_camera_projection(cfg):
    """Build the pinhole camera model used to project a 3D point to pixels.

    ``cfg`` is the GUI configuration; it is expected to contain ``hfov``,
    ``camera_yaw``, ``camera_pitch`` and (optionally) the image size and the
    camera mount pose in the base_link frame.
    """
    width = int(cfg.get('image_width', 640))
    height = int(cfg.get('image_height', 480))
    hfov = float(cfg.get('hfov', 1.396263))
    focal = (width * 0.5) / math.tan(hfov * 0.5)
    return {
        'f': focal,
        'cx': width * 0.5,
        'cy': height * 0.5,
        'width': width,
        'height': height,
        'yaw': float(cfg.get('camera_yaw', -0.08)),
        'pitch': float(cfg.get('camera_pitch', 0.0)),
        'ox': float(cfg.get('camera_x', 2.36)),
        'oy': float(cfg.get('camera_y', 0.0)),
        'oz': float(cfg.get('camera_z', 0.025)),
    }


def project_to_pixel(point, cam):
    """Project a 3D point (in the base_link frame) to camera pixel (u, v).

    Returns None when the point is behind the camera.  The caller still has to
    check that the pixel lies inside the image.
    """
    dx = point[0] - cam['ox']
    dy = point[1] - cam['oy']
    dz = point[2] - cam['oz']

    # base_link -> camera_link: Rz(yaw) then Ry(pitch); invert to get camera
    # frame coordinates of the point.
    cy = math.cos(cam['yaw'])
    sy = math.sin(cam['yaw'])
    x1 = dx * cy + dy * sy
    y1 = -dx * sy + dy * cy
    z1 = dz

    cp = math.cos(cam['pitch'])
    sp = math.sin(cam['pitch'])
    xc = x1 * cp - z1 * sp
    yc = y1
    zc = x1 * sp + z1 * cp

    # REP-103 optical frame: x right, y down, z forward.
    x_opt = -yc
    y_opt = -zc
    z_opt = xc

    if z_opt <= 0.05:
        return None

    u = cam['cx'] + cam['f'] * x_opt / z_opt
    v = cam['cy'] + cam['f'] * y_opt / z_opt
    return (u, v)
