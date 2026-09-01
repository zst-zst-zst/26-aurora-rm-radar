"""Shared helpers for tdt_vision launch files.

Place next to the .launch.py files; importable via:

    import os, sys
    sys.path.insert(0, os.path.dirname(__file__))
    from _common import (...)

The launch/ directory is installed verbatim via ament_auto_package
(INSTALL_TO_SHARE launch), so this module is available at runtime in
share/tdt_vision/launch/_common.py.
"""
from __future__ import annotations

import glob
import os
import yaml

from launch.actions import SetEnvironmentVariable


# ── Hikrobot MVS / libusb workaround ──────────────────────────────────────────
def system_libusb_preload_action():
    """Force-load system libusb so PCL 1.14 finds libusb_set_option.

    Hikrobot MVS ships an older /opt/MVS libusb that is typically prepended to
    LD_LIBRARY_PATH; PCL-linked components break without this preload.
    """
    libusb_path = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"
    if not os.path.exists(libusb_path):
        libusb_path = "/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0"
    existing = os.environ.get("LD_PRELOAD", "").split()
    if os.path.exists(libusb_path) and libusb_path not in existing:
        existing.insert(0, libusb_path)
    return SetEnvironmentVariable("LD_PRELOAD", " ".join(existing))


# ── Workspace config discovery ────────────────────────────────────────────────
def _config_candidates(launch_file: str, filename: str | None = None):
    """Walk up from the launch file looking for config/[filename]."""
    base = os.path.dirname(launch_file)
    rel_steps = ('../../../config', '../../../../config', '../../../../../config')
    candidates = [os.path.abspath(os.path.join(base, rel)) for rel in rel_steps]
    if filename is None:
        return candidates
    return [os.path.join(d, filename) for d in candidates]


def load_runtime_config(launch_file: str) -> dict:
    """Load workspace config/radar_runtime.yaml relative to a launch file."""
    for path in _config_candidates(launch_file, 'radar_runtime.yaml'):
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as fh:
                return yaml.safe_load(fh) or {}
    raise FileNotFoundError('Cannot find config/radar_runtime.yaml from launch path')


def resolve_workspace_config_dir(launch_file: str) -> str:
    """Return the absolute path to the workspace config/ directory."""
    for d in _config_candidates(launch_file):
        if os.path.isdir(d):
            return d
    raise FileNotFoundError('Cannot find workspace config directory')


# ── Team / colour resolution ──────────────────────────────────────────────────
def effective_self_color(calibration_config: dict, runtime_config: dict) -> int:
    """Return effective self-colour (2=red, 0=blue, -1=unknown)."""
    override = runtime_config.get('self_color_override', -1)
    if override in (0, 2):
        return override
    from_calib = calibration_config.get('self_color', -1)
    if from_calib in (0, 2):
        return from_calib
    return -1


def select_calibration_path(calibration_config: dict, runtime_config: dict,
                            key: str, default_path: str) -> str:
    """Pick the team-specific calibration path with sensible fallbacks."""
    self_color = effective_self_color(calibration_config, runtime_config)
    key_red, key_blue = f'{key}_red', f'{key}_blue'

    red_path = calibration_config.get(key_red, calibration_config.get(key, default_path))
    blue_path = calibration_config.get(key_blue, red_path)
    fallback = calibration_config.get(key, red_path)

    if self_color == 2:
        return red_path
    if self_color == 0:
        return blue_path
    return fallback


def warn_if_same_side_calibration(calibration_config: dict,
                                  tag: str,
                                  keys=('out_matrix',)):
    """Warn when red and blue resolve to the same calibration path."""
    for base in keys:
        red_v = str(calibration_config.get(f'{base}_red', calibration_config.get(base, ''))).strip()
        blue_v = str(calibration_config.get(f'{base}_blue', calibration_config.get(base, ''))).strip()
        if red_v and blue_v and red_v == blue_v:
            print(
                f"[{tag}][WARN] {base}_red and {base}_blue are the same path: {red_v}. "
                "Please calibrate both sides separately."
            )


def apply_team_override(runtime_overrides: dict, pre_match_config: dict) -> dict:
    """Honour pre_match.team (0=red, 1=blue) by injecting self_color_override."""
    runtime = dict(runtime_overrides)
    team = str(pre_match_config.get('team', '')).strip()
    if team == '0':
        runtime['self_color_override'] = 2
    elif team == '1':
        runtime['self_color_override'] = 0
    return runtime


# ── Camera brand → intrinsic-params path ──────────────────────────────────────
CAMERA_PARAMS_PATH_HIK = 'src/tdt_vision/camera/config/hik.yaml'
BRAND_CAMERA_PARAMS = {
    'hik': CAMERA_PARAMS_PATH_HIK,
}


def effective_brand(_pre_match_config: dict) -> str:
    """Currently we only ship a Hikvision pipeline."""
    return 'hik'


# ── Serial port autodetect ────────────────────────────────────────────────────
def default_serial_port() -> str:
    """Best-effort default serial port for the gimbal/MCU link."""
    for path in ('/dev/gimbal',):
        if os.path.exists(path):
            return path

    for base_dir in ('/dev/serial/by-id', '/dev/serial/by-path'):
        if os.path.isdir(base_dir):
            entries = sorted(os.listdir(base_dir))
            if entries:
                return os.path.join(base_dir, entries[0])

    for pattern in ('/dev/ttyUSB*', '/dev/ttyACM*'):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    return '/dev/ttyUSB0'


# ── Camera center from extrinsics ────────────────────────────────────────────
def camera_center_from_extrinsics(out_matrix_path: str,
                                  map_height: float = 15.0) -> tuple:
    """Compute camera centre (x, y, z) in **referee frame** from solvePnP YAML.

    Pure-Python Rodrigues; no OpenCV / NumPy dependency.
    Returns (0, 0, 0) if the file cannot be parsed.
    """
    import math
    import re

    try:
        with open(out_matrix_path, 'r', encoding='utf-8') as fh:
            text = fh.read()
    except OSError:
        return (0.0, 0.0, 0.0)

    def _parse(label: str):
        m = re.search(rf'{label}:.*?data:\s*\[(.*?)\]', text, re.DOTALL)
        return [float(x) for x in m.group(1).replace('\n', '').split(',')] if m else None

    rvec = _parse('world_rvec')
    tvec = _parse('world_tvec')
    if not rvec or not tvec or len(rvec) < 3 or len(tvec) < 3:
        return (0.0, 0.0, 0.0)

    # Rodrigues: axis-angle → rotation matrix R
    theta = math.sqrt(sum(r * r for r in rvec))
    if theta < 1e-10:
        return (-tvec[0], -tvec[1] + map_height, -tvec[2])

    k = [r / theta for r in rvec]
    c, s = math.cos(theta), math.sin(theta)
    K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]
    R = [[c * (1 if i == j else 0) + (1 - c) * k[i] * k[j] + s * K[i][j]
          for j in range(3)] for i in range(3)]

    # Camera centre: C = -R^T @ tvec   (extrinsics are in legacy frame)
    cx = -(R[0][0] * tvec[0] + R[1][0] * tvec[1] + R[2][0] * tvec[2])
    cy = -(R[0][1] * tvec[0] + R[1][1] * tvec[1] + R[2][1] * tvec[2])
    cz = -(R[0][2] * tvec[0] + R[1][2] * tvec[1] + R[2][2] * tvec[2])

    cy += map_height  # legacy → referee
    return (cx, cy, cz)
