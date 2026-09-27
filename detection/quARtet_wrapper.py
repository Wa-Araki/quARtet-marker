"""
Convenience layer on top of quARtet.py.

quARtet.py is kept exactly as it was used for the experiments of the paper. This module
adds two things for users of the repository:

* ``quARtetFromSetting`` builds a detector directly from the ``setting.json`` that also
  drives the Fusion 360 add-in, so the fabricated marker and the detection model share
  one source of parameters. It also applies the sign convention the detector needs
  (``XY_tilt_degree = -tilt_degree``), as the experiments of the paper did.
* ``z_flip=True`` reports the pose in the AprilTag frame convention instead of the native
  quARtet convention (see ``to_apriltag_frame``). The default leaves the pose untouched.
"""
import json
import os
import re

import cv2
import numpy as np

try:  # imported as part of the ``detection`` package (e.g. from the repository root)
    from .quARtet import ApriltagDetector, quARtet
except ImportError:  # imported with ``detection/`` itself on sys.path
    from quARtet import ApriltagDetector, quARtet

# 180 degree rotation about the marker x-axis: keeps x, reverses y and z.
_ROT_X_180 = np.diag([1.0, -1.0, -1.0])

SETTING_KEYS = ("tag_family", "tag_size_mm", "whole_size_mm", "tilt_mode", "tilt_degree")


def load_setting(setting_path):
    """Read setting.json and check that the keys needed for detection are present."""
    with open(setting_path, "r") as f:
        cfg = json.load(f)
    missing = [k for k in SETTING_KEYS if k not in cfg]
    if missing:
        raise KeyError(f"{setting_path} lacks the keys required for detection: {missing}")
    return cfg


def tag_ids_from_setting(cfg):
    """
    Infer the tag IDs from the file names in AR0_file_path .. AR3_file_path.

    Works for the bundled images ("0.png" .. "3.png") and for apriltag-imgs names
    ("tag36_11_00003.png" -> 3). Pass ``tag_ids`` explicitly for other naming schemes.
    """
    ids = []
    for i in range(4):
        path = cfg[f"AR{i}_file_path"]
        stem = os.path.splitext(os.path.basename(path))[0]
        m = re.search(r"(\d+)$", stem)
        if m is None:
            raise ValueError(
                f"Cannot infer the tag ID from AR{i}_file_path={path!r}; pass tag_ids explicitly."
            )
        ids.append(int(m.group(1)))
    return ids


def to_apriltag_frame(rvec):
    """
    Convert a quARtet rotation vector to the AprilTag frame convention.

    Native quARtet frame: x along the tag grid, y up, z out of the marker toward the camera.
    AprilTag frame:       x along the tag grid, y down, z into the marker.
    The two differ by a 180 degree rotation about the marker x-axis, so y and z are both
    reversed (reversing z alone would be a reflection, not a rotation). A marker seen
    head-on therefore has R = I in the AprilTag convention, as a single AprilTag does.
    Tag 0 (AR0_file_path) defines the in-plane orientation of the marker axes; the
    translation is unchanged.
    """
    rvec = np.asarray(rvec, dtype=float)
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    rvec_new, _ = cv2.Rodrigues(R @ _ROT_X_180)
    return rvec_new.reshape(rvec.shape)


class quARtetFromSetting(quARtet):
    """
    quARtet detector configured from setting.json, with an optional AprilTag-style frame.

    Args:
        setting_path (str): Path to the setting.json used to generate the marker.
        camera_params (list): Camera intrinsics [fx, fy, cx, cy].
        dist_coeffs (array-like or None): OpenCV distortion coefficients (None = none).
        tag_ids (list or None): IDs of the four tags in the order AR0..AR3. Inferred from
            the image file names when None.
        z_flip (bool): False (default) returns the native quARtet pose. True returns the
            pose in the AprilTag frame convention (see ``to_apriltag_frame``); the axes
            drawn on the frame follow the same convention.
        tag_detector (TagDetectorBase or None): Custom detector; defaults to
            ``ApriltagDetector`` with the ``tag_family`` of setting.json.
        hamming_thr (int): Hamming threshold for the default detector.
        vis_detected_markers_pos (bool): Draw the detected tag outlines and IDs.
    """

    def __init__(
        self,
        setting_path,
        camera_params,
        dist_coeffs=None,
        tag_ids=None,
        z_flip=False,
        tag_detector=None,
        hamming_thr=0,
        vis_detected_markers_pos=True,
    ):
        cfg = load_setting(setting_path)
        if tag_ids is None:
            tag_ids = tag_ids_from_setting(cfg)
        if tag_detector is None:
            tag_detector = ApriltagDetector(cfg["tag_family"], hamming_thr=hamming_thr)
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5)

        # The add-in and the detector define the tilt with opposite signs: a marker generated with
        # tilt_degree = 18 is reconstructed by the detector with XY_tilt_degree = -18 (tag faces rising
        # toward +z, the camera side). The experiments of the paper used exactly this negation.
        super().__init__(
            tag_detector,
            tag_ids,
            cfg["tag_size_mm"],
            cfg["whole_size_mm"],
            camera_params,
            np.asarray(dist_coeffs, dtype=float),
            tilt_mode=cfg["tilt_mode"],
            vis_detected_markers_pos=vis_detected_markers_pos,
            XY_tilt_degree=-cfg["tilt_degree"],
        )
        self.setting = cfg
        self.z_flip = z_flip

    def _draw_attitude(self, frame, rvec, tvec):
        # Draw the axes in the same convention as the returned pose.
        if self.z_flip:
            rvec = to_apriltag_frame(rvec)
        super()._draw_attitude(frame, rvec, tvec)

    def detect(self, frame):
        """
        Same as quARtet.detect(). With z_flip=True the rotation vector is converted to the
        AprilTag frame convention. Returns (tvec [m], rvec [rad], detected_tag_ids).
        """
        tvec, rvec, detected_tag_ids = super().detect(frame)
        if self.z_flip and not np.isscalar(rvec):
            rvec = to_apriltag_frame(rvec)
        return tvec, rvec, detected_tag_ids
