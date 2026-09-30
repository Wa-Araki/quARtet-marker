# quARtet Marker

Design, fabrication, and detection toolchain for the **quARtet marker**, a 3D-printable multi-tag fiducial for robust near-frontal pose estimation, together with the per-measurement experimental datasets of the accompanying paper:

> A. Wakiuchi, H. Sasaki, T. Matsubara, "quARtet Marker: A 3D-Printable Multi-Tag Fiducial for Robust Near-Frontal Pose Estimation" (under review).
> <!-- the arXiv identifier is added at posting and the DOI upon acceptance -->

Four AprilTags are placed at predefined tilts on a single planar substrate, so that informative perspective cues remain available even when the marker is viewed frontally. The marker geometry is defined once, in `setting.json`: the Fusion 360 add-in reads this file to build the printable model, and the detector reconstructs the 3D coordinates of the tag corners from the same file (see Quick start) for one joint `solvePnP` over all detected corners. The fabricated marker and the pose-estimation model therefore stay consistent without manual re-measurement.

## Repository layout

| Path | Contents |
|---|---|
| `marker_generator/` | Fusion 360 **Python add-in** (`quARtet_maker2.py`). Reads `setting.json` and builds the marker as a fully editable model in the Fusion 360 workspace (no direct STL export), so it can be merged with other 3D-printed parts (e.g. fixtures) before printing. Bundled `0.png`–`3.png` are AprilTag `tag36h11` IDs 0–3. |
| `detection/` | Pose-estimation code. `quARtet.py`: AprilTag detection (pyapriltags) and one joint `solvePnP` over all detected tag corners (up to 16) using the shared marker geometry; this file is the detector exactly as used for the paper. `quARtet_wrapper.py`: convenience layer that configures the detector from `setting.json` and offers the optional AprilTag frame convention (`z_flip`). `IV4_wrapper.py`: capture wrapper for the Keyence IV4 camera used in the paper (any calibrated 2D camera works). |
| `printing/` | STL meshes for the evaluated markers: the three quARtet layouts and the single-tag baseline, each split into a white base part + black tag part for two-color printing, plus a flat tag plate for separate-part (insert) fabrication. See `printing/README.md` for the file list and the print settings used in the paper. |
| `experiments/data/` | Canonical CSV datasets of the paper's experiments (orientation / position grids, closed-loop pose-hold logs with a per-run summary, hand-eye recordings and transforms, swing-down in-grasp slip). The analysis steps that turn these files into the paper's tables are documented in Supplementary Section S3 of the paper. All translations are in meters and rotations are Rodrigues vectors in radians. The fixed-camera files contain measurement columns only: for release, the single-tag files were converted from centimeters (the unit in which the single-tag detector was run) and the acquisition notebook's auxiliary columns and empty rows were removed; the paper's analysis used the raw files with the same conversion. |

## Quick start

```bash
pip install numpy opencv-python pyapriltags
```

**Estimating a pose** (run from the repository root):

```python
import cv2
import numpy as np
from detection.quARtet_wrapper import quARtetFromSetting

camera_params = [fx, fy, cx, cy]   # intrinsics from your camera calibration
dist_coeffs = np.zeros(5)          # or the calibrated distortion coefficients

marker = quARtetFromSetting("marker_generator/setting.json", camera_params, dist_coeffs)

frame = cv2.imread("image.jpg")                    # BGR image
tvec, rvec, detected_ids = marker.detect(frame)    # tvec in m, rvec as a Rodrigues vector in rad
```

`quARtetFromSetting` reads the marker geometry (`whole_size_mm`, `tag_size_mm`, `tilt_degree`, `tilt_mode`) and `tag_family` from the same `setting.json` that generated the marker, infers the tag IDs from the image file names (`0.png` to `3.png`, or apriltag-imgs names such as `tag36_11_00003.png`; otherwise pass `tag_ids=[...]` in the order AR0 to AR3), and hands them to the detector class `quARtet` in `detection/quARtet.py`. That class is the detector exactly as used for the paper and can also be instantiated directly. Note one sign convention when doing so: the add-in and the detector define the tilt with opposite signs, so a marker generated with `tilt_degree = 18` is reconstructed with `XY_tilt_degree = -18` (the wrapper applies this negation, and the experiments of the paper used it). With the wrong sign the reconstructed geometry is the mirror image of the printed marker and the pose estimate is invalid.

`detect()` returns `numpy.nan` for both vectors when no tag of the marker is found (the third element is then the placeholder `[0]`, not a detected tag ID), and it draws the detected tags and the pose axes onto `frame` (disable the tag outlines with `vis_detected_markers_pos=False`).

**Pose convention:** the returned pose is that of the marker frame expressed in the camera frame (OpenCV convention). As in the paper, the marker frame has its origin at the center of the marker footprint on the base plane of the tag geometry (the plane from which the four tags are tilted), and its x- and y-axes are aligned with the tag grid, following the orientation of tag 0 (`AR0_file_path`). By default the z-axis points out of the marker toward the camera side (the tilted tag faces rise toward +z), so a marker viewed head-on has a rotation of about 180° relative to the camera frame; the released pose logs in `experiments/data` use this convention. With `quARtetFromSetting(..., z_flip=True)` the pose is reported in the AprilTag frame convention instead (x right, y down, z into the marker; a head-on, upright marker gives R = I, as a single AprilTag does). This conversion is a 180° rotation about the marker x-axis, so y and z are both reversed (reversing z alone would be a reflection, not a rotation). The translation is unchanged, and the axes drawn on the frame follow the selected convention.

**Scope of this repository:** the marker toolchain (generator, detector, print files) and the experimental datasets. The robot-control and analysis code used for the paper's experiments is specific to the laboratory setup and is not part of the release; the analysis conventions are documented in the paper's Supplementary Section S3.

**Generating a marker:** open Fusion 360 → Utilities → Add-Ins → Scripts, add `marker_generator/quARtet_maker2.py`, edit `setting.json` (layout `tilt_mode`: `diagonal` / `pitch` / `pitch_r` (reversed-pitch); `roll` and `roll_r` are also implemented but were not evaluated in the paper; sizes in mm), and run. The add-in opens a file dialog; select the `setting.json` to use. Set `AR0_file_path` to `AR3_file_path` to the absolute paths of the tag images on your machine: relative paths such as those in the bundled example are resolved against the working directory of the Fusion 360 process, not against the script folder, and may fail. The marker appears as an editable model; export for printing after any desired edits.

**Fabrication (as used in the paper):** Raise3D E2, black + white PLA, layer height 0.1 mm, infill 10 %. Ready-to-print STL files for all evaluated markers are in `printing/`.

## Notes

- To use other tag IDs or families, point `AR0_file_path` to `AR3_file_path` at the corresponding [apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs) files, set `tag_family` accordingly, and pass the matching `tag_ids` to the detector.
- The inner-taper parameter supports separate-part fabrication, in which the tags are printed separately and then inserted into the base. A negative value (the paper uses -1°) narrows the tag piece toward its interior so that it can be inserted into the base socket and seat snugly; a positive value would widen with depth and could not be inserted. Adjust the magnitude to your printer's fitting tolerance.

## License

The contents of this repository (code, STL files, and datasets) are released under the MIT License. See [`LICENSE`](LICENSE).

The bundled tag images `marker_generator/0.png` to `3.png` (`tag36h11` IDs 0 to 3) are taken from [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs), which is distributed under the BSD 2-Clause License.
