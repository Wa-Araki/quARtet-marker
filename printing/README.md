# Printing files

STL meshes for the markers evaluated in the paper (35 mm footprint, tag36h11 IDs 0–3, base tilt 18°, exported from the Fusion 360 add-in's generated model). Each marker is split into **two parts, `*_white.stl` (base) and `*_black.stl` (tag pattern)**, so that two-color printing keeps its per-part extruder assignment.

| File | Marker | Part |
|---|---|---|
| `quartet_diagonal_white.stl` / `quartet_diagonal_black.stl` | quARtet, diagonal layout | white base / black tags (tilted in place) |
| `quartet_pitch_white.stl` / `quartet_pitch_black.stl` | quARtet, pitch layout | white base / black tags (tilted in place) |
| `quartet_pitch_r_white.stl` / `quartet_pitch_r_black.stl` | quARtet, reversed-pitch layout | white base / black tags (tilted in place) |
| `quartet_tags_flat_black.stl` | quARtet (any layout) | the four tags as a flat plate, for **separate-part fabrication**: print flat, then insert into the base sockets (inner taper −1° ensures a snug fit; adjust to your printer's tolerance) |
| `single_tag_white.stl` / `single_tag_black.stl` | single planar tag (baseline) | white base / black tag |

## Print settings used in the paper

- Printer: Raise3D E2 (any FDM printer with a 0.4 mm nozzle should work; dual extruders needed only for integrated two-color printing)
- Material: PLA, black + white
- Method used in the paper: separate-part fabrication (`*_white.stl` printed in white, the tag parts printed separately in black and inserted into the base sockets; the inner taper provides the fit)
- Layer height: **0.1 mm**
- Infill: **10 %**

To generate markers with other sizes, layouts, or tag IDs, edit `../marker_generator/setting.json` and re-run the Fusion 360 add-in (see the top-level README); the same configuration then drives detection, so no re-measurement is needed. The parameter names in `setting.json` are those of Table 1 in the paper, and the bundled example carries the paper's fabrication values.
