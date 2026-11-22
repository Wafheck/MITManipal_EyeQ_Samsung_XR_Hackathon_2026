# MITManipal_EyeQ_Samsung_XR_Hackathon_2026

## Track 01 – Synthetic Eye Movement & Virtual Camera Rendering

A textured, rigged human face in Blender, driven by a deterministic Python wrapper.
Both eyeballs rotate in 3D about their own centres to fixate a common target placed
35 cm in front of the eye midpoint. A locked camera 40 cm in front of the face renders
1024 × 1024 PNGs, and every frame gets a ground-truth JSON record (inputs, 3D eye
rotations, gaze vectors, 2D pupil centres and the camera matrix).

## Bundle layout

```
team_name.zip
├── scene.blend          # textured face mesh, eye bones (eye.L / eye.R), materials, camera, lights
├── wrapper/eyes.py      # eyes.move(h, v) / eyes.reset() / eyes.render()
├── renders/             # 10 clockwise frames, frame_00.png … frame_09.png
├── metadata.json        # calibration + one record per render
├── contact_sheet.png    # all 10 frames + eye close-ups with GT pupil markers
└── README.md
```

Optional extra (not part of the required layout): `videos/` with eye-motion demo videos, created by
`--video` (see "Eye-motion videos" below).

## Setup

**Option A: Blender (recommended).** Install **Blender 5.3** (the version we built and tested on).
`scene.blend` was saved in 5.3, so older versions may refuse to open it.
Nothing else is needed; the wrapper uses only Blender's bundled Python.

**Option B: plain Python.** `pip install bpy` gives Blender as a Python module. The wheel
must match your Python version; for example, `bpy` 5.3 needs Python 3.13. (This build has no
built-in video encoder, so `--video` writes PNG frames and encodes them with `ffmpeg` if it is on your PATH.)

## Run

From the folder that contains `scene.blend`:

```bash
# 10 clockwise frames -> renders/ + metadata.json
blender -b scene.blend --python wrapper/eyes.py

# five-direction self-check (left/right/up/down/reset) -> directions/ + directions.json
blender -b scene.blend --python wrapper/eyes.py -- --mode directions

# both, with more samples
blender -b scene.blend --python wrapper/eyes.py -- --mode all --samples 64

# instant geometry self-test (no rendering): pixel shifts, eyelids + pass/fail
blender -b scene.blend --python wrapper/eyes.py -- --check

# fast low-sample renders into preview/ (never touches renders/)
blender -b scene.blend --python wrapper/eyes.py -- --preview --mode directions

# eye-motion videos for the three inputs -> videos/left.mp4, right.mp4, motion.mp4
blender -b scene.blend --python wrapper/eyes.py -- --video all

# same thing without the Blender app (Option B)
python wrapper/eyes.py --mode all
```

| Option | Default | Meaning |
|---|---|---|
| `--mode` | `sequence` | `sequence`, `directions` or `all` |
| `--samples` | 32 (from the scene) | Cycles samples; denoising is on (videos default to 8) |
| `--frames` | 10 | number of clockwise frames |
| `--range` | 0.05 | target travel in metres for \|h\| = 1 or \|v\| = 1 |
| `--convention` | `viewer` | which side counts as "left/right" (see below) |
| `--out` | project folder | where `renders/`, `metadata.json`, `directions/` and `videos/` go |
| `--lid-follow` | 0.8 | eyelid travel vs cornea travel on vertical gaze (0 = lids frozen) |
| `--video` | off | `left`, `right`, `motion` or `all`: render eye-motion mp4s (each ≤ 15 s) into `videos/` |
| `--seconds` | 5 (left/right), 10 (motion) | video length, capped at 15 s |
| `--fps` | 24 | video frame rate |
| `--video-scale` | 50 | video render size in percent (50 → 512 × 512) |
| `--video-range` | 0.10 | gaze travel (m) for \|h\| = 1 in videos, larger than the stills so the motion is clearly visible |

Each frame takes about 1 minute on a 2-core CPU, about 15 s on a MacBook, and a few seconds on a
modern laptop GPU. To use a GPU, open the scene, pick it under
*Preferences → System → Cycles Render Devices*, and set *Render Properties → Device* to GPU.

## Python API

Use it inside Blender's Python console or a script, or from Python with `bpy` installed:

```python
import sys; sys.path.append("wrapper")
import eyes

eyes.move(1, 0)                   # look right
rec = eyes.render("right.png")    # renders and returns the ground-truth record
eyes.move(0.7071, 0.7071)         # up-right diagonal
eyes.eyelids(1.0)                 # close both eyelids (0 = open, 1 = closed), gaze unchanged
eyes.move(0, -1, 0.5)             # optional 3rd argument sets the eyelids too: look down, half-closed
eyes.reset()                      # centre gaze, eyelids open
print(eyes.state())               # current ground truth without rendering
eyes.animate("motion")            # eye-motion video -> videos/motion.mp4 ("left" and "right" too)
```

Inputs `h`, `v` are normalised to **[-1, +1]** and clamped. `h > 0` is right and `v > 0` is up.
The eyelid closure is clamped to **[0, 1]**.

## How the rig works

* **Mesh.** The CC0 MakeHuman base mesh, scaled to metres and cropped to the head. Skin is a
  procedural shader with subsurface scattering, mottling and pore bump. Lips and eyelids are
  tinted with the CC0 MakeHuman mask textures. Eyebrows are procedural.
* **Eyes.** One sphere per eye (radius 13.8 mm, interpupillary distance 61.5 mm, taken from
  MakeHuman's eye helpers). The eye shader works in each eyeball's local space: pupil
  (4.1 mm), iris (12.2 mm) with radial fibres, limbal ring and sclera. A clear-coat layer
  gives a wet cornea with a catchlight.
* **Bones.** The armature `FaceRig` has a `head` bone with children `eye.L` and `eye.R`, whose
  heads sit exactly at the eyeball centres. Each eye bone carries a **Damped Track**
  constraint aimed at the `FixationTarget` empty. The eyes therefore rotate spherically about
  their own centres (no 2D sliding) and **converge** on the target the way real eyes do.
  At centre gaze each eye turns about 5° inwards.
* **Mapping.** `move(h, v)` puts the target at `(s·h·R, −0.35, v·R)` with R = 5 cm
  (the ±5 cm motion bound), which is about ±8.1° of gaze. That gives a pupil shift of about
  12 px in the 1024² image. Use `--range` for larger sweeps.
* **Eyelids.** On load the wrapper adds two shape keys to the face (`scene.blend` itself is unchanged):
  * `GazeLids`: both lids follow vertical gaze (upper lid travels 0.8× the cornea's shift, the
    lower lid 0.3× of that). `--lid-follow 0` freezes them.
  * `LidClose`: eyelid closure for blinks, half-closed and fully closed eyes, set with
    `eyelids(0..1)` or `move(h, v, closure)`. The lid corners stay fixed.

## Calibration

| Item | Value |
|---|---|
| World frame | metres, Z up; **eye midpoint = origin**; face looks along −Y |
| Fixation target | 35 cm in front of the eye midpoint, travel ±5 cm |
| Camera | locked at (0, −0.40, 0), looking +Y; 85 mm lens on a 36 mm sensor; f = 2417.8 px; principal point (512, 512) |
| Output | 1024 × 1024 PNG, Cycles, fixed seed, constant lighting (key, soft key, fill and rim lights; only the key light shows in the cornea) |

### Left/right convention (important)

The subject's **left eye (`L`) is at +X, so it appears on the RIGHT side of the image.**

By default (`--convention viewer`), *right* means **image right**. `move(1, 0)` sends both
pupils towards the right of the picture, and "clockwise" in the 10-frame sequence is clockwise
as you look at the images: top → right → bottom → left.
Pass `--convention subject` to flip *h* so that "right" means the subject's own right.

## metadata.json

```jsonc
{
  "calibration": {
    "image_size": [1024, 1024],
    "camera_matrix": [[2417.78, 0, 512], [0, 2417.78, 512], [0, 0, 1]],    // K, pixels
    "camera_extrinsic_world_to_cv": [[...], [...], [...]],               // [R|t], OpenCV axes
    "fixation_distance_m": 0.35, "camera_distance_m": 0.4, "gaze_range_m": 0.05, "lid_follow": 0.8, ...
  },
  "frames": [
    {
      "frame": 3,
      "theta": 1.884956,                         // θk = 2πk/10
      "input": {"h": 0.951057, "v": -0.309017},
      "eyelid_closure": 0.0,                     // 0 = open, 1 = closed
      "fixation_target_world": [x, y, z],
      "eye_rotation_L": [rx, ry, rz],            // XYZ Euler (rad), world frame, relative to straight-ahead rest
      "eye_rotation_R": [rx, ry, rz],
      "eye_yaw_pitch_deg_L": [yaw, pitch],       // yaw + = towards image right, pitch + = up
      "gaze_dir_L": [x, y, z],                   // unit optical axis, world frame
      "eye_center_3D_L": [x, y, z],
      "pupil_center_3D_L": [x, y, z],            // front pole of the eyeball
      "pupil_center_2D_L": [x, y],               // pixels, origin top-left, y down
      "pupil_visible_L": true,                   // false if an eyelid covers the pupil centre
      "...": "same keys for R",
      "camera_matrix": [[...]],
      "image": "frame_03.png"
    }
  ]
}
```

`pupil_center_2D = K·[R|t]·pupil_center_3D`. This was checked against Blender's own projection
and agrees to < 0.001 px. `contact_sheet.png` draws these points on the renders so you can
verify them by eye. `pupil_visible_*` comes from a ray cast from the camera to the pupil point,
so it also reports when a closed or lowered eyelid hides the pupil centre.

## Self-check

`--mode directions` renders centre / left / right / up / down / reset and tests the 2D pupil
shift of **both** eyes against centre. Left/right must move along x with |dx| > |dy|,
up/down along y, and reset must return exactly to centre. Our run: **5/5 PASS**.
Results go to `directions/directions.json`. `--check` runs the same test without rendering
(about 1 second) and also checks the eyelids: closing them hides the pupil, a half-closed eye
still shows it, and `reset()` opens the lids again.

## Eye-motion videos

`--video left | right | motion | all` renders one mp4 per input into `videos/` (H.264, 24 fps,
512 × 512 by default, each at most 15 s):

* `left` (5 s): both eyes look left, hold with a small natural drift and two blinks, then return to centre.
* `right` (5 s): the mirror image.
* `motion` (10 s, "generate motion"): look left, sweep right, look up, roll the eyes one full
  clockwise turn, return to centre, with three blinks. The eyelids follow the gaze throughout.

The motion is keyframed on the fixation target and the lid shape keys, so it also plays on
Blender's timeline. All three inputs use `eyes.animate(kind)` underneath. Videos use a larger gaze
travel (`--video-range`, 0.10 m) than the 10 official stills so the movement is clearly visible.
Budget roughly 15–30 minutes for `--video all` on a laptop CPU (use `--seconds 1` for a quick test).

## Development (GitHub repo)

The GitHub repo also contains:

| Path | What |
|---|---|
| `docs/WRAPPER_GUIDE.md` | start here if you're changing the wrapper |
| `tests/test_eyes.py` | 12 tests that run in about 1 s with no rendering: `blender -b scene.blend --python tests/test_eyes.py` |
| `tools/eyes_panel.py` | live H/V gaze sliders in Blender's sidebar (N → Eyes) |
| `tools/build_scene.py` | regenerates `scene.blend` from `assets/` |
| `tools/contact_sheet.py` | rebuilds `contact_sheet.png` from `renders/` + `metadata.json` |
| `tools/make_demo.py` | renders a smooth gaze animation for the demo video |
| `tools/package.py` | builds `CollegeName_TeamName.zip` in the required layout (never overwrites) |

## Rebuilding the scene

`scene.blend` is generated by `tools/build_scene.py` from `assets/` (in the GitHub repo):

```bash
blender -b --python tools/build_scene.py
python tools/contact_sheet.py        # needs: pip install pillow
```

## Credits

The base mesh and the lip/eyelid mask textures come from MakeHuman / MPFB2
(© Data Collection AB, Joel Palmius, Jonas Hauquier), released under **CC0 1.0**.
All shaders, rigging and code are our own.
