"""
eyes.py - deterministic eye-articulation wrapper for scene.blend  (Track 01)
============================================================================

QUICK START
-----------
    blender -b scene.blend --python wrapper/eyes.py                         # official: 10 frames + metadata.json
    blender -b scene.blend --python wrapper/eyes.py -- --check              # 2 s, no rendering: geometry self-test
    blender -b scene.blend --python wrapper/eyes.py -- --preview            # fast low-sample frames -> preview/
    blender -b scene.blend --python wrapper/eyes.py -- --mode directions    # 5-direction pass/fail renders
    blender -b scene.blend --python wrapper/eyes.py -- --video all          # videos/left.mp4, right.mp4, motion.mp4 (<= 15 s each)
    python wrapper/eyes.py --check                                          # same, with `pip install bpy`

    In the Blender app: open scene.blend -> Scripting tab -> open this file -> Run Script.
    Nothing renders; you can then type  eyes.move(1, 0)  in the Python console and watch the viewport.

PUBLIC API  (the names the evaluators call - do not rename)
----------
    move(h, v)          h, v in [-1, +1]  ->  both eyes fixate the target, returns state()
                        (optional 3rd arg closure 0..1 also sets the eyelids)
    eyelids(closure)    0 = open ... 1 = closed, gaze unchanged, returns state()
    animate(kind)       kind = "left" | "right" | "motion": keyframed eye-motion video (mp4)
    reset()             centre gaze, returns state()
    render(path=None)   writes a 1024x1024 PNG, returns the ground-truth record
    state()             ground-truth record for the current pose (no rendering)

FILE MAP
--------
    1. CONFIG ............ every tunable number lives here
    2. SCENE ACCESS ...... loading scene.blend, finding the rig / target / camera
    3. CORE API .......... move / reset / render
    4. GROUND TRUTH ...... rotations, gaze, 2D pupil projection, camera matrices
    5. BATCH RUNS ........ 10-frame clockwise sequence, 5-direction check
    6. CLI ............... argument parsing / entry point

See docs/WRAPPER_GUIDE.md for conventions, recipes and gotchas.
"""
import argparse
import glob
import json
import math
import os
import shutil
import subprocess
import sys

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Matrix, Vector

__version__ = "1.3.0"

# =============================================================================
# 1. CONFIG
# =============================================================================
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                         # folder that holds scene.blend
DEFAULT_BLEND = os.path.join(ROOT, "scene.blend")

GAZE_RANGE_M = 0.05      # how far the fixation target moves for |h| = 1 or |v| = 1 (spec: +/- 5 cm)
CONVENTION = "viewer"    # "viewer": h > 0 -> pupils move to IMAGE right
                         # "subject": h > 0 -> pupils move to the SUBJECT's right (image left)
LID_FOLLOW = 0.8         # eyelids follow vertical gaze: upper lid travels LID_FOLLOW x the cornea's vertical
                         # displacement (0 = lids frozen, as in v1.1). Set via --lid-follow / configure().
LID_LOWER_RATIO = 0.3    # lower lid moves this fraction of the upper lid's travel
LID_REF_DEG = 10.0       # pitch (deg) at which the "GazeLids" shape key is authored (value 1.0)
LID_OPEN_HALF_M = 0.0045 # half-height of the open eye (lid margin to eyeball centre), metres
LID_OPEN_HALF_W_M = 0.012  # half-width of the lid opening: corners further out than this never move
VIDEO_RANGE_M = 0.10     # gaze travel used by the demo videos (official frames keep the spec's 0.05)
VIDEO_SAMPLES = 8        # Cycles samples for videos
VIDEO_SCALE = 50         # render-resolution percentage for videos (50 -> 512x512); stills stay 1024x1024
VIDEO_FPS = 24
VIDEO_MAX_S = 15.0       # hard cap on one video's length
VIDEO_SECONDS = {"left": 5.0, "right": 5.0, "motion": 10.0}   # default length of each input's video
N_FRAMES = 10            # clockwise sequence length (theta_k = 2*pi*k/N)
PREVIEW_SAMPLES = 4      # Cycles samples used by --preview (official renders use the scene's 32)

# Names of objects/bones inside scene.blend that this file relies on.
# If you rename anything in Blender, change it here too.
RIG = "FaceRig"
FACE = "Face"
LID_KEY = "GazeLids"                         # shape key the wrapper creates on Face (idempotent): lids follow vertical gaze
CLOSE_KEY = "LidClose"                       # shape key the wrapper creates on Face (idempotent): 0 = open, 1 = closed
TARGET = "FixationTarget"
CAMERA = "Camera"
EYE_BONES = {"L": "eye.L", "R": "eye.R"}     # L = subject's left eye (+X, appears on IMAGE RIGHT)

# Fallbacks if the scene's custom properties are missing (build_scene.py writes them).
DEFAULT_FIXATION_M = 0.35
DEFAULT_CAMERA_M = 0.40
DEFAULT_EYE_RADIUS_M = 0.0138

_state = {"h": 0.0, "v": 0.0, "lid": 0.0}   # last inputs passed to move() / eyelids()


def configure(gaze_range_m=None, convention=None, lid_follow=None):
    """Change GAZE_RANGE_M / CONVENTION / LID_FOLLOW at runtime (the CLI calls this)."""
    global GAZE_RANGE_M, CONVENTION, LID_FOLLOW
    if lid_follow is not None:
        LID_FOLLOW = max(0.0, float(lid_follow))
    if gaze_range_m is not None:
        GAZE_RANGE_M = float(gaze_range_m)
    if convention is not None:
        if convention not in ("viewer", "subject"):
            raise ValueError("convention must be 'viewer' or 'subject'")
        CONVENTION = convention


def _h_sign():
    return 1.0 if CONVENTION == "viewer" else -1.0


# =============================================================================
# 2. SCENE ACCESS
# =============================================================================
def load(blend_path=DEFAULT_BLEND):
    """Open scene.blend unless the rig is already loaded (e.g. you opened it in Blender)."""
    if RIG not in bpy.data.objects:
        bpy.ops.wm.open_mainfile(filepath=os.path.abspath(blend_path))
    return bpy.context.scene


def _objs():
    load()
    d = bpy.data.objects
    _ensure_lids(d[FACE], d[RIG])
    return d[RIG], d[TARGET], d[CAMERA]


def _smoothstep(a, b, x):
    t = max(0.0, min(1.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def _ensure_lids(face, rig):
    """Create the lid shape keys on Face if they aren't there yet (idempotent, no scene changes).

    "GazeLids": value +1 = lids raised by what a ~LID_REF_DEG upward gaze needs; negative values
                mirror it (lids drop when looking down). Driven by _drive_lids().
    "LidClose" : value 0 = open, 1 = closed (upper lid meets the lower lid just below the eyeball
                centre; the corners stay put). Driven by eyelids() / move(..., closure).
    Built from the mesh around each eyeball centre, so no change to scene.blend / build_scene.py is
    needed and the result is identical on every run."""
    sk = face.data.shape_keys
    have = set(sk.key_blocks.keys()) if sk is not None else set()
    if LID_KEY in have and CLOSE_KEY in have:
        return
    if sk is None:
        face.shape_key_add(name="Basis", from_mix=False)
    radius = float(bpy.context.scene.get("eye_radius_m", DEFAULT_EYE_RADIUS_M))
    travel = radius * math.sin(math.radians(LID_REF_DEG))        # cornea's vertical shift at LID_REF_DEG
    to_local = face.matrix_world.inverted()
    centres = [to_local @ (rig.matrix_world @ rig.pose.bones[b].head) for b in EYE_BONES.values()]
    h0, xa = LID_OPEN_HALF_M, LID_OPEN_HALF_W_M
    gaze_dz, close_dz = [], []
    for v in face.data.vertices:
        co = Vector(v.co)
        gz = cz = 0.0
        for c in centres:
            d = co - c
            if d.y > -0.003:                                       # behind the front of the eyeball
                continue
            # look: lids follow vertical gaze (per unit LID_FOLLOW; scaled by the key's value)
            w = (1.0 - _smoothstep(0.004, 0.012, abs(d.z))) * (1.0 - _smoothstep(0.004, 0.011, abs(d.x)))
            if w > 0.0:
                gz += w * travel * (1.0 if d.z > 0 else LID_LOWER_RATIO)
            # close: lids meet at z = -0.3*h(x); h(x) = half-height of the opening (elliptical)
            ax = abs(d.x)
            if ax < xa:
                h = h0 * math.sqrt(1.0 - (ax / xa) ** 2)
                if d.z > 0:
                    cz -= (1.0 - _smoothstep(h0, h0 + 0.010, d.z)) * 1.3 * h
                else:
                    cz += (1.0 - _smoothstep(h0, h0 + 0.006, -d.z)) * 0.7 * h
        gaze_dz.append(gz)
        close_dz.append(cz)
    for name, dzs, lo, hi in ((LID_KEY, gaze_dz, -3.0, 3.0), (CLOSE_KEY, close_dz, 0.0, 1.0)):
        if name in have:
            continue
        key = face.shape_key_add(name=name, from_mix=False)
        key.slider_min, key.slider_max = lo, hi
        for i, v in enumerate(face.data.vertices):
            key.data[i].co = Vector(v.co) + Vector((0.0, 0.0, dzs[i]))


def _scene_value(key, default):
    return float(bpy.context.scene.get(key, default))


# =============================================================================
# 3. CORE API
# =============================================================================
def move(h, v, closure=None):
    """Point both eyes at the fixation target for normalised input (h, v).

    h, v are clamped to [-1, +1]. h > 0 = right (see CONVENTION), v > 0 = up.
    closure (optional) sets the eyelids too: 0 = open ... 1 = closed; None keeps the current value.
    The target is placed at (s*h*R, -fixation, v*R) in world metres; each eye bone
    has a Damped Track constraint to it, so the eyes rotate about their own centres
    and converge. Returns state().
    """
    h = max(-1.0, min(1.0, float(h)))
    v = max(-1.0, min(1.0, float(v)))
    if closure is not None:
        _state["lid"] = max(0.0, min(1.0, float(closure)))
    _, target, _ = _objs()
    fix = _scene_value("fixation_dist_m", DEFAULT_FIXATION_M)
    target.location = (_h_sign() * h * GAZE_RANGE_M, -fix, v * GAZE_RANGE_M)
    bpy.context.view_layer.update()          # re-evaluate constraints NOW, before reading poses
    _drive_lids()
    _state["h"], _state["v"] = h, v
    return state()


def _drive_lids():
    """Set the GazeLids slider from the mean vertical gaze of the two eyes (so it also tracks
    --range changes) and the LidClose slider from the requested closure, then re-evaluate so ray
    casts / renders see the moved lids."""
    keys = bpy.data.objects[FACE].data.shape_keys.key_blocks
    pitch = sum(math.degrees(math.asin(max(-1.0, min(1.0, _eye_pose(s)[2].z)))) for s in EYE_BONES) / 2
    keys[LID_KEY].value = LID_FOLLOW * pitch / LID_REF_DEG
    keys[CLOSE_KEY].value = _state["lid"]
    bpy.context.view_layer.update()


def eyelids(closure):
    """Open / close both eyelids without moving the gaze: 0 = open, 1 = closed (clamped).
    Returns state()."""
    _state["lid"] = max(0.0, min(1.0, float(closure)))
    _objs()
    _drive_lids()
    return state()


def reset():
    """Centre gaze (both eyes fixate the point straight ahead at 35 cm) and open the eyelids.
    Returns state()."""
    return move(0.0, 0.0, 0.0)


def render(path=None, samples=None):
    """Render the current gaze to `path` (PNG) and return its ground-truth record.

    path    default: output/gaze_h{h}_v{v}.png next to scene.blend
    samples Cycles samples for this render only (None = the scene's setting)
    """
    scene = load()
    if path is None:
        path = os.path.join(ROOT, "output", f"gaze_h{_state['h']:+.3f}_v{_state['v']:+.3f}.png")
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    old_samples = scene.cycles.samples
    if samples is not None:
        scene.cycles.samples = int(samples)
    try:
        scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
    finally:
        scene.cycles.samples = old_samples   # don't leave the scene modified
    rec = state()
    rec["image"] = os.path.basename(path)
    return rec


# =============================================================================
# 4. GROUND TRUTH
# =============================================================================
def _eye_pose(side):
    """(rotation-from-rest 3x3, eye centre, unit gaze direction) in world frame."""
    rig, _, _ = _objs()
    pb = rig.pose.bones[EYE_BONES[side]]
    posed = (rig.matrix_world @ pb.matrix).to_3x3()
    rest = (rig.matrix_world @ pb.bone.matrix_local).to_3x3()
    rot = posed @ rest.inverted()                         # rotation from straight-ahead rest
    centre = rig.matrix_world @ pb.head                   # bone head = eyeball centre
    gaze = (posed @ Vector((0, 1, 0))).normalized()       # bone +Y axis = optical axis
    return rot, centre, gaze


def camera_intrinsics():
    """3x3 K matrix in pixels (square pixels, principal point from lens shift)."""
    _, _, cam = _objs()
    r = bpy.context.scene.render
    W, H = r.resolution_x, r.resolution_y
    c = cam.data
    vertical = c.sensor_fit == "VERTICAL"
    f_px = c.lens / (c.sensor_height if vertical else c.sensor_width) * (H if vertical else W)
    cx = W / 2 - c.shift_x * max(W, H)
    cy = H / 2 + c.shift_y * max(W, H)
    return [[f_px, 0.0, cx], [0.0, f_px, cy], [0.0, 0.0, 1.0]]


def camera_extrinsics():
    """World -> camera, OpenCV axes (x right, y down, z forward), as a 3x4 [R|t]."""
    _, _, cam = _objs()
    flip = Matrix(((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1)))
    m = flip @ cam.matrix_world.inverted()
    return [[m[r][c] for c in range(4)] for r in range(3)]


def project(p_world):
    """3D world point -> [x, y] in pixels (origin top-left, y down)."""
    _, _, cam = _objs()
    scene = bpy.context.scene
    W, H = scene.render.resolution_x, scene.render.resolution_y
    ndc = world_to_camera_view(scene, cam, Vector(p_world))
    return [ndc.x * W, (1.0 - ndc.y) * H]


def pupil_visible(p_world):
    """True if nothing (an eyelid) sits between the camera and the 3D point `p_world`.
    Ray-cast against the evaluated scene, stopping 0.5 mm short of the point so the eyeball's
    own surface doesn't count. Uses the viewport subdivision level, so it is exact to ~0.1 mm."""
    _, _, cam = _objs()
    origin = cam.matrix_world.translation
    d = Vector(p_world) - origin
    dist = d.length
    hit = bpy.context.scene.ray_cast(bpy.context.evaluated_depsgraph_get(), origin, d / dist,
                                     distance=dist - 0.0005)[0]
    return not hit


def state():
    """Ground-truth record for the current pose. No rendering. Key names are part of
    the metadata.json format - change them only together with the README."""
    _, target, _ = _objs()
    radius = _scene_value("eye_radius_m", DEFAULT_EYE_RADIUS_M)
    rec = {"input": {"h": round(_state["h"], 6), "v": round(_state["v"], 6)},
           "eyelid_closure": round(_state["lid"], 4),            # 0 = open, 1 = closed
           "fixation_target_world": [round(x, 6) for x in target.location]}
    for side in ("L", "R"):
        rot, centre, gaze = _eye_pose(side)
        pupil3d = centre + gaze * radius                  # front pole of the eyeball
        yaw = math.degrees(math.atan2(gaze.x, -gaze.y))                       # + = image right
        pitch = math.degrees(math.atan2(gaze.z, math.hypot(gaze.x, gaze.y)))  # + = up
        rec[f"eye_rotation_{side}"] = [round(a, 6) for a in rot.to_euler("XYZ")]
        rec[f"eye_yaw_pitch_deg_{side}"] = [round(yaw, 4), round(pitch, 4)]
        rec[f"gaze_dir_{side}"] = [round(x, 6) for x in gaze]
        rec[f"eye_center_3D_{side}"] = [round(x, 6) for x in centre]
        rec[f"pupil_center_3D_{side}"] = [round(x, 6) for x in pupil3d]
        rec[f"pupil_center_2D_{side}"] = [round(x, 3) for x in project(pupil3d)]
        rec[f"pupil_visible_{side}"] = pupil_visible(pupil3d)            # False = covered by an eyelid
    rec["camera_matrix"] = [[round(x, 6) for x in r] for r in camera_intrinsics()]
    return rec


def calibration():
    """Scene-wide constants written at the top of metadata.json."""
    _, _, cam = _objs()
    r = bpy.context.scene.render
    return {
        "units": "metres; world Z-up; eye midpoint at origin; face looks along -Y",
        "image_size": [r.resolution_x, r.resolution_y],
        "pixel_convention": "origin top-left, x right, y down",
        "camera_position_world": [round(x, 6) for x in cam.matrix_world.translation],
        "camera_distance_m": _scene_value("camera_dist_m", DEFAULT_CAMERA_M),
        "fixation_distance_m": _scene_value("fixation_dist_m", DEFAULT_FIXATION_M),
        "gaze_range_m": GAZE_RANGE_M,
        "lid_follow": LID_FOLLOW,
        "convention": CONVENTION,
        "camera_matrix": camera_intrinsics(),
        "camera_extrinsic_world_to_cv": [[round(x, 6) for x in r] for r in camera_extrinsics()],
        "eye_radius_m": _scene_value("eye_radius_m", DEFAULT_EYE_RADIUS_M),
    }


# =============================================================================
# 5. BATCH RUNS
# =============================================================================
def clockwise_inputs(n=N_FRAMES):
    """[(k, theta, h, v)]: theta_k = 2*pi*k/n, h = sin, v = cos -> starts at top, clockwise."""
    return [(k, 2 * math.pi * k / n, math.sin(2 * math.pi * k / n), math.cos(2 * math.pi * k / n))
            for k in range(n)]


def sequence_records(n=N_FRAMES):
    """Ground truth for the clockwise sequence WITHOUT rendering (used by --check and tests)."""
    out = []
    for k, th, h, v in clockwise_inputs(n):
        rec = move(h, v)
        out.append({"frame": k, "theta": round(th, 6), **rec, "image": f"frame_{k:02d}.png"})
    reset()
    return out


def run_sequence(out_dir=ROOT, n=N_FRAMES, samples=None):
    """Render the n clockwise frames to out_dir/renders and write out_dir/metadata.json."""
    frames = []
    for k, th, h, v in clockwise_inputs(n):
        move(h, v)
        rec = render(os.path.join(out_dir, "renders", f"frame_{k:02d}.png"), samples)
        frames.append({"frame": k, "theta": round(th, 6), **rec})
        print(f"[eyes] frame {k}: h={h:+.3f} v={v:+.3f} "
              f"L={rec['pupil_center_2D_L']} R={rec['pupil_center_2D_R']}", flush=True)
    reset()
    meta = {"calibration": calibration(), "frames": frames}
    with open(os.path.join(out_dir, "metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)
    return meta


# name, h, v  (None = call reset() instead of move())
DIRECTIONS = [("center", 0, 0), ("left", -1, 0), ("right", 1, 0),
              ("up", 0, 1), ("down", 0, -1), ("reset", None, None)]


def check_directions(records):
    """Pass/fail for left/right/up/down/reset vs centre, on the 2D pupil shift of BOTH eyes.
    `records` maps direction name -> state() record. Returns {name: "PASS"/"FAIL"}."""
    c = records["center"]
    s = _h_sign()
    result = {}
    for name, rec in records.items():
        if name == "center":
            continue
        ok = True
        for side in ("L", "R"):
            dx = rec[f"pupil_center_2D_{side}"][0] - c[f"pupil_center_2D_{side}"][0]
            dy = rec[f"pupil_center_2D_{side}"][1] - c[f"pupil_center_2D_{side}"][1]
            rec[f"shift_px_{side}"] = [round(dx, 3), round(dy, 3)]
            if name == "left":
                ok &= (s * dx < -0.5) and abs(dy) < abs(dx)
            elif name == "right":
                ok &= (s * dx > 0.5) and abs(dy) < abs(dx)
            elif name == "up":
                ok &= dy < -0.5 and abs(dx) < abs(dy)          # image y grows downwards
            elif name == "down":
                ok &= dy > 0.5 and abs(dx) < abs(dy)
            elif name == "reset":
                ok &= abs(dx) < 1e-3 and abs(dy) < 1e-3
        result[name] = "PASS" if ok else "FAIL"
    return result


def run_directions(out_dir=ROOT, samples=None, render_images=True):
    """Five-direction self-check. With render_images=False nothing is rendered (instant)."""
    records = {}
    for name, h, v in DIRECTIONS:
        rec = reset() if name == "reset" else move(h, v)
        if render_images:
            rec = render(os.path.join(out_dir, "directions", f"{name}.png"), samples)
        records[name] = rec
    checks = check_directions(records)
    if render_images:
        with open(os.path.join(out_dir, "directions", "directions.json"), "w") as f:
            json.dump({"calibration": calibration(), "checks": checks,
                       "records": [{"name": k, **r} for k, r in records.items()]}, f, indent=2)
    print("[eyes] direction check:", checks, flush=True)
    reset()
    return checks, records


def run_check():
    """--check: geometry-only self-test in ~2 s. Prints a table, returns True if all pass."""
    print(f"[eyes] v{__version__}  range={GAZE_RANGE_M} m  convention={CONVENTION}")
    checks, records = run_directions(render_images=False)
    for name, rec in records.items():
        if name != "center":
            print(f"  {name:6s} {checks[name]}  shift L={rec['shift_px_L']}  R={rec['shift_px_R']} px")
    print("  clockwise sequence (pupil L, px):")
    for fr in sequence_records():
        print(f"    k={fr['frame']}  h={fr['input']['h']:+.3f} v={fr['input']['v']:+.3f}  "
              f"L={fr['pupil_center_2D_L']}  R={fr['pupil_center_2D_R']}")
    closed = eyelids(1.0)
    half = eyelids(0.5)
    opened = reset()
    lids_ok = (not closed["pupil_visible_L"] and not closed["pupil_visible_R"]
               and half["pupil_visible_L"] and opened["pupil_visible_L"] and opened["eyelid_closure"] == 0.0)
    print(f"  eyelids: closed -> pupil hidden={not closed['pupil_visible_L']}, "
          f"half-open visible={half['pupil_visible_L']}, reset opens={opened['eyelid_closure'] == 0.0}  "
          f"{'PASS' if lids_ok else 'FAIL'}")
    ok = all(v == "PASS" for v in checks.values()) and lids_ok
    print("[eyes] CHECK", "OK" if ok else "FAILED", flush=True)
    return ok


# =============================================================================
# 5b. ANIMATION / VIDEOS  (--video left | right | motion | all)
# =============================================================================
def _blink(t, t0, dur=0.32):
    """Eyelid closure 0..1 for a blink starting at t0 (s): closes and reopens smoothly."""
    u = (t - t0) / dur
    return math.sin(math.pi * u) ** 2 if 0.0 <= u <= 1.0 else 0.0


def _lerp_path(pts, t):
    """Smoothstep-eased interpolation through [(t, h, v), ...] (constant before/after the ends)."""
    if t <= pts[0][0]:
        return pts[0][1], pts[0][2]
    for (t0, h0, v0), (t1, h1, v1) in zip(pts, pts[1:]):
        if t <= t1:
            f = _smoothstep(t0, t1, t)
            return h0 + (h1 - h0) * f, v0 + (v1 - v0) * f
    return pts[-1][1], pts[-1][2]


def _track(kind, t):
    """(h, v, closure) at time t (s, in the input's own default timeline) for "left", "right" or "motion".

    left / right : centre -> look to that side -> hold with tiny natural drift + blinks -> back to centre.
    motion       : "Generate Motion" - look left, sweep right, look up, roll the eyes one full clockwise
                   turn, return to centre, with blinks along the way (eyelids also follow vertical gaze)."""
    if kind in ("left", "right"):
        sgn = -1.0 if kind == "left" else 1.0
        amp = _smoothstep(0.6, 1.1, t) - _smoothstep(3.4, 3.9, t)
        h = sgn * amp + 0.03 * amp * math.sin(2 * math.pi * 1.3 * t)        # micro drift while fixating
        v = 0.025 * amp * math.sin(2 * math.pi * 0.9 * t + 1.0)
        return h, v, max(_blink(t, 2.3), _blink(t, 4.3))
    if t < 5.0:
        h, v = _lerp_path([(0.0, 0, 0), (0.8, 0, 0), (1.4, -1, 0), (2.4, -1, 0), (3.4, 1, 0),
                           (3.9, 1, 0), (4.5, 0, 1), (5.0, 0, 1)], t)
    elif t < 7.5:                                                            # eye roll, clockwise from the top
        th = 2 * math.pi * _smoothstep(5.0, 7.5, t)
        h, v = math.sin(th), math.cos(th)
    else:
        h, v = _lerp_path([(7.5, 0, 1), (8.3, 0, 0)], t)
    return h, v, max(_blink(t, 1.9), _blink(t, 4.6), _blink(t, 8.8))


def _video_kind(kind):
    k = str(kind).lower().replace("generate_motion", "motion").replace("generate motion", "motion")
    if k not in VIDEO_SECONDS:
        raise ValueError("video input must be one of: left, right, motion")
    return k


def animate(kind="motion", path=None, fps=VIDEO_FPS, seconds=None, samples=VIDEO_SAMPLES,
            scale=VIDEO_SCALE, gaze_range_m=VIDEO_RANGE_M):
    """Render an eye-motion video for one of the three inputs: "left", "right" or "motion".

    The motion is keyframed on the fixation target and the lid shape keys (so the same animation also
    plays on Blender's timeline), then rendered with Blender's own H.264 encoder -> mp4 (no external
    tools). If this Blender build has no FFMPEG output (e.g. `pip install bpy`), PNG frames are written
    and encoded with an `ffmpeg` binary if one is on PATH.

    path    default <project>/videos/<kind>.mp4        seconds  default 5 s (left/right), 10 s (motion), max 15
    samples Cycles samples (default 8)                scale    render-size percentage (50 -> 512x512)
    gaze_range_m  gaze travel for |h| = 1 in the video (default 0.10 m so the motion is clearly visible)
    Returns the path of the video (or of the PNG frame folder if it could not be encoded)."""
    kind = _video_kind(kind)
    scene = load()
    _, target, _ = _objs()
    face_keys = bpy.data.objects[FACE].data.shape_keys
    base = VIDEO_SECONDS[kind]
    dur = max(0.5, min(float(seconds) if seconds else base, VIDEO_MAX_S))
    n = max(2, int(round(dur * fps)))
    path = os.path.abspath(path or os.path.join(ROOT, "videos", f"{kind}.mp4"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    r, ims = scene.render, scene.render.image_settings
    saved = dict(fmt=ims.file_format, filepath=r.filepath, fps=r.fps, pct=r.resolution_percentage,
                 f0=scene.frame_start, f1=scene.frame_end, cur=scene.frame_current,
                 samples=scene.cycles.samples, rng=GAZE_RANGE_M)
    try:
        configure(gaze_range_m)
        # pass 1: solve every frame with the normal wrapper (move + lids), remember the result
        plan = []
        for i in range(n):
            h, v, c = _track(kind, i / fps * (base / dur))
            move(h, v, c)
            plan.append((tuple(target.location), face_keys.key_blocks[LID_KEY].value,
                         face_keys.key_blocks[CLOSE_KEY].value))
        # pass 2: keyframe them all (inserted only now, so animation can't overwrite pass 1)
        for i, (loc, gaze_lid, close_lid) in enumerate(plan):
            f = i + 1
            target.location = loc
            target.keyframe_insert("location", frame=f)
            face_keys.key_blocks[LID_KEY].value = gaze_lid
            face_keys.key_blocks[LID_KEY].keyframe_insert("value", frame=f)
            face_keys.key_blocks[CLOSE_KEY].value = close_lid
            face_keys.key_blocks[CLOSE_KEY].keyframe_insert("value", frame=f)
        scene.frame_start, scene.frame_end, r.fps = 1, n, int(fps)
        scene.frame_set(1)
        scene.cycles.samples = int(samples)
        r.resolution_percentage = int(scale)
        try:
            ims.file_format = "FFMPEG"
            r.ffmpeg.format, r.ffmpeg.codec = "MPEG4", "H264"
            r.ffmpeg.constant_rate_factor, r.ffmpeg.ffmpeg_preset = "MEDIUM", "GOOD"
            r.ffmpeg.audio_codec = "NONE"
            native = True
        except (TypeError, AttributeError):
            native = False
        if native:
            r.filepath = path
            print(f"[eyes] rendering {kind}: {n} frames @ {fps} fps ({n / fps:.1f} s) -> {path}", flush=True)
            bpy.ops.render.render(animation=True)
            if not os.path.exists(path):                    # Blender may have added a frame-range suffix
                stem = os.path.splitext(path)[0]
                made = sorted(g for g in glob.glob(stem + "*.mp4"))
                if made:
                    os.replace(made[-1], path)
            result = path
        else:
            frames = os.path.splitext(path)[0] + "_frames"
            os.makedirs(frames, exist_ok=True)
            ims.file_format = "PNG"
            r.filepath = os.path.join(frames, "f_")
            print(f"[eyes] rendering {kind}: {n} PNG frames -> {frames}", flush=True)
            bpy.ops.render.render(animation=True)
            exe = shutil.which("ffmpeg")
            if exe:
                subprocess.run([exe, "-y", "-loglevel", "error", "-framerate", str(int(fps)), "-i",
                                os.path.join(frames, "f_%04d.png"), "-pix_fmt", "yuv420p", "-c:v", "libx264",
                                path], check=True)
                shutil.rmtree(frames, ignore_errors=True)       # the mp4 is the deliverable
                result = path
            else:
                print("[eyes] no FFMPEG output in this Blender build and no ffmpeg on PATH; "
                      f"PNG frames left in {frames}", flush=True)
                result = frames
        print(f"[eyes] {kind}: done -> {result}", flush=True)
        return result
    finally:
        ims.file_format, r.filepath, r.fps = saved["fmt"], saved["filepath"], saved["fps"]
        r.resolution_percentage = saved["pct"]
        scene.frame_start, scene.frame_end = saved["f0"], saved["f1"]
        scene.cycles.samples = saved["samples"]
        target.animation_data_clear()
        face_keys.animation_data_clear()
        scene.frame_set(saved["cur"])
        configure(saved["rng"])
        reset()


# =============================================================================
# 6. CLI
# =============================================================================
def _cli_args():
    if "--" in sys.argv:                                # blender ... --python eyes.py -- ARGS
        return sys.argv[sys.argv.index("--") + 1:]
    if os.path.basename(sys.argv[0]) == "eyes.py":      # python eyes.py ARGS   (pip bpy)
        return sys.argv[1:]
    return []                                           # blender ... --python eyes.py


def main(argv=None):
    p = argparse.ArgumentParser(description="Render calibrated eye-gaze frames (Track 01).")
    p.add_argument("--blend", default=DEFAULT_BLEND, help="scene .blend (if not already open)")
    p.add_argument("--out", default=None, help="output root (default: project folder; preview/ with --preview)")
    p.add_argument("--mode", choices=["sequence", "directions", "all"], default="sequence")
    p.add_argument("--check", action="store_true", help="geometry self-test only, no rendering")
    p.add_argument("--preview", action="store_true",
                   help=f"fast {PREVIEW_SAMPLES}-sample renders into preview/ (never touches renders/)")
    p.add_argument("--samples", type=int, default=None, help="Cycles samples (default: scene's 32)")
    p.add_argument("--frames", type=int, default=N_FRAMES)
    p.add_argument("--range", type=float, default=GAZE_RANGE_M, help="target travel (m) for |h| = 1")
    p.add_argument("--convention", choices=["viewer", "subject"], default=CONVENTION)
    p.add_argument("--lid-follow", type=float, default=LID_FOLLOW,
                   help="eyelid travel vs cornea travel on vertical gaze (0 = lids frozen)")
    p.add_argument("--video", choices=["left", "right", "motion", "all"], default=None,
                   help="render an eye-motion video (mp4, <= 15 s) for this input into videos/")
    p.add_argument("--seconds", type=float, default=None, help="video length (default 5 s left/right, 10 s motion; max 15)")
    p.add_argument("--fps", type=int, default=VIDEO_FPS)
    p.add_argument("--video-scale", type=int, default=VIDEO_SCALE, help="video render size in percent (50 -> 512x512)")
    p.add_argument("--video-range", type=float, default=VIDEO_RANGE_M, help="gaze travel (m) for |h| = 1 in videos")
    a = p.parse_args(_cli_args() if argv is None else argv)

    load(a.blend)
    configure(a.range, a.convention, a.lid_follow)
    if a.check:
        sys.exit(0 if run_check() else 1)
    if a.video:
        for kind in (["left", "right", "motion"] if a.video == "all" else [a.video]):
            animate(kind, os.path.join(a.out or ROOT, "videos", f"{kind}.mp4"), a.fps, a.seconds,
                    a.samples if a.samples is not None else VIDEO_SAMPLES, a.video_scale, a.video_range)
        return
    samples = a.samples if a.samples is not None else (PREVIEW_SAMPLES if a.preview else None)
    out = a.out or (os.path.join(ROOT, "preview") if a.preview else ROOT)
    if a.mode in ("sequence", "all"):
        run_sequence(out, a.frames, samples)
    if a.mode in ("directions", "all"):
        run_directions(out, samples)
    reset()


if __name__ == "__main__":
    # Make `import eyes` / `eyes.move(...)` work in Blender's Python console after Run Script.
    sys.modules.setdefault("eyes", sys.modules[__name__])
    if bpy.app.background or _cli_args():
        main()
    else:
        load()
        import builtins
        builtins.eyes = sys.modules[__name__]
        print("[eyes] loaded. In the Python console try:  eyes.move(1, 0)   eyes.reset()   "
              "eyes.state()   eyes.render()")
