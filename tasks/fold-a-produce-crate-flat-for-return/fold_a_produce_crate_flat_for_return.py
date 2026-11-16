"""An empty green folding produce crate (a returnable plastic container, 400 x 300 x 174 mm, 0.88 kg) stands up on the
returns bench, its 400 mm length pointing away from the robot, so one short end wall faces the robot and the other
faces away.

The crate folds the way such crates do: each short end wall is latched to the two long side walls by a sprung catch at
each of its top corners. Pressing a catch's thumb button in frees that corner; with both buttons of an end wall
pressed together the wall can be tipped in, and it folds down onto the base. Only once both end walls are down are the
long walls free: each is then pushed in over its stop and folds down on top of the end walls. Folded, the crate is
8 cm tall.

Task: fold the empty crate flat for the return trip, and finish with both arms back where they started.

What the physics asks for: an end wall's two buttons are 20 cm apart, too far for one hand, and while either catch is
latched the wall stays up. A button pressed from outside alone shoves the 0.88 kg crate along the bench (it slides at
about 3 N); squeezing the button and the wall's top rail together in one hand holds it in without moving the crate.

Run it:  inspect-robots run --embodiment yam_arena -E task=fold_a_produce_crate_flat_for_return.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the crate (crate frame: origin at the middle of the base's underside, x along its 400 mm length, z up) ------------
CX, CY = 0.230, 0.000  # where it stands on the bench
HL, HW, H = 0.200, 0.150, 0.174  # half length, half width, height
T = 0.010  # walls and rims are 10 mm
FLOOR_T = 0.006
SIDE_RIM_H, END_RIM_H = 0.068, 0.048  # the long walls hinge on the side rims' tops, the end walls on the end rims'
EW_H = H - END_RIM_H  # end wall 126 mm tall
EW_HW = 0.135  # end wall 270 mm wide (5 mm clear of the long walls each side)
LW_H = H - SIDE_RIM_H  # long wall 106 mm tall
LW_HL = 0.198  # long wall 396 mm long
EW_RAIL_R, LW_RAIL_R = 0.008, 0.007  # round top rails: 16 mm on the end walls, 14 mm on the long walls
SLOT_HW, SLOT_Z0, SLOT_Z1 = 0.060, 0.072, 0.108  # the end walls' 120 x 36 mm handhold (pivot frame)
BTN_Y = 0.100  # the catches' thumb buttons: 100 mm either side of the middle, on the top rail's outer face
BTN_T, BTN_W, BTN_H = 0.006, 0.016, 0.016
BTN_TRAVEL = 0.006  # a button stands 6 mm proud; pushed flush it is fully in
BTN_FREE = 0.0035  # pressed this far in, a catch's bolt is out of its socket in the long wall
BTN_K, BTN_PRELOAD = 1000.0, 0.003  # 3 N at rest, 9 N pressed flush
MASS_BASE, MASS_END, MASS_LONG, MASS_BTN = 0.40, 0.09, 0.13, 0.01  # 0.88 kg in all

# end walls: "far" (+x, away from the robot) and "near" (-x); long walls "left" (+y) and "right" (-y)
ENDS = {"far": +1.0, "near": -1.0}
SIDES = {"left": +1.0, "right": -1.0}
FOLD = 90.0  # degrees from upright to lying flat

# catch and lock springs (hinge drives, N m per degree)
K_LATCH, D_LATCH = 1.2, 0.002  # a latched end wall (or a long wall held by a latched end wall): 3 deg takes ~30 N
D_FREE = 2.0e-4  # a free wall swings against light hinge friction
D_FLAT = 0.002  # a wall lying flat settles
FORCE_DEG = 3.0  # a latched wall pushed in this far has had its catch forced (broken)
RELATCH_DEG = 1.0  # an end wall swung back up to within this, its buttons out, latches again
FLAT_DEG = 5.0  # a wall within this of lying flat is folded
HEIGHT_MAX = 0.090  # the folded crate is 80 mm tall; full score needs it under 90 mm
LEVEL_DEG = 3.0
FALLEN_Z = 0.10
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

GREEN = (0.16, 0.50, 0.25)
DARK = (0.05, 0.17, 0.08)
BTN_GREEN = (0.10, 0.34, 0.16)
FLOOR_SLOT = (0.07, 0.24, 0.11)


def _slots(face_x, y0, y1, z0, z1, n, normal, w=None, depth=0.001):
    """A row of n dark vent slots on a wall face (visual only): along y between y0..y1 at heights z0..z1, the face at
    ``face_x`` along ``normal`` ("x" or "y")."""
    out = []
    pitch = (y1 - y0) / n
    sw = w if w is not None else pitch * 0.6
    for i in range(n):
        c = y0 + pitch * (i + 0.5)
        if normal == "x":
            out.append(BoxShape((face_x, c, (z0 + z1) / 2), (depth, sw, z1 - z0), DARK, collide=False))
        else:
            out.append(BoxShape((c, face_x, (z0 + z1) / 2), (sw, depth, z1 - z0), DARK, collide=False))
    return out


def _base() -> Body:
    shapes = [
        BoxShape((0.0, 0.0, FLOOR_T / 2), (2 * HL, 2 * HW, FLOOR_T), GREEN),
        BoxShape((0.0, HW - T / 2, SIDE_RIM_H / 2), (2 * HL, T, SIDE_RIM_H), GREEN),
        BoxShape((0.0, -(HW - T / 2), SIDE_RIM_H / 2), (2 * HL, T, SIDE_RIM_H), GREEN),
        BoxShape((HL - T / 2, 0.0, END_RIM_H / 2), (T, 2 * (HW - T), END_RIM_H), GREEN),
        BoxShape((-(HL - T / 2), 0.0, END_RIM_H / 2), (T, 2 * (HW - T), END_RIM_H), GREEN),
    ]
    # the floor's moulded open lattice (a grid of small slots between ribs) and the rims' vents (visual)
    nx, ny = 14, 9
    px, py = (2 * (HL - T) - 0.024) / nx, (2 * (HW - T) - 0.024) / ny
    for i in range(nx):
        for j in range(ny):
            x = -(HL - T) + 0.012 + px * (i + 0.5)
            y = -(HW - T) + 0.012 + py * (j + 0.5)
            shapes.append(BoxShape((x, y, FLOOR_T + 0.0005), (px * 0.62, py * 0.62, 0.001), FLOOR_SLOT, collide=False))
    for s in (+1, -1):
        shapes += _slots(s * (HW + 0.0005), -HL + 0.03, HL - 0.03, 0.022, 0.050, 8, "y")
        shapes += _slots(s * (HW - T - 0.0005), -HL + 0.03, HL - 0.03, 0.022, 0.050, 8, "y")
    return Body.build("crate_base", (CX, CY, TOP), MASS_BASE, shapes, color=GREEN, friction=(0.35, 0.35),
                      slick=True, solver_iterations=32)


def _end_pivot(e: str) -> np.ndarray:
    s = ENDS[e]
    return np.array([CX + s * (HL - T), CY, TOP + END_RIM_H])


def _end_wall(e: str) -> Body:
    """Origin on the hinge line (the end rim's inner top edge); the wall stands outboard of it."""
    s = ENDS[e]
    xc = s * T / 2
    shapes = [
        BoxShape((xc, 0.0, SLOT_Z0 / 2), (T, 2 * EW_HW, SLOT_Z0), GREEN),
        BoxShape((xc, (SLOT_HW + EW_HW) / 2, (SLOT_Z0 + SLOT_Z1) / 2), (T, EW_HW - SLOT_HW, SLOT_Z1 - SLOT_Z0), GREEN),
        BoxShape((xc, -(SLOT_HW + EW_HW) / 2, (SLOT_Z0 + SLOT_Z1) / 2), (T, EW_HW - SLOT_HW, SLOT_Z1 - SLOT_Z0), GREEN),
        BoxShape((xc, 0.0, (SLOT_Z1 + EW_H - EW_RAIL_R) / 2), (T, 2 * EW_HW, EW_H - EW_RAIL_R - SLOT_Z1), GREEN),
        CylShape((xc, 0.0, EW_H - EW_RAIL_R), EW_RAIL_R, 2 * EW_HW, GREEN, axis="Y"),
    ]
    for f in (+1, -1):  # vents on both faces of the lower panel
        shapes += _slots(xc + f * (T / 2 + 0.0005), -EW_HW + 0.02, EW_HW - 0.02, 0.015, 0.055, 6, "x")
    p = _end_pivot(e)
    return Body.build(f"end_{e}", p, MASS_END, shapes, color=GREEN, friction=(0.45, 0.4), solver_iterations=32)


def _btn_local(e: str, side: float) -> np.ndarray:
    """A button's centre at rest, in its end wall's frame."""
    s = ENDS[e]
    return np.array([s * (T / 2 + EW_RAIL_R + BTN_T / 2), side * BTN_Y, EW_H - EW_RAIL_R])


def _button(e: str, k: str) -> Body:
    side = SIDES[k]
    s = ENDS[e]
    shapes = [BoxShape((0.0, 0.0, 0.0), (BTN_T, BTN_W, BTN_H), BTN_GREEN),
              BoxShape((s * BTN_T / 2, 0.0, 0.0), (0.001, BTN_W * 0.7, 0.002), DARK, collide=False)]
    return Body.build(f"btn_{e}_{k}", _end_pivot(e) + _btn_local(e, side), MASS_BTN, shapes, color=BTN_GREEN,
                      friction=(0.6, 0.5), solver_iterations=32)


def _side_pivot(k: str) -> np.ndarray:
    return np.array([CX, CY + SIDES[k] * (HW - T), TOP + SIDE_RIM_H])


def _long_wall(k: str) -> Body:
    s = SIDES[k]
    yc = s * T / 2
    shapes = [
        BoxShape((0.0, yc, (LW_H - LW_RAIL_R) / 2), (2 * LW_HL, T, LW_H - LW_RAIL_R), GREEN),
        CylShape((0.0, yc, LW_H - LW_RAIL_R), LW_RAIL_R, 2 * LW_HL, GREEN, axis="X"),
    ]
    for f in (+1, -1):
        for z0, z1 in ((0.012, 0.040), (0.050, 0.078)):
            shapes += _slots(yc + f * (T / 2 + 0.0005), -LW_HL + 0.025, LW_HL - 0.025, z0, z1, 9, "y")
    return Body.build(f"side_{k}", _side_pivot(k), MASS_LONG, shapes, color=GREEN, friction=(0.45, 0.4),
                      solver_iterations=32)


_HINGE_LIMITS = {"far": (-FOLD, 0.0), "near": (0.0, FOLD), "left": (0.0, FOLD), "right": (-FOLD, 0.0)}
for _e in ENDS:
    hinge(f"h_{_e}", "crate_base", f"end_{_e}", tuple(_end_pivot(_e).tolist()), axis="Y",
          limits_deg=_HINGE_LIMITS[_e], spring=K_LATCH, damping=D_LATCH, target_deg=0.0)
for _k in SIDES:
    hinge(f"h_{_k}", "crate_base", f"side_{_k}", tuple(_side_pivot(_k).tolist()), axis="X",
          limits_deg=_HINGE_LIMITS[_k], spring=K_LATCH, damping=D_LATCH, target_deg=0.0)
for _e, _s in ENDS.items():
    for _k, _side in SIDES.items():
        # a button slides in along the wall's normal (-s); its spring's target sits beyond its outer stop (preload)
        lim = (-BTN_TRAVEL, 0.0) if _s > 0 else (0.0, BTN_TRAVEL)
        slider(f"sl_{_e}_{_k}", f"end_{_e}", f"btn_{_e}_{_k}", tuple((_end_pivot(_e) + _btn_local(_e, _side)).tolist()),
               axis="X", limits=lim, stiffness=BTN_K, damping=2.0, target=_s * BTN_PRELOAD, max_force=20.0)

OBJECTS = [_base(), _end_wall("far"), _end_wall("near"), _long_wall("left"), _long_wall("right")] + \
    [_button(e, k) for e in ENDS for k in SIDES]
WALLS = ["end_far", "end_near", "side_left", "side_right"]
MOVING = ["crate_base"] + WALLS


# -- followed after every control step ----------------------------------------------------------------------------

def _fresh() -> dict:
    return {"latched": {e: True for e in ENDS}, "locked": {k: True for k in SIDES}, "flat_set": set(),
            "forced": False, "fallen": False, "s": {}, "tcps": None, "max_press": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CRATE_DEBUG")), "steps": 0}


def _drive(name: str, spring: float, damping: float) -> None:
    bodykit.set_hinge_drive(name, target_deg=0.0, spring=spring, damping=damping)


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for e in ENDS:
        _drive(f"h_{e}", K_LATCH, D_LATCH)
    for k in SIDES:
        _drive(f"h_{k}", K_LATCH, D_LATCH)


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _fold_deg(name: str, rb, rw) -> float:
    """How far a wall has turned in from upright (degrees; 90 = lying flat), from the base's and the wall's poses."""
    r = rb.T @ rw
    if name.startswith("end_"):
        a = math.degrees(math.atan2(float(r[0, 2]), float(r[0, 0])))  # about the crate's y axis
        return -a if name == "end_far" else a
    a = math.degrees(math.atan2(float(r[2, 1]), float(r[1, 1])))  # about the crate's x axis
    return a if name == "side_left" else -a


@on_step
def _follow(env) -> None:
    try:
        _follow_inner(env)
    except Exception as exc:  # noqa: BLE001  (never let the follower end a trial)
        if not _EP.get("err"):
            _EP["err"] = repr(exc)[:300]
            print(f"[crate] follower error: {_EP['err']}", file=sys.stderr, flush=True)


def _follow_inner(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    e = _EP
    pb, rb = _pose(base, origin, "crate_base")
    fold, poses = {}, {}
    for w in WALLS:
        pw, rw = _pose(base, origin, w)
        poses[w] = (pw, rw)
        fold[w] = _fold_deg(w, rb, rw)
    press = {}
    for end, s in ENDS.items():
        pw, rw = poses[f"end_{end}"]
        for k, side in SIDES.items():
            pbtn, _ = _pose(base, origin, f"btn_{end}_{k}")
            local = rw.T @ (pbtn - pw)
            press[(end, k)] = float(-s * (local[0] - _btn_local(end, side)[0]))
            e["max_press"][(end, k)] = max(e["max_press"].get((end, k), 0.0), press[(end, k)])
    # the catches: both buttons of an end wall pressed in together free it; swung back up with them out, it latches
    for end in ENDS:
        f = fold[f"end_{end}"]
        both = all(press[(end, k)] >= BTN_FREE for k in SIDES)
        if e["latched"][end]:
            if f > FORCE_DEG:
                e["forced"] = True
            if both:
                e["latched"][end] = False
                _drive(f"h_{end}", 0.0, D_FREE)
        elif f < RELATCH_DEG and all(press[(end, k)] < 0.001 for k in SIDES):
            e["latched"][end] = True
            e["flat_set"].discard(f"end_{end}")
            _drive(f"h_{end}", K_LATCH, D_LATCH)
    # a long wall is held up by the catches of any latched end wall
    any_latched = any(e["latched"].values())
    for k in SIDES:
        f = fold[f"side_{k}"]
        if e["locked"][k]:
            if f > FORCE_DEG:
                e["forced"] = True
            if not any_latched:
                e["locked"][k] = False
                _drive(f"h_{k}", 0.0, D_FREE)
        elif any_latched and f < RELATCH_DEG:
            e["locked"][k] = True
            e["flat_set"].discard(f"side_{k}")
            _drive(f"h_{k}", K_LATCH, D_LATCH)
    # a free wall lying flat settles (more hinge friction), and frees up again if lifted
    for w in WALLS:
        name = "h_" + w.split("_", 1)[1]
        free = not (e["latched"].get(w[4:], False) if w.startswith("end_") else e["locked"][w[5:]])
        if not free:
            continue
        if fold[w] > FOLD - 3.0 and w not in e["flat_set"]:
            e["flat_set"].add(w)
            _drive(name, 0.0, D_FLAT)
        elif fold[w] < FOLD - 10.0 and w in e["flat_set"]:
            e["flat_set"].discard(w)
            _drive(name, 0.0, D_FREE)
    # the folded height: the highest point of any rail or rim above the bench
    tops = [pb[2] + (rb @ np.array([0.0, 0.0, SIDE_RIM_H]))[2]]
    for end, s in ENDS.items():
        pw, rw = poses[f"end_{end}"]
        c = pw + rw @ np.array([s * T / 2, 0.0, EW_H - EW_RAIL_R])
        tops.append(c[2] + EW_RAIL_R)
    for k, s in SIDES.items():
        pw, rw = poses[f"side_{k}"]
        c = pw + rw @ np.array([0.0, s * T / 2, LW_H - LW_RAIL_R])
        tops.append(c[2] + LW_RAIL_R)
    height = max(tops) - TOP
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rb[2, 2])))))
    if pb[2] < TOP - FALLEN_Z:
        e["fallen"] = True
    flat = {w: abs(fold[w] - FOLD) <= FLAT_DEG for w in WALLS}
    e["tcps"] = _tcps(base, origin)
    e["s"] = {"fold": fold, "flat": flat, "height": height, "tilt": tilt, "on_bench": abs(pb[2] - TOP) < 0.01}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, TOP]) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = e["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            print(f"[crate] t={_DEBUG['steps'] / 20:5.2f}s fold={ {w[-4:]: round(v, 1) for w, v in fold.items()} }"
                  f" press={ {f'{a[0][0]}{b[0]}': round(v * 1000, 1) for (a, b), v in press.items()} }"
                  f" latched={e['latched']} locked={e['locked']} forced={e['forced']} base={mm(pb)}"
                  f" tilt={tilt:.1f} h={height * 1000:.0f}mm L={mm(tl)} R={mm(tr)} fingers={[round(v, 1) for v in fj]}"
                  f" home={_arms_home()} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 for good once the crate has fallen off the bench or a catch has been forced (a latched wall pushed in more
    than 3 degrees). Otherwise, at the end: 1.0 with all four walls lying flat (within 5 degrees), the long walls over
    the end walls, the crate under 90 mm tall, its base flat on the bench and both arms back where they started; else
    0.2 for each end wall lying flat plus 0.2 for each long wall lying flat over both flat end walls (at most 0.8)."""
    e = _EP
    s = e["s"]
    if e["forced"] or e["fallen"] or not s:
        return 0.0
    flat = s["flat"]
    ends = flat["end_far"] and flat["end_near"]
    part = 0.2 * (flat["end_far"] + flat["end_near"])
    if ends:
        part += 0.2 * (flat["side_left"] + flat["side_right"])
    if (part >= 0.8 - 1e-9 and s["height"] < HEIGHT_MAX and s["tilt"] < LEVEL_DEG and s["on_bench"]
            and _arms_home()):
        return 1.0
    return round(min(part, 0.8), 3)


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the crate still: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in MOVING)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands go to the near end wall's top corners and pinch its top rail with the thumb button under the outer
# finger: the squeeze presses both buttons in without pushing the crate along. Holding them in, both hands tip the wall
# a few degrees in, let go, and it drops flat. The same at the far end wall (the grips lean forward to reach it). With
# both end walls down the long walls are free: each hand pushes its long wall in over its stop and it drops flat on
# top. Both arms go back to where they started.

def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


LEAN = -15.0  # the pinching grips lean forward (fingers toward +x) 15 degrees
PINCH_FINGER = -0.020  # pre-open a little wider than rail + button (22 mm)
PINCH_DZ = 0.006  # TCP this far above the rail's axis
TIP_DX, TIP_DZ = 0.030, -0.004  # holding the buttons in, tip the wall in this far at its rail
OVER = 0.030  # clearance above a rail before descending (the reach ceiling is about TOP+0.21)
PUSH_Z = 0.150  # the long walls are pushed in at this height above the bench
PUSH_FROM, PUSH_TO = 0.175, 0.095  # |y| of the pushing TCP: start outside the wall, end well over its stop


def _pinch_rot(arm: str) -> np.ndarray:
    return _ry(LEAN) @ grip_rotation(-math.pi / 2 if arm == "left" else math.pi / 2)


def _push_rot(arm: str) -> np.ndarray:
    return grip_rotation(-math.pi / 2 if arm == "left" else math.pi / 2)


def _end_wall_steps(arm: str, end: str) -> list:
    s = ENDS[end]
    y = SIDES[arm] * BTN_Y
    x = CX + s * (HL - T / 2)
    z = TOP + H - EW_RAIL_R + PINCH_DZ
    r = _pinch_rot(arm)
    return [
        Waypoint(2.0, (x, y, z + OVER), PINCH_FINGER, f"over {end} corner", r),
        Waypoint(1.5, (x, y, z), PINCH_FINGER, "down round the rail", r),
        Waypoint(1.0, (x, y, z), FINGER_CLOSED, "squeeze the button in", r),
        Waypoint(0.6, (x, y, z), FINGER_CLOSED, "hold", r),
        Waypoint(1.5, (x - s * TIP_DX, y, z + TIP_DZ), FINGER_CLOSED, "tip the wall in", r),
        Waypoint(0.8, (x - s * TIP_DX, y, z + TIP_DZ), PINCH_FINGER, "let go", r),
        Waypoint(1.2, (x - s * TIP_DX, y, z + OVER), PINCH_FINGER, "up", r),
    ]


def _arm_plan(arm: str) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    home_p, home_r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    home_p = tuple(float(v) for v in home_p)
    sy = SIDES[arm]
    pr = _push_rot(arm)
    steps = _end_wall_steps(arm, "near") + _end_wall_steps(arm, "far")
    steps += [
        Waypoint(2.0, (CX, sy * PUSH_FROM, TOP + PUSH_Z + 0.04), FINGER_CLOSED, "over the long wall, outside", pr),
        Waypoint(1.2, (CX, sy * PUSH_FROM, TOP + PUSH_Z), FINGER_CLOSED, "down beside it", pr),
        Waypoint(2.0, (CX, sy * PUSH_TO, TOP + PUSH_Z), FINGER_CLOSED, "push it in over its stop", pr),
        Waypoint(1.0, (CX, sy * PUSH_TO, TOP + PUSH_Z + 0.04), FINGER_CLOSED, "up", pr),
        Waypoint(2.5, home_p, FINGER_CLOSED, "home", home_r),
        Waypoint(4.0, home_p, FINGER_CLOSED, "wait", home_r),
    ]
    return steps


def demo():
    return {"left": _arm_plan("left"), "right": _arm_plan("right")}
