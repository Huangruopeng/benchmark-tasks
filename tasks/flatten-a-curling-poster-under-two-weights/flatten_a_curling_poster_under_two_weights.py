"""On the station table lies a poster that has been rolled up for a long time: its middle lies flat but both ends
curl up off the table, each end wrapped round a wooden hanging rail. Two 500 g brass weights stand beside it.

Task: flatten the poster under the two weights. Each curled end is pulled out flat by its rail and held down while the
other hand sets a weight on it; let go of an end with nothing on it and it curls straight back up. Pulling the first
end out drags the whole poster along unless something holds it, so the other hand presses the poster's middle down
meanwhile. Once both ends lie flat under a weight, both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=flatten_a_curling_poster_under_two_weights.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the poster: 34 cm along x (toward and away from the arms), 32 cm wide; a flat middle and two curling ends --------
CX = 0.28  # the middle of the poster, forward
WIDTH = 0.32  # across (y)
MID = 0.09  # the flat middle panel, along x
SLATS = (0.030, 0.030, 0.055)  # each end: three panels out from the middle, the last one under the rail
THICK = 0.003
CURL_DEG = 35.0  # each fold of a curled end, at rest (the end rises 105 degrees in all)
PRELOAD_DEG = 10.0  # the springs pull toward 10 degrees past that, so a curled end rests against its stop
SPRINGS = (0.0050, 0.0030, 0.0016)  # N m per degree, from the middle out
SLAT_MASS = (0.012, 0.012, 0.018)
RAIL_R, RAIL_MASS = 0.009, 0.025
FLAT_DEG = 12.0  # an end counts as flat with every panel within 12 degrees of the table ...
RAIL_DOWN = 0.006  # ... and its rail's underside within 6 mm of it

# -- two brass weights: a 4.4 cm cylinder 3.5 cm tall with a knob on top; 500 g --------------------------------------
W_R, W_H, KNOB_R, KNOB_H = 0.022, 0.035, 0.008, 0.020
W_MASS = 0.50
W_START = {"weight_near": np.array([0.21, 0.22]), "weight_far": np.array([0.21, -0.22])}

PAPER, SKY, SUN, HILL, HILL2, INK, WOOD, BRASS = (0.95, 0.94, 0.90), (0.45, 0.70, 0.90), (0.98, 0.78, 0.25), \
    (0.35, 0.62, 0.35), (0.22, 0.48, 0.30), (0.20, 0.20, 0.25), (0.60, 0.42, 0.25), (0.80, 0.65, 0.25)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _ry(deg: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a turn ``deg`` about +y."""
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def _chain(side: float, angles) -> list[tuple[np.ndarray, float]]:
    """Each end panel's inner edge (x, z) and its rise (degrees) with the folds at ``angles`` (from the middle out),
    ``side`` +1 for the far end (out along +x), -1 for the near end; and the rail's middle last."""
    x, z, rise, out = CX + side * MID / 2, 0.0, 0.0, []
    for length, a in zip(SLATS, angles):
        rise += a
        out.append((np.array([x, z]), rise))
        x += side * length * math.cos(math.radians(rise))
        z += length * math.sin(math.radians(rise))
    a = math.radians(rise)  # the rail sits on the outer edge, its underside on the panel's face
    out.append((np.array([x - side * RAIL_R * math.sin(a), z + RAIL_R * math.cos(a)]), rise))
    return out


CURLED = [CURL_DEG] * 3
FLAT = [0.0] * 3

# picture colours for the panels, from the near end to the far end (a hill landscape)
_COLOURS = {("near", 2): HILL2, ("near", 1): HILL2, ("near", 0): HILL, "mid": HILL, ("far", 0): SKY, ("far", 1): SKY,
            ("far", 2): SKY}


def _panel(name: str, side: float, k: int, inner: np.ndarray, rise: float) -> Body:
    """An end panel: its body unrotated, its origin on its inner fold line; its board tilted ``rise`` degrees."""
    length = SLATS[k]
    pitch = -side * rise  # about +y: the far end's outer edge rises with a negative turn
    c = np.array([side * length / 2 * math.cos(math.radians(rise)), 0.0, length / 2 * math.sin(math.radians(rise))])
    lift = np.array([math.sin(math.radians(pitch)) * THICK / 2, 0.0, math.cos(math.radians(pitch)) * THICK / 2])
    q = _ry(pitch)
    tag = "far" if side > 0 else "near"
    shapes = [BoxShape(tuple((c + lift).tolist()), (length, WIDTH, THICK), PAPER, quat_xyzw=q)]
    shapes.append(BoxShape(tuple((c + lift * 2.05).tolist()), (length * 0.98, WIDTH * 0.92, 0.0004),
                           _COLOURS[(tag, k)], quat_xyzw=q, collide=False))
    if tag == "far" and k == 1:  # the sun (seen only)
        shapes.append(BoxShape(tuple((c + lift * 2.3 + [0.0, 0.07, 0.0]).tolist()), (0.026, 0.026, 0.0004), SUN,
                               quat_xyzw=q, collide=False))
    if tag == "near" and k == 2:  # a title band (seen only)
        shapes.append(BoxShape(tuple((c + lift * 2.3).tolist()), (0.012, WIDTH * 0.6, 0.0004), INK, quat_xyzw=q,
                               collide=False))
    return Body.build(name, pos=(inner[0], 0.0, TOP + inner[1]), mass=SLAT_MASS[k], shapes=shapes, color=PAPER,
                      friction=(0.6, 0.5), damping=(1.0, 4.0))


def _middle() -> Body:
    shapes = [BoxShape((0.0, 0.0, THICK / 2), (MID, WIDTH, THICK), PAPER),
              BoxShape((0.0, 0.0, THICK + 0.0002), (MID * 0.98, WIDTH * 0.92, 0.0004), HILL, collide=False),
              BoxShape((-0.012, 0.03, THICK + 0.0004), (0.02, 0.05, 0.0004), HILL2, collide=False)]
    return Body.build("poster_middle", pos=(CX, 0.0, TOP), mass=0.025, shapes=shapes, color=PAPER, friction=(0.6, 0.5),
                      damping=(0.5, 2.0))


def _rail(name: str, at: np.ndarray) -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), RAIL_R, WIDTH + 0.012, WOOD, axis="Y")]
    for s in (-1.0, 1.0):  # end caps (seen only)
        shapes.append(CylShape((0.0, s * (WIDTH / 2 + 0.008), 0.0), RAIL_R * 1.15, 0.006, (0.45, 0.30, 0.18),
                               axis="Y", collide=False))
    return Body.build(name, pos=(at[0], 0.0, TOP + at[1]), mass=RAIL_MASS, shapes=shapes, color=WOOD,
                      friction=(1.0, 0.9), damping=(0.2, 1.0))


def _weight(name: str) -> Body:
    at = W_START[name]
    shapes = [CylShape((0.0, 0.0, W_H / 2), W_R, W_H, BRASS),
              CylShape((0.0, 0.0, W_H + 0.001), W_R * 0.7, 0.002, BRASS, collide=False),
              CylShape((0.0, 0.0, W_H + KNOB_H / 2), KNOB_R, KNOB_H, BRASS),
              SphereShape((0.0, 0.0, W_H + KNOB_H), KNOB_R * 1.15, BRASS)]
    return Body.build(name, pos=(at[0], at[1], TOP), mass=W_MASS, shapes=shapes, color=BRASS, friction=(0.8, 0.7),
                      com=(0.0, 0.0, 0.016))


def _ends() -> tuple[list[Body], list]:
    bodies, joints = [], []
    for side, tag in ((-1.0, "near"), (1.0, "far")):
        pts = _chain(side, CURLED)
        parent = "poster_middle"
        for k in range(3):
            inner, rise = pts[k]
            name = f"{tag}_panel_{k}"
            bodies.append(_panel(name, side, k, inner, rise))
            # each fold: 0 as curled; flattening turns the panel +35 (far) / -35 (near) about y against its spring
            lim = (-0.5, CURL_DEG + 8.0) if side > 0 else (-CURL_DEG - 8.0, 0.5)
            joints.append(dict(name=f"{tag}_fold_{k}", body0=parent, body1=name,
                               anchor=(inner[0], 0.0, TOP + inner[1]), axis="Y", limits_deg=lim, spring=SPRINGS[k],
                               damping=SPRINGS[k] * 0.02, target_deg=-side * PRELOAD_DEG))
            parent = name
        rail_at = pts[3][0]
        bodies.append(_rail(f"{tag}_rail", rail_at))
        joints.append(dict(name=f"{tag}_rail_spin", body0=parent, body1=f"{tag}_rail",
                           anchor=(rail_at[0], 0.0, TOP + rail_at[1]), axis="Y"))
    return bodies, joints


_END_BODIES, _JOINTS = _ends()
WEIGHTS = ("weight_near", "weight_far")
OBJECTS = [_middle(), *_END_BODIES, _weight("weight_near"), _weight("weight_far")]
for _j in _JOINTS:
    hinge(**_j)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("POSTER_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _end_state(base, origin, tag: str, side: float) -> dict:
    """How flat an end lies (each panel's tilt off the table, the rail's height) and whether a weight sits on it."""
    tilts = []
    for k in range(3):
        _, r = _pose(base, origin, f"{tag}_panel_{k}")
        pitch = math.radians(-side * CURL_DEG * (k + 1))  # the board's tilt in its own frame, as it starts curled
        n = r @ np.array([math.sin(pitch), 0.0, math.cos(pitch)])  # the board's face normal, in the world
        tilts.append(math.degrees(math.acos(max(-1.0, min(1.0, float(n[2]))))))
    rail, _ = _pose(base, origin, f"{tag}_rail")
    flat = max(tilts) <= FLAT_DEG and rail[2] - RAIL_R <= TOP + RAIL_DOWN
    p_out, _ = _pose(base, origin, f"{tag}_panel_2")  # the panel under the rail: its inner fold line
    weighted = None
    for w in WEIGHTS:
        wp, wr = _pose(base, origin, w)
        along = side * float(wp[0] - p_out[0])  # outward from the outer panel's inner fold (it lies flat)
        on_it = (-0.035 <= along <= SLATS[2] + 0.005 and abs(float(wp[1] - p_out[1])) <= WIDTH / 2 and
                 wp[2] <= TOP + THICK + 0.012 and float(wr[2, 2]) > math.cos(math.radians(15.0)))
        if on_it:
            weighted = w
    return {"tilt": max(tilts), "rail_z": float(rail[2] - TOP), "flat": flat, "weight": weighted}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    near = _end_state(base, origin, "near", -1.0)
    far = _end_state(base, origin, "far", 1.0)
    _EP["tcps"] = tcps = _tcps(base, origin)
    _EP["s"] = {"near": near, "far": far}
    _DEBUG["steps"] += 1
    if _DEBUG["on"] and _DEBUG["steps"] % 10 == 0:
        tl, tr = tcps
        mid, _ = _pose(base, origin, "poster_middle")
        wn, _ = _pose(base, origin, "weight_near")
        wf, _ = _pose(base, origin, "weight_far")
        print(f"[poster] t={_DEBUG['steps'] / 20:5.1f}s near tilt={near['tilt']:5.1f} rail={near['rail_z'] * 1000:5.1f}"
              f" flat={near['flat']} w={near['weight']} | far tilt={far['tilt']:5.1f} rail={far['rail_z'] * 1000:5.1f}"
              f" flat={far['flat']} w={far['weight']} | mid={np.round(mid * 1000).astype(int).tolist()} "
              f"wn={np.round(wn * 1000).astype(int).tolist()} wf={np.round(wf * 1000).astype(int).tolist()} "
              f"home={_arms_home()} L={np.round(tl * 1000).astype(int).tolist()} "
              f"R={np.round(tr * 1000).astype(int).tolist()} score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    ends = [st.get(t, {}) for t in ("near", "far")]
    held = [bool(e.get("flat")) and e.get("weight") is not None for e in ends]
    distinct = held[0] and held[1] and ends[0]["weight"] != ends[1]["weight"]
    return {"held": sum(held), "both": distinct, "home": _arms_home()}


def _points(s) -> float:
    if s["both"] and s["home"]:
        return 1.0
    if s["both"]:
        return 0.5
    return 0.25 if s["held"] >= 1 else 0.0


def score(objects) -> float:
    """Judged at the end. An end is held flat when each of its three panels lies within 12 degrees of the table, its
    rail's underside is within 6 mm of the table, and a weight stands upright (within 15 degrees) on its outer panel
    (the one under the rail, or within 3.5 cm inward of it), down on the poster. 1.0 with both ends held flat, each by
    its own weight, and both arms back where they started (each TCP within 5 cm of its start); 0.5 with both ends held
    flat; 0.25 with one end held flat; else 0.0."""
    return _points(_status())


_HOME: dict = {}


def _arms_home() -> bool:
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
    """Full score with the poster and both weights still."""
    names = ["poster_middle", "near_rail", "far_rail", *WEIGHTS] + [f"{t}_panel_{k}" for t in ("near", "far")
                                                                   for k in range(3)]
    return score(objects) >= 1.0 and all(objects[n].resting() for n in names)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand presses the poster's middle down, fingers closed. The right takes the near rail where it lies on top
# of its curl and draws it out along the unfolding path until the near end lies flat, and holds it down there. The left
# lets go of the middle, takes the near weight by its knob and sets it on the near end's outer panel. The right lets
# go. The left takes the far rail and draws it out flat the same way and holds it; the right takes the far weight and
# sets it on the far end. Both let go and go back to where they started.

L_TOP = grip_rotation(-math.pi / 2)  # left: top-down, fingers closing along x
R_TOP = grip_rotation(math.pi / 2)  # right: top-down, fingers closing along x
RAIL_OPEN = -(RAIL_R + 0.006)
KNOB_OPEN = -0.022  # about 4.4 cm between the fingers, round the 1.6 cm knob
HOLD_Y = 0.11  # where along the rail it is held (the left on +y, the right on -y)
PLACE_Y = 0.09  # where across the poster a weight goes (the other side)
PRESS = (CX, 0.08)  # where the left presses the middle down
PATH_STEPS, PATH_S = 12, 0.35
LEAD_S, LEAD_STEPS = 3.0, 15
CARRY_Z = TOP + 0.13


def _rail_path(side: float, k: int) -> np.ndarray:
    """The rail's middle (x, z) with the folds ``k``/PATH_STEPS of the way from curled to flat."""
    f = k / PATH_STEPS
    return _chain(side, [CURL_DEG * (1.0 - f)] * 3)[3][0]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _unroll(side: float, y: float, rot, overshoot: float) -> list[Waypoint]:
    """Take a curled end's rail at ``y`` and draw it out along the unfolding path, flat, pressed down."""
    p0 = _rail_path(side, 0)
    at = np.array([p0[0], y, TOP + p0[1]])
    out = [_w(1.0, at, RAIL_OPEN, "down round the rail, on top of the curl", rot),
           _w(0.6, at, FINGER_CLOSED, "take the rail", rot)]
    for k in range(1, PATH_STEPS + 1):
        p = _rail_path(side, k)
        extra = overshoot * k / PATH_STEPS
        out.append(_w(PATH_S, [p[0] + side * extra, y, TOP + p[1] - 0.005 * k / PATH_STEPS], FINGER_CLOSED,
                      "draw it out: the end unrolls flat", rot))
    return out


def _carry_weight(arm_rot, name: str, dst) -> list[Waypoint]:
    src = W_START[name]
    knob = TOP + W_H + 0.014  # the TCP: the fingertips just above the weight's top, round the knob
    dst_z = knob + THICK + 0.004
    return [_w(1.8, [src[0], src[1], knob + 0.06], KNOB_OPEN, f"over the {name.replace('_', ' ')}'s knob", arm_rot),
            _w(1.4, [src[0], src[1], knob], KNOB_OPEN, "down round the knob", arm_rot),
            _w(0.4, [src[0], src[1], knob], KNOB_OPEN, "settle", arm_rot),
            _w(0.6, [src[0], src[1], knob], FINGER_CLOSED, "take it", arm_rot),
            _w(1.0, [src[0], src[1], CARRY_Z], FINGER_CLOSED, "lift it", arm_rot),
            _w(2.0, [dst[0], dst[1], CARRY_Z], FINGER_CLOSED, "carry it over the end's outer panel", arm_rot),
            _w(1.2, [dst[0], dst[1], dst_z], FINGER_CLOSED, "set it down on the poster", arm_rot),
            _w(0.6, [dst[0], dst[1], dst_z], KNOB_OPEN, "let go", arm_rot),
            _w(0.8, [dst[0], dst[1], dst_z + 0.06], KNOB_OPEN, "rise", arm_rot)]


def _outer_mid(side: float) -> float:
    """x of the middle of an end's outer panel when it lies flat."""
    return CX + side * (MID / 2 + SLATS[0] + SLATS[1] + SLATS[2] / 2)


def demo():
    # right: unroll the near end and hold it
    p0 = _rail_path(-1.0, 0)
    right = _lead_in("right", [p0[0], -HOLD_Y, TOP + p0[1] + 0.05], R_TOP, "over the near rail", RAIL_OPEN)
    L_PRESS = grip_rotation(0.0)  # fingers closing along y: the closed gripper is thinnest along x, between the curls
    left = _lead_in("left", [PRESS[0], PRESS[1], TOP + 0.19], L_PRESS, "high over the poster's middle", FINGER_CLOSED)
    left += [_w(1.4, [PRESS[0], PRESS[1], TOP + 0.04], FINGER_CLOSED, "down between the curled ends", L_PRESS),
             _w(0.8, [PRESS[0], PRESS[1], TOP + THICK + 0.012], FINGER_CLOSED, "press the middle down", L_PRESS)]
    t_pressed = sum(w.duration for w in left)
    t_r = sum(w.duration for w in right)
    if t_pressed > t_r:
        right.append(_w(t_pressed - t_r, right[-1].pos, RAIL_OPEN, "wait for the middle to be held", R_TOP))
    right += _unroll(-1.0, -HOLD_Y, R_TOP, 0.006)
    near_flat_at = sum(w.duration for w in right)
    right.append(_w(0.5, right[-1].pos, FINGER_CLOSED, "hold the near end flat", R_TOP))
    left.append(_w(max(near_flat_at - sum(w.duration for w in left), 0.3), left[-1].pos, FINGER_CLOSED,
                   "keep the middle down while the near end is drawn out", L_PRESS))
    left += [_w(1.6, [PRESS[0], PRESS[1], TOP + 0.19], FINGER_CLOSED, "rise straight up, clear of the curled end",
                L_PRESS)]
    left += _carry_weight(L_TOP, "weight_near", (_outer_mid(-1.0) + 0.006, PLACE_Y))
    w1_down = sum(w.duration for w in left) - 1.4
    right.append(_w(max(w1_down - sum(w.duration for w in right), 0.3), right[-1].pos, FINGER_CLOSED,
                    "hold it flat until the weight is on it", R_TOP))
    held = right[-1].pos
    right += [_w(0.6, held, RAIL_OPEN, "let go of the rail", R_TOP),
              _w(0.8, [held[0], held[1], held[2] + 0.07], RAIL_OPEN, "rise", R_TOP),
              _w(1.5, [0.30, -0.20, TOP + 0.13], RAIL_OPEN, "out of the way", R_TOP)]
    # left: unroll the far end and hold it; the right sets the far weight on it
    p0 = _rail_path(1.0, 0)
    up = np.array(left[-1].pos)
    left += [_w(0.8, [up[0], up[1], TOP + 0.19], RAIL_OPEN, "up", L_TOP),
             _w(1.6, [p0[0], HOLD_Y, TOP + 0.19], RAIL_OPEN, "high over the far rail", L_TOP),
             _w(1.0, [p0[0], HOLD_Y, TOP + p0[1] + 0.04], RAIL_OPEN, "down over it", L_TOP)]
    left += _unroll(1.0, HOLD_Y, L_TOP, 0.006)
    far_flat_at = sum(w.duration for w in left)
    left.append(_w(0.5, left[-1].pos, FINGER_CLOSED, "hold the far end flat", L_TOP))
    right.append(_w(max(far_flat_at - 6.0 - sum(w.duration for w in right), 0.3), right[-1].pos, RAIL_OPEN,
                    "wait", R_TOP))
    right += _carry_weight(R_TOP, "weight_far", (_outer_mid(1.0) - 0.006, -PLACE_Y))
    w2_down = sum(w.duration for w in right) - 1.4
    left.append(_w(max(w2_down - sum(w.duration for w in left), 0.3), left[-1].pos, FINGER_CLOSED,
                   "hold it flat until the weight is on it", L_TOP))
    held = left[-1].pos
    left += [_w(0.6, held, RAIL_OPEN, "let go of the rail", L_TOP),
             _w(0.8, [held[0], held[1], held[2] + 0.07], RAIL_OPEN, "rise", L_TOP)]
    left += _go_home("left", left[-1].pos, L_TOP, FINGER_OPEN)
    right += _go_home("right", right[-1].pos, R_TOP, FINGER_OPEN)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
