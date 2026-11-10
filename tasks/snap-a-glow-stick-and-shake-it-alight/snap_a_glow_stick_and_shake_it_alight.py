"""On the station table lies a 30 cm emergency glow stick in its cardboard packet tray, pale and unlit: a clear plastic
tube with a thin glass vial inside it. Further out stands a long black slate tray.

Task: light the glow stick. Pick it up out of its packet tray with both hands, one near each end, and bend it in the
middle until the glass vial inside snaps (a bright spot appears where the two liquids first mix); straighten it again,
shake it back and forth three times so that it lights up all along, then lay it in the black tray and take both arms
back to where they started. Bending takes a hand on each half: one hand alone just carries the stick along, straight.

Run it:  inspect-robots run --embodiment yam_arena -E task=snap_a_glow_stick_and_shake_it_alight.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the glow stick: two halves on a stiff hinge at its middle (the tube bends there), lying along y ----------------
R = 0.0095  # tube radius (19 mm across)
HALF_L = 0.150  # each half's length (30 cm overall)
SX, SY = 0.280, 0.000  # the middle of the stick in its packet
SZ = TOP + 0.0145 + R  # its axis, lying in the packet (the packet's floor 14.5 mm above the table)
MID = np.array([SX, SY, SZ])
HALF_MASS = 0.020
BEND_SPRING = 0.003  # N m per degree: the tube's stiffness, pulling it back straight
BEND_DAMPING = 2.0e-5
SNAP_DEG = 30.0  # bent this far (the two halves' axes this far apart): the glass vial inside snaps
# -- the shake rule ----------------------------------------------------------------------------------------------------
STROKE = 0.025  # a shake stroke: the stick's middle moves at least this far one way ...
TURN = 0.008  # ... then at least this far back
STROKES_TO_LIGHT = 6  # three shakes (back and forth) after the snap light it all along
# -- the black slate tray --------------------------------------------------------------------------------------------
TX0, TX1, TY0, TY1 = 0.350, 0.440, -0.200, 0.200
T_FLOOR, T_RIM_H, T_RIM_W = 0.006, 0.012, 0.010
IN_TRAY_Z = TOP + T_FLOOR + R + 0.012  # a half lies in the tray: its middle at most this high, over the floor
# -- the packet tray ---------------------------------------------------------------------------------------------------
PX0, PX1, PY0, PY1 = SX - 0.030, SX + 0.030, -0.175, 0.175
P_FLOOR = 0.0145  # the cradle's top
CARD_T = 0.003
RAIL_GAP = R + 0.003  # the rails' inner faces this far either side of the stick's axis
RAIL_Y = 0.060  # the rails run only this far either side of the middle (the hands take the stick outside them)

PARK = (0.40, 0.0, 0.30)  # the glow sleeves wait here, under the table, until the stick lights
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

TUBE, TUBE_DARK, CAP_W, CAP_G = (0.82, 0.88, 0.66), (0.70, 0.78, 0.55), (0.93, 0.93, 0.92), (0.60, 0.62, 0.63)
GLOW, SPARK = (0.36, 1.00, 0.22), (0.80, 1.00, 0.55)
CARD, CARD_DARK, CARD_INK = (0.20, 0.36, 0.70), (0.14, 0.26, 0.52), (0.95, 0.80, 0.15)
SLATE, SLATE_EDGE = (0.10, 0.10, 0.11), (0.17, 0.17, 0.18)


def _packet() -> Body:
    """The cardboard packet tray: a flat blue card with a raised cradle at its middle that the stick lies on, and two
    short rails along the cradle that stop it rolling (under the stick's ends the card is flat, clear of the fingers)."""
    cx, cy = (PX0 + PX1) / 2, (PY0 + PY1) / 2
    lx, ly = PX1 - PX0, PY1 - PY0
    shapes = [BoxShape((0.0, 0.0, CARD_T / 2), (lx, ly, CARD_T), CARD),
              BoxShape((0.0, 0.0, P_FLOOR / 2), (lx, 2 * RAIL_Y, P_FLOOR), CARD_DARK)]
    for s in (-1.0, 1.0):
        x = s * (RAIL_GAP + 0.004)
        shapes.append(BoxShape((x, 0.0, P_FLOOR + 0.006), (0.008, 2 * RAIL_Y, 0.012), CARD_DARK))
        shapes.append(BoxShape((s * (lx / 2 - 0.004), 0.0, CARD_T + 0.0004), (0.004, ly - 0.01, 0.0008), CARD_INK,
                               collide=False))
    for y in (-0.135, 0.135):  # printed bands at the card's ends
        shapes.append(BoxShape((0.0, y, CARD_T + 0.0004), (lx - 0.012, 0.018, 0.0008), CARD_INK, collide=False))
    return Body.build("packet", pos=(cx, cy, TOP), mass=0.05, kinematic=True, color=CARD, shapes=shapes,
                      friction=(0.7, 0.6))


def _tray() -> Body:
    """A long black slate serving tray with a low rim."""
    cx, cy = (TX0 + TX1) / 2, (TY0 + TY1) / 2
    lx, ly = TX1 - TX0, TY1 - TY0
    shapes = [BoxShape((0.0, 0.0, T_FLOOR / 2), (lx, ly, T_FLOOR), SLATE)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx - T_RIM_W) / 2, 0.0, T_RIM_H / 2), (T_RIM_W, ly, T_RIM_H), SLATE_EDGE))
        shapes.append(BoxShape((0.0, s * (ly - T_RIM_W) / 2, T_RIM_H / 2), (lx - 2 * T_RIM_W, T_RIM_W, T_RIM_H),
                               SLATE_EDGE))
    return Body.build("tray", pos=(cx, cy, TOP), mass=1.0, kinematic=True, color=SLATE, shapes=shapes,
                      friction=(0.8, 0.7))


def _half(name: str, s: float) -> Body:
    """One half of the glow stick, its frame at the stick's middle: a clear tube (pale with the unlit liquid), the
    glass vial's shadow inside, and the white end cap (with the hanging loop on the left half). ``s`` = +1: the left
    half (toward +y); -1: the right half."""
    shapes = [CylShape((0.0, s * HALF_L / 2, 0.0), R, HALF_L, TUBE, axis="Y"),
              CylShape((0.0, s * (HALF_L / 2 - 0.004), 0.0), 0.0035, HALF_L - 0.03, TUBE_DARK, collide=False,
                       axis="Y"),
              CylShape((0.0, s * (HALF_L - 0.006), 0.0), R + 0.0012, 0.012, CAP_W, axis="Y"),
              CylShape((0.0, s * (HALF_L + 0.0015), 0.0), R * 0.7, 0.003, CAP_W, collide=False, axis="Y")]
    if s > 0:  # the hanging loop on the left end cap
        shapes.append(CylShape((0.0, HALF_L + 0.008, 0.0), 0.006, 0.003, CAP_G, collide=False, axis="X"))
    return Body.build(name, pos=tuple(MID.tolist()), mass=HALF_MASS, shapes=shapes, color=TUBE, friction=(1.2, 1.1),
                      damping=(0.0, 0.2))


def _sleeve(name: str, s: float, length: float, radius: float, color) -> Body:
    """What the lit liquid looks like, drawn over a half of the stick (in the half's own frame): parked under the
    table, then moved onto the half after every control step once it shows."""
    shapes = [CylShape((0.0, s * length / 2, 0.0), radius, length, color, collide=False, axis="Y")]
    return Body.build(name, pos=PARK, mass=0.01, kinematic=True, color=color, shapes=shapes)


BEND = hinge("bend", "stick_r", "stick_l", anchor=tuple(MID.tolist()), axis="Z", limits_deg=(-60.0, 60.0),
             spring=BEND_SPRING, damping=BEND_DAMPING, target_deg=0.0)

GLOW_LEN = HALF_L - 0.012  # the liquid fills the tube up to the end caps
OBJECTS = [_packet(), _tray(), _half("stick_r", -1.0), _half("stick_l", 1.0),
           _sleeve("spark_r", -1.0, 0.035, R + 0.0007, SPARK), _sleeve("spark_l", 1.0, 0.035, R + 0.0007, SPARK),
           _sleeve("glow_r", -1.0, GLOW_LEN, R + 0.0006, GLOW), _sleeve("glow_l", 1.0, GLOW_LEN, R + 0.0006, GLOW)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"snapped": False, "strokes": 0, "a": None, "f": None, "dmax": 0.0, "max_bend": 0.0, "tcps": None,
            "s": {}, "shown": set()}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GLOW_DEBUG")), "steps": 0}


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


def _raw_pose(base, name) -> np.ndarray:
    return as_numpy(base.scene[name].data.root_link_pose_w)[0]


def _write_pose(base, name, pose) -> None:
    import torch

    asset = base.scene[name]
    asset.write_root_pose_to_sim(torch.tensor([list(map(float, pose))], dtype=torch.float32, device=asset.device))


def bend_deg(rl, rr) -> float:
    """How far the stick is bent at its middle: the angle between the two halves' axes, degrees."""
    c = float(np.dot(rl[:, 1], rr[:, 1]))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _shake(p) -> None:
    """Count shake strokes of the stick's middle: at least STROKE one way, then TURN back."""
    if _EP["a"] is None:
        _EP["a"], _EP["f"], _EP["dmax"] = p.copy(), p.copy(), 0.0
        return
    d = float(np.linalg.norm(p - _EP["a"]))
    if d > _EP["dmax"]:
        _EP["f"], _EP["dmax"] = p.copy(), d
    elif _EP["dmax"] >= STROKE and float(np.linalg.norm(p - _EP["f"])) >= TURN:
        _EP["strokes"] += 1
        _EP["a"] = _EP["f"].copy()
        _EP["f"], _EP["dmax"] = p.copy(), float(np.linalg.norm(p - _EP["a"]))


def lit() -> bool:
    return _EP["snapped"] and _EP["strokes"] >= STROKES_TO_LIGHT


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pl_raw, pr_raw = _raw_pose(base, "stick_l"), _raw_pose(base, "stick_r")
    pl, rl = pl_raw[:3] - origin, rotation(pl_raw[3:7])
    pr, rr = pr_raw[:3] - origin, rotation(pr_raw[3:7])
    bend = bend_deg(rl, rr)
    _EP["max_bend"] = max(_EP["max_bend"], bend)
    if not _EP["snapped"] and bend >= SNAP_DEG:
        _EP["snapped"] = True
        _EP["a"] = None  # shakes count from the snap on
    if _EP["snapped"] and not lit():
        _shake((pl + pr) / 2)
    show = {"spark_l", "spark_r"} if _EP["snapped"] else set()
    if lit():
        show = {"glow_l", "glow_r"}
    for name in ("spark_l", "spark_r", "glow_l", "glow_r"):
        if name in show:
            _write_pose(base, name, pl_raw if name.endswith("_l") else pr_raw)
        elif name in _EP["shown"]:
            _write_pose(base, name, [*(np.asarray(PARK) + origin).tolist(), 0.0, 0.0, 0.0, 1.0])
    _EP["shown"] = show
    in_tray = all(TX0 + T_RIM_W <= c[0] <= TX1 - T_RIM_W and TY0 + T_RIM_W <= y <= TY1 - T_RIM_W and c[2] <= IN_TRAY_Z
                  for c, y in ((p + rot[:, 1] * s * HALF_L / 2, (p + rot[:, 1] * s * HALF_L)[1])
                               for p, rot, s in ((pl, rl, 1.0), (pr, rr, -1.0))))
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"in_tray": in_tray}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = _EP["tcps"]
            mid = (pl + pr) / 2
            print(f"[glow] t={_DEBUG['steps'] / 20:5.1f}s mid={np.round((mid - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" bend={bend:5.1f} max={_EP['max_bend']:5.1f} snapped={_EP['snapped']} strokes={_EP['strokes']}"
                  f" lit={lit()} in_tray={in_tray} yawL={math.degrees(math.atan2(rl[1, 0], rl[0, 0])):6.1f}"
                  f" yawR={math.degrees(math.atan2(rr[1, 0], rr[0, 0])):6.1f}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the stick lit all along (bent until its vial snapped, then shaken three times), lying in the black
    tray and both arms back where they started, at the end; 0.5 with it lit; 0.25 with its vial snapped; else 0.0.
    (A snapped vial stays snapped, and a lit stick stays lit.)"""
    if not _EP["snapped"]:
        return 0.0
    if not lit():
        return 0.25
    if _EP["s"].get("in_tray") and _arms_home():
        return 1.0
    return 0.5


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
    """Full score with both halves of the stick still in the tray: the trial ends there."""
    return score(objects) >= 1.0 and objects["stick_l"].resting() and objects["stick_r"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands come straight down on the stick, one on each half 11 cm out from the middle, fingers closing across it,
# and lift it out of the packet. They bend it at the middle, each hand swinging its half forward round the middle and
# turning with it, until the halves are 44 degrees apart (the vial snaps at 30), then straighten it. They shake it
# forward and back three times, carry it out over the black tray, lay it down in it, let go and go back to where they
# started.

GRIP_R = 0.110  # the hands hold the stick this far either side of its middle
PRE_OPEN = -(R + 0.006)
GRIP_DZ = 0.005  # the TCP this far above the stick's axis
OVER = 0.060
LIFT_Z = TOP + 0.105
CARRY_Z = TOP + 0.070  # carried out over the tray this low (joint4 has room there)
BEND_EACH = 22.0  # degrees each half swings (44 apart)
SHAKE_DX, SHAKE_S = 0.040, 0.32
TRAY_MID = np.array([(TX0 + TX1) / 2, 0.0, TOP + T_FLOOR + R])
LEAD_S, LEAD_STEPS = 3.0, 15


def _grips(mid, beta_deg: float) -> dict:
    """Each hand's TCP pose with the stick's middle at ``mid`` and each half swung ``beta_deg`` forward (+x) round it."""
    b = math.radians(beta_deg)
    out = {}
    for arm, s in (("left", 1.0), ("right", -1.0)):
        p = np.asarray(mid, float) + GRIP_R * np.array([math.sin(b), s * math.cos(b), 0.0]) + [0.0, 0.0, GRIP_DZ]
        yaw = -(math.pi / 2 + b) if s > 0 else (math.pi / 2 + b)
        out[arm] = (p, grip_rotation(yaw))
    return out


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    state = {"mid": MID.copy(), "beta": 0.0}

    def both(t, mid, beta, finger, label, dz=0.0):
        for arm, (p, r) in _grips(mid, beta).items():
            plan[arm].append(_w(t, p + [0.0, 0.0, dz], finger, label, r))

    def move(t, mid, beta, label, n=4, finger=FINGER_CLOSED):
        m0, b0 = state["mid"], state["beta"]
        m1 = np.asarray(mid, float)
        for j in range(1, n + 1):
            f = j / n
            both(t / n, m0 + (m1 - m0) * f, b0 + (beta - b0) * f, finger, label)
        state["mid"], state["beta"] = m1, beta

    for arm in ("left", "right"):
        p, r = _grips(MID, 0.0)[arm]
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, p + [0.0, 0.0, OVER], r),
                                 "over the glow stick", PRE_OPEN)
    both(0.5, MID, 0.0, PRE_OPEN, "over the glow stick", OVER)
    both(1.2, MID, 0.0, PRE_OPEN, "down round the stick")
    both(0.6, MID, 0.0, FINGER_CLOSED, "take the stick near both ends")
    up = np.array([SX, SY, LIFT_Z])
    move(1.2, up, 0.0, "lift it out of the packet", n=3)
    move(1.6, up, BEND_EACH, "bend it at the middle until the vial inside snaps", n=8)
    move(0.4, up, BEND_EACH, "bend it at the middle until the vial inside snaps", n=1)
    move(1.2, up, 0.0, "straighten it", n=6)
    for k in range(3):
        move(SHAKE_S, up + [SHAKE_DX, 0.0, 0.0], 0.0, "shake it: forward ...", n=2)
        move(SHAKE_S, up - [SHAKE_DX, 0.0, 0.0], 0.0, "... and back", n=2)
    move(SHAKE_S / 2, up, 0.0, "hold it still, lit", n=1)
    over = TRAY_MID + [0.0, 0.0, CARRY_Z - TRAY_MID[2]]
    move(1.6, over, 0.0, "carry it out over the black tray", n=4)
    move(1.2, TRAY_MID + [0.0, 0.0, 0.003], 0.0, "lay it down in the tray", n=3)
    both(0.6, TRAY_MID + [0.0, 0.0, 0.003], 0.0, PRE_OPEN, "let go")
    both(1.0, TRAY_MID, 0.0, PRE_OPEN, "rise clear of the stick", OVER)
    for arm in ("left", "right"):
        p, r = _grips(TRAY_MID, 0.0)[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p + [0.0, 0.0, OVER], r), np.array(READY, float),
                                 "back to where it started", PRE_OPEN)
        last = plan[arm][-1]
        plan[arm].append(_w(3.0, last.pos, PRE_OPEN,
                            "hold still: the trial ends once the full score has held 1 s with both arms home",
                            last.yaw))
    return plan
