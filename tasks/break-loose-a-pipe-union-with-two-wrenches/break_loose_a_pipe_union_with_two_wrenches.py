"""On the station table runs a short pipe line: a pipe section resting in two cradles nearest the arms, a union nut
joining it to the next pipe piece, and that piece screwed into a fixed elbow beyond. Two wrenches are already on: one on the union
nut, its handle up to the right; a backup wrench on the pipe piece's hex, its handle up to the left.

Task: break the union loose and take the pipe section off. One hand presses the union nut's wrench down, turning the
nut loose; the other holds the backup wrench down, so the pipe piece cannot turn. Loose, the union comes apart: the pipe
section is lifted out of its cradles and laid on the table. With the backup wrench not held, the union nut's turn drags
the pipe piece round with it and loosens the wrong joint, at the elbow (its thread is the less stiff of the two).

Run it:  inspect-robots run --embodiment yam_arena -E task=break_loose_a_pipe_union_with_two_wrenches.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset, on_step,  # noqa: E402
                     rotation, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the pipe line, along x at y = 0 ---------------------------------------------------------------------------------
PZ = TOP + 0.07  # its axis
PIPE_R = 0.012
NEAR_X = (0.15, 0.264)  # the pipe section (toward the arms) and its union tail, resting in two cradles
CRADLE_X = (0.156, 0.255)
SECTION_MASS = 0.15
NUT_X = (0.265, 0.290)  # the union nut
BODY_X = (0.291, 0.360)  # the pipe piece, with its hex
HEX_X = (0.330, 0.352)
ELBOW_X = (0.361, 0.40)  # the fixed elbow

# -- the wrenches: each a handle 14 cm long from its nut's axis, up 25 degrees from level, with a flat pad at its end ---
HANDLE_L, HANDLE_UP = 0.14, math.radians(25.0)
PAD = (0.030, 0.030, 0.008)

# -- the threads: each holds until turned hard enough, then turns against a drag; only the loosening way (+ about +x).
# A spring holds each where it last stopped; pushed SLIP past that, it gives (the spring's hold moves to it) ----------
NUT_HOLD, ELBOW_HOLD = 0.15, 0.03  # N m per degree: the union nut holds to about 0.3 N m, the elbow joint 0.06
SLIP = 2.0  # degrees
NUT_DRAG = 0.08  # N m per degree/s, while turning
ELBOW_DRAG = 0.017
TURN_MAX = 120.0

STEEL, DARK, BRASS, RED, BLUE = (0.62, 0.64, 0.66), (0.20, 0.21, 0.23), (0.78, 0.64, 0.30), (0.75, 0.15, 0.12), \
    (0.15, 0.30, 0.70)

# -- scoring ----------------------------------------------------------------------------------------------------------
LOOSE = 25.0  # the union nut turned at least this far loose: broken loose (the union comes apart)
TAKEN = 0.05  # the pipe section this far from where it lay: taken off
ON_TABLE = 0.035  # ... and its axis lower than this above the table: laid on the table
HOME_WITHIN = 0.05
WRONG, WRONG_CLEAN = 12.0, 8.0  # the elbow joint turned this far: loosened (0 points); under this: left alone
LET_GO = 0.04
SUCCESS_HOLD_S = 1.0


def _qx(a: float) -> tuple[float, float, float, float]:
    return (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))


def _handle(side: float, x: float, color) -> list:
    """A wrench's handle and pad, from the axis toward ``side`` (-1: -y, right; +1: +y, left), up HANDLE_UP."""
    a = side * HANDLE_UP  # about +x: +y side turned up by +a, -y side up by -a
    d = np.array([0.0, side * math.cos(HANDLE_UP), math.sin(HANDLE_UP)])
    mid, end = d * (0.02 + HANDLE_L) / 2, d * HANDLE_L
    return [BoxShape((x, float(mid[1]), float(mid[2])), (0.016, HANDLE_L - 0.02, 0.010), color, quat_xyzw=_qx(a)),
            BoxShape((x, float(end[1]), float(end[2]) + PAD[2] / 2), PAD, DARK, quat_xyzw=_qx(a))]


def _near() -> Body:
    """The pipe section's two cradles, and the elbow with its riser (fixed). Origin on the axis at x = 0."""
    h = PZ - PIPE_R - TOP
    shapes = []
    for x in CRADLE_X:
        shapes += [BoxShape((x, 0.0, -PIPE_R - h / 2), (0.014, 0.03, h), DARK),
                   *[BoxShape((x, s * (PIPE_R + 0.004), -PIPE_R + 0.005), (0.014, 0.006, 0.010), DARK) for s in (-1.0, 1.0)]]
    shapes += [BoxShape(((ELBOW_X[0] + ELBOW_X[1]) / 2, 0.0, 0.0), (ELBOW_X[1] - ELBOW_X[0], 0.036, 0.036), BRASS),
              CylShape((ELBOW_X[1] - 0.018, 0.0, -(PZ - TOP) / 2), PIPE_R, PZ - TOP, STEEL)]  # the elbow's riser
    return Body.build("pipeline", pos=(0.0, 0.0, PZ), mass=5.0, shapes=shapes, kinematic=True, color=STEEL,
                      friction=(0.05, 0.05), slick=True)  # smooth cradles: the section turns freely in them


SECTION_AT = np.array([(NEAR_X[0] + NEAR_X[1]) / 2, 0.0, PZ])


def _section() -> Body:
    """The pipe section with the union's tail on its end; origin at its middle, on the axis."""
    shapes = [CylShape((0.0, 0.0, 0.0), PIPE_R, NEAR_X[1] - NEAR_X[0], STEEL, axis="X"),
              CylShape(((NEAR_X[1] - NEAR_X[0]) / 2 - 0.004, 0.0, 0.0), 0.016, 0.008, BRASS, axis="X")]  # the tail
    return Body.build("pipe_section", pos=tuple(SECTION_AT.tolist()), mass=SECTION_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=STEEL)


def _body() -> Body:
    """The pipe piece with its hex and the backup wrench; origin on the axis at x = 0 (mass on the axis)."""
    shapes = [CylShape(((BODY_X[0] + BODY_X[1]) / 2, 0.0, 0.0), PIPE_R, BODY_X[1] - BODY_X[0], STEEL, axis="X"),
              CylShape(((HEX_X[0] + HEX_X[1]) / 2, 0.0, 0.0), 0.018, HEX_X[1] - HEX_X[0], DARK, axis="X"),
              *_handle(+1.0, (HEX_X[0] + HEX_X[1]) / 2, BLUE)]
    return Body.build("pipe_piece", pos=(0.0, 0.0, PZ), mass=0.4, shapes=shapes, friction=(1.0, 0.9), color=STEEL,
                      com=((BODY_X[0] + BODY_X[1]) / 2, 0.0, 0.0))


def _nut() -> Body:
    """The union nut with its wrench; origin on the axis at x = 0 (mass on the axis)."""
    shapes = [CylShape(((NUT_X[0] + NUT_X[1]) / 2, 0.0, 0.0), 0.021, NUT_X[1] - NUT_X[0], BRASS, axis="X"),
              *_handle(-1.0, (NUT_X[0] + NUT_X[1]) / 2, RED)]
    return Body.build("union_nut", pos=(0.0, 0.0, PZ), mass=0.2, shapes=shapes, friction=(1.0, 0.9), color=BRASS,
                      com=((NUT_X[0] + NUT_X[1]) / 2, 0.0, 0.0), ignore=("pipeline", "pipe_section"))  # it turns on the tail


OBJECTS = [_near(), _section(), _body(), _nut()]
# the union, tight, holds the pipe section to the pipe piece; turned loose, it comes apart
glue("union_held", "pipe_section", "pipe_piece", anchor=(NEAR_X[1], 0.0, PZ))
# the turn limits stop each joint at its tight end (1 degree past where it starts): nothing turns the tightening way
hinge("elbow_thread", "pipeline", "pipe_piece", anchor=(BODY_X[1], 0.0, PZ), axis="X", limits_deg=(-1.0, TURN_MAX),
      spring=ELBOW_HOLD, damping=ELBOW_DRAG, target_deg=0.0)
hinge("union_thread", "pipe_piece", "union_nut", anchor=(NUT_X[1], 0.0, PZ), axis="X", limits_deg=(-1.0, TURN_MAX),
      spring=NUT_HOLD, damping=NUT_DRAG, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}, "worst_elbow": 0.0, "held": {"elbow_thread": 0.0, "union_thread": 0.0},
            "apart": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("UNION_DEBUG")), "steps": 0}


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


def _roll(r: np.ndarray) -> float:
    """A body's turn about +x, degrees."""
    return math.degrees(math.atan2(float(r[2, 1]), float(r[1, 1])))


def _pad(side: float, x: float, turn_deg: float) -> np.ndarray:
    a = side * HANDLE_UP + math.radians(turn_deg) if side > 0 else math.pi - HANDLE_UP + math.radians(turn_deg)
    return np.array([x, HANDLE_L * math.cos(a), PZ + HANDLE_L * math.sin(a)])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["pipe_piece"].data.root_link_pose_w)[0]
    n = as_numpy(base.scene["union_nut"].data.root_link_pose_w)[0]
    elbow = _roll(rotation(p[3:7]))
    nut = _roll(rotation(n[3:7])) - elbow
    _EP["worst_elbow"] = max(_EP["worst_elbow"], abs(elbow))
    if not _EP["apart"] and nut >= LOOSE:  # the union is loose: it comes apart
        set_glue_on("union_held", False)
        _EP["apart"] = True
    sec = as_numpy(base.scene["pipe_section"].data.root_link_pose_w)[0][:3] - origin
    for name, angle in (("elbow_thread", elbow), ("union_thread", nut)):  # turned hard past its hold: it gives
        if angle - _EP["held"][name] > SLIP:
            _EP["held"][name] = angle
            set_hinge_drive(name, target_deg=angle)
    _EP["tcps"] = _tcps(base, origin)
    pads = [_pad(-1.0, (NUT_X[0] + NUT_X[1]) / 2, elbow + nut), _pad(+1.0, (HEX_X[0] + HEX_X[1]) / 2, elbow)]
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in pads)
    _EP["s"] = {"elbow": elbow, "nut": nut, "clear": clear, "taken": float(np.linalg.norm(sec - SECTION_AT))}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[union] t={_DEBUG['steps'] / 20:5.1f}s nut_loose={nut:6.1f}deg elbow={elbow:6.1f}deg "
                  f"worst_elbow={_EP['worst_elbow']:5.1f} clear={clear} apart={_EP['apart']} "
                  f"section_moved={_EP['s']['taken'] * 1000:5.0f}mm "
                  f"rightTCP-nut_pad={np.round((tr - pads[0]) * 1000).astype(int).tolist()} "
                  f"leftTCP-backup_pad={np.round((tl - pads[1]) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"nut": st.get("nut", 0.0), "elbow": max(_EP["worst_elbow"], abs(st.get("elbow", 0.0))),
            "clear": bool(st.get("clear")), "taken": _EP["apart"] and st.get("taken", 0.0) >= TAKEN}


def _points(s) -> float:
    if s["elbow"] >= WRONG:
        return 0.0
    if s["nut"] >= LOOSE and s["elbow"] < WRONG_CLEAN and s["taken"] and s["clear"]:
        return 1.0
    if s["nut"] >= LOOSE:
        return 0.5
    return 0.25 if s["nut"] >= 10.0 else 0.0


def score(objects) -> float:
    """0.0 if the elbow joint was ever turned 12 degrees loose (the wrong joint). Else 1.0 with the union nut turned at
    least 25 degrees loose, the elbow joint under 8 degrees, the pipe section taken off (at least 5 cm from where it
    lay) and laid on the table (its axis less than 3.5 cm above it), both hands clear of the wrenches, and both arms
    back where they started; 0.5 with the nut 25 degrees loose otherwise; 0.25 with it 10 degrees loose; else 0.0."""
    pts = _points(_status())
    on_table = float(objects["pipe_section"].pos[2]) < TOP + ON_TABLE
    if pts >= 1.0 and not (on_table and _arms_home()):
        return 0.5  # loose and taken off, but not laid on the table or the arms not back
    return pts


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
    """Full score with the pipe section still: the trial ends there."""
    return score(objects) >= 1.0 and objects["pipe_section"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the backup wrench's pad, fingers closed, and presses it down, holding the pipe
# piece. The right hand comes straight down on the union nut's wrench pad, fingers closed, and presses it down along its
# arc, turning the nut 40 degrees loose: the union comes apart. The left rises and goes back to where it started; the
# right takes the pipe section by its middle, lifts it out of its cradles, lays it on the table and goes back to where
# it started.

PRESS = 0.005  # aimed this far into a pad
TIP = 0.013  # closed fingertips below the TCP
GRIP = grip_rotation(0.0)
TURN_TO = 40.0
LEAD_S, LEAD_STEPS = 3.0, 15


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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _on_pad(pad: np.ndarray, press: float) -> np.ndarray:
    return pad + [0.0, 0.0, PAD[2] + TIP - press]


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    pad = _pad(+1.0, (HEX_X[0] + HEX_X[1]) / 2, 0.0)
    out = _lead_in("left", _on_pad(pad, -0.03), GRIP, "over the backup wrench's pad", FINGER_CLOSED)
    out += [_w(1.0, _on_pad(pad, 0.0), FINGER_CLOSED, "down onto it", GRIP),
            _w(0.6, _on_pad(pad, PRESS), FINGER_CLOSED, "press it down: the pipe piece cannot turn", GRIP)]
    held_at = sum(w.duration for w in out)
    up = _on_pad(pad, -0.015) + [0.0, 0.04, 0.0]
    out += [_w(max(release_at - held_at, 0.3), _on_pad(pad, PRESS), FINGER_CLOSED, "hold it down", GRIP),
            _w(1.0, up, FINGER_CLOSED, "rise and draw back", GRIP)]
    out += _go_home("left", up, GRIP, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED, "wait at the start", out[-1].yaw))
    return out, held_at


SECTION_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y, across the pipe
LAY_AT = np.array([0.20, -0.13])  # where the right lays the pipe section on the table
IN_HAND = 0.002  # the right TCP this far above the pipe's axis


def _right(start: float) -> list[Waypoint]:
    x = (NUT_X[0] + NUT_X[1]) / 2
    out = _lead_in("right", _on_pad(_pad(-1.0, x, 0.0), -0.03), GRIP, "over the union nut wrench's pad", FINGER_CLOSED)
    out += [_w(max(start - LEAD_S, 0.3), _on_pad(_pad(-1.0, x, 0.0), 0.0), FINGER_CLOSED, "down onto it", GRIP)]
    for k in range(1, 9):
        out.append(_w(0.5, _on_pad(_pad(-1.0, x, TURN_TO * k / 8), PRESS), FINGER_CLOSED,
                      "press it down along its swing: the nut turns loose, the union comes apart", GRIP))
    end = _pad(-1.0, x, TURN_TO)
    grip = SECTION_AT + [0.0, 0.0, IN_HAND]
    lay = np.array([LAY_AT[0], LAY_AT[1], TOP + PIPE_R + 0.003 + IN_HAND])
    out += [_w(0.5, _on_pad(end, 0.0), FINGER_CLOSED, "ease off", GRIP),
            _w(1.0, _on_pad(end, -0.04), FINGER_CLOSED, "rise", GRIP),
            _w(1.5, grip + [0.0, 0.0, 0.05], FINGER_OPEN, "over the pipe section's middle", SECTION_GRIP),
            _w(1.0, grip, FINGER_OPEN, "down round it", SECTION_GRIP),
            _w(0.6, grip, FINGER_CLOSED, "take it", SECTION_GRIP),
            _w(1.2, grip + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it out of its cradles: the pipe is off", SECTION_GRIP),
            _w(1.5, lay + [0.0, 0.0, 0.04], FINGER_CLOSED, "over the table beside the pipe line", SECTION_GRIP),
            _w(0.8, lay, FINGER_CLOSED, "lay it down", SECTION_GRIP),
            _w(0.5, lay, FINGER_OPEN, "let go", SECTION_GRIP),
            _w(0.8, lay + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", SECTION_GRIP)]
    out += _go_home("right", lay + [0.0, 0.0, 0.05], SECTION_GRIP, FINGER_OPEN)
    return out


def demo():
    _, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_turned = held_at + 4.5 + 0.5  # the nut turned and eased off
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(t_turned, total)
    return {"left": left, "right": right}
