"""A small folding camp stool stands open on the station table: two beech-wood frames crossed in an X on a steel
pivot bolt, a green canvas seat slung between their top bars, which run away from the arms. Folding it brings the two
top bars together, and that is where fingers get caught: the inner faces of both top bars carry yellow and black
warning stripes (the pinch point).

Task: fold the stool shut and lay it down. Pushed at one bar, the light stool slides or tips over instead of folding,
so both hands push: each from outside its own top bar (the left hand the left bar, the right hand the right bar),
squeezing the bars together until the stool is shut, with no gripper between the bars as they close. Then the hands
draw back, the right arm goes back to where it started, the left tips the folded stool over so it lies flat on the
table, and goes back to where it started too.

Run it:  inspect-robots run --embodiment yam_arena -E task=fold_a_camp_stool_without_pinching_a_gripper.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, segment, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stool: two frames crossed in an X, pivoting about a bolt along x (its bars run away from the arms) ------------
SX, SY = 0.25, 0.0  # the pivot's x and y (the stool's middle)
LEG = 0.16  # each leg, from its foot bar's middle to its top bar's middle; the pivot halfway
BAR_R = 0.011  # top and foot bars: round wooden dowels
LEG_T = 0.018  # legs: square section
THETA0 = math.radians(36.0)  # open: each leg this far from vertical
THETA_SHUT = math.asin(2 * BAR_R / LEG)  # shut: the two top bars touch
FOLD_SHUT = math.degrees(2 * (THETA0 - THETA_SHUT))  # the fold (relative turn of the frames) when shut, about 58 deg
OUTER_X, INNER_X = 0.110, 0.084  # the outer frame's legs, the inner frame's legs (each +-x from the middle)
OUTER_L, INNER_L = 0.240, 0.186  # their bars' lengths
PIVOT = np.array([SX, SY, TOP + BAR_R + (LEG / 2) * math.cos(THETA0)])
FRAME_MASS = 0.17
FEET_FRICTION = (0.35, 0.3)  # plastic feet on the table
HOLD = 0.03  # N m per degree: the pivot holds the stool where it is left ...
SLIP = 2.0  # ... until it is turned this many degrees past (then the task file moves the hold there)
DRAG = 0.002

# -- the canvas seat, slung between the top bars (drawn only; it sags as the bars come together) ----------------------
SEAT_HALF = (LEG * math.sin(THETA0) - 1.4 * BAR_R) / 2  # each half of the seat, bar to middle
SEAT_L = 0.17

BEECH, STEEL, CANVAS, YELLOW, BLACK = (0.80, 0.62, 0.40), (0.62, 0.64, 0.66), (0.16, 0.36, 0.30), \
    (0.98, 0.80, 0.08), (0.08, 0.08, 0.08)

# -- scoring ----------------------------------------------------------------------------------------------------------
SHUT = FOLD_SHUT - 8.0  # folded at least this far (deg): shut (the top bars within about 1.5 cm)
HALF = FOLD_SHUT / 2
ZONE_FROM = 34.0  # once folded this far (the bars' inner faces under about 3.5 cm apart) ...
ZONE_HALF = 0.025  # ... a TCP within this of the middle between the bars (across their gap) ...
ZONE_UP = 0.03  # ... and of their height, between their ends, is caught in the pinch point
LYING = math.radians(60.0)  # laid down: the stool's up axis at least this far from vertical
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _frame(name: str, side: float, legs_x: float, length: float, mass: float) -> Body:
    """One frame, origin at the pivot. ``side`` +1: its top bar on the left (+y), its feet on the right; -1: the
    reverse."""
    s, c = math.sin(THETA0), math.cos(THETA0)
    top = np.array([0.0, side * (LEG / 2) * s, (LEG / 2) * c])
    foot = np.array([0.0, -side * (LEG / 2) * s, -(LEG / 2) * c])
    shapes = [CylShape(tuple(top.tolist()), BAR_R, length, BEECH, axis="X"),
              CylShape(tuple(foot.tolist()), BAR_R, length, BEECH, axis="X")]
    for sx in (-1.0, 1.0):
        off = np.array([sx * legs_x, 0.0, 0.0])
        shapes.append(segment(foot + off, top + off, LEG_T, BEECH))
        shapes.append(CylShape((sx * (legs_x + LEG_T / 2 + 0.002), 0.0, 0.0), 0.007, 0.004, STEEL, collide=False,
                               axis="X"))  # the pivot bolt's heads
    n = 7  # warning stripes on the top bar's inner side (facing the other bar)
    span = (2 * legs_x - LEG_T) * 0.9
    for i in range(n):
        x = -span / 2 + span * (i + 0.5) / n
        shapes.append(BoxShape((x, top[1] - side * (BAR_R - 0.0015), top[2]), (span / n, 0.003, 0.016),
                               YELLOW if i % 2 == 0 else BLACK, collide=False))
    return Body.build(name, pos=tuple(PIVOT.tolist()), mass=mass, shapes=shapes, friction=FEET_FRICTION, color=BEECH,
                      com=(0.0, 0.0, 0.0), damping=(0.0, 0.05))


def _seat_half(name: str, side: float) -> Body:
    """Half of the canvas seat, drawn only; origin at its middle. Starts flat between the bars."""
    s, c = math.sin(THETA0), math.cos(THETA0)
    edge = PIVOT + [0.0, side * ((LEG / 2) * s - BAR_R * 0.7), (LEG / 2) * c + BAR_R * 0.7]
    mid = edge + [0.0, -side * SEAT_HALF / 2, 0.0]
    return Body.build(name, pos=tuple(mid.tolist()), mass=0.001, kinematic=True, color=CANVAS,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (SEAT_L, SEAT_HALF, 0.003), CANVAS, collide=False)])


OBJECTS = [_frame("outer_frame", 1.0, OUTER_X, OUTER_L, FRAME_MASS),
           _frame("inner_frame", -1.0, INNER_X, INNER_L, FRAME_MASS - 0.02),
           _seat_half("seat_left", 1.0), _seat_half("seat_right", -1.0)]
# folding turns the outer frame +x and the inner frame -x: the inner frame's turn against the outer one goes negative
hinge("pivot", "outer_frame", "inner_frame", anchor=tuple(PIVOT.tolist()), axis="X",
      limits_deg=(-FOLD_SHUT, 0.5), spring=HOLD, damping=DRAG, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": 0.0, "pinched": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STOOL_DEBUG")), "steps": 0}


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


def _quat_xyzw(m: np.ndarray) -> list[float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return [x / n, y / n, z / n, w / n]  # the order the simulator's root poses use


def _place(base, origin, name, pos, rot) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(pos, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *_quat_xyzw(rot)]], dtype=torch.float32,
                                              device=asset.device))


def _top_bar(pos, rot, side: float) -> np.ndarray:
    s, c = math.sin(THETA0), math.cos(THETA0)
    return pos + rot @ np.array([0.0, side * (LEG / 2) * s, (LEG / 2) * c])


def _drape(base, origin, tl, tr, up, along) -> None:
    """Hang the two canvas halves from the top bars' upper inner edges, sagging so each keeps its length."""
    u = tr - tl
    u /= max(float(np.linalg.norm(u)), 1e-6)
    el = tl + (up + u) * BAR_R * 0.7
    er = tr + (up - u) * BAR_R * 0.7
    half = float(np.linalg.norm(er - el)) / 2
    sag = math.sqrt(max(SEAT_HALF ** 2 - half ** 2, 0.0))
    n = np.cross(along, u)
    if np.dot(n, up) < 0:
        n = -n
    low = (el + er) / 2 - n * sag
    for name, e in (("seat_left", el), ("seat_right", er)):
        y = low - e  # the half's local y runs from its bar to the low middle; its local x along the bars
        y /= max(float(np.linalg.norm(y)), 1e-6)
        x = along - np.dot(along, y) * y
        x /= max(float(np.linalg.norm(x)), 1e-6)
        _place(base, origin, name, (e + low) / 2, np.column_stack([x, y, np.cross(x, y)]))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    a = as_numpy(base.scene["outer_frame"].data.root_link_pose_w)[0]
    b = as_numpy(base.scene["inner_frame"].data.root_link_pose_w)[0]
    a_pos, a_rot = a[:3] - origin, rotation(a[3:7])
    b_pos, b_rot = b[:3] - origin, rotation(b[3:7])
    rel = a_rot.T @ b_rot
    turn = math.degrees(math.atan2(rel[2, 1], rel[1, 1]))  # the inner frame's turn against the outer, about x
    fold = -turn
    if abs(turn - _EP["hold"]) > SLIP:  # turned past its hold: the pivot gives, and holds there
        _EP["hold"] = turn
        set_hinge_drive("pivot", target_deg=turn)
    tl, tr = _top_bar(a_pos, a_rot, 1.0), _top_bar(b_pos, b_rot, -1.0)
    along = a_rot[:, 0]
    up = a_rot[:, 2] + b_rot[:, 2]
    up /= max(float(np.linalg.norm(up)), 1e-6)
    tilt = math.acos(max(-1.0, min(1.0, float(up[2]))))
    _drape(base, origin, tl, tr, up, along)
    _EP["tcps"] = _tcps(base, origin)
    mid = (tl + tr) / 2
    u = tr - tl
    u /= max(float(np.linalg.norm(u)), 1e-6)
    w = np.cross(along, u)
    caught = []
    for t in _EP["tcps"]:
        r = t - mid
        caught.append(abs(float(r @ u)) < ZONE_HALF and abs(float(r @ w)) < ZONE_UP
                      and abs(float(r @ along)) < OUTER_L / 2 + 0.01)
    if fold >= ZONE_FROM and any(caught):
        _EP["pinched"] = True
    _EP["s"] = {"fold": fold, "tilt": tilt, "caught": caught}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            t_l, t_r = _EP["tcps"]
            print(f"[stool] t={_DEBUG['steps'] / 20:5.1f}s fold={fold:5.1f} hold={_EP['hold']:6.1f} "
                  f"tilt={math.degrees(tilt):5.1f} pivot={np.round(a_pos * 1000).astype(int).tolist()} "
                  f"bars={np.round(tl * 1000).astype(int).tolist()}/{np.round(tr * 1000).astype(int).tolist()} "
                  f"caught={caught} pinched={_EP['pinched']} "
                  f"L={np.round(t_l * 1000).astype(int).tolist()} R={np.round(t_r * 1000).astype(int).tolist()} "
                  f"home={_arms_home()} score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[arm])[0], float)
                         for arm in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def _status() -> dict:
    st = _EP["s"]
    return {"fold": st.get("fold", 0.0), "lying": st.get("tilt", 0.0) >= LYING, "pinched": _EP["pinched"],
            "home": _arms_home()}


def _points(s) -> float:
    if s["pinched"]:
        return 0.0
    if s["fold"] >= SHUT and s["lying"] and s["home"]:
        return 1.0
    if s["fold"] >= SHUT:
        return 0.5
    return 0.25 if s["fold"] >= HALF else 0.0


def score(objects) -> float:
    """0.0 if a gripper was ever caught in the pinch point: any TCP within 2.5 cm of the middle between the two top
    bars (across their gap) and within 3 cm of their height, between their ends, while the stool was folded 34
    degrees or more. Otherwise, at the end: 1.0 with the stool shut (folded at least 50 degrees, its top bars within
    about 1.5 cm), lying down (its up axis at least 60 degrees from vertical) and both arms back where they started;
    0.5 with the stool shut; 0.25 with it folded at least halfway (29 degrees); else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with both frames of the stool still."""
    return score(objects) >= 1.0 and objects["outer_frame"].resting() and objects["inner_frame"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Each hand comes in from its own side, level, fingers closed and pointing across the stool: the left from the left,
# the right from the right, both at the stool's middle. Their fingertips meet the outsides of the two top bars and push
# them toward each other along their swing (rising with them) until the stool is shut, both hands staying outside the
# bars. They draw back; the right goes back to where it started while the left pushes the folded stool's top to the
# right, tipping it over so it lies flat on the table. Then the left goes back to where it started too.


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


L_GRIP = _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2)  # level, pointing right (-y), fingers closing along x
R_GRIP = _rx(math.pi / 2) @ grip_rotation(math.pi / 2)  # level, pointing left (+y), fingers closing along x
TIP = 0.013  # closed fingertips this far past the TCP
CLEAR = BAR_R + TIP + 0.002  # the TCP this far outside a bar's middle: the fingertips just off it
SQUEEZE = 0.010  # aim this far into each bar at the end
DOWN = -0.002  # the TCP this far below the bar's middle
STEPS, STEP_S = 10, 0.5
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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _push_at(theta: float, side: float, squeeze: float = 0.0) -> np.ndarray:
    """Where a hand pushes the top bar of the frame on ``side`` (+1 left, -1 right) with the legs ``theta`` from
    vertical: its fingertips just outside the bar, level with its middle, along its swing (the pivot rises as the
    stool folds)."""
    y = SY + side * ((LEG / 2) * math.sin(theta) + CLEAR - squeeze)
    z = TOP + BAR_R + LEG * math.cos(theta) + DOWN
    return np.array([SX, y, z])


def _hand(side: float, arm: str, grip) -> tuple[list[Waypoint], np.ndarray]:
    name = "left" if side > 0 else "right"
    start = _push_at(THETA0, side) + [0.0, side * 0.03, 0.0]
    out = _lead_in(arm, start + [0.0, side * 0.02, 0.02], grip, f"level with the {name} top bar, outside it",
                   FINGER_CLOSED)
    out += [_w(1.0, start, FINGER_CLOSED, "fingertips pointing at the bar from outside", grip),
            _w(0.6, _push_at(THETA0, side), FINGER_CLOSED, "against the bar", grip)]
    for k in range(1, STEPS + 1):
        th = THETA0 + (THETA_SHUT - THETA0) * k / STEPS
        out.append(_w(STEP_S, _push_at(th, side, SQUEEZE * k / STEPS), FINGER_CLOSED,
                      "push the bars together, staying outside them", grip))
    shut = _push_at(THETA_SHUT, side, SQUEEZE)
    back = shut + [0.0, side * 0.05, 0.0]
    out += [_w(0.6, shut, FINGER_CLOSED, "hold it shut", grip),
            _w(0.8, back, FINGER_CLOSED, "draw back", grip)]
    return out, back


TIP_PUSH = 0.07  # the left pushes the folded stool's top this far to the right, tipping it over


def demo():
    left, l_end = _hand(1.0, "left", L_GRIP)
    right, r_end = _hand(-1.0, "right", R_GRIP)
    right += _go_home("right", r_end, R_GRIP, FINGER_CLOSED)
    right_done = sum(w.duration for w in right)
    beside = _push_at(THETA_SHUT, 1.0) + [0.0, 0.01, -0.005]  # outside the folded stool's left top bar
    left += [_w(max(right_done - sum(w.duration for w in left), 0.8), beside, FINGER_CLOSED,
                "beside the folded stool's top (the right goes home first)", L_GRIP),
             _w(1.5, beside + [0.0, -TIP_PUSH, 0.0], FINGER_CLOSED, "tip it over to the right", L_GRIP),
             _w(0.8, beside + [0.0, -TIP_PUSH + 0.05, 0.01], FINGER_CLOSED, "draw back", L_GRIP)]
    left += _go_home("left", left[-1].pos, L_GRIP, FINGER_CLOSED)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, FINGER_CLOSED,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
