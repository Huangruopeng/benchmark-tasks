"""Refit the two washed air filters in a wall-mounted mini-split's indoor unit.

A ductless mini-split's indoor unit hangs on the wall in front of the robot (82 cm wide). Its front panel has been lifted
off its hinges for cleaning and set aside, so the filter bay is open, and it is empty: the unit's two mesh air filters
were taken out and washed. They lie dry, face up, on the table in front of the unit, each with its pull tab standing up
at the edge nearer the robot.

Task: put both filters back in, each in its own bay, and take the hands away.

Each filter (33.5 x 18 cm, 5 mm frame) goes back the way it came out. Its top edge has to go up into a pocket behind
a lip at the top of the bay, so the filter is tilted top-in under the lip first, then swung upright while it is
lifted, so its bottom edge clears the two lugs at the bottom of the bay, and then let down behind the lugs onto the
ledge. Seated, it is held at the top by the lip and at the bottom by the lugs.

Run it:  inspect-robots run --embodiment yam_arena -E task=refit_a_mini_splits_washed_air_filters.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the indoor unit (kinematic), on the wall; heights above the table -----------------------------------------------
X_FRONT, X_BACK, X_WALL = 0.385, 0.625, 0.645  # the unit's front, its back on the wall, the wall's face behind
Y_HALF = 0.410  # half its width (82 cm)
Z_UNDER, Z_TOP = 0.140, 0.405  # its underside and top
XF = 0.430  # the filters' mid-plane, seated
F_T, F_W, F_H = 0.005, 0.335, 0.180  # a filter's frame thickness, width and height
BAY_Y = (0.180, -0.180)  # the two bays' centres (left, right)
Z_LEDGE = 0.180  # a seated filter's bottom edge rests here
LUG = (XF - 0.0085, XF - 0.0035, Z_LEDGE, Z_LEDGE + 0.005)  # the bottom lugs in front of the filter's bottom edge: x0 x1 z0 z1
LUG_DY = 0.120  # two lugs per bay, this far either side of its centre
LIP = (XF - 0.0085, XF - 0.0040, 0.350, 0.380)  # the top lip in front of the pocket: x0 x1 z0 z1
POCKET_TOP = 0.376  # the pocket's ceiling: a seated filter (top edge at 0.360) can be lifted 16 mm
GRILLE_X = XF + 0.003  # the coil grille's face, behind the filters
X_COIL = XF + 0.015  # the body behind the grille
Z_BODY, Z_COVER = 0.165, 0.380  # the top of the body under the bay; the underside of the top cover
# -- the filters' frame (installed orientation; origin at the middle of the bottom edge, on the mid-plane) -----------
TAB = (-0.0325, -0.0025, 0.002, 0.005, 0.012)  # the pull tab: x0 x1 (out from the frame), z0 z1, half width
F_MASS = 0.045
F_LYING_X = 0.120  # lying face up on the table, each filter's bottom edge (with its tab) is this far forward
# -- the rules ------------------------------------------------------------------------------------------------------
SEAT_X, SEAT_Z, SEAT_TILT, SEAT_Y = 0.0025, 0.003, 3.0, 0.010
OUT_X = X_FRONT - 0.030  # a fingertip centre in front of this is out of the unit
SUCCESS_HOLD_S = 1.0

WHITE, GREY, DARK, MESH, FRAME, WALL = (0.94, 0.94, 0.93), (0.80, 0.80, 0.79), (0.15, 0.16, 0.17), (0.55, 0.57, 0.60), \
    (0.40, 0.42, 0.45), (0.86, 0.84, 0.80)


def _box(lo, hi, color, collide=True, rel=None) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if rel is not None:
        lo, hi = rel(lo), rel(hi)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def _q_ry(deg: float) -> tuple:
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


LYING = _ry(90.0)  # a filter lying face up: its height along +x, its tab pointing up


# -- the unit and the wall -------------------------------------------------------------------------------------------------

def _unit() -> Body:
    o = np.array([0.53, 0.0, TOP + 0.26])
    r = lambda p: np.asarray(p, float) - o  # noqa: E731
    t = TOP
    sh = [
        _box((X_WALL, -0.60, t), (X_WALL + 0.02, 0.60, t + 0.62), WALL, rel=r),  # the wall
        _box((X_FRONT, -Y_HALF, t + Z_UNDER), (X_BACK, Y_HALF, t + Z_BODY), WHITE, rel=r),  # lower body
        _box((X_FRONT, -Y_HALF, t + Z_COVER), (X_BACK, Y_HALF, t + Z_TOP), WHITE, rel=r),  # top cover
        _box((X_FRONT, -Y_HALF, t + Z_BODY), (X_BACK, -0.390, t + Z_COVER), WHITE, rel=r),  # right end
        _box((X_FRONT, 0.390, t + Z_BODY), (X_BACK, Y_HALF, t + Z_COVER), WHITE, rel=r),  # left end
        _box((X_COIL, -0.390, t + Z_BODY), (X_BACK, 0.390, t + Z_COVER), GREY, rel=r),  # the body behind the coil
        _box((GRILLE_X, -0.390, t + Z_BODY), (X_COIL, 0.390, t + POCKET_TOP), DARK, rel=r),  # the coil's grille
        _box(((XF - 0.0035), -0.390, t + Z_BODY), (GRILLE_X, 0.390, t + Z_LEDGE), GREY, rel=r),  # the ledge
        _box((LIP[0], -0.390, t + LIP[2]), (LIP[1], 0.390, t + LIP[3]), WHITE, rel=r),  # the top lip
        _box((LIP[1], -0.390, t + POCKET_TOP), (GRILLE_X, 0.390, t + Z_COVER), GREY, rel=r),  # the pocket's ceiling
        _box(((XF - 0.0085), -0.006, t + Z_BODY), (GRILLE_X, 0.006, t + Z_COVER), WHITE, rel=r),  # the rib between the bays
    ]
    for yc in BAY_Y:
        s = 1.0 if yc > 0 else -1.0
        g = yc + s * (F_W / 2 + 0.003)  # the outer guide strip
        sh.append(_box(((XF - 0.0035), min(g, g + s * 0.003), t + Z_LEDGE), (GRILLE_X, max(g, g + s * 0.003), t + Z_LEDGE + F_H),
                       GREY, rel=r))
        for dy in (-LUG_DY, LUG_DY):
            sh.append(_box((LUG[0], yc + dy - 0.010, t + LUG[2]), (LUG[1], yc + dy + 0.010, t + LUG[3]), WHITE,
                           rel=r))
    # render-only: the display window and the louvre at the bottom
    sh.append(_box((X_FRONT - 0.001, 0.25, t + Z_UNDER + 0.010), (X_FRONT, 0.32, t + Z_UNDER + 0.020), (0.25, 0.55, 0.85),
                   False, rel=r))
    return Body.build("unit", pos=tuple(o.tolist()), mass=12.0, kinematic=True, color=WHITE, shapes=sh,
                      friction=(0.4, 0.35))


def _filter_shapes():
    w, h, t = F_W / 2, F_H, F_T / 2
    sh = [BoxShape((0.0, 0.0, 0.005), (F_T, F_W, 0.010), FRAME),  # bottom rail
          BoxShape((0.0, 0.0, h - 0.005), (F_T, F_W, 0.010), FRAME),  # top rail
          BoxShape((0.0, w - 0.004, h / 2), (F_T, 0.008, h - 0.020), FRAME),  # side rails
          BoxShape((0.0, -w + 0.004, h / 2), (F_T, 0.008, h - 0.020), FRAME),
          BoxShape((0.0, 0.0, h / 2), (F_T, 0.008, h - 0.020), FRAME),  # middle ribs
          BoxShape((0.0, 0.0, h / 2), (F_T, F_W - 0.016, 0.008), FRAME),
          BoxShape((0.0, 0.0, h / 2), (0.001, F_W - 0.016, h - 0.020), MESH, collide=False)]  # the mesh
    x0, x1, z0, z1, hw = TAB
    sh.append(BoxShape(((x0 + x1) / 2, 0.0, (z0 + z1) / 2), (x1 - x0, 2 * hw, z1 - z0), FRAME))
    del t
    return sh


def _filter(name: str, yc: float) -> Body:
    return Body.build(name, pos=(F_LYING_X, yc, TOP + F_T / 2), mass=F_MASS, shapes=_filter_shapes(), color=FRAME,
                      start_xyzw=_q_ry(90.0), friction=(0.5, 0.4), solver_iterations=24, com=(0.0, 0.0, F_H / 2),
                      contact_offset=0.0015)


OBJECTS = [_unit(), _filter("filter_left", BAY_Y[0]), _filter("filter_right", BAY_Y[1])]


# -- followed after every control step ---------------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"s": {}, "tcps": None, "steps": 0, "last": None}


_EP = _fresh()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _tcps(base, origin):
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


def filter_seat(pf, rf, yc) -> dict:
    """How a filter sits against its bay (pf: the middle of its bottom edge, on its mid-plane; rf)."""
    up = rf @ np.array([0.0, 0.0, 1.0])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    top = pf + up * F_H
    dx, dz, dy = float(pf[0] - XF), float(pf[2] - (TOP + Z_LEDGE)), float(pf[1] - yc)
    seated = abs(dx) <= SEAT_X and abs(dz) <= SEAT_Z and tilt <= SEAT_TILT and abs(dy) <= SEAT_Y
    top_in = float(top[0]) > LIP[1] and TOP + LIP[2] < float(top[2]) <= TOP + POCKET_TOP + 0.002 and abs(dy) < 0.03
    return {"seated": bool(seated), "top_in": bool(top_in), "dx": dx, "dz": dz, "dy": dy, "tilt": tilt}


def measure(filters, tcps) -> dict:
    seats = [filter_seat(pf, rf, yc) for (pf, rf), yc in zip(filters, BAY_Y)]
    out = all(float(t[0]) < OUT_X for t in tcps) if tcps else True
    return {"seats": seats, "n_seated": sum(s["seated"] for s in seats), "n_top_in": sum(s["top_in"] for s in seats),
            "hands_out": out}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    filters = [_pose(base, origin, n) for n in ("filter_left", "filter_right")]
    _EP["tcps"] = _tcps(base, origin)
    s = measure(filters, _EP["tcps"])
    cur = np.concatenate([filters[0][0], filters[1][0]])
    last = _EP.get("last")
    s["still"] = last is not None and float(np.abs(cur - last).max()) < 0.0003
    _EP["last"] = cur
    _EP["s"] = s
    _EP["steps"] += 1


# -- scoring --------------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: both filters seated, each in its own bay (on the ledge behind its bottom lugs with its top edge in the
    pocket behind the lip: its bottom edge within 2.5 mm of the seat front to back and 3 mm in height, upright within
    3 degrees, centred in its bay within 10 mm), and both fingertip centres 3 cm or more in front of the unit; 0.75:
    both seated, a hand still at the unit; 0.5: one filter seated; 0.25: none seated, but a filter's top edge is in
    its pocket behind the lip; 0 otherwise."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["n_seated"] == 2:
        return 1.0 if s["hands_out"] else 0.75
    if s["n_seated"] == 1:
        return 0.5
    return 0.25 if s["n_top_in"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP["s"].get("still", False)


# -- scripted demo ------------------------------------------------------------------------------------------------------------
# Each hand takes one filter by its pull tab (straight down, fingers either side of the tab), lifts it and turns it
# upright, tilted top-in, in front of its bay. It slides the filter's top edge in under the lip, swings the filter
# upright while lifting it, so the top edge rises into the pocket and the bottom edge passes over the lugs, then lets it
# down onto the ledge, lets go and draws back. Both hands then go back to where they started.

LEAD_S, LEAD_STEPS = 3.0, 15
TILT0 = 28.0  # degrees: the filter's tilt, top-in, as its top goes under the lip
CZ = {"left": -1.0, "right": 1.0}
GRIP = np.array([-0.020, 0.0, 0.0035])  # where a filter is held: on its tab, 2 cm out from the frame (filter frame)
AIM = {arm: np.array([0.007, 0.0, 0.012]) for arm in ("left", "right")}  # the arms run about 7 mm short and 12 mm
# low of their targets here, holding a filter out at the bay
TAB_OPEN = -0.011  # 22 mm between the pads round the 3 mm tab
Z_CLEAR = 0.191  # the filter's bottom edge while it is swung upright (the lugs' tops are at 0.185)


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - np.dot(close, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _good_ik(arm: str, pos, rot, near=None) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    sols = []
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.3, 1.9), (1.0, 1.7), (-1.0, 0.0), (0.0,), (-1.0, 0.0, 1.0)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            sols.append((q, float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
    if not sols:
        raise ValueError(f"no IK solution for the {arm} hand at {np.round(pos, 4)}")
    if near is not None:
        roomy = [s for s in sols if s[1] > 0.15] or sols
        return min(roomy, key=lambda s: float(np.linalg.norm(s[0] - near)))[0]
    return max(sols, key=lambda s: s[1])[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs=LEAD_S, steps=LEAD_STEPS) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(secs / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_OPEN, r)

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def lead(arm, pos, rot, finger, label, secs=LEAD_S):
        p0, _, r0 = cur[arm]
        q0 = np.array(READY, float) if not plan[arm] else _good_ik(arm, p0, r0, near=np.array(READY, float))
        plan[arm] += _joint_path(arm, q0, _good_ik(arm, pos, rot, near=q0), label, finger, secs)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r, near=np.array(READY, float)), np.array(READY, float), label, f)
        cur[arm] = (np.asarray(plan[arm][-1].pos, float), f, plan[arm][-1].yaw)

    def held(arm, bottom, tilt):
        """The hand's pose for the filter's bottom-edge middle at ``bottom`` (world), tilted top-in by ``tilt``."""
        rf = _ry(tilt)
        fit = _frame((1.0, 0.0, 0.0), (0.0, 0.0, CZ[arm]))
        return np.asarray(bottom, float) + rf @ GRIP + AIM[arm], rf @ fit

    for arm, yc in (("left", BAY_Y[0]), ("right", BAY_Y[1])):
        fit = _frame((1.0, 0.0, 0.0), (0.0, 0.0, CZ[arm]))
        down = LYING @ fit
        f0 = np.array([F_LYING_X, yc, TOP + F_T / 2])
        g0 = f0 + LYING @ GRIP
        lead(arm, g0 + np.array([0.0, 0.0, 0.07]), down, TAB_OPEN, "above the filter's tab, open")
        go(arm, 1.2, g0, TAB_OPEN, "down round the tab", n=2)
        go(arm, 0.6, g0, FINGER_CLOSED, "take the tab")
        go(arm, 1.0, g0 + np.array([0.0, 0.0, 0.06]), FINGER_CLOSED, "lift it")
        # upright, tilted top-in, in front of the bay: its top edge 3 cm in front of where it goes, below the lip
        s0, c0 = math.sin(math.radians(TILT0)), math.cos(math.radians(TILT0))
        b_pre = np.array([XF - F_H * s0 - 0.030, yc, TOP + LIP[2] - 0.006 - F_H * c0])
        p, r = held(arm, b_pre + np.array([-0.04, 0.0, 0.0]), TILT0)
        lead(arm, p, r, FINGER_CLOSED, "turn it upright, tilted top-in, in front of its bay")
        p, r = held(arm, b_pre, TILT0)
        go(arm, 1.0, p, FINGER_CLOSED, "up to the bay", rot=r, n=2)
        p, r = held(arm, b_pre + np.array([0.030, 0.0, 0.0]), TILT0)
        go(arm, 1.5, p, FINGER_CLOSED, "top edge in under the lip", rot=r, n=3)
        # swing it upright about its top edge while lifting: the top rises into the pocket, the bottom over the lugs
        z_b0 = b_pre[2]
        for k in range(1, 9):
            th = TILT0 * (1 - k / 8)
            zb = z_b0 + (TOP + Z_CLEAR - z_b0) * k / 8
            b = np.array([XF - F_H * math.sin(math.radians(th)), yc, zb])
            p, r = held(arm, b, th)
            go(arm, 0.35, p, FINGER_CLOSED, "swing it upright into the pocket, over the lugs", rot=r)
        p, r = held(arm, np.array([XF, yc, TOP + Z_LEDGE - 0.001]), 0.0)
        go(arm, 1.0, p, FINGER_CLOSED, "let it down behind the lugs onto the ledge", rot=r, n=2)
        wait(arm, 0.4, "steady")
        go(arm, 0.6, p, TAB_OPEN, "let go")
        go(arm, 1.2, p + np.array([-0.07, 0.0, 0.0]), TAB_OPEN, "draw the hand out", n=2)
    home("left")
    home("right")
    end = max(dur("left"), dur("right")) + 3.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still")
    return plan
