"""Childproof a low cabinet: fit a U-shaped slide lock across its two door knobs.

A low two-door cabinet (a TV console) stands on the floor in front of the robot: two doors meeting in the middle, on
self-closing hinges, each with a round knob near the meeting edge (knobs 9 cm apart). On the floor in front of it lie
the two parts of a cabinet slide lock: the U (two 6 mm plastic rods 17 cm long, 2.1 cm apart inside, joined at one end
by a flat grip) and the lock block (5.7 cm tall, two holes that slide along the rods).

Task: fit the lock so that it holds the two doors together, then take both hands away.

The U's rods have to go behind both knob heads, one rod above both knob stems and one below, and slide in until its
end meets the first stem; the U has to be held there while the block's two holes are put onto the two rod ends beyond
the second knob and the block is slid along the rods until it meets the second stem. Fitted snug, the U and the block
enclose both stems, and the doors cannot be pulled apart.

Run it:  inspect-robots run --embodiment yam_arena -E task=fit_a_slide_lock_across_two_cabinet_knobs.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the cabinet (a low TV console on the floor): door fronts at x = XD ------------------------------------------------------------------------------
XD, DOOR_T = 0.370, 0.018
DOOR_Y0, DOOR_Y1 = 0.0015, 0.295  # each door's inner (meeting) and outer (hinge) edge, |y|
DOOR_Z0, DOOR_Z1 = TOP + 0.004, TOP + 0.270
DOOR_MASS = 1.10  # a 29 x 27 cm door of 19 mm MDF
HINGE_SPRING = 0.004  # N m per degree: self-closing hinges, about 0.1 N m holding a door shut
# -- the knobs (on the doors) ---------------------------------------------------------------------------------------
KY, KZ = 0.045, TOP + 0.200  # each knob's centre: y = +-KY
STEM_R, STEM_L = 0.006, 0.025  # its stem, from the door face out
HEAD_R, HEAD_T = 0.016, 0.012  # its head
# -- the lock: the U (its frame: origin at the middle of the grip's inner face, rods along -y, z up) --------------------
ROD_R, ROD_DZ, ROD_L = 0.003, 0.0135, 0.170  # rod radius; rods at z = +-ROD_DZ; length from the grip
GRIP = (0.010, 0.012, 0.040)  # the U's flat grip: x (thickness), y, z
U_MASS = 0.030
# the block (its frame: origin between its holes; holes along y at z = +-ROD_DZ)
BLK_Y, BLK_X0, BLK_X1, BLK_H = 0.020, -0.0085, 0.0065, 0.057  # length along the rods; x extent; height
HOLE, FUNNEL = 0.0090, 0.0040  # square holes 9 mm; the entry face chamfered 4 mm deep
BLK_MASS = 0.025
# where the parts start: lying flat on the floor in front of the cabinet, rods along -y
U_START = np.array([0.200, 0.150, TOP + GRIP[0] / 2])
BLK_START = np.array([0.200, -0.150, TOP - BLK_X0 - 0.002])
# -- the rules -------------------------------------------------------------------------------------------------------
X_LEGS = XD - STEM_L / 2  # the rods' plane, fitted: halfway along the stems
SNUG = 2 * KY + 2 * STEM_R + 0.010  # inner length of the fitted lock for a pass: the stems' span plus 1 cm
OUT_X = XD - 0.060  # a fingertip centre in front of this is clear of the lock (2 cm in front of the knob heads)
SUCCESS_HOLD_S = 1.0

WHITE, NICKEL, SHADOW, TOPC = (0.93, 0.93, 0.91), (0.72, 0.73, 0.74), (0.10, 0.10, 0.10), (0.55, 0.55, 0.52)
U_COLOR, BLK_COLOR = (0.95, 0.95, 0.93), (0.90, 0.90, 0.88)


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


LYING = _ry(90.0)  # how the U and the block lie on the floor: their z axis along +x, their x axis down
LYING_Q = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


# -- the cabinet (kinematic): sides, face frame, back, countertop -----------------------------------------------------------

def _cabinet() -> Body:
    o = np.array([XD + 0.12, 0.0, TOP])
    r = lambda p: np.asarray(p, float) - o  # noqa: E731
    f0 = XD + DOOR_T + 0.0005  # the face frame's front
    sh = [_box(r((f0, 0.275, TOP)), r((f0 + 0.018, 0.315, TOP + 0.280)), WHITE),  # face frame: left stile
          _box(r((f0, -0.315, TOP)), r((f0 + 0.018, -0.275, TOP + 0.280)), WHITE),  # right stile
          _box(r((f0, -0.315, TOP + 0.250)), r((f0 + 0.018, 0.315, TOP + 0.280)), WHITE),  # top rail
          _box(r((f0, -0.315, TOP)), r((0.76, -0.297, TOP + 0.280)), WHITE),  # the sides
          _box(r((f0, 0.297, TOP)), r((0.76, 0.315, TOP + 0.280)), WHITE),
          _box(r((0.74, -0.315, TOP)), r((0.76, 0.315, TOP + 0.280)), SHADOW),  # the back, in shadow
          _box(r((XD - 0.022, -0.330, TOP + 0.280)), r((0.77, 0.330, TOP + 0.300)), TOPC)]  # the top board
    return Body.build("cabinet", pos=tuple(o.tolist()), mass=20.0, kinematic=True, color=WHITE, shapes=sh,
                      friction=(0.5, 0.4))


def _door(side: float) -> Body:
    """side +1: the left door (hinged at its left edge, y = +DOOR_Y1), -1: the right."""
    yc = side * (DOOR_Y0 + DOOR_Y1) / 2
    o = np.array([XD + DOOR_T / 2, yc, (DOOR_Z0 + DOOR_Z1) / 2])
    r = lambda p: np.asarray(p, float) - o  # noqa: E731
    sh = [_box(r((XD, side * DOOR_Y0 if side > 0 else -DOOR_Y1, DOOR_Z0)),
               r((XD + DOOR_T, DOOR_Y1 if side > 0 else -DOOR_Y0, DOOR_Z1)), WHITE)]
    # a shallow frame on the door face (shaker style): four strips, render only
    for lo, hi in (((XD - 0.002, side * 0.012, DOOR_Z1 - 0.036), (XD, side * 0.284, DOOR_Z1 - 0.030)),
                   ((XD - 0.002, side * 0.012, DOOR_Z0 + 0.030), (XD, side * 0.284, DOOR_Z0 + 0.036))):
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        sh.append(_box(r(lo), r(hi), (0.86, 0.86, 0.84), collide=False))
    # the knob: stem from the door face out, then the head
    sh.append(CylShape(tuple(r((XD - STEM_L / 2, side * KY, KZ)).tolist()), STEM_R, STEM_L + 0.001, NICKEL, axis="X"))
    sh.append(CylShape(tuple(r((XD - STEM_L - HEAD_T / 2, side * KY, KZ)).tolist()), HEAD_R, HEAD_T, NICKEL, axis="X"))
    sh.append(CylShape(tuple(r((XD - STEM_L - HEAD_T - 0.0015, side * KY, KZ)).tolist()), HEAD_R - 0.004, 0.003, NICKEL,
                       axis="X"))
    return Body.build("door_left" if side > 0 else "door_right", pos=tuple(o.tolist()), mass=DOOR_MASS, shapes=sh,
                      color=WHITE, friction=(0.5, 0.4), solver_iterations=32, damping=(0.0, 0.5))


for _s, _name in ((1.0, "door_left"), (-1.0, "door_right")):
    hinge(f"{_name}_hinge", "cabinet", _name, anchor=(XD + DOOR_T, _s * DOOR_Y1, (DOOR_Z0 + DOOR_Z1) / 2), axis="Z",
          limits_deg=(-95.0, 0.0) if _s > 0 else (0.0, 95.0), spring=HINGE_SPRING, damping=0.002,
          target_deg=2.0 * _s)  # pulled a little past shut: held against the face frame


# -- the lock -------------------------------------------------------------------------------------------------------------

def _u_shapes():
    sh = [BoxShape((0.0, GRIP[1] / 2, 0.0), GRIP, U_COLOR)]
    for s in (-1.0, 1.0):
        sh.append(CylShape((0.0, -ROD_L / 2, s * ROD_DZ), ROD_R, ROD_L, U_COLOR, axis="Y"))
        sh.append(CylShape((0.0, -ROD_L - 0.0025, s * ROD_DZ), ROD_R - 0.0012, 0.005, U_COLOR, axis="Y"))  # its end
    return sh


def _block_shapes():
    """The block: two square holes along y with chamfered entries on its +y face (the face the rod ends go into)."""
    y0, y1 = -BLK_Y / 2, BLK_Y / 2
    yi = y1 - FUNNEL  # where the chamfers end and the 9 mm holes begin
    h, f = HOLE / 2, HOLE / 2 + FUNNEL
    x0, x1, z1 = BLK_X0, BLK_X1, BLK_H / 2
    c = BLK_COLOR
    sh = []
    # the solid parts round the holes, along their whole length (front, back, top, bottom, middle)
    sh.append(_box((x0, y0, -z1), (-f, y1, z1), c))  # front (towards the robot)
    sh.append(_box((f, y0, -z1), (x1, y1, z1), c)) if x1 > f else None
    sh.append(_box((-f, y0, ROD_DZ + f), (f, y1, z1), c))  # top
    sh.append(_box((-f, y0, -z1), (f, y1, -ROD_DZ - f), c))  # bottom
    sh.append(_box((-f, y0, -ROD_DZ + f), (f, y1, ROD_DZ - f), c))  # middle
    # behind the chamfers, the holes' own walls (between f and h)
    for zc in (ROD_DZ, -ROD_DZ):
        sh.append(_box((-f, y0, zc + h), (f, yi, zc + f), c))
        sh.append(_box((-f, y0, zc - f), (f, yi, zc - h), c))
        sh.append(_box((-f, y0, zc - h), (-h, yi, zc + h), c))
        sh.append(_box((h, y0, zc - h), (min(f, x1), yi, zc + h), c))
        # the chamfers: four 45-degree plates from the hole's edge at yi out to the entry face at y1
        for n in ((0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)):
            n = np.array(n)
            p1 = np.array([0.0, yi, zc]) + n * (h + 0.0007)
            p2 = np.array([0.0, y1, zc]) + n * (f + 0.0007)
            sh.append(plate(tuple(p1.tolist()), tuple(p2.tolist()), 2 * f, 0.0014, tuple((n - np.array([0, 1.0, 0]))
                                                                                     .tolist()), c))
    # a release button on the front, render only
    sh.append(_box((x0 - 0.002, -0.005, -0.006), (x0, 0.005, 0.006), (0.55, 0.75, 0.90), collide=False))
    return [s for s in sh if s is not None]


OBJECTS = [_cabinet(), _door(1.0), _door(-1.0),
           Body.build("u_bar", pos=tuple(U_START.tolist()), mass=U_MASS, shapes=_u_shapes(), color=U_COLOR,
                      start_xyzw=LYING_Q, friction=(0.5, 0.4), solver_iterations=32, contact_offset=0.0015),
           Body.build("lock_block", pos=tuple(BLK_START.tolist()), mass=BLK_MASS, shapes=_block_shapes(),
                      color=BLK_COLOR, start_xyzw=LYING_Q, friction=(0.5, 0.4), solver_iterations=32,
                      contact_offset=0.0015)]


# -- followed after every control step --------------------------------------------------------------------------------------

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


def _door_angle(rd) -> float:
    x_axis = rd @ np.array([1.0, 0.0, 0.0])
    return math.degrees(math.atan2(float(x_axis[1]), float(x_axis[0])))


def measure(pu, ru, pb, rb, door_deg, tcps) -> dict:
    """The lock from the U's pose (pu: the middle of its grip's inner face; ru), the block's (pb; rb), the doors'
    angles (degrees, 0 shut) and where the fingertip centres are."""
    u_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ru[2, 2])))))
    stems = []
    for s in (1.0, -1.0):
        stems.append(ru.T @ (np.array([X_LEGS, s * KY, KZ]) - pu))  # each stem's middle, in the U's frame
    inside = (-ROD_L + 0.005 < min(st[1] for st in stems)) and max(st[1] for st in stems) < -STEM_R + 0.002
    straddles = all(abs(float(st[2])) <= ROD_DZ - ROD_R - STEM_R + 0.0015 for st in stems)
    in_gap = XD - STEM_L + ROD_R - 0.001 <= float(pu[0]) <= XD - ROD_R + 0.001
    u_on = bool(inside and straddles and in_gap and u_tilt < 15.0)
    b = ru.T @ (pb - pu)  # the block in the U's frame
    rel = ru.T @ rb
    b_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rel[2, 2])))))
    b_yaw = math.degrees(math.acos(max(-1.0, min(1.0, float(rel[1, 1])))))
    engaged = float(b[1]) + BLK_Y / 2 >= -ROD_L + 0.005 and float(b[1]) - BLK_Y / 2 <= -0.002
    b_on = bool(abs(float(b[0])) < 0.003 and abs(float(b[2])) < 0.003 and engaged and b_tilt < 8.0 and b_yaw < 8.0)
    length = -(float(b[1]) + BLK_Y / 2)  # the fitted lock's inner length: grip face to block face
    snug = b_on and length <= SNUG
    shut = all(abs(a) < 3.0 for a in door_deg)
    out = all(float(t[0]) < OUT_X for t in tcps) if tcps else True
    return {"u_on": u_on, "b_on": b_on, "length": length, "snug": snug, "shut": shut, "hands_out": out,
            "u_tilt": u_tilt, "b_rel": b, "stems": stems}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pu, ru = _pose(base, origin, "u_bar")
    pb, rb = _pose(base, origin, "lock_block")
    doors = [_door_angle(_pose(base, origin, n)[1]) for n in ("door_left", "door_right")]
    _EP["tcps"] = _tcps(base, origin)
    s = measure(pu, ru, pb, rb, doors, _EP["tcps"])
    last = _EP.get("last")
    cur = np.concatenate([pu, pb])
    s["still"] = last is not None and float(np.abs(cur - last).max()) < 0.0003
    _EP["last"] = cur
    _EP["s"] = s
    _EP["steps"] += 1


# -- scoring --------------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the lock fitted snug across both knobs (the U's rods behind both knob heads, one above and one below both
    stems; the block on both rods; grip to block at most the stems' span plus 1 cm, so the doors cannot be pulled
    more than about 1 cm apart), both doors shut and both fingertip centres 6 cm or more in front of the doors; 0.75:
    fitted snug and shut, a hand still on it; 0.5: the U round both stems and the block on both rods, not snug (or a
    door open); 0.25: the U round both stems, the block not on both rods; 0 otherwise."""
    s = _EP["s"]
    if not s or not s["u_on"]:
        return 0.0
    if s["b_on"] and s["snug"] and s["shut"]:
        return 1.0 if s["hands_out"] else 0.75
    return 0.5 if s["b_on"] else 0.25


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP["s"].get("still", False)


# -- scripted demo ------------------------------------------------------------------------------------------------------------
# The left hand picks the U up by its grip (straight down, fingers on the grip's ends), turns it upright with its rods
# pointing right, brings it beside the left knob and slides it in behind both knob heads until the grip nearly meets
# the left stem, and holds it there. The right hand picks the block up the same way, brings it beyond the right knob
# with its holes on the rods' line, pushes it onto the rod ends and along them until it meets the right stem. Both let go
# and draw back.

LEAD_S, LEAD_STEPS = 3.0, 15
PITCH = 0.0  # both hands level when fitting
CZ = {"left": -1.0, "right": 1.0}  # each hand's closing direction when fitting (down for the left, up for the right)
U_AIM = np.array([-0.008, 0.0, 0.0])  # the U sits 8 mm out past the fingertip centre once picked off the floor
B_AIM = np.array([0.0, 0.0, -0.014])  # picked off the floor, the block sits 19 mm off the fingers' middle
GRIP_OPEN = -0.030
U_OPEN = -0.0235  # 47 mm: 3.5 mm each side of the U's 40 mm grip
B_OPEN = -0.0340  # 68 mm: 5.5 mm each side of the 57 mm block


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

    pr = PITCH
    fit = {arm: _frame((math.cos(pr), 0.0, -math.sin(pr)), (0.0, 0.0, CZ[arm])) for arm in ("left", "right")}
    down = {arm: LYING @ fit[arm] for arm in ("left", "right")}  # the same grip on the part lying on the floor

    # -- the left hand: the U ----------------------------------------------------------------------------------------
    g_u = np.array([0.0, GRIP[1] / 2, 0.0])  # where the U is held: the middle of its grip
    u_pick = U_START + LYING @ g_u
    lead("left", u_pick + np.array([0.0, 0.0, 0.08]), down["left"], U_OPEN, "above the U's grip, open")
    go("left", 1.2, u_pick, U_OPEN, "down round the grip's ends", n=2)
    go("left", 0.6, u_pick, FINGER_CLOSED, "take the U")
    go("left", 1.0, u_pick + np.array([0.0, 0.0, 0.10]), FINGER_CLOSED, "lift it")
    u_fit = np.array([X_LEGS, 2 * STEM_R / 2 + KY + 0.004, KZ + 0.0045]) + U_AIM  # the grip's face 4 mm from the stem
    u_pre = u_fit + np.array([0.0, ROD_L + 0.035, 0.0])  # rod ends 3.5 cm left of the fitted grip line... clear
    lead("left", u_pre + g_u + np.array([-0.06, 0.0, 0.0]), fit["left"], FINGER_CLOSED, "turn it upright, rods right")
    go("left", 1.2, u_pre + g_u, FINGER_CLOSED, "rods level with the knob stems, beside the left knob", n=2)
    go("left", 4.0, u_fit + g_u, FINGER_CLOSED, "slide it in behind both knob heads", n=8)

    # -- the right hand: the block ------------------------------------------------------------------------------------
    b_pick = BLK_START.copy()
    lead("right", b_pick + np.array([0.0, 0.0, 0.08]), down["right"], B_OPEN, "above the block, open")
    go("right", 1.2, b_pick, B_OPEN, "down round its ends", n=2)
    go("right", 0.6, b_pick, FINGER_CLOSED, "take the block")
    go("right", 1.0, b_pick + np.array([0.0, 0.0, 0.10]), FINGER_CLOSED, "lift it")
    tips_y = u_fit[1] - ROD_L
    b_fit = np.array([X_LEGS, -KY - STEM_R - BLK_Y / 2 - 0.001, u_fit[2]]) + B_AIM
    b_pre = np.array([X_LEGS, tips_y - BLK_Y / 2 - 0.012, u_fit[2]]) + B_AIM
    lead("right", b_pre + np.array([-0.06, -0.02, 0.0]), fit["right"], FINGER_CLOSED, "turn it upright, holes along")
    t0 = max(dur("left"), dur("right"))
    wait("right", t0 - dur("right"), "wait for the U")
    go("right", 1.2, b_pre, FINGER_CLOSED, "holes in line with the rod ends", n=2)
    go("left", 0.0001, cur["left"][0], FINGER_CLOSED, "hold the U")
    wait("left", dur("right") - dur("left"), "hold the U where it is")
    go("right", 3.5, b_fit, FINGER_CLOSED, "onto the rods and along them to the right stem", n=7)
    wait("left", dur("right") - dur("left") + 0.5, "hold the U where it is")
    wait("right", 0.5, "steady")
    # -- let go and draw back -----------------------------------------------------------------------------------------
    for arm in ("left", "right"):
        p, f, r = cur[arm]
        go(arm, 0.6, p, GRIP_OPEN, "let go")
        go(arm, 1.2, p - r[:, 2] * 0.10, GRIP_OPEN, "draw the hand back", n=2)
    home("left")
    home("right")
    end = max(dur("left"), dur("right")) + 2.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still")
    return plan
