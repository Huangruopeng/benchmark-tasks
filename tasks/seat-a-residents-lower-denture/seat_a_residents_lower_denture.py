"""Put a care-home resident's clean lower denture back in her mouth and seat it on her lower ridge.

A care-training manikin head (a seated resident) faces the robot with her mouth open: the mouth opening is 6.6 cm
wide and 4.9 cm high, her upper denture is in, and behind her lower lip her lower ridge (the toothless gum) runs
round in a U with her tongue inside it. Her clean lower full denture lies on a paper towel on the
bedside table, its front 15 mm over the table's edge.

Task: put the lower denture in her mouth, seated on her lower ridge, then take the hand away.

The denture is a horseshoe: a pink base with a channel underneath that fits over the ridge, and a row of teeth on
top. It is 6.1 cm across, only 5 mm narrower than the mouth, so it goes in square and centred; inside, it has to come
down onto the ridge back first, its two inner flanges either side of the tongue, then the front is let down and the
front teeth are pressed down until the channel sits on the ridge all round. The lower lip is soft (a sprung flap
here): the hand's lower finger has to push it down to get the denture's front low enough.

Run it:  inspect-robots run --embodiment yam_arena -E task=seat_a_residents_lower_denture.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the mouth (world): the lips' front plane, the mouth's centre ----------------------------------------------------
XL, ZM = 0.355, TOP + 0.200
LIP_T = 0.012  # the lips' thickness, front to back
OPEN_Y, OPEN_Z = 0.033, 0.023  # half the mouth opening's width and height
LIP_TOP = ZM - 0.026  # the lower lip's top, at rest (just below the ridge's crest)
CAV_Y, CAV_X1, FLOOR = 0.038, XL + 0.095, ZM - 0.046  # the mouth inside: half width, back, floor
# -- the lower ridge: a U of gum round the tongue, open at the back; its crest -------------------------------------
XC = XL + 0.050  # the arch's centre (the front arc's centre)
R_ARCH, ARCH_BACK = 0.020, 0.020  # the arch's centre line: front arc radius; the denture's sides run back this far
RIDGE_BACK = 0.032  # the ridge runs on back to the retromolar pads, past the denture's ends
RIDGE_W = 0.008
CREST = ZM - 0.025
TONGUE = (XC - 0.002, XL + 0.090, 0.010, ZM - 0.027)  # x0 (1 cm behind the front inner flange), x1, half width, top (below the crest)
# -- the lower lip: a flap hinged along its bottom, sprung up ---------------------------------------------------------
LIP_LO_Z0 = ZM - 0.047
LIP_SPRING = 0.0025  # N m per degree: about 2 N on its top edge pushes it down 1 cm
# -- the denture (its frame: origin on the arch centre at the channel's ceiling; z up; x back) -----------------------
D_FLANGE, D_BASE, D_TEETH = 0.007, 0.005, 0.009  # flange depth below the ceiling, base thickness, teeth height
D_MASS = 0.025
SEAT = np.array([XC, 0.0, CREST])  # the denture's origin when seated
# where it starts: on the bedside table, on a paper towel, its front 15 mm over the table's front edge
TABLE = (0.245, 0.360, -0.250, -0.060, TOP + 0.170)  # x0 x1 y0 y1 top
CUP = (0.315, -0.205, 0.040, TABLE[4] + 0.075)  # the empty denture cup beside it: x, y, lid radius, lid top
D_START = np.array([TABLE[0] - 0.015 + R_ARCH + 0.0105, -0.120, TABLE[4] + D_FLANGE])
# -- the rules -------------------------------------------------------------------------------------------------------
SEAT_XY, SEAT_Z, SEAT_YAW, SEAT_TILT = 0.0025, 0.002, 5.0, 4.0
OUT_X = XL - 0.030  # a fingertip centre in front of this is out of the mouth
SUCCESS_HOLD_S = 1.0

SKIN, LIPC, GUM, TOOTH, DARK, CUPC, WOOD = (0.86, 0.70, 0.60), (0.75, 0.45, 0.45), (0.85, 0.45, 0.50), \
    (0.96, 0.95, 0.90), (0.25, 0.12, 0.12), (0.55, 0.75, 0.90), (0.60, 0.45, 0.30)


def _box(lo, hi, color, collide=True, rel=None) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if rel is not None:
        lo, hi = rel(lo), rel(hi)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _yawq(a: float) -> tuple:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _arch(radius_mid: float, width: float, z0: float, z1: float, color, n_arc=9, n_side=2, collide=True, back=None):
    """Boxes along the U (frame: arch centre at the origin, front toward -x, sides running back to +ARCH_BACK)."""
    out = []
    for k in range(n_arc):  # the front arc, from the left side round to the right
        a0 = math.pi / 2 + math.pi * k / n_arc
        a1 = math.pi / 2 + math.pi * (k + 1) / n_arc
        am = (a0 + a1) / 2
        length = 2 * radius_mid * math.sin((a1 - a0) / 2) + 0.0012
        c = (radius_mid * math.cos(am), radius_mid * math.sin(am), (z0 + z1) / 2)
        out.append(BoxShape(c, (width, length, z1 - z0), color, quat_xyzw=_yawq(am), collide=collide))
    back = ARCH_BACK if back is None else back
    for s in (-1.0, 1.0):  # the straight sides
        for j in range(n_side):
            x0, x1 = back * j / n_side, back * (j + 1) / n_side
            out.append(BoxShape(((x0 + x1) / 2, s * radius_mid, (z0 + z1) / 2), (x1 - x0 + 0.0008, width, z1 - z0),
                                color, collide=collide))
    return out


# -- the resident's head (kinematic): face, upper lip and mouth corners, the inside of the mouth -----------------------

def _head() -> Body:
    o = np.array([XL + 0.06, 0.0, ZM])
    r = lambda p: np.asarray(p, float) - o  # noqa: E731
    f0, f1 = XL, XL + LIP_T
    sh = [
        _box((f0, -0.050, ZM + OPEN_Z), (f1, 0.050, ZM + 0.045), LIPC, rel=r),  # upper lip
        _box((f0, OPEN_Y, ZM - 0.047), (f1, 0.050, ZM + OPEN_Z), SKIN, rel=r),  # left of the mouth
        _box((f0, -0.050, ZM - 0.047), (f1, -OPEN_Y, ZM + OPEN_Z), SKIN, rel=r),  # right of the mouth
        _box((XL - 0.010, -0.045, ZM - 0.085), (f1, 0.045, ZM - 0.047), SKIN, rel=r),  # chin
        _box((XL - 0.024, -0.012, ZM + 0.050), (XL, 0.012, ZM + 0.090), SKIN, rel=r),  # nose
        _box((f0, -0.075, ZM + 0.045), (XL + 0.12, 0.075, ZM + 0.150), SKIN, rel=r),  # the face above
        _box((f1, -0.075, ZM - 0.085), (XL + 0.12, -0.050, ZM + 0.045), SKIN, rel=r),  # cheeks
        _box((f1, 0.050, ZM - 0.085), (XL + 0.12, 0.075, ZM + 0.045), SKIN, rel=r),
        # inside the mouth
        _box((f1, -CAV_Y, FLOOR - 0.010), (CAV_X1, CAV_Y, FLOOR), DARK, rel=r),  # floor
        _box((f1, CAV_Y, FLOOR), (CAV_X1, 0.050, ZM + 0.040), DARK, rel=r),  # inner cheeks
        _box((f1, -0.050, FLOOR), (CAV_X1, -CAV_Y, ZM + 0.040), DARK, rel=r),
        _box((CAV_X1, -0.050, FLOOR), (CAV_X1 + 0.010, 0.050, ZM + 0.040), DARK, rel=r),  # throat
        _box((f1, -0.034, ZM + OPEN_Z), (XL + 0.070, 0.034, ZM + 0.040), TOOTH, rel=r),  # upper denture, in place
        _box((TONGUE[0], -TONGUE[2], FLOOR), (TONGUE[1], TONGUE[2], TONGUE[3]), LIPC, rel=r),  # tongue
        _box((XL + 0.10, -0.050, ZM - 0.200), (XL + 0.18, 0.050, ZM - 0.090), SKIN, collide=False, rel=r),  # neck
        _box((XL + 0.06, -0.200, ZM - 0.330), (XL + 0.26, 0.200, ZM - 0.200), (0.55, 0.65, 0.80), collide=False,
             rel=r),  # shoulders, in a gown
    ]
    sh.append(SphereShape(tuple(r((XL + 0.140, 0.0, ZM + 0.070)).tolist()), 0.090, SKIN, collide=False))  # skull
    for sy in (-1.0, 1.0):
        sh.append(SphereShape(tuple(r((XL + 0.125, sy * 0.090, ZM + 0.040)).tolist()), 0.022, SKIN, collide=False))  # ears
    sh.append(SphereShape(tuple(r((XL + 0.145, 0.0, ZM + 0.100)).tolist()), 0.092, (0.85, 0.85, 0.85),
                          collide=False))  # grey hair
    ridge = _arch(R_ARCH, RIDGE_W, FLOOR - ZM, CREST - ZM, GUM, back=RIDGE_BACK)
    for b in ridge:  # into the head's frame
        sh.append(BoxShape(tuple((np.array(b.centre) + np.array([XC, 0.0, ZM]) - o).tolist()), b.size, GUM,
                           quat_xyzw=b.quat_xyzw))
    # the bedside table and the denture cup (kinematic, part of the scene)
    cx, cy, cr, ctop = CUP
    tx0, tx1, ty0, ty1, ttop = TABLE
    sh.append(_box((tx0, ty0, TOP), (tx1, ty1, ttop), WOOD, rel=r))  # bedside table
    sh.append(_box((tx0 + 0.005, -0.150, ttop), (tx0 + 0.065, -0.090, ttop + 0.0005), (0.97, 0.97, 0.97), False,
                   rel=r))  # paper towel
    sh.append(CylShape(tuple(r((cx, cy, ctop - 0.0375)).tolist()), cr - 0.004, 0.071, CUPC))  # cup
    sh.append(CylShape(tuple(r((cx, cy, ctop - 0.002)).tolist()), cr, 0.004, CUPC))  # lid
    return Body.build("head", pos=tuple(o.tolist()), mass=5.0, kinematic=True, color=SKIN, shapes=sh,
                      friction=(0.6, 0.5))


def _lower_lip() -> Body:
    o = np.array([XL + LIP_T / 2, 0.0, (LIP_LO_Z0 + LIP_TOP) / 2])
    sh = [BoxShape((0.0, 0.0, 0.0), (LIP_T, 2 * OPEN_Y - 0.002, LIP_TOP - LIP_LO_Z0), LIPC)]
    return Body.build("lower_lip", pos=tuple(o.tolist()), mass=0.02, shapes=sh, color=LIPC, friction=(0.6, 0.5),
                      solver_iterations=24, damping=(0.5, 2.0))


hinge("lower_lip_hinge", "head", "lower_lip", anchor=(XL + LIP_T, 0.0, LIP_LO_Z0), axis="Y", limits_deg=(-40.0, 0.5),
      spring=LIP_SPRING, damping=0.0004, target_deg=0.3)


def _denture() -> Body:
    sh = _arch(R_ARCH + 0.001, 0.019, 0.0, D_BASE, GUM)  # the base over the channel
    sh += _arch(R_ARCH + 0.0085, 0.004, -D_FLANGE, 0.0, GUM)  # outer flange: a 4 mm rolled border
    sh += _arch(R_ARCH - 0.0075, 0.002, -D_FLANGE, 0.0, GUM)  # inner flange (13 mm channel over the 8 mm ridge)
    sh += _arch(R_ARCH, 0.008, D_BASE, D_BASE + D_TEETH, TOOTH, n_arc=9, n_side=2)  # the teeth
    return Body.build("denture", pos=tuple(D_START.tolist()), mass=D_MASS, shapes=sh, color=GUM,
                      friction=(0.5, 0.4), solver_iterations=32, com=(-0.004, 0.0, 0.004), contact_offset=0.0012)


OBJECTS = [_head(), _lower_lip(), _denture()]


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


def measure(pd, rd, tcps) -> dict:
    up = rd @ np.array([0.0, 0.0, 1.0])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    fwd = rd @ np.array([1.0, 0.0, 0.0])
    yaw = abs(math.degrees(math.atan2(float(fwd[1]), float(fwd[0]))))
    off = float(np.hypot(pd[0] - SEAT[0], pd[1] - SEAT[1]))
    dz = float(pd[2] - SEAT[2])
    in_mouth = pd[0] > XL + 0.020 and abs(pd[1]) < CAV_Y and FLOOR - 0.005 < pd[2] < ZM + OPEN_Z
    seated = in_mouth and off <= SEAT_XY and abs(dz) <= SEAT_Z and yaw <= SEAT_YAW and tilt <= SEAT_TILT
    on_ridge = in_mouth and off <= 0.008 and abs(dz) <= 0.008 and tilt <= 15.0 and yaw <= 15.0
    out = all(float(t[0]) < OUT_X for t in tcps) if tcps else True
    return {"off": off, "dz": dz, "yaw": yaw, "tilt": tilt, "in_mouth": bool(in_mouth), "seated": bool(seated),
            "on_ridge": bool(on_ridge), "hands_out": out}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pd, rd = _pose(base, origin, "denture")
    _EP["tcps"] = _tcps(base, origin)
    s = measure(pd, rd, _EP["tcps"])
    last = _EP.get("last")
    s["still"] = last is not None and float(np.linalg.norm(pd - last)) < 0.0003
    _EP["last"] = pd
    _EP["s"] = s
    _EP["steps"] += 1


# -- scoring --------------------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the denture seated on the lower ridge (centred within 2.5 mm, its channel within 2 mm of sitting on the
    ridge, turned under 5 degrees, tilted under 4 degrees) and both fingertip centres 3 cm or more in front of the
    lips; 0.75: seated, a hand still at the mouth; 0.5: on the ridge but not seated (within 8 mm, tilted under 15
    degrees); 0.25: in the mouth, not on the ridge; 0 otherwise (still out of the mouth, or dropped)."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["seated"]:
        return 1.0 if s["hands_out"] else 0.75
    if s["on_ridge"]:
        return 0.5
    return 0.25 if s["in_mouth"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and _EP["s"].get("still", False)


# -- scripted demo ------------------------------------------------------------------------------------------------------------
# The right hand takes the denture by its front (fingers above its front teeth and under its front flange, where it
# overhangs the cup's lid), carries it level to the mouth and slides it in, square and centred, above the ridge. It
# tips the denture's back down onto the ridge (the hand pitching down, its lower finger pushing the lower lip down),
# lets go so the front drops onto the ridge, then presses the front teeth down with its closed fingers and draws back.

LEAD_S, LEAD_STEPS = 3.0, 15
GRIP = np.array([-R_ARCH - 0.005, 0.0, (D_BASE + D_TEETH - D_FLANGE) / 2])  # the arch's front, mid-height (denture)
AIM = np.array([-0.016, 0.0, 0.0])  # once it is in the hand: it sits this much further out of the fingers
PICK_AIM = np.array([-0.012, 0.0, 0.0])  # where the fingers close on it, on the table
HANG = 4.5  # degrees: held by its front, the denture hangs back-down this much
HAND_TIP = 14.0  # degrees: the hand pitches down this much as it lowers it, so its back reaches the ridge first
BACK_DZ = -0.006  # m: aim the back flanges this far below their seat (the arm stops short; the ridge stops them)
PRE_OPEN = -0.0175  # 35 mm between the pads round the 21 mm front
REL_OPEN = -0.013  # let go: 26 mm, just clear of the front


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - np.dot(close, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


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

    fit = _frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0))

    def held(origin, tilt_front_up=0.0):
        """The hand's pose for the denture's origin at ``origin``, its front tipped up by the angle given."""
        rd = _ry(tilt_front_up)
        return np.asarray(origin, float) + rd @ GRIP + AIM, rd @ fit

    a = "right"
    p, r = held(D_START)
    p = p - AIM + PICK_AIM
    lead(a, p + np.array([-0.05, 0.0, 0.06]), r, PRE_OPEN, "above and in front of the denture, open")
    go(a, 1.0, p + np.array([-0.05, 0.0, 0.0]), PRE_OPEN, "down in front of its front teeth", n=2)
    go(a, 1.2, p, PRE_OPEN, "fingers above its front teeth and under its front flange", n=2)
    go(a, 0.6, p, FINGER_CLOSED, "take the denture by its front")
    go(a, 1.0, p + np.array([0.0, 0.0, 0.03]), FINGER_CLOSED, "lift it off the lid")
    # level, centred, at the mouth's mid-height, just in front of the lips
    z_mid = ZM - GRIP[2]
    o_pre = np.array([XL - (ARCH_BACK + 0.012), 0.0, z_mid])
    p, r = held(o_pre)
    lead(a, p, r, FINGER_CLOSED, "to the mouth, square and centred")
    o_in = np.array([XC - 0.002, 0.0, z_mid])
    p, r = held(o_in)
    go(a, 4.0, p, FINGER_CLOSED, "slide it in through the lips, above the ridge", n=10)
    # lower it onto the ridge back first: held by its front, it hangs back-down by about HANG degrees, so the hand
    # stays level (tilted only HAND_TIP) and comes straight down until the back flanges sit on the crest
    back = np.array([ARCH_BACK, 0.0, -D_FLANGE])
    for k in range(1, 6):
        ht = HAND_TIP * k / 5
        dt = HANG + ht
        b_world = SEAT + back + np.array([0.0, 0.0, (z_mid - SEAT[2]) * (1 - k / 5) + BACK_DZ * k / 5])
        o = b_world - _ry(dt) @ back
        p = o + _ry(dt) @ GRIP + AIM
        go(a, 0.6, p, FINGER_CLOSED, "lower its back onto the ridge", rot=_ry(ht) @ fit)
    wait(a, 0.4, "steady")
    go(a, 0.6, cur[a][0], REL_OPEN, "let go: the front drops onto the ridge")
    p_rel = cur[a][0] + np.array([-0.014, 0.0, 0.0])
    go(a, 0.8, p_rel, REL_OPEN, "fingers straight back off the front")
    go(a, 0.6, p_rel + np.array([0.0, 0.0, 0.004]), REL_OPEN, "level the hand", rot=fit)
    # press the front teeth down with the closed fingers' underside
    teeth_top = SEAT + np.array([-R_ARCH, 0.0, D_BASE + D_TEETH])
    press = teeth_top + np.array([0.004, 0.0, 0.010]) + AIM
    go(a, 0.8, press + np.array([0.0, 0.0, 0.010]), FINGER_CLOSED, "closed fingers over the front teeth", rot=fit)
    go(a, 0.8, press + np.array([0.0, 0.0, -0.003]), FINGER_CLOSED, "press the front teeth down", rot=fit)
    wait(a, 0.4, "hold")
    go(a, 1.2, press + np.array([-0.07, 0.0, 0.006]), FINGER_CLOSED, "draw the hand out of the mouth", rot=fit, n=2)
    home(a)
    end = max(dur("left"), dur("right")) + 2.0
    for arm in ("left", "right"):
        wait(arm, end - dur(arm), "hold still")
    return plan
