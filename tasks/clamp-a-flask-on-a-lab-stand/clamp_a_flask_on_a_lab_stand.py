"""A lab stand (a cast-iron base plate and a steel rod) carries a spring flask clamp on a boss head. Its arm reaches
toward the middle of the table and ends in two cork-lined jaws, which a spring keeps shut. A red lever on top of the
arm opens them while it is pressed down. A 100 mL round-bottom flask of borosilicate glass stands in a cork ring to
the right.

Task: clamp the flask on the stand. One hand presses the lever down and keeps it down, so the jaws stay open. The other
lifts the flask out of its cork ring by its neck and sets the neck between the open jaws, against the clamp's back. The
lever is then let go, so the jaws close on the neck; the flask is let go, hanging in the clamp above the stand's base,
and both arms go back to where they started. The flask must not be dropped or tipped over.

Run it:  inspect-robots run --embodiment yam_arena -E task=clamp_a_flask_on_a_lab_stand.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stand and its clamp ------------------------------------------------------------------------------------------
CX = 0.28  # the clamp's middle line (x) ...
JAW_Y = 0.0  # ... the jaws' middle (y) ...
JAW_Z = TOP + 0.095  # ... and height
PLATE = ((CX - 0.065, CX + 0.065), (-0.05, 0.28), 0.012)  # the base plate: x span, y span, thickness
ROD = (CX, 0.250, 0.006, 0.16)  # x, y, radius, height above the plate
ARM_W, ARM_H = 0.012, 0.010
YOKE_Y = 0.020  # the clamp's back, which the neck goes against
PAD = (0.006, 0.040, 0.016)  # a jaw's cork pad: thick, long (y), tall
PAD_Y = (-0.022, 0.018)
JAW_OPEN = 0.025  # each jaw's travel, fully open (m)
JAW_K, JAW_F = 2000.0, 15.0  # the spring closing each jaw (N/m) and its most force (N)
PIVOT = np.array([CX, 0.120, TOP + 0.120])  # the lever's pivot (an x axis)
LEVER_LEN = 0.058
PAD_LEVER = (0.025, 0.025, 0.006)  # the lever's rubber pad, at its end
LEVER_MIN_DEG, LEVER_FULL_DEG = -15.0, -12.0  # its stop, and where the jaws are fully open
LEVER_SPRING = 0.008  # N m per degree, back to level

# -- the flask (100 mL round-bottom, borosilicate) and its cork ring ----------------------------------------------------
BULB_R, NECK_R, NECK_TOP, LIP = 0.030, 0.0125, 0.0875, (0.015, 0.005)
JAW_ON_NECK = 0.042  # the jaws close on the neck this far above the bulb's middle
RING = (np.array([CX, -0.170]), 0.0225, 0.040, 0.015)  # the cork ring: middle, hole radius, outer radius, height
SIT_Z = TOP + RING[3] + math.sqrt(BULB_R ** 2 - RING[1] ** 2)  # the bulb's middle, sitting in the ring
FLASK_MASS = 0.09

STEEL, CHROME, BLACK, IRON = (0.45, 0.46, 0.48), (0.80, 0.81, 0.83), (0.08, 0.08, 0.09), (0.18, 0.18, 0.20)
CORK, RED, GLASS = (0.72, 0.55, 0.36), (0.78, 0.10, 0.08), (0.80, 0.90, 0.95)

# -- scoring ----------------------------------------------------------------------------------------------------------
IN_X, IN_Y = 0.007, 0.016  # the neck this close to the jaws' middle line, and within their length
GAP = (0.020, 0.030)  # the jaws' gap when closed on the neck (m)
UPRIGHT_DEG = 10.0
CLEAR = 0.005  # the hanging flask's bottom at least this far above the base plate
DROP_SPEED, TIP_DEG = 1.0, 45.0
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _stand() -> Body:
    """Base plate, rod, boss head, arm, the clamp's back (yoke) and the lever's bracket; origin on the plate's top under
    the jaws."""
    o = np.array([CX, JAW_Y, TOP + PLATE[2]])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    (x0, x1), (y0, y1), th = PLATE
    zt = TOP + th
    arm_y0, arm_y1 = YOKE_Y + 0.010, ROD[1]
    shapes = [BoxShape(at((x0 + x1) / 2, (y0 + y1) / 2, TOP + th / 2), (x1 - x0, y1 - y0, th), IRON),
              CylShape(at(ROD[0], ROD[1], zt + ROD[3] / 2), ROD[2], ROD[3], CHROME),
              BoxShape(at(ROD[0], ROD[1], JAW_Z), (0.030, 0.030, 0.030), BLACK),  # the boss head
              CylShape(at(ROD[0] + 0.022, ROD[1], JAW_Z), 0.008, 0.014, BLACK, axis="X"),  # its thumbscrew
              BoxShape(at(CX, (arm_y0 + arm_y1) / 2, JAW_Z), (ARM_W, arm_y1 - arm_y0, ARM_H), CHROME),  # the arm
              BoxShape(at(CX, YOKE_Y + 0.005, JAW_Z), (0.080, 0.010, 0.012), CHROME),  # the yoke the jaws run in
              BoxShape(at(CX, PIVOT[1], (JAW_Z + PIVOT[2]) / 2), (0.016, 0.012, PIVOT[2] - JAW_Z), CHROME)]  # bracket
    return Body.build("stand", pos=tuple(o.tolist()), mass=3.0, kinematic=True, shapes=shapes, color=IRON)


def _jaw(name, side) -> Body:
    """A jaw: a cork pad on a steel back; origin on the pad's face. Starts shut (2 mm gap)."""
    face_x = CX + side * 0.001
    yc = (PAD_Y[0] + PAD_Y[1]) / 2
    shapes = [BoxShape((side * PAD[0] / 2, 0.0, 0.0), (PAD[0], PAD[1], PAD[2]), CORK),
              BoxShape((side * (PAD[0] + 0.004), 0.002, 0.0), (0.008, PAD[1] + 0.004, PAD[2]), STEEL)]
    return Body.build(name, pos=(face_x, JAW_Y + yc, JAW_Z), mass=0.03, shapes=shapes, color=CORK,
                      friction=(0.9, 0.8))


def _lever() -> Body:
    """The red lever: a bar from its pivot toward the rod, with a black rubber pad on its end; origin at the pivot."""
    shapes = [BoxShape((0.0, LEVER_LEN / 2, 0.0), (0.010, LEVER_LEN, 0.006), RED),
              BoxShape((0.0, LEVER_LEN - PAD_LEVER[1] / 2, 0.003 + PAD_LEVER[2] / 2), PAD_LEVER, BLACK),
              CylShape((0.0, 0.0, 0.0), 0.005, 0.020, CHROME, axis="X")]
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=0.02, shapes=shapes, color=RED,
                      com=(0.0, 0.02, 0.0), friction=(1.0, 0.9))


def _ring() -> Body:
    c, r_in, r_out, h = RING
    shapes = []
    for k in range(12):
        a = k * math.pi / 6
        m = (r_in + r_out) / 2
        shapes.append(BoxShape((m * math.cos(a), m * math.sin(a), h / 2), (r_out - r_in, 2 * m * math.tan(math.pi / 12)
                                                                          + 0.002, h), CORK, yaw=a))
    return Body.build("cork_ring", pos=(c[0], c[1], TOP), mass=0.05, kinematic=True, shapes=shapes, color=CORK)


def _flask() -> Body:
    """The flask; origin at the bulb's middle, the neck up its z axis."""
    neck0 = math.sqrt(BULB_R ** 2 - NECK_R ** 2) - 0.003
    shapes = [SphereShape((0.0, 0.0, 0.0), BULB_R, GLASS),
              CylShape((0.0, 0.0, (neck0 + NECK_TOP) / 2), NECK_R, NECK_TOP - neck0, GLASS),
              CylShape((0.0, 0.0, NECK_TOP - LIP[1] / 2), LIP[0], LIP[1], GLASS)]
    return Body.build("flask", pos=(RING[0][0], RING[0][1], SIT_Z), mass=FLASK_MASS, shapes=shapes, color=GLASS,
                      friction=(0.8, 0.7), com=(0.0, 0.0, 0.01))


OBJECTS = [_stand(), _jaw("jaw_far", 1.0), _jaw("jaw_near", -1.0), _lever(), _ring(), _flask()]
for _n, _s in (("jaw_far", 1.0), ("jaw_near", -1.0)):
    slider(f"{_n}_slide", "stand", _n, anchor=(CX + _s * 0.001, JAW_Y, JAW_Z), axis="X",
           limits=(0.0, JAW_OPEN + 0.001) if _s > 0 else (-JAW_OPEN - 0.001, 0.0), stiffness=JAW_K, damping=20.0,
           target=0.0, max_force=JAW_F)
hinge("lever_pivot", "stand", "lever", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(LEVER_MIN_DEG, 0.0),
      spring=LEVER_SPRING, damping=0.0002, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"dropped": False, "in_fork": False, "tcps": None, "poses": {}, "open": 0.0, "set": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FLASK_DEBUG")), "steps": 0}


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


def _lever_deg(rot) -> float:
    """The lever's turn about x (negative: pressed down)."""
    return math.degrees(math.atan2(float(rot[2, 1]), float(rot[1, 1])))


def _neck_point(pos, rot) -> np.ndarray:
    return np.asarray(pos, float) + rot @ np.array([0.0, 0.0, JAW_ON_NECK])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    poses = {}
    for name in ("flask", "lever", "jaw_far", "jaw_near"):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        poses[name] = (p[:3] - origin, rotation(p[3:7]))
    _EP["poses"] = poses
    deg = _lever_deg(poses["lever"][1])
    opening = min(1.0, max(0.0, deg / LEVER_FULL_DEG))
    _EP["open"] = opening
    target = JAW_OPEN * opening
    if _EP["set"] is None or abs(target - _EP["set"]) > 0.0005:  # the lever drives both jaws apart
        set_slider_drive("jaw_far_slide", target=target)
        set_slider_drive("jaw_near_slide", target=-target)
        _EP["set"] = target
    fp, fr = poses["flask"]
    vel = as_numpy(base.scene["flask"].data.root_lin_vel_w)[0]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(fr[2, 2])))))
    if float(vel[2]) < -DROP_SPEED or tilt > TIP_DEG:
        _EP["dropped"] = True
    n = _neck_point(fp, fr)
    gap = float(poses["jaw_far"][0][0] - poses["jaw_near"][0][0])
    if abs(n[0] - CX) < IN_X + 0.01 and PAD_Y[0] < n[1] < PAD_Y[1] and abs(n[2] - JAW_Z) < 0.02 and gap > 0.030:
        _EP["in_fork"] = True  # the neck between the open jaws
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            print(f"[flask] t={_DEBUG['steps'] / 20:5.1f}s lever={deg:6.1f}deg open={opening:4.2f} gap={gap * 1000:5.1f}mm "
                  f"neck={np.round((n - [CX, JAW_Y, JAW_Z]) * 1000).astype(int).tolist()} tilt={tilt:4.1f} "
                  f"bottom={(fp[2] - BULB_R - TOP - PLATE[2]) * 1000:5.1f}mm in_fork={_EP['in_fork']} "
                  f"clamped={st['clamped']} dropped={_EP['dropped']} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"R-lip={np.round((tr - fp - fr @ [0, 0, NECK_TOP]) * 1000).astype(int).tolist()} "
                  f"home={st['home']} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    poses = _EP["poses"]
    clamped = False
    if poses:
        fp, fr = poses["flask"]
        n = _neck_point(fp, fr)
        gap = float(poses["jaw_far"][0][0] - poses["jaw_near"][0][0])
        upright = float(fr[2, 2]) > math.cos(math.radians(UPRIGHT_DEG))
        hanging = float(fp[2]) - BULB_R >= TOP + PLATE[2] + CLEAR
        clamped = (abs(n[0] - CX) < IN_X and PAD_Y[0] + 0.006 < n[1] < PAD_Y[1] and abs(n[2] - JAW_Z) < 0.012
                   and GAP[0] < gap < GAP[1] and upright and hanging)
    return {"clamped": clamped, "in_fork": _EP["in_fork"], "dropped": _EP["dropped"], "home": _arms_home()}


def _points(s) -> float:
    if s["dropped"]:
        return 0.0
    if s["clamped"] and s["home"]:
        return 1.0
    if s["clamped"]:
        return 0.5
    return 0.25 if s["in_fork"] else 0.0


def score(objects) -> float:
    """Judged at the end. 0.0 if the flask was ever dropped (falling faster than 1 m/s) or tipped past 45 degrees.
    Clamped: the flask upright within 10 degrees and hanging (its bottom at least 5 mm above the stand's base plate),
    its neck between the closed jaws (their gap 20 to 30 mm, the neck's axis within 7 mm of their middle line and
    within their length, at their height within 12 mm). Otherwise 1.0: clamped, and both arms back where they started
    (each gripper's fingertip centre (TCP) within 5 cm of its start); 0.5: clamped; 0.25: the neck between the open
    jaws at some point; else 0.0."""
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
    """Full score with the flask still."""
    return score(objects) >= 1.0 and objects["flask"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the lever's pad with its fingers closed and presses it down: the jaws open. The right hand
# comes straight down over the flask, fingers closing across its neck just under the lip, lifts it out of the cork ring,
# carries it to the open jaws and slides the neck in between them, against the clamp's back, then lowers it 1 cm. The
# left lets the lever up: the jaws close on the neck. The right lets go and rises; both arms go back to where they
# started.

LEFT_G = grip_rotation(0.0)  # fingers closing along y (closed, they press the pad)
RIGHT_G = grip_rotation(0.0)  # fingers closing along y, across the neck (the way it is carried and pushed in)
UNDER_LIP = 0.014  # the right's TCP this far below the flask's top
PRE_OPEN = -(LIP[0] + 0.008)
FINGER_REACH = 0.013
PRESS = 0.030
LIFT = 0.010  # carried this much higher than it will hang, lowered once the neck is between the jaws
PUSH = 0.0045  # aim the neck this far past the clamp's back
HANG_Z = JAW_Z - JAW_ON_NECK  # the bulb's middle, hanging
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


def _sweep(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _sweep(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _sweep(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _pad_top() -> np.ndarray:
    return PIVOT + np.array([0.0, LEVER_LEN - PAD_LEVER[1] / 2, 0.003 + PAD_LEVER[2]])


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    touch = _pad_top() + [0.0, 0.0, FINGER_REACH]
    out = _lead_in("left", touch + [0.0, 0.0, 0.03], LEFT_G, "over the lever's pad", FINGER_CLOSED)
    out += [_w(1.0, touch + [0.0, 0.0, 0.004], FINGER_CLOSED, "down onto it", LEFT_G),
            _w(1.0, touch - [0.0, 0.0, PRESS], FINGER_CLOSED, "press the lever down: the jaws open", LEFT_G)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), touch - [0.0, 0.0, PRESS], FINGER_CLOSED,
               "keep it down while the neck goes in", LEFT_G),
            _w(1.0, touch + [0.0, 0.0, 0.03], FINGER_CLOSED, "let the lever up: the jaws close on the neck", LEFT_G)]
    out += _go_home("left", touch + [0.0, 0.0, 0.03], LEFT_G, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, held_at


def _right(start: float, let_go_at: float) -> tuple[list[Waypoint], float, float]:
    g = RIGHT_G
    lip_top = SIT_Z + NECK_TOP
    take = np.array([RING[0][0], RING[0][1], lip_top - UNDER_LIP])
    hang_tcp = HANG_Z + NECK_TOP - UNDER_LIP
    neck_in_y = YOKE_Y - NECK_R + PUSH
    front = np.array([CX, PAD_Y[0] - 0.045, hang_tcp + LIFT])
    inside = np.array([CX, neck_in_y, hang_tcp + LIFT])
    down = np.array([CX, neck_in_y, hang_tcp])
    out = _lead_in("right", take + [0.0, 0.0, 0.06], g, "over the flask", PRE_OPEN)
    out += [_w(1.0, take, PRE_OPEN, "down round its neck", g),
            _w(0.6, take, FINGER_CLOSED, "take the neck under the lip", g),
            _w(1.4, take + [0.0, 0.0, front[2] - take[2] + 0.01], FINGER_CLOSED, "lift it out of the cork ring", g),
            _w(2.4, front + [0.0, 0.0, 0.01], FINGER_CLOSED, "carry it in front of the clamp", g),
            _w(0.6, front, FINGER_CLOSED, "line the neck up with the jaws", g)]
    t = sum(w.duration for w in out)
    if start > t:
        out.append(_w(start - t, front, FINGER_CLOSED, "wait for the jaws to open", g))
    out += [_w(1.6, inside, FINGER_CLOSED, "slide the neck in between the jaws, against the clamp's back", g),
            _w(0.8, down, FINGER_CLOSED, "lower it to the jaws' height", g)]
    placed_at = sum(w.duration for w in out)
    if let_go_at > placed_at:
        out.append(_w(let_go_at - placed_at, down, FINGER_CLOSED, "hold it while the jaws close", g))
    back = down + [-0.07, 0.0, 0.015]  # toward the arms: the open fingers pass either side of the neck
    out += [_w(0.5, down, PRE_OPEN, "let go: the flask hangs in the clamp", g),
            _w(0.5, down + [0.0, 0.0, 0.015], PRE_OPEN, "rise off the neck", g),
            _w(1.0, back, PRE_OPEN, "draw back toward the arms, the fingers either side of the neck", g)]
    out += _go_home("right", back, g, FINGER_OPEN)
    return out, placed_at, 0.0


def demo():
    _, held_at = _left(0.0, 0.0)
    _, placed_at, _ = _right(held_at, 0.0)
    release_at = placed_at + 0.3
    let_go_at = release_at + 1.0 + 0.6  # the lever up, then the jaws shut
    right, _, _ = _right(held_at, let_go_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(release_at, total)
    return {"left": left, "right": right}
