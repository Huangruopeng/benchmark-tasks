"""A lab stand (a 6 x 9 in cast-iron base with a 24 in x 1/2 in rod) stands at the back left. Through two boss heads
and a 1/2 in crossbar it holds a remote-release spring flask clamp, built from clamp_drawing.py: an aluminium arm tube
reaching toward the middle of the table and ending in two cork-lined jaws, which springs keep shut. A red lever on the
arm opens them while it is pressed down: its stub pushes a push rod inside the tube, whose wedge spreads the jaws. A
100 mL round-bottom flask of borosilicate glass stands in a cork ring to the right.

Task: clamp the flask on the stand. One hand presses the lever down and keeps it down, so the jaws stay open. The other
lifts the flask out of its cork ring by its neck and sets the neck between the open jaws, against the clamp's head. The
lever is then let go, so the jaws close on the neck; the flask is let go, hanging in the clamp clear of the table,
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

# -- the stand (a 6 x 9 in cast-iron base with a 24 in x 1/2 in rod), its crossbar and the clamp ---------------------------
# The stand stands at the back left, out of the arms' way; a 1/2 in crossbar runs forward from it to the clamp's arm.
CX = 0.28  # the clamp's middle line (x) ...
JAW_Y = 0.0  # ... the jaws' middle (y) ...
JAW_Z = TOP + 0.095  # ... and height
INCH = 0.0254
BASE = (9 * INCH, 6 * INCH, 0.013)  # the cast-iron base: long (x), wide (y), thick
ROD_XY, ROD_R, ROD_H = (0.48, 0.23), INCH / 4, 24 * INCH  # the stand's rod: where, radius (1/2 in), height
BASE_C = (ROD_XY[0] + 0.025 - BASE[0] / 2, ROD_XY[1])  # the rod is screwed in 2.5 cm from the base's back end
CROSS_Z = JAW_Z - 0.025  # the crossbar (a 1/2 in x 12 in support rod along x), held under the clamp's arm ...
CROSS_X0, CROSS_LEN = CX - 0.030, 12 * INCH  # ... from here back
TUBE_R = INCH / 4  # the clamp's arm: a 1/2 in aluminium tube along y, at the jaws' height; the push rod runs inside
ARM_END_Y = ROD_XY[1] + 0.02
BOSS = 0.030  # the two boss heads (stainless, about 3 cm)
HEAD = (0.150, 0.030, 0.016)  # the clamp head: long (x), deep (y), tall; its front face is the neck's stop
YOKE_Y = 0.020  # the clamp head's front face, which the neck goes against
PAD = (0.006, 0.040, 0.016)  # a jaw's cork pad: thick, long (y), tall
PAD_Y = (-0.022, 0.018)
GAP0 = 0.002  # the jaws' gap when shut on their stops
# The linkage: pressing the lever turns its 25 mm stub, which pushes the push rod toward the jaws; the rod's wedge
# (tan 1.5 per side) spreads the two jaws' rollers. Each jaw is pushed shut by its own compression spring.
STUB = 0.025  # the lever's stub, pivot to push rod (m)
WEDGE = 1.5  # jaw travel per mm of push rod travel (wedge flank 56.3 degrees from the rod's axis)
LINK = STUB * WEDGE  # each jaw opens LINK * sin(lever angle)
LEVER_STOP_DEG = 30.0  # the lever's stop, pressed: the jaws are fully open there
JAW_OPEN = LINK * math.sin(math.radians(LEVER_STOP_DEG))  # each jaw's travel, fully open (m)
JAW_F0, JAW_K = 3.0, 250.0  # each jaw's spring: preload on its stop (N) and rate (N/m)
ETA = 0.9  # the linkage's efficiency (roller followers on the wedge)
PIVOT = np.array([CX, 0.120, TOP + 0.120])  # the lever's pivot (an x axis), STUB above the push rod
PAD_ARM = 0.050  # pivot to the middle of the lever's pad
PAD_LEVER = (0.025, 0.025, 0.006)  # the lever's rubber pad, at its end
LEVER_LEN = PAD_ARM + PAD_LEVER[1] / 2
LEVER_DX = 0.0225  # the lever's arm is cranked this far to the side (+x) of the tube, so it passes beside it pressed


def lever_torque(deg: float) -> float:
    """The torque (N m) the two jaw springs put on the lever through the linkage when it is pressed ``deg``."""
    a = math.radians(deg)
    return 2 * (JAW_F0 + JAW_K * LINK * math.sin(a)) * LINK * math.cos(a) / ETA


def pad_force(deg: float) -> float:
    """The downward force (N) on the lever's pad that holds it pressed ``deg``."""
    return lever_torque(deg) / (PAD_ARM * math.cos(math.radians(deg)))


# The simulated lever's spring matches lever_torque at rest and at the stop (linear in between).
LEVER_SPRING = (lever_torque(LEVER_STOP_DEG) - lever_torque(0.0)) / LEVER_STOP_DEG  # N m per degree
LEVER_PRELOAD_DEG = lever_torque(0.0) / LEVER_SPRING  # its target, above the rest stop

# -- the flask (100 mL round-bottom, borosilicate) and its cork ring ----------------------------------------------------
BULB_R, NECK_R, NECK_TOP, LIP = 0.032, 0.0125, 0.0875, (0.015, 0.005)
JAW_ON_NECK = 0.042  # the jaws close on the neck this far above the bulb's middle
RING = (np.array([CX, -0.170]), 0.0225, 0.040, 0.015)  # the cork ring: middle, hole radius, outer radius, height
SIT_Z = TOP + RING[3] + math.sqrt(BULB_R ** 2 - RING[1] ** 2)  # the bulb's middle, sitting in the ring
FLASK_MASS = 0.09

STEEL, CHROME, BLACK, IRON = (0.45, 0.46, 0.48), (0.80, 0.81, 0.83), (0.08, 0.08, 0.09), (0.18, 0.18, 0.20)
ALU = (0.70, 0.72, 0.75)
CORK, RED, GLASS = (0.72, 0.55, 0.36), (0.78, 0.10, 0.08), (0.80, 0.90, 0.95)

# -- scoring ----------------------------------------------------------------------------------------------------------
IN_X, IN_Y = 0.007, 0.016  # the neck this close to the jaws' middle line, and within their length
GAP = (0.020, 0.030)  # the jaws' gap when closed on the neck (m)
OPEN_GAP = 0.030  # the jaws count as open wider than this
UPRIGHT_DEG = 10.0
CLEAR = 0.005  # the hanging flask's bottom at least this far above the table
DROP_SPEED, TIP_DEG = 1.0, 45.0
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _stand() -> Body:
    """Base, rod, the two boss heads, the crossbar, the clamp's arm, its head and the lever's bracket; one fixed body,
    origin on the table under the jaws."""
    o = np.array([CX, JAW_Y, TOP])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    bl, bw, bt = BASE
    rx, ry = ROD_XY
    zt = TOP + bt
    head_y = YOKE_Y + HEAD[1] / 2
    shapes = [BoxShape(at(BASE_C[0], BASE_C[1], TOP + bt / 2), (bl, bw, bt), IRON),  # the base
              CylShape(at(rx, ry, zt + ROD_H / 2), ROD_R, ROD_H, CHROME),  # the rod
              BoxShape(at(rx, ry, CROSS_Z), (BOSS, BOSS, BOSS), STEEL),  # boss head 1, on the rod
              CylShape(at(rx, ry - 0.022, CROSS_Z), 0.007, 0.014, BLACK, axis="Y"),  # its wing screw
              CylShape(at(CROSS_X0 + CROSS_LEN / 2, ry, CROSS_Z), INCH / 4, CROSS_LEN, CHROME, axis="X"),  # crossbar
              BoxShape(at(CX, ry, (CROSS_Z + JAW_Z) / 2), (BOSS, BOSS, BOSS + 0.015), STEEL),  # boss head 2
              CylShape(at(CX + 0.022, ry, (CROSS_Z + JAW_Z) / 2), 0.007, 0.014, BLACK, axis="X"),  # its wing screw
              CylShape(at(CX, (YOKE_Y + HEAD[1] + ARM_END_Y) / 2, JAW_Z), TUBE_R, ARM_END_Y - YOKE_Y - HEAD[1], ALU,
                       axis="Y"),  # the clamp's arm (tube)
              BoxShape(at(CX, head_y, JAW_Z), HEAD, ALU),  # the clamp head
              BoxShape(at(CX, PIVOT[1], (JAW_Z + PIVOT[2]) / 2 + 0.002), (0.016, 0.014, PIVOT[2] - JAW_Z + 0.006),
                       ALU)]  # the lever's bracket (a split collar on the tube, carrying the pivot pin)
    return Body.build("stand", pos=tuple(o.tolist()), mass=3.0, kinematic=True, shapes=shapes, color=IRON)


def _jaw(name, side) -> Body:
    """A jaw: a cork pad on a steel back; origin on the pad's face. Starts shut on its stop."""
    face_x = CX + side * GAP0 / 2
    yc = (PAD_Y[0] + PAD_Y[1]) / 2
    shapes = [BoxShape((side * PAD[0] / 2, 0.0, 0.0), (PAD[0], PAD[1], PAD[2]), CORK),
              BoxShape((side * (PAD[0] + 0.004), 0.002, 0.0), (0.008, PAD[1] + 0.004, PAD[2]), STEEL)]
    return Body.build(name, pos=(face_x, JAW_Y + yc, JAW_Z), mass=0.03, shapes=shapes, color=CORK,
                      friction=(0.9, 0.8))


def _lever() -> Body:
    """The red lever, a cranked bell crank: a hub on the pivot pin above the tube, a stub down to the push rod, and an
    arm toward the stand, LEVER_DX to the side of the tube, with a black rubber pad on its end; origin at the pivot."""
    d = LEVER_DX
    shapes = [BoxShape((d, LEVER_LEN / 2, 0.0), (0.010, LEVER_LEN, 0.006), RED),
              BoxShape((d, LEVER_LEN - PAD_LEVER[1] / 2, 0.003 + PAD_LEVER[2] / 2), PAD_LEVER, BLACK),
              BoxShape((d / 2, 0.0, 0.0), (d + 0.010, 0.010, 0.006), RED),  # the crank, hub to arm
              BoxShape((0.0, 0.0, -STUB / 2), (0.006, 0.008, STUB), RED),  # the stub, into the tube's slot
              CylShape((d / 2, 0.0, 0.0), 0.005, d + 0.020, CHROME, axis="X")]  # the hub on its pin
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=0.025, shapes=shapes, color=RED,
                      com=(d, 0.025, 0.0), friction=(1.0, 0.9))


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
    slider(f"{_n}_slide", "stand", _n, anchor=(CX + _s * GAP0 / 2, JAW_Y, JAW_Z), axis="X",
           limits=(0.0, JAW_OPEN + 0.0005) if _s > 0 else (-JAW_OPEN - 0.0005, 0.0), stiffness=JAW_K, damping=20.0,
           target=-_s * JAW_F0 / JAW_K, max_force=15.0)
hinge("lever_pivot", "stand", "lever", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(-LEVER_STOP_DEG, 0.0),
      spring=LEVER_SPRING, damping=0.0003, target_deg=LEVER_PRELOAD_DEG)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"dropped": False, "in_fork": False, "tcps": None, "poses": {}, "open": 0.0, "set": {}}


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
    wedge = LINK * math.sin(math.radians(min(max(-deg, 0.0), LEVER_STOP_DEG)))  # where the wedge holds each jaw
    _EP["open"] = wedge / JAW_OPEN
    for name, side in (("jaw_far", 1.0), ("jaw_near", -1.0)):
        x = side * (float(poses[name][0][0]) - (CX + side * GAP0 / 2))  # this jaw's opening (m)
        # The wedge only pushes the jaws apart. A jaw it holds follows it; a jaw clear of it (held wider by the neck,
        # or with the lever up) is pushed shut by its preloaded spring alone.
        target = wedge if wedge > 0.0005 and x <= wedge + 0.0015 else -JAW_F0 / JAW_K
        if _EP["set"].get(name) is None or abs(target - _EP["set"][name]) > 0.0002:
            set_slider_drive(f"{name}_slide", target=side * target)
            _EP["set"][name] = target
    fp, fr = poses["flask"]
    vel = as_numpy(base.scene["flask"].data.root_lin_vel_w)[0]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(fr[2, 2])))))
    if float(vel[2]) < -DROP_SPEED or tilt > TIP_DEG:
        _EP["dropped"] = True
    n = _neck_point(fp, fr)
    gap = float(poses["jaw_far"][0][0] - poses["jaw_near"][0][0])
    if abs(n[0] - CX) < IN_X + 0.01 and PAD_Y[0] < n[1] < PAD_Y[1] and abs(n[2] - JAW_Z) < 0.02 and gap > OPEN_GAP:
        _EP["in_fork"] = True  # the neck between the open jaws
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            print(f"[flask] t={_DEBUG['steps'] / 20:5.1f}s lever={deg:6.1f}deg open={_EP["open"]:4.2f} gap={gap * 1000:5.1f}mm "
                  f"neck={np.round((n - [CX, JAW_Y, JAW_Z]) * 1000).astype(int).tolist()} tilt={tilt:4.1f} "
                  f"bottom={(fp[2] - BULB_R - TOP) * 1000:5.1f}mm in_fork={_EP['in_fork']} "
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
        hanging = float(fp[2]) - BULB_R >= TOP + CLEAR
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
    Clamped: the flask upright within 10 degrees and hanging (its bottom at least 5 mm above the table), its neck
    between the closed jaws (their gap 20 to 30 mm, the neck's axis within 7 mm of their middle line, along their
    length at least 6 mm in from their front ends, and at their height within 12 mm). Otherwise 1.0: clamped, and both
    arms back where they started (each gripper's fingertip centre (TCP) within 5 cm of its start); 0.5: clamped; 0.25:
    the neck between the open jaws at some point; else 0.0."""
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
PRESS = 0.035  # aim this far below the pad's top at the stop
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


def _pad_top(deg: float = 0.0) -> np.ndarray:
    """The middle of the top of the lever's pad, the lever pressed ``deg`` degrees."""
    a = math.radians(-deg)
    rot = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
    return PIVOT + rot @ np.array([LEVER_DX, PAD_ARM, 0.003 + PAD_LEVER[2]])


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    touch = _pad_top() + [0.0, 0.0, FINGER_REACH]
    full = _pad_top(LEVER_STOP_DEG) + [0.0, 0.0, FINGER_REACH - PRESS]  # aimed past the stop: the arm gives
    up = touch + [0.0, 0.0, 0.03]
    out = _lead_in("left", up, LEFT_G, "over the lever's pad", FINGER_CLOSED)
    out += [_w(1.0, touch + [0.0, 0.0, 0.004], FINGER_CLOSED, "down onto it", LEFT_G),
            _w(1.2, full, FINGER_CLOSED, "press the lever down to its stop: the jaws open", LEFT_G)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), full, FINGER_CLOSED, "keep it down while the neck goes in", LEFT_G),
            _w(1.0, up, FINGER_CLOSED, "let the lever up: the jaws close on the neck", LEFT_G)]
    out += _go_home("left", up, LEFT_G, FINGER_CLOSED)
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
