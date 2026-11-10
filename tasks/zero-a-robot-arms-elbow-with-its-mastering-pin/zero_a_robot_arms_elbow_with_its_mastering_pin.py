"""On the station table stands a small six-axis robot arm, orange with grey joints, its base at the left and its arm
reaching across the table toward the right. Its elbow's brake has been released for mastering (zeroing the joint after
its motor was changed), so its forearm hangs down from the elbow on a free joint, resting on its stop. Two yellow zero
marks sit on top of the elbow, one on the fixed housing and one on the forearm's hub; they line up only when the forearm
is level. On the front face of the elbow is the zeroing hole. A steel zeroing pin with a red knurled grip lies in its
holder at the front left of the table.

Task: zero the elbow. Take the zeroing pin from its holder. With one hand lift the forearm until the two zero marks line
up and hold it there, while the other hand pushes the pin into the zeroing hole until it is home and the elbow's green
light comes on. Then let go of the forearm and the pin (the pin now holds the forearm level) and take both arms back to
where they started. The pin goes all the way in only when the marks line up, and the forearm drops back onto its stop
whenever it is let go before the pin is home, so one hand cannot do it alone.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=zero_a_robot_arms_elbow_with_its_mastering_pin.py --instruction "..." --scorer task_score
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, glue, hinge,  # noqa: E402
                     on_reset, on_step, rotation, segment, set_glue_on, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the robot. The elbow's axis runs along x (forward), through (XM, YE, ZE); the forearm reaches toward -y ----------
XM, YE, ZE = 0.320, 0.080, TOP + 0.115
BASE = np.array([0.320, 0.240])  # the robot's base on the table
SHOULDER_Z = TOP + 0.075
HUB_R, HUB_W = 0.030, 0.026  # the forearm's hub, between the two cheeks of the elbow housing
CHEEK_R, CHEEK_T = 0.037, 0.010
CHEEK_X = (XM - HUB_W / 2 - 0.001 - CHEEK_T / 2, XM + HUB_W / 2 + 0.001 + CHEEK_T / 2)  # front and rear cheek
FRONT = CHEEK_X[0] - CHEEK_T / 2  # the front cheek's face (the zeroing hole's mouth)
HOLE = np.array([FRONT, YE + 0.022, ZE])  # the zeroing hole's mouth; the pin goes in along +x
CHEEK_DEPTH = CHEEK_T + 0.001  # the pin's tip this deep: through the front cheek, at the hub's face
PIN_HOME = 0.024  # the pin's tip this deep: home, 1.3 cm into the hub
PIN_AT_HUB = 0.016  # past this, the pin is in the hub (it gets there only with the marks lined up)
ZERO_TOL_DEG = 4.0  # the marks line up within this
HANG_DEG = 30.0  # the forearm hangs this far below level, on its stop
FOREARM_MASS = 0.30
FOREARM_COM_Y = -0.075  # from the axis, along the forearm
ELBOW_DAMP = 0.0004  # N m s/deg: the released joint turns freely (light gearbox drag)
# -- the zeroing pin: its frame's origin at its tip, +x along it toward the tip ---------------------------------------
PIN_R, SHAFT_L = 0.004, 0.040
GRIP_W, GRIP_L, GRIP_LO, GRIP_HI = 0.014, 0.030, 0.008, 0.026  # its red handle: width, length, below/above the axis
GRIP_C = -(SHAFT_L + GRIP_L / 2)  # the handle's middle, along the pin
PIN_MASS = 0.08
PIN_START = np.array([0.205 - GRIP_C, 0.165, TOP + 0.034])  # its tip, lying in its holder, pointing +x
CATCH_R, CATCH_TILT = 0.006, 12.0  # caught by the hole within this of its axis, and this tilted
# -- colours ----------------------------------------------------------------------------------------------------------
ORANGE, GREY, DARK, STEEL = (0.93, 0.42, 0.08), (0.60, 0.62, 0.64), (0.16, 0.16, 0.18), (0.74, 0.75, 0.77)
RED, YELLOW, BLACK, ALU = (0.78, 0.10, 0.08), (0.98, 0.84, 0.08), (0.04, 0.04, 0.04), (0.70, 0.71, 0.73)
LED_OFF, LED_ON = (0.10, 0.22, 0.12), (0.25, 1.00, 0.30)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _robot() -> Body:
    """Base, turret, shoulder, upper arm and the elbow housing's two cheeks (fixed); origin on the elbow's axis."""
    def at(x, y, z):
        return (x - XM, y - YE, z - ZE)

    bx, by = float(BASE[0]), float(BASE[1])
    s = [CylShape(at(bx, by, TOP + 0.010), 0.062, 0.020, DARK),  # the base plate
         CylShape(at(bx, by, TOP + 0.036), 0.048, 0.032, ORANGE),  # the turret
         CylShape(at(bx, by, SHOULDER_Z), 0.036, 0.062, GREY, axis="X"),  # the shoulder
         segment(at(bx, by, SHOULDER_Z), at(XM, YE + 0.040, ZE), 0.044, ORANGE),  # the upper arm
         CylShape(at(XM, YE + 0.056, ZE), 0.024, 0.050, GREY, axis="X")]  # the elbow's motor housing
    for i, cx in enumerate(CHEEK_X):  # the elbow housing's cheeks either side of the forearm's hub
        s.append(CylShape(at(cx, YE, ZE), CHEEK_R, CHEEK_T, ORANGE, axis="X"))
        s.append(BoxShape(at(cx, YE + 0.022, ZE), (CHEEK_T, 0.044, 2 * CHEEK_R - 0.004), ORANGE))
    s += [CylShape(at(FRONT - 0.0003, HOLE[1], HOLE[2]), 0.0058, 0.0008, BLACK, collide=False, axis="X"),  # the hole
          CylShape(at(FRONT - 0.0002, HOLE[1], HOLE[2]), 0.0085, 0.0005, STEEL, collide=False, axis="X"),  # its bush
          # the housing's zero mark (a yellow arrow on top of the front cheek, pointing back at the hub's mark)
          BoxShape(at(CHEEK_X[0], YE, ZE + CHEEK_R + 0.0004), (CHEEK_T - 0.001, 0.005, 0.0008), YELLOW, collide=False),
          BoxShape(at(CHEEK_X[1], YE, ZE + CHEEK_R + 0.0004), (CHEEK_T - 0.001, 0.005, 0.0008), YELLOW, collide=False),
          # the joint's status light on top of the elbow housing: dim until zeroed
          CylShape(at(XM, YE + 0.056, ZE + 0.0245), 0.0065, 0.004, LED_OFF, collide=False),
          CylShape(at(XM, YE + 0.056, ZE + 0.0255), 0.0068, 0.004, LED_ON, collide=False),  # lit (shown when zeroed)
          # the cable dress along the upper arm
          segment(at(bx - 0.026, by - 0.02, SHOULDER_Z + 0.02), at(XM - 0.026, YE + 0.06, ZE + 0.012), 0.008, DARK,
                  collide=False)]
    return Body.build("robot", pos=(XM, YE, ZE), mass=10.0, kinematic=True, shapes=s, color=ORANGE,
                      friction=(0.8, 0.7))


LED_SHAPE = 14  # index of the lit status light in _robot()'s shapes


def _forearm() -> Body:
    """The forearm: hub, beam, wrist and tool flange; origin on the elbow's axis, level (zeroed) when unrotated."""
    s = [CylShape((0.0, 0.0, 0.0), HUB_R, HUB_W, GREY, axis="X"),
         BoxShape((0.0, -0.088, 0.0), (0.034, 0.126, 0.036), ORANGE),  # the forearm's beam
         CylShape((0.0, -0.163, 0.0), 0.016, 0.026, GREY, axis="Y"),  # the wrist
         SphereShape((0.0, -0.163, 0.0), 0.017, GREY),
         CylShape((0.0, -0.180, 0.0), 0.021, 0.008, DARK, axis="Y"),  # the tool flange
         # its zero mark, on top of the hub (lined up with the housing's marks only when level)
         BoxShape((0.0, 0.0, HUB_R + 0.0004), (HUB_W - 0.002, 0.005, 0.0008), YELLOW, collide=False),
         BoxShape((0.0, -0.088, 0.0184), (0.020, 0.040, 0.0006), DARK, collide=False)]  # a label plate
    for k in range(4):  # the flange's bolt holes
        a = math.pi / 4 + k * math.pi / 2
        s.append(CylShape((0.012 * math.cos(a), -0.1845, 0.012 * math.sin(a)), 0.002, 0.0008, STEEL, collide=False,
                          axis="Y"))
    h = math.radians(HANG_DEG) / 2
    return Body.build("forearm", pos=(XM, YE, ZE), mass=FOREARM_MASS, shapes=s, color=ORANGE, friction=(1.0, 0.9),
                      com=(0.0, FOREARM_COM_Y, 0.0), start_xyzw=(math.sin(h), 0.0, 0.0, math.cos(h)),
                      damping=(0.0, 0.05))


def _holder() -> Body:
    """The pin's holder: a grooved aluminium block under the shaft and a pad under the grip."""
    gx = PIN_START[0] + GRIP_C
    bot = PIN_START[2] - PIN_R  # the shaft rests on the groove's floor
    x0, x1 = PIN_START[0] - SHAFT_L + 0.004, PIN_START[0] - 0.004
    cx = (x0 + x1) / 2
    s = [BoxShape((cx - gx, 0.0, (bot - TOP) / 2), (x1 - x0, 0.030, bot - TOP), ALU)]
    for sy in (-1.0, 1.0):  # the groove's walls, with room either side of the shaft
        s.append(BoxShape((cx - gx, sy * 0.0120, bot - TOP + 0.002), (x1 - x0, 0.009, 0.004), ALU))
    pad = PIN_START[2] - GRIP_LO  # the handle rests on a narrow pad
    s.append(BoxShape((0.0, 0.0, (pad - TOP) / 2), (0.020, 0.008, pad - TOP), DARK))
    s.append(BoxShape((0.0, 0.0, 0.002), (0.030, 0.030, 0.004), DARK))
    return Body.build("holder", pos=(gx, PIN_START[1], TOP), mass=1.0, kinematic=True, shapes=s, color=ALU,
                      friction=(0.6, 0.5))


def _pin() -> Body:
    s = [CylShape((-SHAFT_L / 2, 0.0, 0.0), PIN_R, SHAFT_L, STEEL, axis="X"),
         CylShape((-0.0015, 0.0, 0.0), PIN_R - 0.0012, 0.003, STEEL, axis="X"),  # (a chamfered tip)
         BoxShape((GRIP_C, 0.0, (GRIP_HI - GRIP_LO) / 2), (GRIP_L, GRIP_W, GRIP_HI + GRIP_LO), RED),
         CylShape((-SHAFT_L + 0.002, 0.0, 0.0), 0.0065, 0.004, STEEL, axis="X")]  # its collar
    for k in range(4):  # grip ribs
        s.append(BoxShape((GRIP_C, 0.0, -GRIP_LO + 0.006 + 0.0065 * k), (GRIP_L + 0.0006, GRIP_W + 0.0006, 0.0015),
                          DARK, collide=False))
    return Body.build("pin", pos=tuple(PIN_START.tolist()), mass=PIN_MASS, shapes=s, color=STEEL, friction=(1.2, 1.1),
                      com=(GRIP_C + 0.004, 0.0, 0.006), ignore=("robot", "forearm"))


def _pendant() -> Body:
    """The robot's teach pendant lying on the table (scenery)."""
    s = [BoxShape((0.0, 0.0, 0.009), (0.075, 0.120, 0.018), DARK),
         BoxShape((0.0, 0.006, 0.0182), (0.058, 0.080, 0.0006), (0.20, 0.32, 0.45), collide=False),  # its screen
         CylShape((0.0, -0.047, 0.021), 0.009, 0.008, RED, collide=False),  # its emergency stop
         CylShape((0.0, -0.047, 0.0185), 0.012, 0.002, YELLOW, collide=False)]
    return Body.build("pendant", pos=(0.430, -0.170, TOP), mass=0.8, kinematic=True, shapes=s, color=DARK, yaw=0.3)


hinge("elbow", "robot", "forearm", anchor=(XM, YE, ZE), axis="X", limits_deg=(-60.0, HANG_DEG), damping=ELBOW_DAMP)
glue("zero_lock", "robot", "forearm", anchor=(XM, YE, ZE))  # the pin home: holds the forearm level (switched on then)
thread("pin_in_hole", "robot", "pin", anchor0=tuple((HOLE - [XM, YE, ZE]).tolist()), axis0=(1.0, 0.0, 0.0),
       anchor1=(0.0, 0.0, 0.0), axis1=(1.0, 0.0, 0.0), slide_limits=(-0.004, PIN_HOME + 0.002), turn_damping=0.002,
       slide_stiffness=4000.0, slide_damping=8.0, slide_max_force=40.0)

_ROBOT_PATH: list = []
_author_glues_plain = bodykit._author_glues


def _author_glues(stage, prim_path: str, spec) -> None:
    """bodykit's glues, then: the zero lock starts off (or it would pull the forearm level as the sim settles), and
    the lit status light starts hidden."""
    _author_glues_plain(stage, prim_path, spec)
    if spec.name == "forearm":
        from pxr import UsdPhysics

        UsdPhysics.FixedJoint(stage.GetPrimAtPath(f"{prim_path}/zero_lock")).GetJointEnabledAttr().Set(False)
    if spec.name == "robot":
        from pxr import UsdGeom

        _ROBOT_PATH.clear()
        _ROBOT_PATH.append(prim_path)
        UsdGeom.Imageable(stage.GetPrimAtPath(f"{prim_path}/body/s{LED_SHAPE}")).MakeInvisible()


bodykit._author_glues = _author_glues

OBJECTS = [_robot(), _forearm(), _holder(), _pin(), _pendant()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"caught": False, "pinned": False, "lit": False, "deg": HANG_DEG, "s": None, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("ZERO_DEBUG")), "steps": 0}


def _show_led(on: bool) -> None:
    if not _ROBOT_PATH:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(f"{_ROBOT_PATH[0]}/body/s{LED_SHAPE}")
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_glue_on("zero_lock", False)
    _show_led(False)


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _, rf = _pose(base, origin, "forearm")
    pp, rp = _pose(base, origin, "pin")
    # the forearm's turn about the elbow's axis: positive hanging down, 0 level (zeroed)
    d = rf @ np.array([0.0, -1.0, 0.0])
    deg = math.degrees(math.atan2(-float(d[2]), -float(d[1])))
    _EP["deg"] = deg
    rel = pp - HOLE
    s = float(rel[0])  # the pin's tip, this deep in the hole
    radial = float(np.hypot(rel[1], rel[2]))
    axis = rp @ np.array([1.0, 0.0, 0.0])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[0])))))
    zeroed = abs(deg) <= ZERO_TOL_DEG
    if not _EP["caught"]:
        if radial <= CATCH_R and tilt <= CATCH_TILT and -0.008 <= s <= 0.004:
            _EP["caught"] = True
            set_slide_target("pin_in_hole", s)
            set_thread_on("pin_in_hole", True)
    else:
        if s < -0.006:  # drawn back out of the hole: free again
            _EP["caught"] = False
            set_thread_on("pin_in_hole", False)
        elif _EP["pinned"] or zeroed:
            set_slide_target("pin_in_hole", min(s, PIN_HOME))
        else:  # the marks are not lined up: the tip stops against the hub's face
            set_slide_target("pin_in_hole", min(s, CHEEK_DEPTH))
    pinned = _EP["caught"] and s >= PIN_AT_HUB and (_EP["pinned"] or zeroed)
    if pinned != _EP["pinned"]:
        _EP["pinned"] = pinned
        set_glue_on("zero_lock", pinned)  # the pin in the hub holds the forearm level
        _show_led(pinned)
    _EP["s"] = s if _EP["caught"] else None
    _EP["tcps"] = _tcps(base, origin)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, tr = _EP["tcps"]
            grip = pp + rp @ np.array([GRIP_C, 0.0, 0.0])
            fa = np.array([XM, YE, ZE]) + rf @ np.array([0.0, -R_GRIP, 0.0])
            print(f"[zero] t={_DEBUG['steps'] / 20:5.1f}s deg={deg:6.2f} caught={_EP['caught']} s={s * 1000:6.1f}"
                  f" radial={radial * 1000:5.1f} tilt={tilt:5.1f} pinned={_EP['pinned']}"
                  f" L-grip={np.round((tl - grip) * 1000).astype(int).tolist()}"
                  f" R-forearm={np.round((tr - fa) * 1000).astype(int).tolist()}"
                  f" pin={np.round((pp - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """At the end: 1.0 with the elbow pinned at its zero (the pin home, the marks lined up, the light on) and both arms
    back where they started; 0.5 with the elbow pinned at its zero; 0.25 with the pin in the zeroing hole; else 0.0."""
    if _EP["pinned"]:
        return 1.0 if _arms_home() else 0.5
    return 0.25 if _EP["caught"] else 0.0


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
    """Full score with the forearm and the pin still: the trial ends there."""
    return score(objects) >= 1.0 and objects["forearm"].resting() and objects["pin"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the pin out of its holder by its grip, straight down, fingers closing across it, and brings it in
# front of the zeroing hole. Meanwhile the right hand comes straight down on the hanging forearm near its wrist, fingers
# closing across it, and lifts it along its arc until it is level (the fingers let it pivot between them), and holds it
# there. The left pushes the pin into the hole until it is home; the green light comes on and the pin holds the forearm.
# Both hands let go and go back to where they started.

def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


DOWN_L = grip_rotation(0.0)  # straight down, fingers closing along y (across the pin)
DOWN_R = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (across the forearm)
R_GRIP = 0.140  # the right hand holds the forearm this far out from the elbow's axis
R_ABOVE = 0.006  # its TCP this far above the forearm's centre line
R_SAG = 0.012  # (the arm sags this much holding the forearm up: aim this much higher)
PIN_IN_HAND = 0.011  # the left TCP this far above the pin's axis (the middle of its handle)
LEAD_S, LEAD_STEPS = 3.0, 15
LIFT_S, LIFT_STEPS = 3.0, 12
PUSH_S = 2.5
OFF = 0.05


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    key = (arm, tuple(np.round(np.asarray(pos, float), 5)), tuple(np.round(np.asarray(rot, float), 5).ravel()))
    if key in _IK_CACHE:
        return _IK_CACHE[key]
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
    _IK_CACHE[key] = best[0]
    return best[0]


_IK_CACHE: dict = {}


def _sweep(arm: str, q0, q1, label: str, finger, secs=LEAD_S, steps=LEAD_STEPS) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(secs / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _on_arc(deg: float, extra: float = 0.0) -> np.ndarray:
    """The right TCP on the forearm, turned ``deg`` below level."""
    a = math.radians(deg)
    return np.array([XM, YE - R_GRIP * math.cos(a), ZE - R_GRIP * math.sin(a) + R_ABOVE + extra])


def _right(t_release: float) -> list[Waypoint]:
    from yam_arena.constants import READY

    hang = _on_arc(HANG_DEG)
    out = _sweep("right", np.array(READY, float), _good_ik("right", hang + [0.0, 0.0, OFF], DOWN_R),
                 "over the hanging forearm near its wrist", FINGER_OPEN)
    out += [_w(1.0, hang, -0.024, "down round the forearm", DOWN_R),
            _w(0.6, hang, FINGER_CLOSED, "take hold of the forearm", DOWN_R)]
    for k in range(1, LIFT_STEPS + 1):
        deg = HANG_DEG * (1 - k / LIFT_STEPS)
        out.append(_w(LIFT_S / LIFT_STEPS, _on_arc(deg, R_SAG * k / LIFT_STEPS), FINGER_CLOSED,
                      "lift the forearm until the zero marks line up", DOWN_R))
    level = _on_arc(0.0, R_SAG)
    out.append(_w(max(t_release - _dur(out), 0.5), level, FINGER_CLOSED, "hold it level while the pin goes in",
                  DOWN_R))
    out += [_w(0.5, level, FINGER_OPEN, "let go: the pin holds the forearm", DOWN_R),
            _w(0.8, level + [0.0, 0.0, 0.03], FINGER_OPEN, "rise", DOWN_R)]
    out += _sweep("right", _good_ik("right", level + [0.0, 0.0, 0.03], DOWN_R), np.array(READY, float),
                  "back to where it started", FINGER_OPEN)
    return out


def _pin_tcp(depth: float) -> np.ndarray:
    """The left TCP on the pin's grip with the pin's tip ``depth`` into the hole."""
    return np.array([HOLE[0] + depth + GRIP_C, HOLE[1], HOLE[2] + PIN_IN_HAND])


def _left(t_push: float) -> tuple[list[Waypoint], float]:
    from yam_arena.constants import READY

    pick = PIN_START + np.array([GRIP_C, 0.0, PIN_IN_HAND])
    out = _sweep("left", np.array(READY, float), _good_ik("left", pick + [0.0, 0.0, OFF], DOWN_L),
                 "over the zeroing pin in its holder", FINGER_OPEN)
    out += [_w(1.0, pick, -0.016, "down round the pin's grip", DOWN_L),
            _w(0.6, pick, FINGER_CLOSED, "take the pin", DOWN_L),
            _w(0.8, pick + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it out of its holder", DOWN_L)]
    ready = _pin_tcp(-0.020)
    out.append(_w(2.0, ready, FINGER_CLOSED, "bring it in front of the zeroing hole", DOWN_L))
    out.append(_w(max(t_push - _dur(out), 0.3), ready, FINGER_CLOSED, "wait for the marks to line up", DOWN_L))
    home = _pin_tcp(PIN_HOME + 0.006)  # (pushed a little past home: the arm lags)
    for k in range(1, 6):
        out.append(_w(PUSH_S / 5, ready + (home - ready) * k / 5, FINGER_CLOSED, "push the pin into the hole",
                      DOWN_L))
    out.append(_w(0.6, home, FINGER_CLOSED, "the pin is home: the light comes on", DOWN_L))
    t_home = _dur(out)
    back = _pin_tcp(PIN_HOME)
    out += [_w(0.5, back, FINGER_OPEN, "let go of the pin", DOWN_L),
            _w(0.8, back + [-0.02, 0.0, OFF], FINGER_OPEN, "draw back and rise", DOWN_L)]
    out += _sweep("left", _good_ik("left", back + [-0.02, 0.0, OFF], DOWN_L), np.array(READY, float),
                  "back to where it started", FINGER_OPEN)
    return out, t_home


def demo():
    t_level = LEAD_S + 1.0 + 0.6 + LIFT_S + 0.8  # the right hand has the forearm level by then
    left, t_home = _left(t_level)
    right = _right(t_home + 0.4)
    t_end = max(_dur(right), _dur(left))
    plan = {"left": left, "right": right}
    for arm in ("left", "right"):
        last = plan[arm][-1]
        dt = t_end - _dur(plan[arm])
        if dt > 1e-6:
            plan[arm].append(_w(dt, last.pos, last.finger, "hold still", last.yaw))
        plan[arm].append(_w(3.0, last.pos, last.finger, "hold still: the trial ends once the full score has held 1 s",
                            last.yaw))
    return plan
