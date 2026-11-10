"""On the station table lies an open flute case: a black case lined with blue velvet, its lid open flat behind it. In
the case lie the two parts of a silver flute: the long body with its keys, in the front row, and the head joint with
its lip plate, in the back row, turned in its slot so that the lip plate faces up and toward the arms.

Task: put the flute together. Take the body in one hand and the head joint in the other, push the head joint's tenon
all the way into the body's socket, and turn the head joint until its embouchure hole (the hole in the lip plate)
lines up with the keys. Then lay the assembled flute on the open lid, keys up, and take both arms back to where they
started.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=assemble_a_flute_and_line_up_its_embouchure_hole_with_the_keys.py --instruction "..." ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, _quat_from_matrix, allow_matrix_grips,  # noqa: E402
                     as_numpy, on_reset, on_step, rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the case: the base (two rows of velvet-lined slots) and the lid lying open flat behind it ---------------------------
CX0, CX1, CY0, CY1 = 0.175, 0.330, -0.220, 0.300
C_FLOOR = TOP + 0.006  # the slots' floor
SADDLE = TOP + 0.022  # the velvet saddles the parts lie on
C_TOP = TOP + 0.036  # the base's rim
LX0, LX1, LID_T = 0.332, 0.440, 0.016
BED_H = 0.012  # three padded cradles on the lid, the assembled flute lies in them
CRADLE_Y = (-0.070, 0.060, 0.260)  # where they are along the flute, from its socket's mouth
LID_RIM = 0.012  # the lid's rim stands this high above its velvet
LID = TOP + LID_T  # the lid's velvet, open flat
ROW_BODY, ROW_HEAD = 0.222, 0.284  # the two rows' middle lines (x)
RAIL_X, RAIL_H = 0.0085, 0.003  # little velvet rails either side of each tube keep it from rolling
# -- the flute (3/4 size): octagonal silver tubes along their own +y axis --------------------------------------------
W = 0.015  # the tubes across
BODY_L = 0.340  # the body, from its socket's mouth (its origin) to the end of its foot
TENON = 0.025  # the head joint's tenon, from its tip (the head joint's origin) back
HEAD_L = 0.165  # the head joint, tip to crown
LIP_Y = -0.112  # the lip plate's middle along the head joint
BODY0 = np.array([ROW_BODY, -0.060, SADDLE + 0.0084])  # the body's socket mouth, lying in the case, keys up
HEAD0 = np.array([ROW_HEAD, -0.008, SADDLE + 0.0084])  # the head joint's tenon tip, lying in the case
HEAD_ROLL0 = math.radians(45.0)  # the head joint lies turned this far about its axis (lip plate up and away from the arms)
BODY_MASS, HEAD_MASS = 0.14, 0.05
LAY = np.array([0.380, -0.060, LID + 0.0165])  # the socket's mouth with the flute lying in its bed on the lid
# -- the fit -----------------------------------------------------------------------------------------------------------
CAPTURE_R, CAPTURE_DEG, CAPTURE_IN = 0.008, 12.0, 0.012  # the tenon's tip near the socket's axis, in its mouth
K_FIT, SLIP, D_FIT = 2000.0, 0.001, 3.0  # the tenon slides in against about 2 N and stays where it is left
TURN_DAMPING = 0.05  # N m s/rad: the snug fit resists turning the head joint in the socket ...
K_TURN, TURN_SLIP = 0.03, 2.0  # ... and holds it where it is left (N m per degree, up to this many degrees)
SEATED = TENON - 0.003  # the tenon this far in: all the way home
OUT = -0.004  # pulled this far back out of the mouth: apart again
ALIGN_DEG = 16.0  # the embouchure hole within this of the keys' line: lined up
KEYS_UP_DEG = 45.0  # the flute lies keys up with its keys within this of straight up

SILVER, SILVER_HI, SILVER_DK, PAD, DARK = (0.80, 0.81, 0.83), (0.92, 0.93, 0.94), (0.58, 0.59, 0.62), \
    (0.85, 0.80, 0.66), (0.05, 0.05, 0.06)
CASE_C, VELVET, VELVET_DK, CLASP = (0.06, 0.06, 0.07), (0.12, 0.18, 0.46), (0.08, 0.12, 0.32), (0.72, 0.70, 0.64)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _q(m: np.ndarray) -> tuple:
    return _quat_from_matrix(m)


def _tube(y0: float, y1: float, w: float, color, collide=True) -> list:
    """A round tube section along y from y0 to y1, ``w`` across."""
    return [CylShape((0.0, (y0 + y1) / 2, 0.0), w / 2, y1 - y0, color, collide=collide, axis="Y")]


# -- the scene ----------------------------------------------------------------------------------------------------------

def _case() -> Body:
    """The case's base with its two slots and their velvet saddles, and its lid lying open flat behind it."""
    ox, oy = (CX0 + CX1) / 2, (CY0 + CY1) / 2
    lx, ly = CX1 - CX0, CY1 - CY0
    shapes = [BoxShape((0.0, 0.0, (C_FLOOR - TOP) / 2), (lx, ly, C_FLOOR - TOP), CASE_C),
              BoxShape((0.0, 0.0, C_FLOOR - TOP + 0.0005), (lx - 0.012, ly - 0.012, 0.001), VELVET_DK, collide=False)]
    hz = (C_TOP - C_FLOOR) / 2 + C_FLOOR - TOP
    # the rim all round, and the ridges between and beside the two slots (velvet on top)
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, s * (ly / 2 - 0.006), hz), (lx, 0.012, C_TOP - C_FLOOR), CASE_C))
        shapes.append(BoxShape((s * (lx / 2 - 0.006), 0.0, hz), (0.012, ly, C_TOP - C_FLOOR), CASE_C))
    mid = (ROW_BODY + ROW_HEAD) / 2 - ox
    for x0 in (CX0 + 0.006, CX1 - 0.006):
        shapes.append(BoxShape((x0 - ox, 0.0, C_TOP - TOP + 0.0005), (0.013, ly, 0.001), VELVET, collide=False))
    shapes.append(BoxShape((mid, 0.0, C_FLOOR - TOP + 0.002), (0.006, ly - 0.012, 0.004), VELVET, collide=False))
    # saddles: two under the body, two under the head joint
    for row, ys in ((ROW_BODY, (BODY0[1] + 0.050, BODY0[1] + 0.280)), (ROW_HEAD, (HEAD0[1] - 0.045, HEAD0[1] - 0.130))):
        for y in ys:
            shapes.append(BoxShape((row - ox, y - oy, (SADDLE - TOP + C_FLOOR - TOP) / 2), (0.030, 0.022,
                                                                                            SADDLE - C_FLOOR), VELVET))
            for sx in (-1.0, 1.0):  # a groove in the saddle's top
                shapes.append(BoxShape((row + sx * RAIL_X - ox, y - oy, SADDLE - TOP + RAIL_H / 2), (0.006, 0.022,
                                                                                                     RAIL_H), VELVET))
    # the clasps on the front
    for y in (CY0 + 0.10, CY1 - 0.10):
        shapes.append(BoxShape((-lx / 2 - 0.002, y - oy, hz), (0.004, 0.022, 0.014), CLASP, collide=False))
    # the lid, open flat behind the base: black shell, velvet inside (up), its hinges
    lox = (LX0 + LX1) / 2 - ox
    shapes.append(BoxShape((lox, 0.0, (LID_T - 0.001) / 2), (LX1 - LX0, ly, LID_T - 0.001), CASE_C))
    shapes.append(BoxShape((lox, 0.0, LID_T - 0.0005), (LX1 - LX0 - 0.014, ly - 0.014, 0.001), VELVET))
    for y in (CY0 + 0.09, CY1 - 0.09):
        shapes.append(BoxShape((CX1 + 0.001 - ox, y - oy, 0.010), (0.006, 0.030, 0.010), CLASP, collide=False))
    for sx in (-1.0, 1.0):  # the lid's rim: it is a shallow tray
        shapes.append(BoxShape((lox + sx * (LX1 - LX0 - 0.010) / 2, 0.0, (LID_T + LID_RIM) / 2), (0.010, ly, LID_T + LID_RIM),
                               CASE_C))
        shapes.append(BoxShape((lox, sx * (ly - 0.010) / 2, (LID_T + LID_RIM) / 2), (LX1 - LX0, 0.010, LID_T + LID_RIM),
                               CASE_C))
    for dy in CRADLE_Y:  # three padded cradles on the lid for the assembled flute (none under the hand holding it)
        for sx in (-1.0, 1.0):
            shapes.append(BoxShape((LAY[0] + sx * 0.010 - ox, LAY[1] + dy - oy, LID_T + BED_H / 2), (0.008, 0.025, BED_H),
                                   VELVET_DK))
    return Body.build("case", pos=(ox, oy, TOP), mass=2.0, kinematic=True, color=CASE_C, shapes=shapes,
                      friction=(1.4, 1.3))


def _body() -> Body:
    """The flute's body and foot: a silver tube with its socket (barrel) at the origin end, eight key cups on top, the
    key rod along its side and the foot's roller keys."""
    shapes = []
    shapes += _tube(0.000, 0.030, 0.0185, SILVER_HI)  # the socket, round the head joint's tenon
    shapes += _tube(0.030, 0.034, 0.0195, SILVER_DK, collide=False)  # the ring at the socket's end
    shapes += _tube(0.034, 0.300, W, SILVER)
    shapes += _tube(0.300, 0.336, 0.0155, SILVER)
    shapes += _tube(0.336, BODY_L, 0.0185, SILVER_HI)
    top = W / 2
    for y in (0.080, 0.106, 0.132, 0.162, 0.188, 0.214, 0.248, 0.278):  # key cups on short arms, a pad under each
        shapes.append(BoxShape((0.0035, y, top + 0.002), (0.004, 0.004, 0.004), SILVER_DK, collide=False))
        shapes.append(CylShape((0.0, y, top + 0.0045), 0.0064, 0.003, SILVER_HI, collide=False))
        shapes.append(CylShape((0.0, y, top + 0.0032), 0.0058, 0.0012, PAD, collide=False))
        shapes.append(CylShape((0.0, y, top + 0.0061), 0.0020, 0.0006, DARK, collide=False))  # the open hole
    shapes.append(BoxShape((0.0085, 0.180, top + 0.002), (0.0026, 0.230, 0.0026), SILVER_HI, collide=False))  # the rod
    for y in (0.070, 0.150, 0.230, 0.290):  # posts holding the rod
        shapes.append(BoxShape((0.0085, y, top - 0.001), (0.003, 0.003, 0.006), SILVER_DK, collide=False))
    for y in (0.316, 0.327):  # the foot's roller keys
        shapes.append(CylShape((-0.0075, y, top + 0.0015), 0.003, 0.010, SILVER_HI, collide=False, axis="Y"))
    shapes.append(BoxShape((-0.0080, 0.095, top + 0.001), (0.003, 0.016, 0.003), SILVER_HI, collide=False))  # a trill
    return Body.build("flute_body", pos=tuple(BODY0.tolist()), mass=BODY_MASS, shapes=shapes, color=SILVER,
                      friction=(1.2, 1.1), com=(0.0, 0.170, 0.0), damping=(0.0, 0.1))


def _head() -> Body:
    """The head joint: the tenon (origin at its tip), the ring, the tube with the lip plate and its embouchure hole on
    top (its +z), and the crown."""
    shapes = []
    shapes += _tube(-TENON, 0.000, 0.0135, SILVER_HI)
    shapes += _tube(-TENON - 0.004, -TENON, 0.0185, SILVER_DK)
    shapes += _tube(-HEAD_L + 0.012, -TENON - 0.004, W, SILVER)
    shapes += _tube(-HEAD_L, -HEAD_L + 0.012, 0.0175, SILVER_HI)
    shapes.append(SphereShape((0.0, -HEAD_L - 0.004, 0.0), 0.0055, SILVER_HI, collide=False))  # the crown's finial
    top = W / 2
    shapes.append(BoxShape((0.0, LIP_Y, top + 0.0015), (0.012, 0.014, 0.003), SILVER_DK, collide=False))  # the riser
    shapes.append(BoxShape((0.0, LIP_Y, top + 0.0035), (0.019, 0.028, 0.002), SILVER_HI, collide=False))  # lip plate
    shapes.append(BoxShape((0.0, LIP_Y, top + 0.0042), (0.0075, 0.011, 0.0012), DARK, collide=False))  # the hole
    return Body.build("flute_head", pos=tuple(HEAD0.tolist()), mass=HEAD_MASS, shapes=shapes, color=SILVER,
                      friction=(1.2, 1.1), com=(0.0, -0.080, 0.0), damping=(0.0, 0.1), ignore=("flute_body",),
                      start_xyzw=_q(_ry(HEAD_ROLL0)))


thread("fit", "flute_body", "flute_head", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, 1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 1.0, 0.0), slide_limits=(OUT - 0.002, TENON), turn_damping=TURN_DAMPING, slide_stiffness=K_FIT,
       slide_damping=D_FIT, slide_max_force=40.0)

_author_threads_plain = bodykit._author_threads


def _author_threads(stage, prim_path: str, spec) -> None:
    """bodykit's threads, then: a turning spring on the fit, whose target the task file moves to wherever the head
    joint is turned (stick-slip), so it stays where it is left."""
    _author_threads_plain(stage, prim_path, spec)
    if spec.name == "flute_head":
        from pxr import UsdPhysics

        drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/fit"), "rotX")
        drive.GetStiffnessAttr().Set(float(K_TURN))
        drive.GetTargetPositionAttr().Set(0.0)


bodykit._author_threads = _author_threads
_TURN_ATTR: dict = {}


def set_turn_target(deg: float) -> None:
    """Move the fit's turning spring's target (degrees of the head joint's turn against the body)."""
    if "attr" not in _TURN_ATTR:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(bodykit._THREAD_PATHS["fit"])
        _TURN_ATTR["attr"] = UsdPhysics.DriveAPI(prim, "rotX").GetTargetPositionAttr()
    _TURN_ATTR["attr"].Set(float(deg))

OBJECTS = [_case(), _body(), _head()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": False, "hold": 0.0, "turn": 0.0, "tcps": None, "s": {}, "depth": None, "roll": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FLUTE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_thread_on("fit", False)
    set_turn_target(0.0)


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


def in_socket(pb, rb, ph, rh):
    """Where the tenon's tip is against the socket: (distance off the socket's axis, how far in past its mouth, the
    angle between the two parts' axes in degrees)."""
    q = rb.T @ (ph - pb)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(rb[:, 1] @ rh[:, 1])))))
    return math.hypot(q[0], q[2]), float(q[1]), angle


def roll_deg(rb, rh) -> float:
    """How far the embouchure hole is turned off the keys' line, about the flute's axis, degrees."""
    v = rb.T @ rh[:, 2]
    return math.degrees(math.atan2(v[0], v[2]))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "flute_body")
    ph, rh = _pose(base, origin, "flute_head")
    off, d, ang = in_socket(pb, rb, ph, rh)
    roll = roll_deg(rb, rh)
    if not _EP["on"]:
        if off <= CAPTURE_R and -0.002 <= d <= CAPTURE_IN and ang <= CAPTURE_DEG:  # the tenon in the socket's mouth
            set_slide_target("fit", d)
            set_turn_target(roll)
            set_thread_on("fit", True)
            _EP["on"], _EP["hold"], _EP["turn"] = True, d, roll
    elif d < OUT:  # pulled back out: apart again
        set_thread_on("fit", False)
        _EP["on"] = False
    else:
        if abs(d - _EP["hold"]) > SLIP:  # pushed (or pulled) along: it slides, and stays where it is left
            _EP["hold"] = d
            set_slide_target("fit", d)
        if abs(roll - _EP["turn"]) > TURN_SLIP:  # turned: it turns, and stays where it is left
            _EP["turn"] = roll
            set_turn_target(roll)
    seated = _EP["on"] and d >= SEATED
    lined_up = seated and abs(roll) <= ALIGN_DEG
    keys_up = float(rb[2, 2]) >= math.cos(math.radians(KEYS_UP_DEG))
    on_lid = all(LX0 + 0.005 <= p[0] <= LX1 - 0.005 and CY0 <= p[1] <= CY1 and p[2] <= LID + 0.030
                 for p in (pb + rb[:, 1] * 0.17, ph + rh[:, 1] * -0.08))
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"seated": seated, "lined_up": lined_up, "on_lid": on_lid and keys_up}
    _EP["depth"], _EP["roll"] = d, roll
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = _EP["tcps"]
            mm = lambda v: np.round((np.asarray(v) - [0, 0, TOP]) * 1000).astype(int).tolist()  # noqa: E731
            print(f"[flute] t={_DEBUG['steps'] / 20:5.1f}s body={mm(pb)} head={mm(ph)} off={off * 1000:5.1f}mm"
                  f" in={d * 1000:6.1f}mm ang={ang:5.1f} on={_EP['on']} roll={roll:6.1f}"
                  f" keys={math.degrees(math.acos(max(-1.0, min(1.0, float(rb[2, 2]))))):4.1f} keys_up={keys_up}"
                  f" seated={seated} lined={lined_up} on_lid={on_lid} L={mm(tl)} R={mm(trc)} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """At the end: 1.0 with the head joint all the way in the body, its embouchure hole lined up with the keys, the
    flute lying keys up on the open lid and both arms back where they started; 0.5 with the head joint all the way in
    and lined up; 0.25 with it all the way in; else 0.0."""
    s = _EP["s"]
    if not s.get("seated"):
        return 0.0
    if not s.get("lined_up"):
        return 0.25
    if s.get("on_lid") and _arms_home():
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
    """Full score with both parts of the flute still on the lid: the trial ends there."""
    return score(objects) >= 1.0 and objects["flute_body"].resting() and objects["flute_head"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the body by its middle and the right hand the head joint, both from above, fingers closing
# across the tubes, and lift them out of the case. They bring them together in the air in front of the case, the head
# joint's tenon at the socket's mouth, and the right hand pushes it all the way in. Then the right hand leans 45
# degrees, turning the head joint about its axis until the lip plate is on top, over the keys. It lets go and takes the
# head joint again from straight above; together the hands lay the flute on the open lid, keys up, let go and go back
# to where they started.

TUBE_FINGER = -0.014  # open round a tube
RELEASE_FINGER = -0.032  # opened wide to let go, so the fingers do not drag the tube round as they rise
GRIP_DZ = 0.000  # the TCP on the tube's axis
OVER = 0.070
LEAD_S, LEAD_STEPS = 3.0, 15
BODY_GRIP = 0.140  # the left hand holds the body this far from its socket's mouth
HEAD_GRIP = -0.070  # the right hand holds the head joint this far back from its tenon's tip
AIR = np.array([0.300, 0.005, TOP + 0.100])  # the socket's mouth while putting the flute together
TWIST = math.radians(-48.0)
PUSH_PAST = 0.025  # the right hand aims this far past seated: the fit and the soft arms hold it back
BRACE = 0.012  # meanwhile the left hand pushes the body this far toward the head joint
LEFT_LAY_AIM = 0.0  # aim the left hand this much further out over the lid


def _grip_rot(lean: float = 0.0, arm: str = "right") -> np.ndarray:
    """A top-down grip with the fingers closing along x (across a tube lying along y), leaned ``lean`` about the
    tube's axis."""
    return _ry(lean) @ grip_rotation(math.pi / 2 if arm == "right" else -math.pi / 2)


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

    up = np.array([0.0, 0.0, 1.0])
    yv = np.array([0.0, 1.0, 0.0])
    body_pick = BODY0 + yv * BODY_GRIP + up * GRIP_DZ
    head_pick = HEAD0 + yv * HEAD_GRIP + up * GRIP_DZ
    body_air = AIR + yv * BODY_GRIP
    head_start = AIR + yv * (HEAD_GRIP - 0.012)  # the tenon's tip 1.2 cm short of the socket's mouth
    head_home = AIR + yv * (HEAD_GRIP + TENON + PUSH_PAST)
    head_seated = AIR + yv * (HEAD_GRIP + TENON + 0.004)
    body_lay, head_lay = LAY + yv * BODY_GRIP, LAY + yv * (HEAD_GRIP + TENON)
    r0, l0 = _grip_rot(), _grip_rot(arm="left")
    L, R = [], []
    L += _joint_path("left", np.array(READY, float), _good_ik("left", body_pick + up * OVER, l0),
                     "over the flute's body", TUBE_FINGER)
    R += _joint_path("right", np.array(READY, float), _good_ik("right", head_pick + up * OVER, r0),
                     "over the head joint", TUBE_FINGER)
    for seq, pick, what, rr in ((L, body_pick, "body", l0), (R, head_pick, "head joint", r0)):
        seq.append(_w(1.2, pick, TUBE_FINGER, f"down round the {what}", rr))
        seq.append(_w(0.6, pick, FINGER_CLOSED, f"take the {what}", rr))
        seq.append(_w(1.0, pick + up * 0.050, FINGER_CLOSED, f"lift the {what} out of the case", rr))
    L.append(_w(2.0, body_air, FINGER_CLOSED, "hold the body up in front of the case", l0))
    t_push = sum(w.duration for w in R) + 2.0 + 1.0  # when the right hand starts pushing the head joint in
    L.append(_w(t_push - sum(w.duration for w in L), body_air, FINGER_CLOSED, "hold the body still", l0))
    L.append(_w(1.6, body_air - yv * BRACE, FINGER_CLOSED, "hold the body against the push", l0))
    L.append(_w(0.6, body_air - yv * BRACE, FINGER_CLOSED, "hold the body against the push", l0))
    L.append(_w(1.0, body_air - yv * BRACE / 3, FINGER_CLOSED, "hold the body still", l0))
    R.append(_w(2.0, head_start + [0.0, -0.030, 0.0], FINGER_CLOSED, "bring the head joint to the body", r0))
    R.append(_w(1.0, head_start, FINGER_CLOSED, "the tenon at the socket's mouth", r0))
    R.append(_w(1.6, head_home, FINGER_CLOSED, "push the head joint all the way in", r0))
    R.append(_w(0.6, head_seated, FINGER_CLOSED, "push the head joint all the way in", r0))
    for k in range(1, 6):
        R.append(_w(0.5, head_seated, FINGER_CLOSED, "turn the head joint: the lip plate over the keys",
                    _grip_rot(TWIST * k / 5)))
    R.append(_w(0.5, head_seated, FINGER_CLOSED, "turn the head joint: the lip plate over the keys", _grip_rot(TWIST)))
    R.append(_w(0.6, head_seated, TUBE_FINGER, "let go of the head joint", _grip_rot(TWIST)))
    R.append(_w(0.8, head_seated + up * 0.050, TUBE_FINGER, "rise clear", _grip_rot(TWIST)))
    t_l, t_r = sum(w.duration for w in L), sum(w.duration for w in R)
    L.append(_w(t_r - t_l + LEAD_S, body_air - yv * BRACE / 3, FINGER_CLOSED,
                "hold the flute still while the right hand goes back", l0))
    b_lay = body_lay + [LEFT_LAY_AIM, 0.0, 0.002]  # 2 mm above the cradles: let go without pressing it in
    L.append(_w(1.8, (body_air + b_lay) / 2 + up * 0.040, FINGER_CLOSED, "carry the flute over the open lid", l0))
    L.append(_w(1.2, b_lay + up * 0.020, FINGER_CLOSED, "lower it over the lid's bed", l0))
    L.append(_w(1.2, b_lay, FINGER_CLOSED, "lay the flute in the bed on the lid, keys up", l0))
    L.append(_w(0.6, b_lay, FINGER_CLOSED, "lay the flute in the bed on the lid, keys up", l0))
    L.append(_w(0.8, b_lay, RELEASE_FINGER, "let go, opening wide", l0))
    L.append(_w(1.0, b_lay + up * OVER, RELEASE_FINGER, "rise clear", l0))
    plan = {"left": L, "right": R}
    for arm, top_pt, rr in (("left", b_lay, l0), ("right", head_seated, _grip_rot(TWIST))):
        plan[arm] += _joint_path(arm, _good_ik(arm, top_pt + up * (OVER if arm == "left" else 0.050), rr),
                                 np.array(READY, float), "back to where it started",
                                 RELEASE_FINGER if arm == "left" else TUBE_FINGER)
        last = plan[arm][-1]
        plan[arm].append(_w(3.0, last.pos, TUBE_FINGER,
                            "hold still: the trial ends once the full score has held 1 s with both arms home",
                            last.yaw))
    return plan
