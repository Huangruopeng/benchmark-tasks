"""A small tweed-covered guitar practice amp stands on a smooth grey desk pad, switched on (its red pilot light glows).
Its front, with the brown grille cloth and the INPUT jack socket on a cream strip above it, faces the right arm. On its
cream top panel are three black chicken-head knobs in a row, each with a white pointer line and a ring of printed
ticks: VOLUME (nearest the arms), TONE and GAIN. The VOLUME knob is turned part way up. A black leather carry handle
runs front to back along the top's back half. A guitar cable's jack plug lies on the pad in front of the INPUT socket,
pointing at it.

Task: plug the guitar cable in without a loud pop, then put the volume back where it was. Turn the VOLUME knob all the
way down first; only then push the jack plug fully into the INPUT socket; then turn the VOLUME knob back up to exactly
where it was set before (within 15 degrees). Plugging in while the volume is up scores 0. Both arms end where they
started. Once the knob is down, nothing on the amp shows where it was: the setting has to be remembered. The amp is
light and stands on smooth glides, so one hand has to hold it (its handle is handy) while the other works: the plug
needs a firm push to go in, and on an unheld amp that push just slides the amp away.

Run it:  inspect-robots run --embodiment yam_arena -E task=turn_an_amps_volume_down_to_plug_in_then_set_it_back_where_it_was.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_hinge_drive, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the desk pad: smooth melamine, fixed -----------------------------------------------------------------------------
PAD_T = 0.004
PAD_TOP = TOP + PAD_T
PX0, PX1, PY0, PY1 = 0.120, 0.470, -0.250, 0.150
PAD_FRICTION = 0.05
# -- the amp: its origin is the middle of its base (the bottom of its feet); world-aligned, its front facing -y ------
AX, AY = 0.300, 0.040
WX, WY, H = 0.240, 0.130, 0.105  # width (x), depth (y), height of the cabinet
FEET = 0.003
TOPZ = FEET + H  # the cabinet's top, in the amp's frame
AMP_MASS = 0.55
FRONT = -WY / 2  # the front face, in the amp's frame
SX, SZ = -0.050, 0.074  # the INPUT socket's axis on the front face (amp frame x, z)
CH, CH_D = 0.0075, 0.035  # the socket's channel: half width and depth
KNOB_Y = -0.044  # the row of knobs on the top panel (amp frame y)
KNOBS = ("volume", "tone", "gain")
KNOB_X = {"volume": -0.090, "tone": -0.040, "gain": 0.010}
# degrees, the knob's turn on the amp: 0 points to the back (12 o'clock seen from the front), + is anticlockwise from
# above (turning down); fully down at +135 (7:30), full up at -135 (4:30)
ZERO_DEG, MAX_DEG = 135.0, -135.0
START = {"volume": 40.0, "tone": -10.0, "gain": -70.0}  # VOLUME a bit past 3 of 10
K_KNOB = 0.045  # N m per degree: a knob's pot holds it where it was left ...
KNOB_CAP = 0.15  # ... up to this torque (N m); turned harder it turns, and stays where it is left
D_KNOB = 0.0004
SLIP_DEG = 2.0
DOWN_WITHIN = 12.0  # VOLUME counts as down within this of fully down
BACK_WITHIN = 15.0  # ... and as back where it was within this of its start
HANDLE_X0, HANDLE_X1, HANDLE_Y, HANDLE_W = -0.030, 0.090, 0.035, 0.026  # the carry handle on top (amp frame)
# -- the jack plug: its origin is the middle of its moulded barrel, its tip along +y ----------------------------------
TIP_Y = 0.0535
PLUG_MASS = 0.05
PLUG0 = np.array([AX + SX, -0.170, PAD_TOP + 0.0065])
K_JACK, D_JACK, F_JACK = 2000.0, 20.0, 1.5  # the socket's contacts: the plug slides in only when pushed with 1.5 N
SLIP_M = 0.001
DEPTH = 0.027  # the tip this far in: the plug is home (its collar just clear of the socket's nut)
SEATED = DEPTH - 0.004
OUT = -0.006
CAPTURE_R, CAPTURE_IN, CAPTURE_DEG = 0.008, 0.010, 12.0

TWEED, TWEED_D, BROWN = (0.80, 0.68, 0.46), (0.55, 0.42, 0.24), (0.34, 0.19, 0.13)
GOLD, CREAM, BLACK, LEATHER = (0.78, 0.63, 0.30), (0.92, 0.88, 0.76), (0.05, 0.05, 0.05), (0.10, 0.07, 0.05)
CHROME, CHROME_D, WHITE, RED = (0.86, 0.86, 0.88), (0.55, 0.56, 0.58), (0.97, 0.97, 0.95), (0.95, 0.12, 0.08)
PAD_C, INK = (0.30, 0.31, 0.33), (0.12, 0.10, 0.08)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _yawq(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _box(x0, x1, y0, y1, z0, z1, color, collide=True) -> BoxShape:
    return BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color, collide=collide)


def _pad() -> Body:
    cx, cy = (PX0 + PX1) / 2, (PY0 + PY1) / 2
    shapes = [BoxShape((0.0, 0.0, PAD_T / 2), (PX1 - PX0, PY1 - PY0, PAD_T), PAD_C)]
    return Body.build("desk_pad", pos=(cx, cy, TOP), mass=2.0, kinematic=True, color=PAD_C, shapes=shapes,
                      friction=(PAD_FRICTION, PAD_FRICTION), slick=True)


def _amp() -> Body:
    hx, hy = WX / 2, WY / 2
    z0, z1 = FEET, TOPZ
    shapes = []
    for sx in (-1.0, 1.0):  # four black glides
        for sy in (-1.0, 1.0):
            shapes.append(CylShape((sx * (hx - 0.020), sy * (hy - 0.017), FEET / 2), 0.009, FEET, BLACK))
    # the cabinet, tweed, built round a channel behind the INPUT socket
    yb = FRONT + CH_D
    shapes.append(_box(-hx, hx, yb, hy, z0, z1, TWEED))
    shapes.append(_box(-hx, SX - CH, FRONT, yb, z0, z1, TWEED))
    shapes.append(_box(SX + CH, hx, FRONT, yb, z0, z1, TWEED))
    shapes.append(_box(SX - CH, SX + CH, FRONT, yb, z0, SZ - CH, TWEED))
    shapes.append(_box(SX - CH, SX + CH, FRONT, yb, SZ + CH, z1, TWEED))
    # tweed weave: faint darker diagonals on the top and the sides (drawn only)
    for i in range(-7, 8):
        shapes.append(BoxShape((i * 0.017, 0.040, z1 + 0.0003), (0.0014, 0.060, 0.0004), TWEED_D, yaw=math.radians(35),
                               collide=False))
    for sx in (-1.0, 1.0):
        for k in range(5):
            shapes.append(BoxShape((sx * (hx + 0.0003), -0.04 + 0.022 * k, z0 + H / 2), (0.0004, 0.0014, H * 0.9),
                                   TWEED_D, collide=False))
    # black corner caps
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for zc in (z0 + 0.006, z1 - 0.006):
                shapes.append(BoxShape((sx * (hx - 0.005), sy * (hy - 0.005), zc), (0.0135, 0.0135, 0.0135), BLACK,
                                       collide=False))
    # the front: brown grille cloth with gold threads below, a cream strip with the INPUT socket above
    yf = FRONT - 0.0006
    shapes.append(_box(-hx + 0.012, hx - 0.012, yf - 0.0004, yf + 0.0004, z0 + 0.010, 0.060, BROWN, collide=False))
    for k in range(7):
        shapes.append(_box(-hx + 0.014, hx - 0.014, yf - 0.0008, yf - 0.0004, z0 + 0.015 + 0.0062 * k,
                           z0 + 0.0158 + 0.0062 * k, GOLD, collide=False))
    shapes.append(_box(0.040, 0.090, yf - 0.0012, yf - 0.0008, 0.050, 0.056, GOLD, collide=False))  # the logo plate
    shapes.append(_box(-hx + 0.012, hx - 0.012, yf - 0.0004, yf + 0.0004, 0.064, z1 - 0.008, CREAM, collide=False))
    shapes.append(CylShape((SX, FRONT - 0.0014, SZ), 0.0095, 0.0016, CHROME, collide=False, axis="Y"))  # the nut
    shapes.append(CylShape((SX, FRONT - 0.0024, SZ), 0.0058, 0.0008, BLACK, collide=False, axis="Y"))  # the hole
    shapes.append(_box(SX - 0.010, SX + 0.010, yf - 0.0010, yf - 0.0006, SZ - 0.016, SZ - 0.0135, INK, collide=False))
    # the top panel: cream, with each knob's ring of ticks, the pilot light and the power switch
    shapes.append(_box(-hx + 0.006, hx - 0.006, FRONT + 0.004, FRONT + 0.046, z1, z1 + 0.0008, CREAM, collide=False))
    for k in KNOBS:
        for j in range(11):
            a = math.radians(90.0 + ZERO_DEG - 27.0 * j)
            r = 0.0215
            long = j in (0, 10)
            shapes.append(BoxShape((KNOB_X[k] + r * math.cos(a), KNOB_Y + r * math.sin(a), z1 + 0.0010),
                                   (0.0060 if long else 0.0040, 0.0013, 0.0004), INK, yaw=a, collide=False))
    shapes.append(CylShape((0.085, KNOB_Y, z1 + 0.0015), 0.0085, 0.003, CHROME, collide=False))
    shapes.append(CylShape((0.085, KNOB_Y, z1 + 0.0045), 0.0058, 0.005, RED, collide=False))  # the pilot light, on
    shapes.append(CylShape((0.106, KNOB_Y, z1 + 0.0015), 0.0050, 0.003, CHROME, collide=False))
    shapes.append(BoxShape((0.106, KNOB_Y + 0.004, z1 + 0.006), (0.0025, 0.0025, 0.009), CHROME, collide=False))
    # the carry handle: a leather strap on two black end caps, along the top's back half
    for x in (HANDLE_X0, HANDLE_X1):
        shapes.append(_box(x - 0.008, x + 0.008, HANDLE_Y - 0.016, HANDLE_Y + 0.016, z1, z1 + 0.012, BLACK))
        shapes.append(CylShape((x, HANDLE_Y, z1 + 0.0122), 0.003, 0.0008, CHROME, collide=False))
    shapes.append(_box(HANDLE_X0 + 0.004, HANDLE_X1 - 0.004, HANDLE_Y - HANDLE_W / 2, HANDLE_Y + HANDLE_W / 2,
                       z1 + 0.006, z1 + 0.016, LEATHER))
    shapes.append(_box(HANDLE_X0 + 0.010, HANDLE_X1 - 0.010, HANDLE_Y - 0.0110, HANDLE_Y - 0.0100, z1 + 0.0161,
                       z1 + 0.0165, TWEED_D, collide=False))  # stitching
    shapes.append(_box(HANDLE_X0 + 0.010, HANDLE_X1 - 0.010, HANDLE_Y + 0.0100, HANDLE_Y + 0.0110, z1 + 0.0161,
                       z1 + 0.0165, TWEED_D, collide=False))
    return Body.build("amp", pos=(AX, AY, PAD_TOP), mass=AMP_MASS, shapes=shapes, color=TWEED, friction=(0.8, 0.7),
                      com=(0.0, 0.010, 0.040), damping=(0.0, 0.05))


def _knob(k: str) -> Body:
    """A black chicken-head knob in its own frame (its axis), its pointer along +y."""
    shapes = [CylShape((0.0, 0.0, 0.002), 0.012, 0.004, BLACK),
              BoxShape((0.0, 0.006, 0.0125), (0.011, 0.038, 0.017), BLACK),
              BoxShape((0.0, 0.0275, 0.009), (0.006, 0.008, 0.010), BLACK),
              BoxShape((0.0, 0.0125, 0.0212), (0.0024, 0.027, 0.0006), WHITE, collide=False),
              BoxShape((0.0, 0.0305, 0.0142), (0.0020, 0.0006, 0.0080), WHITE, collide=False)]
    pos = (AX + KNOB_X[k], AY + KNOB_Y, PAD_TOP + TOPZ + 0.001)
    return Body.build(f"knob_{k}", pos=pos, mass=0.02, shapes=shapes, color=BLACK, friction=(1.0, 0.9),
                      com=(0.0, 0.006, 0.010), start_xyzw=_yawq(START[k]), damping=(0.0, 0.05))


def _plug() -> Body:
    """The cable's jack plug in its own frame (the middle of its barrel), tip along +y; the cable is a short tail."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (0.013, 0.040, 0.013), BLACK),
              BoxShape((0.0, 0.0, 0.0066), (0.009, 0.030, 0.0004), (0.20, 0.20, 0.21), collide=False),
              CylShape((0.0, 0.0225, 0.0), 0.0062, 0.005, CHROME, axis="Y"),
              CylShape((0.0, 0.036, 0.0), 0.0032, 0.022, CHROME, axis="Y"),
              CylShape((0.0, 0.0475, 0.0), 0.0034, 0.002, BLACK, collide=False, axis="Y"),
              SphereShape((0.0, 0.0505, 0.0), 0.0030, CHROME),
              BoxShape((0.0, -0.027, 0.0), (0.010, 0.014, 0.010), BLACK),
              CylShape((0.0, -0.064, 0.0), 0.0035, 0.060, (0.08, 0.08, 0.09), collide=False, axis="Y")]
    return Body.build("plug", pos=tuple(PLUG0.tolist()), mass=PLUG_MASS, shapes=shapes, color=BLACK,
                      friction=(1.0, 0.9), com=(0.0, 0.002, 0.0), damping=(0.0, 0.1))


for _k in KNOBS:
    hinge(f"turn_{_k}", "amp", f"knob_{_k}", anchor=(AX + KNOB_X[_k], AY + KNOB_Y, PAD_TOP + TOPZ + 0.001), axis="Z",
          limits_deg=(MAX_DEG, ZERO_DEG), spring=K_KNOB, damping=D_KNOB, target_deg=START[_k])

thread("jack", "amp", "plug", anchor0=(SX, FRONT, SZ), axis0=(0.0, 1.0, 0.0), anchor1=(0.0, TIP_Y, 0.0),
       axis1=(0.0, 1.0, 0.0), slide_limits=(OUT - 0.002, DEPTH), turn_damping=0.002, slide_stiffness=K_JACK,
       slide_damping=D_JACK, slide_max_force=F_JACK)

_author_hinges_plain = bodykit._author_hinges


def _author_hinges(stage, prim_path: str, spec) -> None:
    """bodykit's hinges, then: each knob's pot holds it with at most KNOB_CAP (a capped drive acts like friction)."""
    _author_hinges_plain(stage, prim_path, spec)
    if spec.name.startswith("knob_"):
        from pxr import UsdPhysics

        k = spec.name[len("knob_"):]
        drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/turn_{k}"), "angular")
        drive.GetMaxForceAttr().Set(float(KNOB_CAP))


bodykit._author_hinges = _author_hinges

OBJECTS = [_pad(), _amp(), *[_knob(k) for k in KNOBS], _plug()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": dict(START), "turn": dict(START), "on": False, "slide": 0.0, "d": None, "seated": False,
            "unsafe": False, "plugged_at": None, "tcps": None, "amp": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GUITARAMP_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_thread_on("jack", False)


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


def _yaw(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def in_socket(pa, ra, pp, rp):
    """Where the plug's tip is against the INPUT socket: (distance off the socket's axis, how far in past its mouth,
    the angle between the plug's axis and the socket's, degrees)."""
    mouth = pa + ra @ np.array([SX, FRONT, SZ])
    axis = ra[:, 1]
    tip = pp + rp @ np.array([0.0, TIP_Y, 0.0])
    v = tip - mouth
    d = float(v @ axis)
    off = float(np.linalg.norm(v - d * axis))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(axis @ rp[:, 1])))))
    return off, d, ang


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pa, ra = _pose(base, origin, "amp")
    for k in KNOBS:  # each knob's pot: it turns only when pushed past its hold, and stays where it is left
        _, rk = _pose(base, origin, f"knob_{k}")
        rel = ra.T @ rk
        turn = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
        _EP["turn"][k] = turn
        if abs(turn - _EP["hold"][k]) > SLIP_DEG:
            _EP["hold"][k] = turn
            set_hinge_drive(f"turn_{k}", target_deg=turn)
    pp, rp = _pose(base, origin, "plug")
    off, d, ang = in_socket(pa, ra, pp, rp)
    if not _EP["on"]:
        if off <= CAPTURE_R and -0.004 <= d <= CAPTURE_IN and ang <= CAPTURE_DEG:  # the tip in the socket's mouth
            set_slide_target("jack", d)
            set_thread_on("jack", True)
            _EP["on"], _EP["slide"] = True, d
            if _EP["plugged_at"] is None:
                _EP["plugged_at"] = _DEBUG["steps"]
            if _EP["turn"]["volume"] < ZERO_DEG - DOWN_WITHIN:
                _EP["unsafe"] = True  # plugged in with the volume up: a loud pop, latched
    elif d < OUT:  # pulled back out
        set_thread_on("jack", False)
        _EP["on"] = False
    elif abs(d - _EP["slide"]) > SLIP_M:  # pushed (or pulled) past the contacts' hold: it slides, and stays
        _EP["slide"] = d
        set_slide_target("jack", d)
    _EP["d"] = d
    _EP["seated"] = _EP["on"] and d >= SEATED
    _EP["tcps"] = _tcps(base, origin)
    _EP["amp"] = (pa, _yaw(ra))
    _DEBUG["steps"] += 1
    if _DEBUG["on"] and _DEBUG["steps"] % 5 == 0:
        tl, tr = _EP["tcps"]
        mm = lambda v: np.round((np.asarray(v) - [0, 0, TOP]) * 1000).astype(int).tolist()  # noqa: E731
        t = _EP["turn"]
        print(f"[amp] t={_DEBUG['steps'] / 20:5.1f}s vol={t['volume']:6.1f} tone={t['tone']:6.1f}"
              f" gain={t['gain']:6.1f} amp={mm(pa)} yaw={_yaw(ra):5.1f} plug={mm(pp)} off={off * 1000:5.1f}"
              f" in={d * 1000:6.1f} ang={ang:5.1f} on={_EP['on']} seated={_EP['seated']} unsafe={_EP['unsafe']}"
              f" L={mm(tl)} R={mm(tr)} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _volume_back() -> bool:
    return abs(_EP["turn"]["volume"] - START["volume"]) <= BACK_WITHIN


def score(objects) -> float:
    """0.0 once the plug has gone into the INPUT socket while the VOLUME knob was not all the way down. Otherwise, at
    the end: 1.0 with the plug home in the socket, the VOLUME knob back within 15 degrees of where it started and both
    arms back where they started; 0.5 with the plug home and the VOLUME knob back where it was; 0.25 with the plug
    home; else 0.0."""
    if _EP["unsafe"] or not _EP["seated"]:
        return 0.0
    if not _volume_back():
        return 0.25
    if _arms_home():
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
    """Full score with the amp, the VOLUME knob and the plug still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["amp"].resting() and objects["knob_volume"].resting()
            and objects["plug"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the carry handle and holds the amp. The right comes down on the VOLUME knob, takes its
# pointer and turns it anticlockwise all the way down (aimed 5 degrees past, against the stop), lets go and rises. It
# takes the jack plug off the pad by its barrel, lifts it to the INPUT socket's height, lines it up and pushes it home,
# lets go and backs off. Then it takes the VOLUME knob again and turns it clockwise back to where it was set, lets go
# and goes home; the left lets go of the handle and goes home.

LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.050
OVER_L = 0.030  # the left arm reaches no higher over the handle
LEFT_TWIST = -4.5  # degrees: the left's pads twist what they clamp about 4.5 degrees anticlockwise; aim against it
HANDLE_AT = 0.020  # the left hand takes the handle here (amp frame x)
HANDLE_FINGER = -0.022
KNOB_FINGER = -0.014
KNOB_LET_GO = -0.024  # opened well clear of the pointer before rising (a snagged pointer lifts the light amp)
PLUG_FINGER = -0.016
KNOB_TCP = 0.017  # the TCP this far above the panel on a knob's pointer
HANDLE_TCP = 0.014  # ... above the cabinet's top on the handle
PLUG_TCP = 0.0075  # ... above the plug's axis on its barrel
TURN_STEP = 12.0  # degrees per waypoint while turning a knob
DOWN_AIM = ZERO_DEG + 5.0
PAST = 0.015  # the plug pushed this far past home


def _knob_grip(turn: float):
    """The right hand's TCP pose on the VOLUME knob's pointer with the knob turned ``turn`` degrees (the amp unmoved)."""
    p = np.array([AX + KNOB_X["volume"], AY + KNOB_Y, PAD_TOP + TOPZ + KNOB_TCP])
    return p, grip_rotation(math.radians(turn - 90.0))


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.5, 2.0), (1.2, 1.8), (-1.3, -0.8), (0.0,), (-1.0, 0.0, 1.0)):
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


def _turn(R: list, a0: float, a1: float, label: str) -> None:
    n = max(1, int(math.ceil(abs(a1 - a0) / TURN_STEP)))
    for k in range(1, n + 1):
        p, r = _knob_grip(a0 + (a1 - a0) * k / n)
        R.append(_w(0.40, p, FINGER_CLOSED, label, r))


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    L, R = [], []
    up = np.array([0.0, 0.0, OVER])
    ready = np.array(READY, float)

    # the left hand takes the carry handle and holds the amp
    hp = np.array([AX + HANDLE_AT, AY + HANDLE_Y, PAD_TOP + TOPZ + HANDLE_TCP])
    hr = grip_rotation(math.radians(LEFT_TWIST))
    upl = np.array([0.0, 0.0, OVER_L])
    L += _joint_path("left", ready, _good_ik("left", hp + upl, hr), "over the amp's carry handle", HANDLE_FINGER)
    L.append(_w(1.2, hp, HANDLE_FINGER, "down round the handle", hr))
    L.append(_w(0.6, hp, FINGER_CLOSED, "hold the amp by its handle", hr))
    t_held = sum(w.duration for w in L)

    # the right turns the VOLUME knob all the way down
    kp, kr = _knob_grip(START["volume"])
    R += _joint_path("right", ready, _good_ik("right", kp + up, kr), "over the VOLUME knob", KNOB_FINGER)
    R.append(_w(max(0.3, t_held - sum(w.duration for w in R)), kp + up, KNOB_FINGER, "wait for the amp to be held", kr))
    R.append(_w(1.0, kp, KNOB_FINGER, "down round the knob's pointer", kr))
    R.append(_w(0.6, kp, FINGER_CLOSED, "take the knob", kr))
    _turn(R, START["volume"], DOWN_AIM, "turn the volume all the way down")
    p, r = _knob_grip(DOWN_AIM)
    R.append(_w(0.4, p, FINGER_CLOSED, "turn the volume all the way down", r))
    R.append(_w(0.7, p, KNOB_LET_GO, "let go of the knob", r))
    R.append(_w(1.0, p + up, KNOB_LET_GO, "rise clear of the knob", r))

    # it takes the jack plug and pushes it home in the INPUT socket
    g = grip_rotation(math.pi / 2)
    pick = PLUG0 + [0.0, 0.0, PLUG_TCP]
    R.append(_w(1.6, pick + [0.0, 0.0, 0.07], PLUG_FINGER, "over the jack plug", g))
    R.append(_w(1.0, pick, PLUG_FINGER, "down round its barrel", g))
    R.append(_w(0.6, pick, FINGER_CLOSED, "take the plug", g))
    mouth_y = AY + FRONT
    z_in = PAD_TOP + SZ + PLUG_TCP
    x_in = AX + SX
    R.append(_w(1.0, (pick[0], pick[1], z_in + 0.01), FINGER_CLOSED, "lift it to the socket's height", g))
    R.append(_w(1.2, (x_in, mouth_y - 0.018 - TIP_Y, z_in), FINGER_CLOSED, "line it up with the INPUT socket", g))
    R.append(_w(0.6, (x_in, mouth_y - 0.018 - TIP_Y, z_in), FINGER_CLOSED, "line it up with the INPUT socket", g))
    R.append(_w(1.6, (x_in, mouth_y + DEPTH + PAST - TIP_Y, z_in), FINGER_CLOSED, "push it home", g))
    R.append(_w(0.6, (x_in, mouth_y + DEPTH + PAST - TIP_Y, z_in), FINGER_CLOSED, "push it home", g))
    home_p = np.array([x_in, mouth_y + DEPTH - TIP_Y, z_in])
    R.append(_w(0.4, home_p, FINGER_CLOSED, "ease off", g))
    R.append(_w(0.5, home_p, PLUG_FINGER - 0.006, "let go of the plug", g))
    R.append(_w(0.8, home_p + [0.0, 0.0, 0.05], PLUG_FINGER - 0.006, "rise clear of the plug", g))

    # it turns the VOLUME knob back to where it was
    kp, kr = _knob_grip(ZERO_DEG)
    R.append(_w(1.6, kp + up, KNOB_FINGER, "over the VOLUME knob", kr))
    R.append(_w(1.0, kp, KNOB_FINGER, "down round the knob's pointer", kr))
    R.append(_w(0.6, kp, FINGER_CLOSED, "take the knob", kr))
    _turn(R, ZERO_DEG, START["volume"], "turn the volume back to where it was")
    p, r = _knob_grip(START["volume"])
    R.append(_w(0.4, p, FINGER_CLOSED, "turn the volume back to where it was", r))
    R.append(_w(0.7, p, KNOB_LET_GO, "let go of the knob", r))
    R.append(_w(1.0, p + up, KNOB_LET_GO, "rise clear of the knob", r))
    high = np.array([p[0], p[1], PAD_TOP + 0.175])
    R.append(_w(0.8, high, KNOB_LET_GO, "rise well clear of the amp", r))
    away = np.array([p[0] + 0.03, -0.150, PAD_TOP + 0.175])
    R.append(_w(1.0, away, KNOB_LET_GO, "move off to the side", r))
    t_done = sum(w.duration for w in R)
    R += _joint_path("right", _good_ik("right", away, r), ready, "back to where it started", KNOB_LET_GO)

    # the left lets go of the handle and goes home
    L.append(_w(max(0.2, t_done - sum(w.duration for w in L)), hp, FINGER_CLOSED, "hold the amp by its handle",
                hr))
    L.append(_w(0.5, hp, HANDLE_FINGER, "let go of the handle", hr))
    L.append(_w(0.7, hp + upl, HANDLE_FINGER, "rise clear of the handle", hr))
    L += _joint_path("left", _good_ik("left", hp + upl, hr), ready, "back to where it started", HANDLE_FINGER)
    t_end = max(sum(w.duration for w in L), sum(w.duration for w in R)) + 3.0
    for side in (L, R):
        side.append(_w(t_end - sum(w.duration for w in side), side[-1].pos, side[-1].finger,
                       "hold still: the trial ends once the full score has held 1 s with both arms home", side[-1].yaw))
    return {"left": L, "right": R}
