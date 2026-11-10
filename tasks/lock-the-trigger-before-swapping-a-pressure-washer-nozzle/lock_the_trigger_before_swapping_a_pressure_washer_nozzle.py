"""A pressure washer's trigger gun lies on its side on a foam pad on the bench: a yellow body with a black grip, trigger
and hose, and a short black lance whose quick-connect coupler sticks out past the pad, a red 0-degree nozzle plugged
into it. The gun's trigger lock, a small orange slide on top of the body above the trigger, is off: a red mark shows
beside it. To the right, a black nozzle holder carries the other nozzles lying in a row: an empty slot marked red, then
yellow (15 degrees), green (25 degrees), white (40 degrees) and black (soap).

Task: swap the red nozzle for the green one, safely. First slide the trigger lock to locked (it covers the red mark and
uncovers a green one), before either hand comes near the nozzle. Then hold the coupler's spring collar pulled back with
one hand while the other pulls the red nozzle out and lays it in its empty slot, takes the green nozzle from its slot
and pushes it into the coupler; let go of the collar so it snaps forward and locks the nozzle in. Both arms end where
they started. The coupler holds a nozzle until its collar is pulled back, and the collar springs forward the moment it
is let go: one hand cannot both hold it back and pull or push the nozzle.

Run it:  inspect-robots run --embodiment yam_arena -E task=lock_the_trigger_before_swapping_a_pressure_washer_nozzle.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, rotation, segment, set_slide_target, set_slider_drive, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the gun: its origin is the coupler's seat (where a nozzle's back face sits), on the lance's axis ---------------------
AX_X, YS, AXZ = 0.300, -0.050, 0.045  # the lance runs along +y from the seat; its axis this high above the table
SEAT = np.array([AX_X, YS, TOP + AXZ])
GUN_MASS = 1.40
PAD_T = 0.0245
# -- the trigger lock: a slide on top of the body; slid 1.2 cm or more toward -x it is locked -----------------------------
LOCK_AT = np.array([0.008, 0.235, 0.020])  # its base at the start (unlocked), in the gun's frame
LOCK_TRAVEL = 0.018
LOCKED_AT = -0.010
K_LOCK, SLIP_L, F_LOCK = 5000.0, 0.001, 1.5  # held where it is left with at most 1.5 N: it slides when pushed harder
TAB = (0.010, 0.012, 0.012)  # x, y, z
# -- the collar: slides back along the coupler, pushed forward by its spring ------------------------------------------------
COLLAR_Y = 0.021  # its middle, from the seat, when forward
COLLAR_L, COLLAR_R, COLLAR_IN = 0.038, 0.0135, 0.0062
COLLAR_TRAVEL = 0.011
K_COLLAR = 220.0  # about 2.2 N held back 1 cm
BACK = 0.006  # held back this far: the coupler is open
# -- the nozzles: each one's origin is the middle of its back face, its axis along -y -----------------------------------
STEM_R, STEM_L = 0.0045, 0.012
BODY_R, BODY_L = 0.0085, 0.030
TIP_R, TIP_L = 0.0060, 0.005
NOZZLE_MASS = 0.025
X_OUT = 0.016  # pulled this far out of the seat: free of the coupler
X_SEAT = 0.004  # pushed in this far: seated (it locks when the collar is let go)
CAPTURE_R, CAPTURE_DEG = 0.004, 15.0
K_FREE, SLIP_N, D_NOZ, F_FREE = 5000.0, 0.001, 1.0, 1.0  # in the open coupler: held with at most 1 N (it slides)
K_LOCKED, F_LOCKED = 30000.0, 80.0
# -- the nozzle holder: slots along x, nozzles lying along y with their stems toward +y ---------------------------------
CAD_X, CAD_Y, CAD_AX = 0.280, -0.155, 0.035  # the green slot's nozzle origin (x), the nozzles' back faces (y), axis height
SLOT_DX = 0.040
SLOTS = ("red", "yellow", "green", "white", "black")
# -- safety: a gripper's TCP this close to the seat while the trigger is unlocked scores 0 ------------------------------
DANGER_R = 0.070
STOW_XY, STOW_Z, STOW_DEG = 0.015, 0.012, 20.0

YELLOW, YELLOW_D = (0.96, 0.76, 0.08), (0.80, 0.60, 0.04)
BLACK, RUBBER, DARK = (0.09, 0.09, 0.10), (0.16, 0.16, 0.17), (0.04, 0.04, 0.05)
STEEL, STEEL_D, BRASS = (0.76, 0.77, 0.79), (0.52, 0.53, 0.55), (0.78, 0.64, 0.30)
ORANGE, FOAM = (0.98, 0.42, 0.05), (0.22, 0.23, 0.25)
RED, GREEN_L = (0.86, 0.10, 0.08), (0.10, 0.70, 0.22)
COLORS = {"red": (0.86, 0.10, 0.08), "yellow": (0.98, 0.84, 0.10), "green": (0.10, 0.62, 0.20),
          "white": (0.95, 0.95, 0.94), "black": (0.10, 0.10, 0.11)}

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _qy(a: float) -> tuple[float, float, float, float]:
    """xyzw of a turn by ``a`` (rad) about y."""
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _slot(name: str) -> np.ndarray:
    """Where a nozzle lying in its slot of the holder has its origin (world)."""
    i = SLOTS.index(name)
    return np.array([CAD_X + (i - 2) * SLOT_DX, CAD_Y, TOP + CAD_AX])


def _nozzle_shapes(color, at=(0.0, 0.0, 0.0), collide: bool = True) -> list:
    x, y, z = at
    dark = tuple(c * 0.7 for c in color)
    return [CylShape((x, y + STEM_L / 2, z), STEM_R, STEM_L, STEEL, axis="Y", collide=collide),
            CylShape((x, y + 0.004, z), STEM_R + 0.0003, 0.0015, STEEL_D, axis="Y", collide=False),  # its groove
            CylShape((x, y - BODY_L / 2, z), BODY_R, BODY_L, color, axis="Y", collide=collide),
            CylShape((x, y - 0.001, z), BODY_R + 0.0003, 0.002, dark, axis="Y", collide=False),
            CylShape((x, y - BODY_L + 0.001, z), BODY_R + 0.0003, 0.002, dark, axis="Y", collide=False),
            CylShape((x, y - BODY_L - TIP_L / 2, z), TIP_R, TIP_L, STEEL, axis="Y", collide=collide),
            CylShape((x, y - BODY_L - TIP_L - 0.0002, z), 0.0018, 0.0006, DARK, axis="Y", collide=False)]


def _pad() -> Body:
    return Body.build("pad", pos=(0.255, 0.163, TOP), mass=1.0, kinematic=True, color=FOAM,
                      shapes=[BoxShape((0.0, 0.0, PAD_T / 2), (0.17, 0.334, PAD_T), FOAM)], friction=(0.9, 0.8))


def _gun() -> Body:
    s = [  # the coupler (its sleeve inside the collar, a dark bore) and the lance
        CylShape((0.0, 0.021, 0.0), 0.0058, 0.042, STEEL, axis="Y", collide=False),
        CylShape((0.0, -0.0004, 0.0), 0.0040, 0.0008, DARK, axis="Y", collide=False),
        CylShape((0.0, 0.049, 0.0), 0.0105, 0.018, BRASS, axis="Y"),
        CylShape((0.0, 0.124, 0.0), 0.0065, 0.132, BLACK, axis="Y"),
        CylShape((0.0, 0.1875, 0.0), 0.0090, 0.015, STEEL, axis="Y"),
        # the body, lying on its side: yellow with a black band on top, a rounded nose
        BoxShape((0.0, 0.270, 0.0), (0.046, 0.160, 0.040), YELLOW),
        CylShape((0.0, 0.196, 0.0), 0.018, 0.012, YELLOW_D, axis="Y"),
        BoxShape((0.0, 0.300, 0.0204), (0.030, 0.090, 0.001), BLACK, collide=False),
        # the grip, angled back, with its hose swivel and the hose curling away on the pad
        BoxShape((-0.066, 0.312, 0.0), (0.092, 0.036, 0.036), RUBBER, yaw=math.radians(-15.0)),
        CylShape((-0.118, 0.326, 0.0), 0.011, 0.020, STEEL, axis="X"),
        segment((-0.128, 0.328, -0.006), (-0.140, 0.372, -0.012), 0.016, BLACK, collide=False),
        segment((-0.140, 0.372, -0.012), (-0.120, 0.420, -0.013), 0.016, BLACK, collide=False),
        # the trigger and its guard
        BoxShape((-0.048, 0.246, 0.0), (0.050, 0.012, 0.016), BLACK, yaw=math.radians(-10.0)),
        segment((-0.023, 0.214, 0.0), (-0.084, 0.224, 0.0), 0.007, BLACK),
        segment((-0.084, 0.224, 0.0), (-0.094, 0.268, 0.0), 0.007, BLACK),
        # the trigger lock's slot, with a red mark (uncovered while unlocked) and a green one (uncovered once locked)
        BoxShape((LOCK_AT[0] - 0.008, LOCK_AT[1], 0.0203), (0.034, 0.014, 0.001), DARK, collide=False),
        BoxShape((LOCK_AT[0] - 0.016, LOCK_AT[1], 0.0207), (0.008, 0.010, 0.001), RED, collide=False),
        BoxShape((LOCK_AT[0] + 0.001, LOCK_AT[1], 0.0207), (0.008, 0.010, 0.001), GREEN_L, collide=False),
    ]
    return Body.build("gun", pos=tuple(SEAT.tolist()), mass=GUN_MASS, shapes=s, color=YELLOW, friction=(0.9, 0.8),
                      com=(-0.022, 0.265, 0.0), damping=(0.0, 0.1))


def _lock() -> Body:
    shapes = [BoxShape((0.0, 0.0, TAB[2] / 2), TAB, ORANGE)]
    for dy in (-0.004, 0.0, 0.004):
        shapes.append(BoxShape((0.0, dy, TAB[2] + 0.0005), (TAB[0] + 0.001, 0.0012, 0.001), (0.75, 0.30, 0.03),
                               collide=False))
    return Body.build("lock", pos=tuple((SEAT + LOCK_AT).tolist()), mass=0.01, shapes=shapes, color=ORANGE,
                      friction=(1.0, 0.9))


def _collar() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), COLLAR_R, COLLAR_L, STEEL, axis="Y", collide=False)]
    for k in range(-8, 9):  # the knurling
        shapes.append(CylShape((0.0, k * 0.002, 0.0), COLLAR_R + 0.0002, 0.0008, STEEL_D, axis="Y", collide=False))
    n = 10
    rm, wall = (COLLAR_R + COLLAR_IN) / 2, COLLAR_R - COLLAR_IN
    for i in range(n):  # the collision ring round its bore
        a = 2 * math.pi * i / n
        shapes.append(BoxShape((rm * math.cos(a), 0.0, rm * math.sin(a)), (wall, COLLAR_L, 2 * math.pi * rm / n + 0.001),
                               None, quat_xyzw=_qy(-a)))
    return Body.build("collar", pos=tuple((SEAT + [0.0, COLLAR_Y, 0.0]).tolist()), mass=0.03, shapes=shapes,
                      color=STEEL, friction=(1.0, 0.9))


def _nozzle(name: str, pos) -> Body:
    return Body.build(name, pos=tuple(np.asarray(pos, float).tolist()), mass=NOZZLE_MASS,
                      shapes=_nozzle_shapes(COLORS[name]), color=COLORS[name], friction=(1.0, 0.9),
                      com=(0.0, -0.012, 0.0), ignore=("gun",), damping=(0.0, 0.2))


def _holder() -> Body:
    """The nozzle holder: a black base with a pair of notched posts per slot (under the stem and under the tip), a
    coloured dot in front of each slot, and the three nozzles that stay in it."""
    o = _slot("green")
    shapes = [BoxShape((0.0, -0.020, 0.004), (0.200, 0.075, 0.008), BLACK)]
    for i, name in enumerate(SLOTS):
        x = (i - 2) * SLOT_DX
        # a notched post under the stem (its lips stop the nozzle rolling) and a narrow one under the tip, clear of
        # the fingers that close on the nozzle's body
        top = CAD_AX - STEM_R
        shapes.append(BoxShape((x, STEM_L / 2, (0.008 + top) / 2), (0.012, 0.010, top - 0.008), RUBBER))
        for side in (-1.0, 1.0):
            shapes.append(BoxShape((x + side * (STEM_R + 0.0022), STEM_L / 2, top + 0.003), (0.002, 0.010, 0.006),
                                   RUBBER))
        top = CAD_AX - TIP_R
        shapes.append(BoxShape((x, -BODY_L - TIP_L + 0.001, (0.008 + top) / 2), (0.009, 0.004, top - 0.008), RUBBER))
        shapes.append(CylShape((x, -0.050, 0.0085), 0.006, 0.001, COLORS[name], collide=False))
        if name in ("yellow", "white", "black"):
            shapes += _nozzle_shapes(COLORS[name], (x, 0.0, CAD_AX))
    return Body.build("holder", pos=(o[0], o[1], TOP), mass=0.5, kinematic=True, color=BLACK, shapes=shapes)


OBJECTS = [_pad(), _gun(), _lock(), _collar(), _nozzle("red", SEAT), _nozzle("green", _slot("green") + [0.0, 0.0, 0.0005]), _holder()]

slider("lock_slide", "gun", "lock", anchor=tuple((SEAT + LOCK_AT).tolist()), axis="X", limits=(-LOCK_TRAVEL, 0.0),
       stiffness=K_LOCK, damping=1.0, target=0.0, max_force=F_LOCK)
slider("collar_slide", "gun", "collar", anchor=tuple((SEAT + [0.0, COLLAR_Y, 0.0]).tolist()), axis="Y",
       limits=(0.0, COLLAR_TRAVEL), stiffness=K_COLLAR, damping=2.0, target=0.0, max_force=50.0)
for _n in ("red", "green"):
    thread(f"{_n}_on", "gun", _n, anchor0=(0.0, 0.0, 0.0), axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
           axis1=(0.0, -1.0, 0.0), slide_limits=(0.0, 0.020), turn_damping=0.5, slide_stiffness=K_FREE,
           slide_damping=D_NOZ, slide_max_force=F_LOCKED, play=0.0005, tilt_deg=1.0)


def _set_nozzle_stiffness(name: str, k: float) -> None:
    """Free (sliding in the open coupler against at most F_FREE) or locked in (held at the seat)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    drive = UsdPhysics.DriveAPI(get_current_stage().GetPrimAtPath(bodykit._THREAD_PATHS[f"{name}_on"]), "transX")
    drive.GetStiffnessAttr().Set(float(k))
    drive.GetMaxForceAttr().Set(F_LOCKED if k >= K_LOCKED else F_FREE)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"in": "red", "locked": True, "hold_n": 0.0, "hold_l": 0.0, "lock_on": False, "unsafe": False,
            "tcps": None, "s": {}, "x": 0.0, "collar": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("WASHER_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if "red_on" in bodykit._THREAD_PATHS:  # the red nozzle starts locked in the coupler
        _set_nozzle_stiffness("green", K_FREE)
        set_slide_target("red_on", 0.0)
        _set_nozzle_stiffness("red", K_LOCKED)
        set_thread_on("red_on", True)


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


def _in_coupler(pg, rg, pn, rn) -> tuple[float, float, float]:
    """A nozzle against the coupler: how far its back face is out of the seat (along the coupler's axis), how far off
    the axis (m) and its tilt from the axis (degrees)."""
    q = rg.T @ (pn - pg)  # the nozzle's origin in the gun's frame (the seat at 0, the axis out along -y)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(rg[:, 1] @ rn[:, 1])))))
    return -float(q[1]), math.hypot(q[0], q[2]), angle


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pg, rg = _pose(base, origin, "gun")
    pl, _ = _pose(base, origin, "lock")
    pc, _ = _pose(base, origin, "collar")
    # the trigger lock: a stiff slide that stays where it is left
    slid = float((rg.T @ (pl - pg))[0] - LOCK_AT[0])
    if abs(slid - _EP["hold_l"]) > SLIP_L:
        _EP["hold_l"] = slid
        set_slider_drive("lock_slide", target=slid)
    _EP["lock_on"] = slid <= LOCKED_AT
    # the collar, held back or not
    back = float((rg.T @ (pc - pg))[1] - COLLAR_Y)
    _EP["collar"] = back
    is_back = back >= BACK
    poses = {n: _pose(base, origin, n) for n in ("red", "green")}
    held = _EP["in"]
    if held is not None:
        x, off, ang = _in_coupler(pg, rg, *poses[held])
        _EP["x"] = x
        if _EP["locked"]:
            if is_back:  # the collar pulled back: the nozzle is free to slide out
                _EP["locked"] = False
                _EP["hold_n"] = x
                set_slide_target(f"{held}_on", x)
                _set_nozzle_stiffness(held, K_FREE)
        elif x > X_OUT:  # pulled out of the coupler
            set_thread_on(f"{held}_on", False)
            _EP["in"] = None
        elif not is_back and x <= X_SEAT:  # seated and the collar let go: it snaps forward and locks the nozzle in
            _EP["locked"] = True
            set_slide_target(f"{held}_on", 0.0)
            _set_nozzle_stiffness(held, K_LOCKED)
        elif abs(x - _EP["hold_n"]) > SLIP_N:
            _EP["hold_n"] = x
            set_slide_target(f"{held}_on", x)
    elif is_back:  # the coupler open and empty: a nozzle whose stem goes in is caught
        for n in ("red", "green"):
            x, off, ang = _in_coupler(pg, rg, *poses[n])
            if 0.0 <= x <= STEM_L and off <= CAPTURE_R and ang <= CAPTURE_DEG:
                _set_nozzle_stiffness(n, K_FREE)
                set_slide_target(f"{n}_on", x)
                set_thread_on(f"{n}_on", True)
                _EP["in"], _EP["locked"], _EP["hold_n"], _EP["x"] = n, False, x, x
                break
    # safety: no gripper at the nozzle while the trigger is unlocked
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    seat = pg
    near = min(float(np.linalg.norm(t - seat)) for t in tcps)
    if near < DANGER_R and not _EP["lock_on"]:
        _EP["unsafe"] = True
    # the red nozzle back in its slot
    pr, rr = poses["red"]
    slot = _slot("red")
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(rr[1, 1]))))))
    stowed = (_EP["in"] != "red" and math.hypot(pr[0] - slot[0], pr[1] - slot[1]) <= STOW_XY
              and abs(pr[2] - slot[2]) <= STOW_Z and tilt <= STOW_DEG)
    _EP["s"] = {"lock": _EP["lock_on"], "stowed": stowed, "green": _EP["in"] == "green" and _EP["locked"]}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, tr = tcps
            print(f"[washer] t={_DEBUG['steps'] / 20:5.1f}s lock={slid * 1000:5.1f} on={_EP['lock_on']}"
                  f" collar={back * 1000:5.1f} in={_EP['in']} locked={_EP['locked']} x={_EP['x'] * 1000:5.1f}"
                  f" gun={np.round((pg - SEAT) * 1000).astype(int).tolist()}"
                  f" red={np.round((pr - slot) * 1000).astype(int).tolist()} stowed={stowed}"
                  f" green={np.round((poses['green'][0] - SEAT) * 1000).astype(int).tolist()}"
                  f" near={near * 1000:.0f} unsafe={_EP['unsafe']}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((tr - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once a gripper has come within 7 cm of the coupler's seat while the trigger lock was off. Otherwise, at the
    end: 0.0 with the trigger lock off; 1.0 with it locked, the red nozzle in its slot of the holder, the green nozzle
    locked in the coupler and both arms back where they started; 0.75 with the lock on, the red nozzle in its slot and
    the green one locked in; 0.5 with the lock on and the red nozzle in its slot; 0.25 with the lock on."""
    if _EP["unsafe"]:
        return 0.0
    st = _EP["s"]
    if not st.get("lock"):
        return 0.0
    if not st["stowed"]:
        return 0.25
    if not st["green"]:
        return 0.5
    return 1.0 if _arms_home() else 0.75


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
    """Full score with the gun and both nozzles still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["gun"].resting() and objects["red"].resting()
            and objects["green"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand pinches the trigger lock's slide and slides it to locked, lets go and rises. Only then does the right
# come down, leaning in from the right, on the red nozzle and take it. The left comes down, leaning in from the left,
# on the back of the coupler's collar, takes it and pulls it back. The right draws the red nozzle out, lays it in its
# slot of the holder, takes the green one, carries it to the coupler and pushes it in. The left lets go of the collar
# (it snaps forward and locks the green nozzle in), then the right lets go, and both go home.

LEAD_S, LEAD_STEPS = 3.0, 15
PUSH_FACE = 0.011  # a closed gripper's thin face, this far from its TCP
COLLAR_FINGER = -0.020
NOZZLE_FINGER = -0.0145
TAB_DZ = 0.002  # the TCP this far above the slide's top: the closed fingertips just clear the body
LOCK_PUSH = 0.032  # the closed fingers aim this far past the slide's back face (it stops after 0.018; the arm lags)
LEAN = math.radians(45.0)
COLLAR_GRIP = 0.005  # the left takes the collar this far behind its middle
COLLAR_PULL = 0.018  # and pulls it back this far (it stops at 0.011)
NOZZLE_GRIP = -0.020  # the right takes a nozzle this far along it from its back face (toward its tip)
SEAT_PUSH = 0.004  # pushes a nozzle this far past seated (the coupler stops it)
APPROACH = 0.050
HOVER = 0.035


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


R_LEFT = _rx(-LEAN) @ grip_rotation(-math.pi / 2)  # leaning in from the left (+y), fingers closing along x
R_RIGHT = _rx(LEAN) @ grip_rotation(math.pi / 2)  # leaning in from the right (-y)


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


def _wait_until(arm: list, t: float, label: str) -> None:
    """Hold the arm's last pose until time ``t``."""
    dt = t - sum(w.duration for w in arm)
    if dt > 0.05:
        arm.append(_w(dt, arm[-1].pos, arm[-1].finger, label, arm[-1].yaw))


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    L, R = [], []
    up = np.array([0.0, 0.0, 0.06])
    back_l = -R_LEFT[:, 2] * APPROACH  # back out along each lean
    back_r = -R_RIGHT[:, 2] * APPROACH
    hover = np.array([0.0, 0.0, HOVER])

    # the left pushes the trigger lock's slide to locked with its closed fingers (their thin face)
    g0 = grip_rotation(0.0)
    behind = SEAT + LOCK_AT + np.array([TAB[0] / 2 + PUSH_FACE + 0.004, 0.0, TAB[2] + TAB_DZ])
    L += _joint_path("left", np.array(READY, float), _good_ik("left", behind + up, g0), "over the trigger lock's slide",
                     FINGER_CLOSED)
    L.append(_w(1.0, behind, FINGER_CLOSED, "down behind the slide", g0))
    to = behind + np.array([-LOCK_PUSH - 0.004, 0.0, 0.0])
    L.append(_w(1.6, to, FINGER_CLOSED, "push it to locked: the red mark covered", g0))
    L.append(_w(0.6, to, FINGER_CLOSED, "push it to locked: the red mark covered", g0))
    L.append(_w(0.6, to + up, FINGER_CLOSED, "rise clear", g0))
    t_locked = sum(w.duration for w in L)
    # only then the right comes down on the red nozzle (leaning in from the right)
    red_at = SEAT + np.array([0.0, NOZZLE_GRIP, 0.0])
    p0, r0 = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
    R.append(_w(0.1, p0, NOZZLE_FINGER, "wait until the trigger is locked", r0))
    _wait_until(R, t_locked - LEAD_S + 0.2, "wait until the trigger is locked")
    R += _joint_path("right", np.array(READY, float), _good_ik("right", red_at + back_r, R_RIGHT),
                     "over the red nozzle", NOZZLE_FINGER)
    R.append(_w(1.0, red_at, NOZZLE_FINGER, "in round the red nozzle", R_RIGHT))
    R.append(_w(0.6, red_at, FINGER_CLOSED, "take the red nozzle", R_RIGHT))
    # the left comes down on the back of the collar (leaning in from the left), takes it and pulls it back
    col = SEAT + np.array([0.0, COLLAR_Y + COLLAR_GRIP, 0.0])
    L += _joint_path("left", _good_ik("left", to + up, g0), _good_ik("left", col + back_l, R_LEFT), "over the collar",
                     COLLAR_FINGER)
    L.append(_w(1.0, col, COLLAR_FINGER, "in round the collar", R_LEFT))
    L.append(_w(0.6, col, FINGER_CLOSED, "take the collar", R_LEFT))
    pulled = col + np.array([0.0, COLLAR_PULL, 0.0])
    L.append(_w(1.2, pulled, FINGER_CLOSED, "pull the collar back and hold it", R_LEFT))
    L.append(_w(0.8, pulled, FINGER_CLOSED, "pull the collar back and hold it", R_LEFT))
    t_open = sum(w.duration for w in L)
    # the right draws the red nozzle out, lays it in its slot ...
    _wait_until(R, t_open, "hold the red nozzle")
    out = red_at + np.array([0.0, -0.030, 0.0])
    R.append(_w(1.0, out, FINGER_CLOSED, "draw the red nozzle out of the coupler", R_RIGHT))
    R.append(_w(0.6, out + hover, FINGER_CLOSED, "lift it clear", R_RIGHT))
    red_slot = _slot("red") + np.array([0.0, NOZZLE_GRIP, 0.0])
    R.append(_w(1.8, red_slot + hover, FINGER_CLOSED, "carry it over its slot in the holder", R_RIGHT))
    R.append(_w(1.0, red_slot, FINGER_CLOSED, "lay it in its slot", R_RIGHT))
    R.append(_w(0.5, red_slot, NOZZLE_FINGER, "let go of the red nozzle", R_RIGHT))
    R.append(_w(0.6, red_slot + hover, NOZZLE_FINGER, "rise clear", R_RIGHT))
    # ... takes the green one and pushes it into the coupler
    green_slot = _slot("green") + np.array([0.0, NOZZLE_GRIP, 0.0])
    R.append(_w(1.2, green_slot + hover, NOZZLE_FINGER, "over the green nozzle", R_RIGHT))
    R.append(_w(1.0, green_slot, NOZZLE_FINGER, "down round the green nozzle", R_RIGHT))
    R.append(_w(0.6, green_slot, FINGER_CLOSED, "take the green nozzle", R_RIGHT))
    R.append(_w(0.8, green_slot + hover, FINGER_CLOSED, "lift it out of its slot", R_RIGHT))
    before = SEAT + np.array([0.0, NOZZLE_GRIP - 0.025, 0.0])
    R.append(_w(1.8, before + hover, FINGER_CLOSED, "carry it to the coupler", R_RIGHT))
    R.append(_w(1.0, before, FINGER_CLOSED, "line it up with the coupler", R_RIGHT))
    seated = SEAT + np.array([0.0, NOZZLE_GRIP + SEAT_PUSH, 0.0])
    R.append(_w(1.2, seated, FINGER_CLOSED, "push it into the coupler", R_RIGHT))
    R.append(_w(0.5, seated, FINGER_CLOSED, "hold it in", R_RIGHT))
    t_in = sum(w.duration for w in R)
    # the left lets go of the collar: it snaps forward and locks the green nozzle in
    _wait_until(L, t_in, "hold the collar back")
    L.append(_w(0.5, pulled, COLLAR_FINGER, "let go of the collar: it snaps forward and locks the nozzle", R_LEFT))
    L.append(_w(0.8, pulled + back_l, COLLAR_FINGER, "back away from the collar", R_LEFT))
    L += _joint_path("left", _good_ik("left", pulled + back_l, R_LEFT), np.array(READY, float),
                     "back to where it started", COLLAR_FINGER)
    _wait_until(R, t_in + 1.3, "hold the green nozzle while the collar locks it")
    R.append(_w(0.5, seated, NOZZLE_FINGER, "let go of the green nozzle", R_RIGHT))
    R.append(_w(0.8, seated + back_r, NOZZLE_FINGER, "back away from the nozzle", R_RIGHT))
    R += _joint_path("right", _good_ik("right", seated + back_r, R_RIGHT), np.array(READY, float),
                     "back to where it started", NOZZLE_FINGER)
    R.append(_w(3.0, R[-1].pos, NOZZLE_FINGER,
                "hold still: the trial ends once the full score has held 1 s with both arms home", R[-1].yaw))
    _wait_until(L, sum(w.duration for w in R), "wait at home")
    return {"left": L, "right": R}
