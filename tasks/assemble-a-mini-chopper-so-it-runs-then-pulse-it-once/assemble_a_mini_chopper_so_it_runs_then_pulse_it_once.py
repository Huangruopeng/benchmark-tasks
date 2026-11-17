"""On the kitchen table stands a Black+Decker HC150B one-touch mini chopper (1.5 cups), at its real size: its black
motor base (a low base under the bowl, with a switch tower on its right), the clear bowl already set on the base, and
beside it its two loose parts: the blade (a black hub with two stainless blades, standing on its hub, left) and the
clear lid (lying on the table, right, its long tab pointing right).

Task: assemble the chopper so it can run, then pulse it once, and take both arms back to where they started. As the
maker's manual says: the blade goes into the bowl over the centre shaft and is turned until it drops to the bottom
(its hub is keyed to the shaft's drive); the lid is fitted on the bowl with its seal pressed home, its tab lying over
the tower; pressing the textured pad on the tab then runs the motor (a pulse) while it is held, and only when the lid
is fitted (the cover interlock). The blade may be touched only by its hub.

Run it:  inspect-robots run --embodiment yam_arena -E task=assemble_a_mini_chopper_so_it_runs_then_pulse_it_once.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_hinge_drive, set_slide_target, set_thread_on, slider, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the chopper: motor base, switch tower and bowl (the bowl sits in the base's slots: one body). Origin: the bowl's
#    axis on the table. The tower stands on the bowl's left (+y): the unit stands with its tower to the left. ------
AXIS_XY = (0.250, -0.050)
CHOPPER_MASS = 0.42  # base 0.34 kg + bowl 0.08 kg (the whole HC150B, with blade and lid: 1.2 lb = 0.54 kg)
PLINTH_R, PLINTH_H = 0.053, 0.024  # the round base under the bowl
R_IN, WALL_T, RIM_Z, FLOOR_Z = 0.047, 0.003, 0.114, 0.030  # the bowl: inside radius, wall, rim height, floor top
POST_R, POST_TOP = 0.006, 0.064  # the bowl's centre tube over the motor shaft (with a rounded top)
TOWER_R, TOWER_W, TOWER_D, TOWER_TOP = 0.0805, 0.054, 0.080, 0.1125  # its middle from the axis; radial, x; its top
SLOT_T, SLOT_RW, SLOT_DEPTH = 0.024, 0.014, 0.010  # the hole in the tower's top the lid's tab pin goes into
# -- the drive: the motor's cross pin through the foot of the centre tube (its own body, on the motor's axle) -------
KEY_L, KEY_T, KEY_Z = 0.020, 0.003, (0.002, 0.005)  # its length (along x), width, and height above the floor
DRIVE_MASS = 0.02
DRIVE_K, DRIVE_DAMP = 0.01, 1.0e-5  # N m/deg, N m per deg/s: the stopped motor's gear holds the pin
SPIN_W = 25.0  # rad/s: while the motor runs, its drive (and the blade keyed on it) turns this fast
# -- the blade: origin at its hub's bottom, on its axis ---------------------------------------------------------------
HUB_RO, HUB_T, HUB_H = 0.015, 0.002, 0.044  # the hub: outside radius, wall, height
SLEEVE_RI, SLEEVE_T, SLEEVE_TOP = 0.0085, 0.002, 0.030  # the bore that slides over the centre tube
NOTCH_H, NOTCH_DEG = 0.007, 29.0  # two notches at the bore's foot (along x) that fit over the drive's cross pin
FIN_L, FIN_W, FIN_H = 0.016, 0.010, 0.012  # the knob on top of the hub
BLADE_R1, BLADE_W, BLADE_T = 0.038, 0.012, 0.002
BLADE_Z = (0.006, 0.018)  # the two blades' heights on the hub (bi-level: the lower along +x, the upper along -x)
BLADE_MASS = 0.04
BLADE_AT, BLADE_YAW0 = (0.320, -0.170), 60.0  # standing on its hub on the table, its keyway 60 degrees off the key
# -- the lid: origin at the middle of its plate's underside; its tab points +y ---------------------------------------
LID_AT = (0.180, -0.170)
LID_MASS = 0.06
PLATE_R, PLATE_T = 0.053, 0.003
SKIRT_RO, SKIRT_T, SKIRT_H = 0.0455, 0.002, 0.008  # the seal ring that fits inside the bowl's mouth (lower half
SKIRT_LEAD = 0.0055  # narrower by this much: its lead-in)
DOME_R, DOME_H = 0.030, 0.012  # the raised middle (with the oil holes round it)
TAB_W, TAB_END = 0.040, 0.105  # the tab: width, how far it reaches from the axis (a finger loop beyond)
PIN_R, PIN_L = 0.0035, 0.006  # the pin under the tab (into the tower's hole)
# -- the pad: the textured "pulse" pad on the tab (its own body on a sprung slide) ----------------------------------
PAD_R, PAD_H, PAD_PROUD = 0.011, 0.004, 0.005  # its radius, its thickness; its top this far above the tab's top
PAD_MASS = 0.008
PAD_TRAVEL = 0.003
SWITCH_AT, SWITCH_OFF = 0.0020, 0.0010  # its switch closes 2 mm down, opens again once it is back above 1 mm
PAD_K, PAD_PRELOAD, PAD_DAMP = 1500.0, 0.004, 3.0  # N/m; 6 N at rest, 9 N at 2 mm, 10.5 N at its stop
# -- the lid's seal --------------------------------------------------------------------------------------------------
SEAL_K, SEAL_SLIP, SEAL_TOP, SEAL_OFF = 3000.0, 0.0033, 0.012, 0.009  # N/m; 10 N to slide it; its travel; pulled off
CATCH_R, CATCH_DZ, CATCH_TILT = 0.006, (-0.002, 0.007), 8.0  # the seal caught in the bowl's mouth
SEATED_DZ, SEATED_TILT = 0.0015, 3.0  # the lid fitted: its seal home (checked live)
PIN_TOL = math.degrees(math.asin((SLOT_T / 2 - PIN_R) / TOWER_R))  # the pin fits the hole within this turn (6.1)
FIT_YAW = PIN_TOL + 1.5  # the fitted check allows this much (the hole itself stops the pin at 6.1 degrees)
# -- rules -------------------------------------------------------------------------------------------------------------
SEAT_R, SEAT_DZ, SEAT_TILT = 0.003, 0.0025, 8.0  # the blade on its drive (its hub's bottom on the floor)
SEAT_Z = FLOOR_Z
EDGE_NEAR, EDGE_R0 = 0.008, 0.020  # a fingertip within 8 mm of a blade's edges (its cutting part: beyond 2 cm)
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

BLACK, TRIM, DARK, GREY = (0.07, 0.07, 0.08), (0.30, 0.31, 0.33), (0.03, 0.03, 0.035), (0.42, 0.43, 0.46)
CLEAR, CLEAR_D, STEEL, HUB_C = (0.82, 0.88, 0.93), (0.66, 0.74, 0.82), (0.80, 0.81, 0.84), (0.13, 0.13, 0.14)
SILVER, RUBBER = (0.72, 0.73, 0.75), (0.10, 0.10, 0.11)

AXIS = np.array([AXIS_XY[0], AXIS_XY[1], TOP])  # the chopper's origin at the start


def _yawq(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _polar(r: float, deg: float) -> tuple[float, float]:
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def _ring(r: float, z0: float, z1: float, t: float, color, n: int = 24, collide: bool = True, at=(0.0, 0.0)) -> list:
    """A round wall of n flat panels, mid-radius r, from z0 to z1, t thick (color None: clear, collision only)."""
    out = []
    w = 2 * (r + t / 2) * math.tan(math.pi / n) + 0.001
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        out.append(BoxShape((at[0] + r * math.cos(a), at[1] + r * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0), color,
                            yaw=a, collide=collide))
    return out


def _box(x0, x1, y0, y1, z0, z1, color, collide=True) -> BoxShape:
    return BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color, collide=collide)


def _chopper() -> Body:
    """The motor base (a round base under the bowl and a switch tower beside it) with the clear bowl set on it: the
    bowl is drawn as its rim, foot and seams (its clear wall is a collision-only shell), with the grey centre tube."""
    s = [CylShape((0.0, 0.0, PLINTH_H / 2), PLINTH_R, PLINTH_H, BLACK),
         CylShape((0.0, 0.0, PLINTH_H - 0.002), PLINTH_R + 0.0004, 0.004, TRIM, collide=False)]  # its grey band
    # the tower: a block with a hole in its top for the lid's tab pin
    ty0, ty1 = TOWER_R - TOWER_W / 2, TOWER_R + TOWER_W / 2
    tx = TOWER_D / 2
    s.append(_box(-tx, tx, ty0, ty1, 0.0, TOWER_TOP - SLOT_DEPTH, BLACK))
    sy0, sy1 = TOWER_R - SLOT_RW / 2, TOWER_R + SLOT_RW / 2
    zt0 = TOWER_TOP - SLOT_DEPTH
    s += [_box(-tx, -SLOT_T / 2, ty0, ty1, zt0, TOWER_TOP, BLACK), _box(SLOT_T / 2, tx, ty0, ty1, zt0, TOWER_TOP, BLACK),
          _box(-SLOT_T / 2, SLOT_T / 2, ty0, sy0, zt0, TOWER_TOP, BLACK),
          _box(-SLOT_T / 2, SLOT_T / 2, sy1, ty1, zt0, TOWER_TOP, BLACK)]
    s.append(_box(-SLOT_T / 2 + 0.001, SLOT_T / 2 - 0.001, sy0 + 0.001, sy1 - 0.001, zt0, zt0 + 0.0008, GREY,
                  collide=False))  # the switch plunger at the bottom of the hole
    s.append(_box(-tx - 0.0004, -tx, ty0 + 0.010, ty1 - 0.010, 0.040, 0.052, SILVER, collide=False))  # badge (front)
    for fx, fy in ((0.035, -0.030), (-0.035, -0.030), (0.030, 0.100), (-0.030, 0.100)):  # rubber feet
        s.append(CylShape((fx, fy, 0.0015), 0.005, 0.003, RUBBER, collide=False))
    # the bowl
    r_mid = R_IN + WALL_T / 2
    s.append(CylShape((0.0, 0.0, (PLINTH_H + FLOOR_Z) / 2), R_IN + WALL_T, FLOOR_Z - PLINTH_H, CLEAR))  # its floor
    s += _ring(r_mid, PLINTH_H, RIM_Z, WALL_T, None)  # the clear wall: collides, not drawn
    s += _ring(r_mid, RIM_Z - 0.003, RIM_Z, WALL_T + 0.0004, CLEAR_D, collide=False)  # its rim, drawn
    s += _ring(r_mid, PLINTH_H, PLINTH_H + 0.006, WALL_T + 0.0004, CLEAR_D, collide=False)  # its foot, drawn
    for k in range(4):  # faint seams up the wall, so it reads as a clear bowl
        x, y = _polar(R_IN + WALL_T + 0.0002, 45.0 + 90.0 * k)
        s.append(BoxShape((x, y, (FLOOR_Z + RIM_Z) / 2), (0.0006, 0.0015, RIM_Z - FLOOR_Z - 0.006), CLEAR_D,
                          yaw=math.radians(45.0 + 90.0 * k), collide=False))
    s.append(CylShape((0.0, 0.0, (FLOOR_Z + POST_TOP) / 2), POST_R, POST_TOP - FLOOR_Z, GREY))  # the centre tube
    s.append(SphereShape((0.0, 0.0, POST_TOP), POST_R, GREY))  # its rounded top
    return Body.build("chopper", pos=tuple(AXIS.tolist()), mass=CHOPPER_MASS, shapes=s, color=BLACK,
                      friction=(0.9, 0.8), com=(0.0, 0.010, 0.018), solver_iterations=32)


def _drive() -> Body:
    """The motor's drive: a stainless cross pin through the foot of the centre tube (origin: on the axis, at the bowl's
    floor), on the motor's coupling disc hidden in the base below the floor."""
    s = [BoxShape((0.0, 0.0, sum(KEY_Z) / 2), (KEY_L, KEY_T, KEY_Z[1] - KEY_Z[0]), STEEL),
         CylShape((0.0, 0.0, -0.010), 0.025, 0.004, STEEL),  # the coupling disc, inside the base
         CylShape((0.0, 0.0, 0.001), POST_R + 0.0015, 0.002, STEEL, collide=False)]  # its collar at the floor
    return Body.build("drive", pos=(AXIS[0], AXIS[1], TOP + FLOOR_Z), mass=DRIVE_MASS, shapes=s, color=STEEL,
                      friction=(0.5, 0.4), solver_iterations=32, contact_offset=0.001)


def _blade() -> Body:
    s = _ring(HUB_RO - HUB_T / 2, 0.0, HUB_H, HUB_T, HUB_C, n=12)  # the hub
    rs = SLEEVE_RI + SLEEVE_T / 2  # its bore: 16 panels, the foot of the two along x (and their neighbours) left out
    s += _ring(rs, NOTCH_H, SLEEVE_TOP, SLEEVE_T, HUB_C, n=16)
    w = 2 * (rs + SLEEVE_T / 2) * math.tan(math.pi / 16) + 0.001
    for k in range(16):
        a = 2 * math.pi * k / 16
        if abs(math.sin(a)) <= math.sin(math.radians(25.0)):
            continue  # the notches
        s.append(BoxShape((rs * math.cos(a), rs * math.sin(a), NOTCH_H / 2), (SLEEVE_T, w, NOTCH_H), HUB_C, yaw=a))
    s.append(CylShape((0.0, 0.0, HUB_H - 0.001), HUB_RO, 0.002, HUB_C))  # its top
    s.append(_box(-FIN_L / 2, FIN_L / 2, -FIN_W / 2, FIN_W / 2, HUB_H, HUB_H + FIN_H, HUB_C))  # the knob
    for z, sign in ((BLADE_Z[0], 1.0), (BLADE_Z[1], -1.0)):
        r = (HUB_RO + BLADE_R1) / 2
        s.append(BoxShape((sign * r, sign * 0.002, z + BLADE_T / 2), (BLADE_R1 - HUB_RO, BLADE_W, BLADE_T), STEEL,
                          yaw=sign * math.radians(6.0)))
        s.append(BoxShape((sign * (BLADE_R1 - 0.004), sign * 0.002, z + BLADE_T + 0.0003),
                          (0.006, BLADE_W - 0.002, 0.0006), (0.93, 0.94, 0.96), collide=False))  # the bright edge
    return Body.build("blade", pos=(BLADE_AT[0], BLADE_AT[1], TOP), mass=BLADE_MASS, shapes=s, color=HUB_C,
                      yaw=math.radians(BLADE_YAW0), friction=(0.6, 0.5), damping=(0.05, 0.3), com=(0.0, 0.0, 0.014),
                      solver_iterations=32, contact_offset=0.001)


def _lid() -> Body:
    """The clear lid: its plate (collision-only, drawn as its rim), the seal ring under it, the raised middle with the
    oil holes round it, and the long tab (drawn) with the pin under it."""
    s = [CylShape((0.0, 0.0, PLATE_T / 2), PLATE_R, PLATE_T, None)]
    s += _ring(PLATE_R - 0.002, 0.0, PLATE_T + 0.0004, 0.004, CLEAR_D, collide=False)  # its rim, drawn
    s += _ring(SKIRT_RO - SKIRT_T / 2, -SKIRT_H / 2, 0.0, SKIRT_T, None)  # the seal ring
    s += _ring(SKIRT_RO - SKIRT_LEAD - SKIRT_T / 2, -SKIRT_H, -SKIRT_H / 2, SKIRT_T, None, n=20)  # its lead-in
    s += _ring(SKIRT_RO - SKIRT_T / 2, -SKIRT_H, 0.0, SKIRT_T + 0.0004, CLEAR_D, n=24, collide=False)[::3]  # seen
    s.append(CylShape((0.0, 0.0, PLATE_T + DOME_H / 2), DOME_R, DOME_H, None))  # the raised middle
    s += _ring(DOME_R - 0.0015, PLATE_T, PLATE_T + DOME_H, 0.003, CLEAR_D, collide=False)
    s += _ring(DOME_R - 0.006, PLATE_T + DOME_H - 0.0004, PLATE_T + DOME_H + 0.0004, 0.002, CLEAR_D, n=20,
               collide=False)
    for k in range(14):  # the oil holes
        x, y = _polar(0.041, 360.0 * k / 14)
        s.append(CylShape((x, y, PLATE_T + 0.0002), 0.0017, 0.0006, DARK, collide=False))
    s.append(_box(-TAB_W / 2, TAB_W / 2, PLATE_R - 0.004, TAB_END, 0.0, PLATE_T, CLEAR_D))  # the tab
    s += _ring(0.011, 0.0, PLATE_T, 0.004, CLEAR_D, n=12, collide=False, at=(0.0, TAB_END + 0.009))  # finger loop
    s.append(CylShape((0.0, TOWER_R, -PIN_L / 2), PIN_R, PIN_L, CLEAR_D))  # the pin under the tab
    return Body.build("lid", pos=(LID_AT[0], LID_AT[1], TOP + SKIRT_H), mass=LID_MASS, shapes=s, color=CLEAR,
                      friction=(0.8, 0.7), com=(0.0, 0.012, 0.004), solver_iterations=32)


def _pad() -> Body:
    """The pulse pad on the tab: grey, textured, with a power mark (origin: its middle). It is its own body on a sprung
    slide; pressed 2 mm down it closes the motor's switch in the tower, when the lid is fitted."""
    top = PAD_H / 2
    s = [CylShape((0.0, 0.0, 0.0), PAD_R, PAD_H, GREY),
         BoxShape((0.0, 0.003, top + 0.0003), (0.0016, 0.006, 0.0006), (0.92, 0.92, 0.92), collide=False)]
    s += _ring(0.0055, top, top + 0.0006, 0.0013, (0.92, 0.92, 0.92), n=12, collide=False)[1:-1]
    s += _ring(0.0090, top, top + 0.0005, 0.0008, (0.55, 0.56, 0.58), n=16, collide=False)
    return Body.build("pad", pos=(LID_AT[0], LID_AT[1] + TOWER_R, TOP + SKIRT_H + PLATE_T + PAD_PROUD - top),
                      mass=PAD_MASS, shapes=s, color=GREY, friction=(0.8, 0.7), ignore=("lid",), solver_iterations=32)


# the motor's axle: the drive turns on it; stopped, a spring (the motor's gear) holds it
hinge("axle", "chopper", "drive", anchor=(AXIS[0], AXIS[1], TOP + FLOOR_Z), axis="Z", spring=DRIVE_K,
      damping=DRIVE_DAMP, target_deg=0.0)
# the lid's seal in the bowl's mouth: switched on once it is caught; a spring on its slide holds the lid where it was
# left until it is pushed (or pulled) with about 10 N, and its pin keeps it from turning once it is in the tower's hole
thread("seal", "chopper", "lid", anchor0=(0.0, 0.0, RIM_Z), axis0=(0.0, 0.0, 1.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 0.0, 1.0), slide_limits=(0.0, SEAL_TOP), turn_damping=0.002, slide_stiffness=SEAL_K,
       slide_damping=30.0, slide_max_force=40.0)
# the pad's slide: 3 mm of travel, its spring pushing it up against its top stop (preloaded)
_PAD0 = (LID_AT[0], LID_AT[1] + TOWER_R, TOP + SKIRT_H + PLATE_T + PAD_PROUD - PAD_H / 2)
slider("pad_spring", "lid", "pad", anchor=_PAD0, axis="Z", limits=(-PAD_TRAVEL, 0.0), stiffness=PAD_K,
       damping=PAD_DAMP, target=PAD_PRELOAD, max_force=30.0)

OBJECTS = [_chopper(), _drive(), _blade(), _lid(), _pad()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"steps": 0, "on": False, "hold": 0.0, "s": 0.0, "fitted": False, "seated": False, "depth": 0.0,
            "switch": False, "running": False, "starts": 0, "pulses": 0, "pulsed_at": None, "cut": False, "tcps": None,
            "standing": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CHOP_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    try:
        set_thread_on("seal", False)
        set_hinge_drive("axle", target_deg=0.0, spring=DRIVE_K)
    except Exception:  # before the scene exists
        pass


def _grippers(base, origin) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each arm's TCP and the direction its fingers point (world)."""
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        r = rotation(g[3:7])
        out.append((g[:3] - origin + r @ np.array(TCP_OFFSET), r[:, 2]))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _spin(base, name: str, w) -> None:
    import torch

    asset = base.scene[name]
    asset.write_root_velocity_to_sim(torch.tensor([[0.0, 0.0, 0.0, *[float(v) for v in w]]], dtype=torch.float32,
                                                  device=asset.device))


def _yaw(r) -> float:
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = max(0.0, min(1.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    t = _EP["steps"] * 0.05
    pc, rc = _pose(base, origin, "chopper")
    pl, rl = _pose(base, origin, "lid")
    pd, rd = _pose(base, origin, "blade")
    pk, rk = _pose(base, origin, "drive")
    # the lid's seal: caught once the seal ring is in the bowl's mouth; then it stays where it was left until it is
    # pushed (or pulled) with about 10 N; pulled up out of the mouth, it comes off
    v = rc.T @ (pl - pc)
    s, off, tilt = float(v[2] - RIM_Z), math.hypot(v[0], v[1]), _tilt(rc.T @ rl)
    rel = _wrap(_yaw(rl) - _yaw(rc))
    _EP["s"] = s
    if not _EP["on"]:
        if off <= CATCH_R and CATCH_DZ[0] <= s <= CATCH_DZ[1] and tilt <= CATCH_TILT:
            hold = min(max(s, 0.0), SEAL_TOP)
            set_slide_target("seal", hold)
            set_thread_on("seal", True)
            _EP.update(on=True, hold=hold)
    elif s <= SEATED_DZ and _EP["hold"] > 0.0:  # pressed home past its lip: the seal snaps into place
        _EP["hold"] = 0.0
        set_slide_target("seal", 0.0)
    elif s < _EP["hold"] - SEAL_SLIP or s > _EP["hold"] + SEAL_SLIP:  # pushed or pulled past the seal's grip
        _EP["hold"] = min(max(s, 0.0), SEAL_TOP)
        set_slide_target("seal", _EP["hold"])
        if _EP["hold"] >= SEAL_OFF:  # pulled up out of the mouth: it comes off
            set_thread_on("seal", False)
            _EP.update(on=False, hold=0.0)
    # fitted, live: the seal caught and pressed home, the lid level and its tab's pin in the tower's hole
    _EP["fitted"] = bool(_EP["on"] and s <= SEATED_DZ and tilt <= SEATED_TILT and abs(rel) <= FIT_YAW)
    # the blade on its drive: its hub's bottom down at the floor (only with its keyway over the key), on the axis
    u = rc.T @ (pd - pc)
    _EP["seated"] = bool(math.hypot(u[0], u[1]) <= SEAT_R and abs(u[2] - SEAT_Z) <= SEAT_DZ
                         and _tilt(rc.T @ rd) <= SEAT_TILT)
    # fingertips near the blades' cutting parts
    grips = _grippers(base, origin)
    _EP["tcps"] = [g[0] for g in grips]
    for z, sign in ((BLADE_Z[0], 1.0), (BLADE_Z[1], -1.0)):
        a = pd + rd @ np.array([sign * EDGE_R0, sign * 0.002, z + BLADE_T / 2])
        b = pd + rd @ np.array([sign * BLADE_R1, sign * 0.006, z + BLADE_T / 2])
        for tcp, axis in grips:
            for p in (tcp, tcp + PAD_TIP_PAST_TCP * axis):
                if _seg_dist(p, a, b) < EDGE_NEAR + BLADE_W / 2:
                    _EP["cut"] = True
    # the pad: how far it is pushed down its slide; the switch under it closes 2 mm down (opens again above 1 mm). The
    # motor runs while the switch is closed and the lid is fitted (the cover interlock). Every start counts; a pulse is
    # a start with the blade on its drive
    pp, _ = _pose(base, origin, "pad")
    _EP["depth"] = depth = (PLATE_T + PAD_PROUD - PAD_H / 2) - float((rl.T @ (pp - pl))[2])
    _EP["switch"] = depth > SWITCH_OFF if _EP["switch"] else depth >= SWITCH_AT
    running = _EP["switch"] and _EP["fitted"]
    if running and not _EP["running"]:
        _EP["starts"] += 1
        if _EP["seated"] and not _EP["cut"]:
            _EP["pulses"] += 1
            if _EP["pulsed_at"] is None:
                _EP["pulsed_at"] = t
    # the motor: running, it turns its drive (and the blade with it, if it is keyed on); stopped, its gear holds the
    # drive where it stopped
    turn = _wrap(_yaw(rc.T @ rk))
    if running:
        if not _EP["running"]:
            set_hinge_drive("axle", spring=0.0)
        _spin(base, "drive", SPIN_W * rc[:, 2])
        if _EP["seated"]:
            _spin(base, "blade", SPIN_W * rc[:, 2])
    elif _EP["running"]:
        set_hinge_drive("axle", target_deg=turn, spring=DRIVE_K)
    _EP["running"] = running
    _EP["standing"] = bool(_tilt(rc) < 5.0 and pc[2] - TOP < 0.004)
    if _DEBUG["on"] and (_EP["steps"] % 5 == 0 or depth > 0.0005):
        tl, trc = _EP["tcps"]
        print(f"[chop] t={t:5.2f} unit={np.round((pc - AXIS) * 1000).astype(int).tolist()} uyaw={_yaw(rc):5.1f}"
              f" lid_on={int(_EP['on'])} s={s * 1000:5.2f} hold={_EP['hold'] * 1000:5.2f} off={off * 1000:4.1f}"
              f" ltilt={tilt:4.1f} lyaw={rel:5.1f} fitted={int(_EP['fitted'])}"
              f" blade={np.round((u - [0, 0, SEAT_Z]) * 1000, 1).tolist()} byaw={_wrap(_yaw(rc.T @ rd)):6.1f}"
              f" btilt={_tilt(rc.T @ rd):4.1f}"
              f" key={turn:6.1f} seat={int(_EP['seated'])} pad={depth * 1000:.2f} sw={int(_EP['switch'])}"
              f" run={int(running)} starts={_EP['starts']} pulses={_EP['pulses']} cut={int(_EP['cut'])} pulsed={_EP['pulsed_at']}"
              f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
              f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}",
              file=sys.stderr, flush=True)
        fn = []
        for arm in ("left_robot", "right_robot"):
            rob = base.scene[arm]
            fj = [i for i, n in enumerate(rob.joint_names) if n in ("joint7", "joint8")]
            fn.append(np.round(as_numpy(rob.data.joint_pos)[0][fj] * 1000, 1).tolist())
        print(f"[chop]   lid-R={np.round((pl - trc) * 1000, 1).tolist()} blade-R={np.round((pd - trc) * 1000, 1).tolist()}"
              f" fingers L={fn[0]} R={fn[1]}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once a fingertip has come within 1.4 cm of the middle line of either blade's cutting part (the 1.2 cm wide
    blade beyond 2 cm from the hub's axis: about 8 mm from its edges).
    The motor starts when the pad is pressed past its switch point (2 mm of its 3 mm travel) while the lid is fitted;
    a pulse is a start with the blade on its drive. Fitted is checked live, at that moment and at the end: the lid's
    seal caught in the bowl's mouth and pressed home, the lid level, its tab's pin in the tower's hole.
    Otherwise 1.0 when the motor started exactly once and that start was a pulse, with, at the end, the pad let go (its
    switch open), the lid fitted, the blade on its drive, the chopper standing on the table and both arms back where
    they started; 0.75 after at least one pulse; 0.5 with the lid fitted and the blade on its drive at the end; 0.25 with the blade on its drive at the
    end; else 0.0."""
    if _EP["cut"]:
        return 0.0
    if _EP["pulses"] >= 1:
        if (_EP["starts"] == 1 and _EP["pulses"] == 1 and not _EP["switch"] and _EP["fitted"] and _EP["seated"]
                and _EP["standing"] and _arms_home()):
            return 1.0
        return 0.75
    if _EP["fitted"] and _EP["seated"]:
        return 0.5
    return 0.25 if _EP["seated"] else 0.0


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
    """Full score with the chopper, its lid, the pad and the blade still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["chopper"].resting() and objects["lid"].resting()
            and objects["pad"].resting() and objects["blade"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right takes the blade by the knob on its hub, lifts it, carries it over the bowl and sets it down on the centre
# tube, where it rests on the drive's cross pin; it turns the blade until its notches meet the pin and it drops to the
# bottom, and lets go. It takes the lid by its raised middle, carries it over the bowl with its tab over the tower, lowers its
# seal into the bowl's mouth and presses it home (the tab's pin goes into the tower's hole), lets go and goes home.
# Then the left, on the tower's side, presses the pad on the tab with its closed fingers (the motor runs: the blade
# spins), lets it up and goes home. The left waits at home while the right works over the bowl (wrists closer than
# about 15 cm shove each other).

OVER = 0.035
LEAD_S, LEAD_STEPS = 3.0, 15
FIN_FINGER = -0.012
FIN_DZ = 0.002  # the TCP this far above the knob's top (the pads' tips reach 11 mm down it)
LID_FINGER = -0.037
LID_DZ = 0.004  # the TCP this far above the lid's raised middle
CLEAR_RIM = 0.010  # the blade's hub and the lid's seal pass this far above the bowl's rim
HIGH = 0.165  # the TCP's height above the table while the empty hand moves between the bowl and the lid
REST_ON_KEY = KEY_Z[1] + 0.001  # the hub's bottom this far above its seat while it rests on the cross pin
TURN_FROM = 40.0  # the blade comes down this far off the cross pin and is turned onto it
DROP_PRESS = 0.008  # while turning, the hand aims this far lower (it drops once its notches meet the pin)
SEAL_PRESS = 0.032  # the hand aims the lid this far below home (about 13 N through the arm)
PRESS_DEPTH = 0.026  # aim the closed pads' tips this far below the pad's top (it stops 3 mm down; the arm gives)
RELEASE_FINGER = FINGER_OPEN  # fully open before rising off the lid (it hangs off-centre: no finger may close on it)
BLADE_AIM = np.array([0.0, 0.0, 0.0])  # the blade hangs straight under the TCP (measured)
LOW_AIM = np.array([0.005, 0.004, 0.0])  # low over the bowl the right TCP falls short by this much (measured)
LID_AIM = np.array([-0.001, 0.017, 0.0])  # the lid hangs 17 mm off the TCP along the closing axis: aim against it


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
                if room > 0.4:
                    break
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


def _g(yaw_deg: float):
    return grip_rotation(math.radians(yaw_deg))


def _plan() -> dict:
    """Both arms' waypoints."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    L, R = [], []
    ax = AXIS.copy()
    knob = HUB_H + FIN_H + FIN_DZ  # the TCP above the hub's bottom
    # the right: the blade, by its knob, onto the drive
    g_fin = np.array([BLADE_AT[0], BLADE_AT[1], TOP + knob])
    rb = _g(BLADE_YAW0)  # fingers close across the knob
    R += _joint_path("right", np.array(READY, float), _good_ik("right", g_fin + [0, 0, OVER], rb),
                     "over the blade's knob", FIN_FINGER)
    R.append(_w(1.2, g_fin, FIN_FINGER, "down round the knob", rb))
    R.append(_w(0.6, g_fin, FINGER_CLOSED, "take the blade by its hub", rb))
    carry = ax + BLADE_AIM + [0.0, 0.0, RIM_Z + CLEAR_RIM + knob]
    R.append(_w(1.4, g_fin + [0, 0, carry[2] - g_fin[2]], FINGER_CLOSED, "lift it", rb))
    y1 = TURN_FROM  # the cross pin lies along x: the blade comes down 40 degrees off it
    R.append(_w(2.4, carry, FINGER_CLOSED, "carry it over the bowl", _g(y1)))
    R.append(_w(0.6, carry, FINGER_CLOSED, "steady over the centre tube", _g(y1)))
    rest = ax + BLADE_AIM + LOW_AIM + [0.0, 0.0, SEAT_Z + REST_ON_KEY + knob]
    R.append(_w(1.8, rest, FINGER_CLOSED, "lower it over the tube (it rests on the cross pin)", _g(y1)))
    n = 6
    for k in range(1, n + 1):
        d = y1 * (1 - k / n)
        R.append(_w(0.4, rest - [0, 0, DROP_PRESS * k / n], FINGER_CLOSED,
                    "turn it until it drops onto the pin", _g(d)))
    seat = ax + BLADE_AIM + LOW_AIM + [0.0, 0.0, SEAT_Z + knob]
    R.append(_w(0.6, seat - [0, 0, 0.002], FINGER_CLOSED, "down on its seat", _g(0.0)))
    R.append(_w(0.6, seat, FIN_FINGER, "let go of the blade", _g(0.0)))
    high = TOP + HIGH  # well clear of the bowl and the lid
    R.append(_w(1.2, np.array([seat[0], seat[1], high]), FIN_FINGER, "rise clear of the bowl", _g(0.0)))
    # the right: the lid, by its raised middle (fingers closing along y, in line with its tab: the lid can't tip about
    # the line between them)
    rl = _g(0.0)
    lid0 = np.array([LID_AT[0], LID_AT[1], TOP + SKIRT_H])  # its origin at the start
    up = PLATE_T + DOME_H + LID_DZ  # the TCP above the lid's origin
    g_lid = lid0 + [0.0, 0.0, up]
    R.append(_w(1.8, np.array([g_lid[0], g_lid[1], high]), LID_FINGER, "over to the lid", rl))
    R.append(_w(1.0, g_lid + [0, 0, OVER], LID_FINGER, "over the lid", rl))
    R.append(_w(1.2, g_lid, LID_FINGER, "down round its middle", rl))
    R.append(_w(0.6, g_lid, FINGER_CLOSED, "take the lid", rl))
    lifted = g_lid + [0, 0, RIM_Z + CLEAR_RIM + 0.005]
    R.append(_w(1.4, lifted, FINGER_CLOSED, "lift it", rl))
    home = ax + LID_AIM + [0.0, 0.0, RIM_Z + up]  # the TCP with the lid fitted
    above = home + [0, 0, SKIRT_H + CLEAR_RIM]
    R.append(_w(2.4, above, FINGER_CLOSED, "carry it over the bowl, its tab over the tower", rl))
    R.append(_w(0.6, above, FINGER_CLOSED, "steady over the bowl", rl))
    R.append(_w(1.4, home + [0, 0, 0.004], FINGER_CLOSED, "lower its seal into the bowl's mouth", rl))
    R.append(_w(1.2, home - [0, 0, SEAL_PRESS], FINGER_CLOSED, "press it home", rl))
    R.append(_w(0.6, home - [0, 0, SEAL_PRESS], FINGER_CLOSED, "hold it pressed", rl))
    R.append(_w(0.4, home, FINGER_CLOSED, "ease off", rl))
    R.append(_w(0.6, home, RELEASE_FINGER, "let go of the lid", rl))
    R.append(_w(1.0, home + [0, 0, 0.030], RELEASE_FINGER, "rise clear", rl))
    R += _joint_path("right", _good_ik("right", home + [0, 0, 0.030], rl), np.array(READY, float), "back home",
                     RELEASE_FINGER)
    t_right_clear = sum(w.duration for w in R) - LEAD_S * 0.5
    # the left: presses the pad on the lid's tab (over the tower, on its side) once the right is clear
    home_l, home_lr = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    pad = ax + [0.0, TOWER_R, RIM_Z + PLATE_T + PAD_PROUD]  # the pad's top, fitted
    touch = pad + [0.0, 0.0, PAD_TIP_PAST_TCP]
    rp = _g(0.0)
    q_over = _good_ik("left", pad + [0, 0, 0.040], rp)
    L.append(_w(max(0.6, t_right_clear - LEAD_S), home_l, FINGER_CLOSED, "wait at home", home_lr))
    L += _joint_path("left", np.array(READY, float), q_over, "over the pad", FINGER_CLOSED)
    L.append(_w(0.8, touch + [0, 0, 0.004], FINGER_CLOSED, "down onto the pad", rp))
    L.append(_w(0.6, touch - [0, 0, PRESS_DEPTH], FINGER_CLOSED, "press the pad: the motor runs, the blade spins", rp))
    L.append(_w(1.0, touch - [0, 0, PRESS_DEPTH], FINGER_CLOSED, "hold it down a moment", rp))
    L.append(_w(0.5, touch + [0, 0, 0.004], FINGER_CLOSED, "let it up: the motor stops", rp))
    L.append(_w(0.6, pad + [0, 0, 0.040], FINGER_CLOSED, "rise clear", rp))
    L += _joint_path("left", q_over, np.array(READY, float), "back home", FINGER_CLOSED)
    tr = sum(w.duration for w in R)
    tl = sum(w.duration for w in L)
    end = max(tr, tl) + 2.0
    home_r, home_rr = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
    R.append(_w(end - tr, tuple(home_r), RELEASE_FINGER, "hold still", home_rr))
    L.append(_w(end - tl, tuple(home_l), FINGER_CLOSED, "hold still", home_lr))
    return {"left": L, "right": R}


def demo():
    return _plan()
