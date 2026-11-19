"""The cooktop of a 30-inch plug-in coil electric range (GE/Hotpoint style). Its right front burner's 8-inch coil
element (GE WB30M2 type) is dead; a new one lies flat on the deck between the two front burners.

Task: replace the dead coil with the new one.

What the physics asks for: a plug-in element's two terminal blades go through the notch in its chrome drip pan into a
receptacle under the deck. The blades run 25 mm into two channels just below the deck's edge, so they only go in
held level at the right height, pushed straight in. The coil sits inside the pan's raised lip: it comes out only
lifted over the lip, and goes in only with its far edge held up over the lip until the blades are home, then lowered
flat (the makers' method). Lifting a plugged-in coil's far edge much more than an inch levers the receptacle
(strained: the trial fails). The 0.45 kg, 19 cm coil is held at its terminal bracket and at its far rim; a single
gripper on the bracket can't hold it level.

Run it:  inspect-robots run --embodiment yam_arena -E task=replace_the_dead_coil_element_on_an_electric_range.py \\
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
from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the range top -----------------------------------------------------------------------------------------------------
D = TOP + 0.035  # the enamel cooktop deck's surface, 35 mm above the table
DECK_T = 0.003
RX0, RX1 = 0.075, 0.750  # its front edge and the backguard's back
RY0, RY1 = -0.330, 0.432  # 76.2 cm (30 in) wide
C8 = (0.300, -0.130)  # the right front burner: 8-inch element
C6 = (0.300, 0.250)  # the left front burner: 6-inch element
REAR = {"rr": ((0.585, -0.130), 6), "lr": ((0.585, 0.250), 8)}
# 8-inch drip pan (chrome): lip on the deck, a ledge the element's feet stand on, the bowl below
LIP8 = (0.112, 0.124)
LEDGE8 = (0.084, 0.112)
LIP6 = (0.088, 0.099)
LEDGE6 = (0.062, 0.088)
LEDGE_Z = -0.012  # the ledges' top, below the deck surface
BOWL_Z = -0.030
NOTCH_HX = 0.022  # the notch in the 8-inch pan's lip for the element's terminal bracket (44 mm)

# -- the element (GE WB30M2 type): origin at the coil's centre, on the underside of its sheath ------------------------
RINGS = (0.0935, 0.0775, 0.0640, 0.0505, 0.0370, 0.0235)  # six turns
SHEATH_W, SHEATH_H = 0.0075, 0.0065
FOOT_R, FOOT_H = 0.090, 0.006  # two feet at the far ends of the Y-frame
BR_X, BR_Y0, BR_Y1, BR_Z0, BR_Z1 = 0.0125, 0.101, 0.117, -0.006, 0.010  # the steel terminal bracket
BL_X, BL_T, BL_Y1, BL_Z0, BL_Z1 = 0.0065, 0.002, 0.142, -0.0045, 0.0015  # two terminal blades, 25 mm long
COIL_MASS = 0.45
SEAT_Z = D + LEDGE_Z + FOOT_H  # the origin's height when the element stands on its feet in the pan

# -- the receptacle under the deck at the 8-inch well's inner side (+y), in the seated element's frame ------------------
MOUTH_Y, BOTTOM_Y, BACK_Y = 0.118, 0.143, 0.150
CH_HW = 0.002  # each blade channel is 4 mm wide (blade 2 mm)
CH_Z0, CH_Z1 = -0.0075, 0.003  # and 10.5 mm tall (blade 6 mm); its roof is the deck's underside
LEAD = 0.004  # 45-degree lead-in chamfer on the channels' floor
LEAD_MID, LEAD_OUT = 0.0045, 0.006  # and on the walls: the middle wall comes to an edge, the outer ones 6 mm

# -- the rules ---------------------------------------------------------------------------------------------------------
HOME_DEPTH = 0.020  # blades this far in: plugged home (full travel 24 mm)
IN_DEPTH = 0.003  # in the receptacle at all
LEVEL_MM = 3.0  # seated: level within 3 mm across the coil's 19 cm
CENTRE_MM = 6.0  # and centred in the well within 6 mm
STRAIN_DEG = 10.0  # tilted this far with its blades 3 mm or more in: the receptacle is levered (strained)
OUT_OF_WELL = 0.150  # the dead element's centre this far from the right well's centre: out of it
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

ENAMEL, ENAMEL_D = (0.93, 0.93, 0.91), (0.80, 0.80, 0.78)
CHROME, CHROME_D = (0.80, 0.81, 0.83), (0.55, 0.56, 0.58)
SHEATH_NEW, SHEATH_OLD = (0.13, 0.13, 0.14), (0.30, 0.27, 0.24)
STEEL, BLADE = (0.62, 0.63, 0.65), (0.82, 0.80, 0.74)
CERAMIC, KNOB, GLASS = (0.92, 0.91, 0.86), (0.12, 0.12, 0.13), (0.10, 0.11, 0.13)


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _annulus(cx, cy, r0, r1, z0, z1, color, n=48, collide=True, skip=None, clip=None) -> list:
    """Boxes approximating a flat ring round (cx, cy) from radius r0 to r1 (r1 clipped by ``clip(theta)``)."""
    out = []
    for k in range(n):
        th = 2 * math.pi * (k + 0.5) / n
        if skip is not None and skip(th):
            continue
        ro = r1 if clip is None else min(r1, clip(th))
        if ro <= r0 + 0.002:
            continue
        w = 2 * ro * math.tan(math.pi / n) + 0.002
        rm = (r0 + ro) / 2
        out.append(BoxShape((cx + rm * math.cos(th), cy + rm * math.sin(th), (z0 + z1) / 2), (ro - r0, w, z1 - z0),
                            color, yaw=th, collide=collide))
    return out


def _coil_shapes(rings, color, collide_rings=2, z0=0.0, cx=0.0, cy=0.0, collide=True) -> list:
    sh = []
    for i, r in enumerate(rings):
        solid = collide and i < collide_rings
        n = max(10, int(2 * math.pi * r / (0.014 if solid else 0.020)))
        chord = 2 * r * math.sin(math.pi / n) + 0.0015
        for k in range(n):
            th = 2 * math.pi * k / n
            sh.append(BoxShape((cx + r * math.cos(th), cy + r * math.sin(th), z0 + SHEATH_H / 2), (SHEATH_W, chord,
                               SHEATH_H), color, yaw=th, collide=solid))
    return sh


def _range() -> Body:
    """The range's top: the white enamel deck with its two front burner wells (chrome pans, the receptacle for the
    right one under the deck), the cold 6-inch element in the left front well, the rear burners, the backguard with its
    knobs. Fixed. Origin: on the deck surface at (0, 0)."""
    sh = []
    ox, oy = 0.0, 0.0

    # the deck: flat tiles round two squares about the front wells, rings of tiles inside those squares
    zd0, zd1 = -DECK_T, 0.0
    xe = RX1 - 0.040
    h8, h6 = 0.130, 0.105
    s8 = (C8[0] - h8, C8[0] + h8, C8[1] - h8, C8[1] + h8)
    s6 = (C6[0] - h6, C6[0] + h6, C6[1] - h6, C6[1] + h6)
    for lo, hi in (((RX0, RY0), (xe, s8[2])), ((RX0, s8[2]), (s8[0], s8[3])), ((s8[1], s8[2]), (xe, s8[3])),
                   ((RX0, s8[3]), (xe, s6[2])), ((RX0, s6[2]), (s6[0], s6[3])), ((s6[1], s6[2]), (xe, s6[3])),
                   ((RX0, s6[3]), (xe, RY1))):
        sh.append(_box((lo[0], lo[1], zd0), (hi[0], hi[1], zd1), ENAMEL))
    sh += _annulus(C8[0], C8[1], LIP8[1] - 0.004, h8 * 1.42, zd0, zd1, ENAMEL, n=44)
    sh += _annulus(C6[0], C6[1], LIP6[1] - 0.004, h6 * 1.42, zd0, zd1, ENAMEL, n=40)
    # the range's body below the deck (front panel, sides), down to the table
    sh.append(_box((RX0 - 0.004, RY0, TOP - D), (RX0, RY1, zd0), ENAMEL_D, collide=False))
    sh.append(_box((RX0, RY0 - 0.004, TOP - D), (RX1, RY0, 0.002), ENAMEL_D, collide=False))
    sh.append(_box((RX0, RY1, TOP - D), (RX1, RY1 + 0.004, 0.002), ENAMEL_D, collide=False))
    sh.append(_box((RX0 - 0.006, RY0 - 0.004, -0.004), (RX0, RY1 + 0.004, 0.003), ENAMEL, collide=False))  # front lip
    # the backguard: knobs (all OFF) and a clock
    sh.append(_box((RX1 - 0.040, RY0, 0.0), (RX1, RY1, 0.170), ENAMEL))
    sh.append(_box((RX1 - 0.042, RY0 + 0.01, 0.060), (RX1 - 0.040, RY1 - 0.01, 0.140), (0.20, 0.20, 0.22), False))
    for y in (-0.270, -0.190, 0.290, 0.370):
        sh.append(CylShape((RX1 - 0.052, y, 0.100), 0.019, 0.022, KNOB, collide=False, axis="X"))
        sh.append(_box((RX1 - 0.064, y - 0.002, 0.100), (RX1 - 0.062, y + 0.002, 0.116), (0.85, 0.85, 0.85), False))
    sh.append(_box((RX1 - 0.043, 0.020, 0.080), (RX1 - 0.041, 0.100, 0.120), GLASS, False))
    sh.append(_box((RX1 - 0.044, 0.040, 0.094), (RX1 - 0.043, 0.080, 0.106), (0.20, 0.55, 0.30), False))
    # the right front well: the 8-inch chrome pan (lip notched at +y for the terminal bracket), ledge and bowl
    notch = lambda th: abs(math.cos(th)) * LIP8[1] < NOTCH_HX and math.sin(th) > 0  # noqa: E731
    sh += _annulus(C8[0], C8[1], LIP8[0], LIP8[1], -0.002, 0.002, CHROME, n=64, skip=notch)
    sh += _annulus(C8[0], C8[1], LEDGE8[0], LEDGE8[1], LEDGE_Z - 0.002, LEDGE_Z, CHROME, n=36)
    sh += _annulus(C8[0], C8[1], LEDGE8[1], LIP8[0] + 0.001, LEDGE_Z, -0.002, CHROME_D, n=48, skip=notch)
    sh += _annulus(C8[0], C8[1], LEDGE8[0] - 0.003, LEDGE8[0], BOWL_Z, LEDGE_Z - 0.002, CHROME_D, n=24)
    sh.append(CylShape((C8[0], C8[1], BOWL_Z - 0.0015), LEDGE8[0], 0.003, CHROME))
    sh.append(CylShape((C8[0], C8[1], BOWL_Z + 0.0002), 0.020, 0.0004, (0.30, 0.30, 0.32), collide=False))
    # the receptacle under the deck: a porcelain block with two blade channels opening toward the well (-y)
    sx, sy, sz = C8[0], C8[1], SEAT_Z - D  # the seated element's origin in this body's frame

    def rbox(lo, hi, color=CERAMIC, collide=True):
        sh.append(_box((sx + lo[0], sy + lo[1], sz + lo[2]), (sx + hi[0], sy + hi[1], sz + hi[2]), color, collide))

    y0, y1, yb = MOUTH_Y, BOTTOM_Y, BACK_Y
    def rplate(p1, p2, width, normal, t=0.002):
        n = np.asarray(normal, float) / np.linalg.norm(normal)
        a = np.asarray(p1, float) + [sx, sy, sz] + n * t / 2
        b = np.asarray(p2, float) + [sx, sy, sz] + n * t / 2
        sh.append(bodykit.plate(tuple(a.tolist()), tuple(b.tolist()), width, t, tuple(n.tolist()), CERAMIC))

    zc, hz = (CH_Z0 + CH_Z1) / 2, CH_Z1 - CH_Z0
    rbox((-0.020, y0 + LEAD, CH_Z0 - 0.006), (0.020, yb, CH_Z0))  # floor
    rbox((-0.020, y0, CH_Z0 - 0.006), (0.020, y0 + LEAD, CH_Z0 - LEAD))
    rplate((0.0, y0, CH_Z0 - LEAD), (0.0, y0 + LEAD, CH_Z0), 0.040, (0.0, 1.0, -1.0))  # the floor's chamfer
    rbox((-0.020, y1, CH_Z0), (0.020, yb, CH_Z1))  # back
    rbox((-0.020, y0, CH_Z1), (0.020, yb, -sz))  # roof (meets the deck)
    for xa, xb in ((-0.020, -BL_X - CH_HW), (-BL_X + CH_HW, BL_X - CH_HW), (BL_X + CH_HW, 0.020)):
        lw = LEAD_MID if abs(xa + xb) < 1e-6 else LEAD_OUT
        rbox((xa, y0 + lw, CH_Z0), (xb, y1, CH_Z1))  # walls between and beside the channels
        fa = xa + (lw if xa > -0.019 else 0.0)
        fb = xb - (lw if xb < 0.019 else 0.0)
        if fb - fa > 0.0004:
            rbox((fa, y0, CH_Z0), (fb, y0 + lw, CH_Z1))
        if xa > -0.019:  # chamfer on the wall's -x face
            rplate((xa + lw, y0, zc), (xa, y0 + lw, zc), hz, (1.0, 1.0, 0.0))
        if xb < 0.019:  # and on its +x face
            rplate((xb - lw, y0, zc), (xb, y0 + lw, zc), hz, (-1.0, 1.0, 0.0))
    rbox((-0.024, y0 + 0.006, CH_Z0 - 0.012), (0.024, yb + 0.010, CH_Z0 - 0.006), (0.70, 0.70, 0.68), False)  # holder
    # the left front well: 6-inch pan and its cold element, plugged in (terminals toward -y)
    notch6 = lambda th: abs(math.cos(th)) * LIP6[1] < 0.018 and math.sin(th) < 0  # noqa: E731
    sh += _annulus(C6[0], C6[1], LIP6[0], LIP6[1], -0.002, 0.002, CHROME, n=40, skip=notch6)
    sh += _annulus(C6[0], C6[1], LEDGE6[0], LEDGE6[1], LEDGE_Z - 0.002, LEDGE_Z, CHROME, n=28)
    sh += _annulus(C6[0], C6[1], LEDGE6[1], LIP6[0] + 0.001, LEDGE_Z, -0.002, CHROME_D, n=32, skip=notch6)
    sh.append(CylShape((C6[0], C6[1], BOWL_Z - 0.0015), LEDGE6[0], 0.003, CHROME))
    z6 = LEDGE_Z + FOOT_H
    sh += _coil_shapes((0.0700, 0.0565, 0.0430, 0.0295, 0.0160), SHEATH_NEW, collide_rings=3, z0=z6, cx=C6[0], cy=C6[1])
    sh.append(_box((C6[0] - 0.011, C6[1] - 0.088, z6 - 0.006), (C6[0] + 0.011, C6[1] - 0.074, z6 + 0.009), STEEL))
    sh.append(_box((C6[0] - 0.018, C6[1] - 0.100, -0.003), (C6[0] + 0.018, C6[1] - 0.088, 0.0005), CERAMIC, False))
    # the rear burners (cold, plugged in): scenery
    for (cx, cy), size in REAR.values():
        rl = LIP8 if size == 8 else LIP6
        sh += _annulus(cx, cy, rl[0] - 0.01, rl[1], 0.0, 0.002, CHROME, n=28, collide=False)
        sh.append(CylShape((cx, cy, 0.0006), rl[0] - 0.01, 0.0012, CHROME_D, collide=False))
        rings = RINGS if size == 8 else (0.0700, 0.0565, 0.0430, 0.0295, 0.0160)
        sh += _coil_shapes(rings, SHEATH_NEW, collide_rings=0, z0=0.003, cx=cx, cy=cy, collide=False)
        r0 = rings[0]
        sh.append(_box((cx - 0.011, cy - r0 - 0.016, 0.0), (cx + 0.011, cy - r0 - 0.004, 0.014), STEEL, False))
    return Body.build("range", pos=(ox, oy, D), mass=20.0, kinematic=True, shapes=sh, color=ENAMEL,
                      friction=(0.5, 0.45))


def _coil(name: str, pos, color) -> Body:
    """A GE WB30M2-type 8-inch plug-in element: six turns of flattened sheath on a Y-frame (two feet at its far
    ends, a chrome hub), the steel terminal bracket at +y with two blades sticking out 25 mm."""
    sh = _coil_shapes(RINGS, color)
    for x in (-BL_X, BL_X):  # the sheath's two ends running out to the bracket
        sh.append(_box((x - 0.003, RINGS[0], 0.0), (x + 0.003, BR_Y0 + 0.002, SHEATH_H), color))
    for th in (math.pi / 2, math.radians(210), math.radians(330)):  # the Y-frame under the turns
        c, s = math.cos(th), math.sin(th)
        sh.append(BoxShape((0.054 * c, 0.054 * s, -0.001), (0.080, 0.008, 0.002), STEEL, yaw=th, collide=False))
        if th != math.pi / 2:
            sh.append(BoxShape((FOOT_R * c, FOOT_R * s, -FOOT_H / 2), (0.010, 0.008, FOOT_H), STEEL, yaw=th))
    sh.append(CylShape((0.0, 0.0, 0.002), 0.014, 0.004, CHROME, collide=False))
    sh.append(_box((-BR_X, BR_Y0, BR_Z0), (BR_X, BR_Y1, BR_Z1), STEEL))
    for x in (-BL_X, BL_X):
        sh.append(_box((x - BL_T / 2, BR_Y1, BL_Z0), (x + BL_T / 2, BL_Y1, BL_Z1), BLADE))
    return Body.build(name, pos=pos, mass=COIL_MASS, shapes=sh, color=color, friction=(1.0, 0.9),
                      solver_iterations=32, damping=(0.0, 0.2), com=(0.0, 0.008, 0.002))


PROBE = os.environ.get("COIL_PROBE", "")
NEW0 = (0.175, 0.042, D + FOOT_H + 0.0005)  # the new element lies flat on the deck between the front burners
OLD0 = (C8[0], C8[1], SEAT_Z)  # the dead one, plugged in, in the right front well
if PROBE == "new":  # checks only: the dead one is already out, parked behind
    OLD0 = (0.570, 0.250, D + 0.02)
PARK = (0.355, 0.150)  # where the demo sets the dead element down: on the cold left front burner

OBJECTS = [_range(), _coil("old_coil", OLD0, SHEATH_OLD), _coil("new_coil", NEW0, SHEATH_NEW)]
COILS = ("old_coil", "new_coil")

# -- followed after every control step --------------------------------------------------------------------------------


def _fresh() -> dict:
    return {"strained": False, "dropped": False, "why": "", "tcps": None, "s": {}, "max_tilt_in": 0.0,
            "old_out": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("COIL_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


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


def _blades(p, r):
    """Each blade's depth in its receptacle channel (m, 0 if not in it), in the seated element's frame."""
    seat = np.array([C8[0], C8[1], SEAT_Z])
    out = []
    for x in (-BL_X, BL_X):
        tip = p + r @ np.array([x, BL_Y1, (BL_Z0 + BL_Z1) / 2]) - seat
        inside = abs(tip[0] - x) < CH_HW + 0.0015 and CH_Z0 - 0.001 < tip[2] < CH_Z1 + 0.001 and tip[1] > MOUTH_Y
        out.append(min(BOTTOM_Y - MOUTH_Y, tip[1] - MOUTH_Y) if inside else 0.0)
    return out


def _state(p, r) -> dict:
    depth = _blades(p, r)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2]))))
    level_mm = math.sin(math.radians(tilt)) * 2 * RINGS[0] * 1000
    off = math.hypot(p[0] - C8[0], p[1] - C8[1])
    return {"depth": depth, "tilt": tilt, "level_mm": level_mm, "off": off, "z": p[2]}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    st = {}
    for n in COILS:
        p, r = _pose(base, origin, n)
        s = _state(p, r)
        st[n] = s
        if min(s["depth"]) >= IN_DEPTH or max(s["depth"]) >= IN_DEPTH:
            _EP["max_tilt_in"] = max(_EP["max_tilt_in"], s["tilt"])
            if s["tilt"] >= STRAIN_DEG and not _EP["strained"]:
                _EP["strained"], _EP["why"] = True, f"{n} levered the receptacle at {s['tilt']:.1f} deg"
        on_top = RX0 - 0.01 < p[0] < RX1 and RY0 - 0.01 < p[1] < RY1 + 0.01 and p[2] > D + BOWL_Z - 0.02
        if not on_top and not _EP["dropped"]:
            _EP["dropped"], _EP["why"] = True, f"{n} fell off the range"
        s["p"] = p
    o, nw = st["old_coil"], st["new_coil"]
    old_out = o["off"] > OUT_OF_WELL and max(o["depth"]) == 0.0
    seated = (min(nw["depth"]) >= HOME_DEPTH and nw["level_mm"] <= LEVEL_MM and nw["off"] * 1000 <= CENTRE_MM)
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"old_out": old_out, "new_in": max(nw["depth"]) >= IN_DEPTH, "seated": seated}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round(np.asarray(v) * 1000, 1).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            parts = []
            for n in COILS:
                s = st[n]
                parts.append(f"{n[:3]} p={mm(s['p'] - [0, 0, D])} tilt={s['tilt']:.2f} lvl={s['level_mm']:.1f}"
                             f" off={s['off'] * 1000:.1f} depth={[round(d * 1000, 1) for d in s['depth']]}")
            print(f"[coil] t={_DEBUG['steps'] / 20:5.2f}s fingers={[round(v, 1) for v in fj]} {' | '.join(parts)}"
                  f" L={mm(tl - [0, 0, D])} R={mm(tr - [0, 0, D])} s={_EP['s']} strained={_EP['strained']}"
                  f" dropped={_EP['dropped']} {_EP['why']} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 once a coil has fallen off the range, or has been tilted 10 degrees or more with its blades in the
    receptacle (strained). Else, at the end, on live states: 1.0: the new coil seated in the right front well (both
    blades 20 mm or more into the receptacle, level within 3 mm across the coil, centred within 6 mm), the dead coil
    out of that well (its centre 15 cm or more from the well's centre, blades out) and both arms back where they
    started; 0.75: the same with an arm not back; 0.5: the dead coil out and the new coil's blades in the receptacle
    (3 mm or more), not seated; 0.25: the dead coil out; else 0.0."""
    s = _EP["s"]
    if not s or _EP["strained"] or _EP["dropped"]:
        return 0.0
    if not s["old_out"]:
        return 0.0
    if s["seated"]:
        return 1.0 if _arms_home() else 0.75
    return 0.5 if s["new_in"] else 0.25


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
    """Full score with both coils at rest: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in COILS)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands hold a coil: the left by its terminal bracket (fingers closing across it, along x), the right by its outer
# turn at the far rim (fingers closing radially, the inner one in the gap to the next turn). The dead coil: the right
# lifts the far edge over the pan's lip, both draw it 30 mm away from the receptacle (blades out), lift it out and set
# it down on the cold left front burner. The new coil: taken from the deck the same way, carried over the right well
# with its far edge up, its blades pushed straight into the receptacle by the left, then the far edge lowered flat.

ROT_L = grip_rotation(-math.pi / 2)  # closes along x, across the bracket
R_PITCH = math.radians(float(os.environ.get("COIL_RPITCH", -25.0)))  # fingers pitched forward: reaches further
ROT_R = np.array([[math.cos(R_PITCH), 0.0, math.sin(R_PITCH)], [0.0, 1.0, 0.0],
                  [-math.sin(R_PITCH), 0.0, math.cos(R_PITCH)]]) @ grip_rotation(0.0)  # closes along y: radially
L_GRIP = np.array([0.0, (BR_Y0 + BR_Y1) / 2, BR_Z1 + float(os.environ.get("COIL_LZ", 0.004))])
R_GRIP = np.array([0.0, -RINGS[0], SHEATH_H + float(os.environ.get("COIL_RZ", 0.0085))])
L_OPEN, R_OPEN = -(BR_X + 0.006), -float(os.environ.get("COIL_ROPEN", 0.0075))
L_HOLD, R_HOLD = FINGER_CLOSED, FINGER_CLOSED
LEAD_S, LEAD_STEPS = 3.0, 15
PULL = 0.030  # drawn this far from the receptacle
FAR_UP = 0.018  # the far edge lifted over the pan's lip
PUSH_OVER = float(os.environ.get("COIL_PUSHOVER", 0.012))  # the pushing hand aims this far past home (arm lag)
AIM_X = float(os.environ.get("COIL_AIMX", 0.006))  # low poses fall short in x: aim this far beyond, plugging in
SAG = float(os.environ.get("COIL_SAG", 0.004))  # the arms sag under the coil: aim this much higher when carrying


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
                if room > 0.3:
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


def _grips(origin, far_up=0.0, lift=0.0):
    """The two TCPs holding a level coil whose origin is at ``origin``; ``far_up`` raises the far rim (pivoting about
    the bracket), ``lift`` raises both."""
    o = np.asarray(origin, float)
    left = o + L_GRIP + [0.0, 0.0, lift]
    right = o + R_GRIP + [0.0, 0.0, lift + far_up]
    return left, right


def demo():
    from yam_arena.constants import READY

    from yam_arena import ik, station

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_CLOSED, r)
    moved = set()
    rot = {"left": ROT_L, "right": ROT_R}
    one = os.environ.get("COIL_ONEHAND")  # checks only: "left" or "right" has that arm work alone, the other at home

    def go(arm, t, pos, finger, label, n=1):
        p0, _, r0 = cur[arm]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, r0))
        cur[arm] = (p1, finger, r0)

    def both(t, pl, pr, fl, fr, label, n=1):
        go("left", t, pl, fl, label, n)
        go("right", t, pr, fr, label, n)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync():
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), "wait for the other hand")

    def lead(arm, pos, label, finger):
        q0 = np.array(READY, float) if arm not in moved else _good_ik(arm, cur[arm][0], cur[arm][2])
        moved.add(arm)
        plan[arm].extend(_joint_path(arm, q0, _good_ik(arm, pos, rot[arm]), label, finger))
        cur[arm] = (np.asarray(pos, float), finger, rot[arm])

    def take(origin, label):
        gl, gr = _grips(origin)
        lead("left", gl + [0.0, 0.0, 0.050], f"over the {label}'s terminal bracket", L_OPEN)
        lead("right", gr + [0.0, 0.0, 0.050], f"over the {label}'s far rim", R_OPEN)
        sync()
        both(1.2, gl, gr, L_OPEN, R_OPEN, f"down round the bracket and the outer turn of the {label}")
        both(0.8, gl, gr, L_HOLD, R_HOLD, f"take the {label} at both ends")
        wait("left", 0.4)
        wait("right", 0.4)

    seat = np.array([C8[0], C8[1], SEAT_Z])
    # checks only, not the task (COIL_CHECK): "strain" lifts the plugged-in dead coil's far edge 45 mm; "noremove"
    # skips the dead coil and tries the new one into the occupied well; "putback" sets the dead coil back in its well
    # (plugged) after taking it out; "short" pushes the new coil only to 8 mm short of home before lowering it
    chk = os.environ.get("COIL_CHECK", "")
    push_to = seat - [0.0, 0.008 + PUSH_OVER, 0.0] if chk == "short" else seat
    if chk == "strain":
        take(seat, "dead coil")
        gl, gr = _grips(seat, far_up=0.045 + SAG)
        both(1.5, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "lift its far edge 45 mm while it is plugged in")
        wait("left", 1.0)
        wait("right", 1.0)
        gl, gr = _grips(seat)
        both(1.0, gl, gr, L_HOLD, R_HOLD, "lower it again")
        both(0.6, gl, gr, L_OPEN, R_OPEN, "let go")
        both(0.8, gl + [0, 0, 0.05], gr + [0, 0, 0.05], L_OPEN, R_OPEN, "rise clear")
    if chk == "putback":
        take(seat, "dead coil")
        gl, gr = _grips(seat, far_up=FAR_UP + SAG)
        both(1.0, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "lift its far edge over the pan's lip")
        gl, gr = _grips(seat - [0.0, PULL, 0.0], far_up=FAR_UP + SAG)
        both(1.5, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "blades out")
        gl, gr = _grips(seat - [0.0, PULL, 0.0], lift=0.045 + SAG)
        both(1.2, gl, gr, L_HOLD, R_HOLD, "lift it out")
        gl, gr = _grips(seat - [0.0, PULL, 0.0], far_up=FAR_UP + SAG)
        both(1.2, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "lower it back")
        gl2, gr2 = _grips(seat, far_up=FAR_UP + SAG)
        both(1.8, gl2 + [0.0, PUSH_OVER, SAG], gr2, L_HOLD, R_HOLD, "push it back in", n=3)
        gl3, gr3 = _grips(seat)
        both(1.0, gl3 + [0.0, PUSH_OVER, 0.0], gr3 - [0.0, 0.0, 0.004], L_HOLD, R_HOLD, "lower it flat")
        both(0.6, gl3, gr3, L_OPEN, R_OPEN, "let go")
        both(0.8, gl3 + [0, 0, 0.05], gr3 + [0, 0, 0.05], L_OPEN, R_OPEN, "rise clear")
    if PROBE != "new" and chk not in ("strain", "noremove", "putback"):
        # 1. the dead coil: far edge over the lip, blades drawn out, lifted out, set down on the left front burner
        take(seat, "dead coil")
        gl, gr = _grips(seat, far_up=FAR_UP + SAG)
        both(1.0, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "lift its far edge over the pan's lip")
        gl, gr = _grips(seat - [0.0, PULL, 0.0], far_up=FAR_UP + SAG)
        both(1.5, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "draw it straight away from the receptacle: blades out")
        gl, gr = _grips(seat - [0.0, PULL, 0.0], lift=0.045 + SAG)
        both(1.2, gl, gr, L_HOLD, R_HOLD, "lift it out of the well, level")
        park = np.array([PARK[0], PARK[1], D + 0.0005 + FOOT_H + 0.006])
        gl, gr = _grips(park, lift=0.025 + SAG)
        both(2.5, gl, gr, L_HOLD, R_HOLD, "carry it over the new coil to the cold left front burner", n=5)
        gl, gr = _grips(park, lift=SAG)
        both(1.2, gl, gr, L_HOLD, R_HOLD, "set it down")
        both(0.6, gl, gr, L_OPEN, R_OPEN, "let go")
        both(0.8, gl + [0, 0, 0.05], gr + [0, 0, 0.05], L_OPEN, R_OPEN, "rise clear")
    # 2. the new coil: taken from the deck, carried over the right well, plugged in, lowered flat
    take(np.array(NEW0), "new coil")
    gl, gr = _grips(NEW0, lift=0.045 + SAG)
    both(1.2, gl, gr, L_HOLD, R_HOLD, "lift it")
    pre = seat - [-AIM_X, PULL, 0.0]
    push_to = push_to + [AIM_X, 0.0, 0.0]
    gl, gr = _grips(pre, far_up=FAR_UP, lift=0.040 + SAG)
    both(2.5, gl, gr, L_HOLD, R_HOLD, "carry it over the right front well", n=5)
    gl, gr = _grips(pre, far_up=FAR_UP + SAG)
    both(1.2, gl + [0, 0, SAG], gr, L_HOLD, R_HOLD, "lower its blades to the receptacle's mouth, far edge up")
    gl2, gr2 = _grips(push_to, far_up=FAR_UP + SAG)
    over = PUSH_OVER
    go("left", 1.8, gl2 + [0.0, over, SAG], L_HOLD, "push the blades straight into the receptacle", n=3)
    go("right", 1.8, gr2, R_HOLD, "follow along, far edge up", n=3)
    wait("left", 0.6, "keep pushing: blades home")
    wait("right", 0.6)
    gl3, gr3 = _grips(push_to)
    go("left", 1.0, gl3 + [0.0, over, 0.0], L_HOLD, "hold the bracket home")
    go("right", 1.0, gr3 - [0.0, 0.0, 0.004], R_HOLD, "lower the far edge: the coil lies flat on its feet")
    wait("left", 0.6)
    wait("right", 0.6)
    both(0.6, gl3 + [0.0, over * 0.5, 0.0], gr3, L_OPEN, R_OPEN, "let go")
    both(0.8, gl3 + [0.0, 0.0, 0.05], gr3 + [0, 0, 0.05], L_OPEN, R_OPEN, "rise clear")
    # 3. both arms go home and hold still
    for arm in ("left", "right"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), "back to where it started", f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    if one == "left":  # checks only: the right stays home
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
        plan["right"] = [Waypoint(dur("left"), tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
    if one == "right":  # checks only: the left stays home
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        plan["left"] = [Waypoint(dur("right"), tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
    return plan
