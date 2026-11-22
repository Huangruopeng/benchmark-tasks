"""An ear-examination training manikin (a life-size adult head with a soft outer ear and a 25 mm ear canal) lies on
its right side on a gel head ring, its left ear up. A pocket LED otoscope (Welch Allyn Pocket Plus LED 22880 type),
already switched on, rests on its stand; three 4.25 mm adult disposable specula stand tip-down in a holder.

Task: look into the manikin's left ear canal with the otoscope.

What the physics asks for: the scope's tip takes a speculum only by being pushed into one and turned clockwise until
it locks (the makers' twist-on fit). The outer third of the canal is cartilage: at rest its back-upper wall sits
across the opening, leaving a 4 mm slit that the 4.25 mm speculum can't pass without forcing the wall. Drawing the
outer ear up and back (the textbook adult technique) pulls that wall clear and opens the canal to 11 mm; it springs
back as soon as the ear is let go. So one hand holds the ear up and back while the other inserts the speculum, 6 to 13
mm deep, gently. Afterwards the scope goes back on its stand.

Run it:  inspect-robots run --embodiment yam_arena -E task=look_into_the_manikins_left_ear_canal_with_the_otoscope.py \\
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


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat(m) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix (Shepperd's method)."""
    m = np.asarray(m, float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (float(x / n), float(y / n), float(z / n), float(w / n))


# -- the manikin -------------------------------------------------------------------------------------------------------
# She lies on her right side, left ear up: the ear canal points straight down into her head. Her face points away and
# to the robot's right (A), the crown of her head away and to the left (CR); "up and back" for her ear is world +y.
A_AX = np.array([1.0, -1.0, 0.0]) / math.sqrt(2.0)  # anterior (toward her face)
CR_AX = np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)  # toward the crown of her head
CANAL = np.array([0.300, 0.040, TOP + 0.145])  # the canal's opening (meatus), centre, on her skin
R_OUT = 0.0055  # the cartilaginous (outer) canal: 11 mm across when opened
OUT_D = 0.008  # its depth: the outer third of the 25 mm canal
R_BONE = 0.0035  # the bony canal: 7 mm across
CANAL_D = 0.025  # 25 mm to the eardrum
FOLD0 = -0.0015  # at rest the outer canal's back-upper wall stands 1.5 mm past the canal's axis: a 4 mm slit
PULL = 0.007  # drawing the ear up and back slides that wall 7 mm, to flush with the canal (11 mm open)
FOLD_K, FOLD_PRE = 300.0, 0.012  # its spring: 3.6 N at rest (preload), 5.7 N fully drawn
LEAD = 0.008  # the concha's bowl: a 45-degree funnel 8 mm deep round the opening
RIM_C = np.array([0.0, 0.032, 0.004])  # the helix rim's back-upper part (the grip), base centre, from CANAL
RIM_TILT = math.radians(20.0)  # the ear stands off the head, its rim leaning 20 degrees out (toward +y)
RIM_L, RIM_T, RIM_H = 0.024, 0.010, 0.015  # that rim part: 24 mm long, 10 mm thick, 15 mm tall
RIM_UP = _rx(-RIM_TILT) @ np.array([0.0, 0.0, 1.0])
AURICLE_MASS = 0.020

# -- the otoscope (scope frame: built nose-down as used, the handle along -y; origin at the head's centre) -----------
HEAD_W, HEAD_DY, HEAD_H = 0.030, 0.030, 0.036  # the instrument head, 30 mm wide, with the 15 mm tip cone 51 mm deep
NOSE_L, NOSE_R0, NOSE_R1 = 0.015, 0.004, 0.007  # the tip cone (speculum mount): 8 mm at its end, 14 mm at its base
NOSE_TIP = -(HEAD_H / 2 + NOSE_L)  # -0.033
HANDLE_R, HANDLE_L, HANDLE_Z = 0.0105, 0.138, -0.004  # the 21 mm handle, 138 mm long (16.8 cm overall)
HANDLE_Y0 = -HEAD_DY / 2  # where it meets the head
SCOPE_MASS = 0.118  # 71.7 g head and handle + two AA cells
GRIP_G = 0.100  # the right hand takes the handle 100 mm from the head's axis
SPEC_SEAT = NOSE_TIP + 0.009  # a seated speculum's rim (its origin), 9 mm up the tip cone: -0.024
SPEC_L, SPEC_R_TIP, SPEC_R_RIM, SPEC_R_FL = 0.027, 0.002125, 0.0065, 0.008  # 4.25 mm tip, 13 mm rim, 16 mm flange
SPEC_MASS = 0.003
REST0 = np.array([0.255, -0.140, TOP + 0.069])  # the scope's origin on its stand (handle axis TOP+0.065)
HOLDER0 = np.array([0.175, -0.110, TOP])  # the specula holder block (top at TOP+0.060)
HOLDER_H = 0.060
POCKETS = [np.array([-0.025, 0.0]), np.array([0.0, 0.0]), np.array([0.025, 0.0])]  # three, along x
SPEC0 = [HOLDER0 + [p[0], p[1], HOLDER_H + 0.0015] for p in POCKETS]  # each speculum's rim, its flange on the top

SKIN, SKIN_D, CANAL_C, DRUM_C = (0.87, 0.70, 0.60), (0.78, 0.58, 0.50), (0.32, 0.18, 0.16), (0.74, 0.74, 0.70)
LIP_C = (0.84, 0.64, 0.56)
GEL, SHEET = (0.35, 0.62, 0.86), (0.93, 0.94, 0.95)
SCOPE_BLACK, SCOPE_GREY, LENS_C, LED_C = (0.10, 0.10, 0.11), (0.42, 0.43, 0.45), (0.55, 0.70, 0.80), (0.98, 0.98, 0.85)
SPEC_C, HOLDER_C, STAND_C = (0.20, 0.22, 0.24), (0.88, 0.88, 0.86), (0.30, 0.32, 0.36)

# -- the rules ---------------------------------------------------------------------------------------------------------
LOOK_DEPTH = 0.006  # the speculum's tip 6 mm or more inside the canal (and no deeper than 13 mm): examining
BONY_DEPTH = 0.013  # deeper than 13 mm: into the bony canal (painful): the trial fails
LOOK_S = 1.0  # the look lasts 1 s
FORCE_LIMIT = 3.0  # N, the scope or speculum pressing on her ear or head
FORCE_HOLD_S = 0.1
ENGAGE = (0.006, 0.016)  # the tip cone this far into a speculum (and within 2.5 mm of its axis) can lock it
ENGAGE_LAT = 0.0035
LOCK_TURN = 20.0  # turned this far clockwise (seen from the eyepiece) while engaged: locked
REST_WITHIN = 0.015  # the scope's origin this close to where it started, nose down within 15 degrees: on its stand
EAR_FREE = 0.001  # the ear back within 1 mm of rest: let go
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
PHYSICS_DT = 1.0 / 240.0

# extra visual-only shapes drawn with the bodies (ellipsoids and capsules), by body name
_VISUALS: dict[str, list] = {}


def _ell(body, centre, radii, color, rot=None):
    _VISUALS.setdefault(body, []).append(("ell", tuple(map(float, centre)), tuple(map(float, radii)), color,
                                          _quat(np.eye(3) if rot is None else rot)))


def _cap(body, p1, p2, r, color):
    a, b = np.asarray(p1, float), np.asarray(p2, float)
    d = b - a
    ln = float(np.linalg.norm(d))
    d /= ln
    ref = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(ref, d)
    x /= np.linalg.norm(x)
    y = np.cross(d, x)
    _VISUALS.setdefault(body, []).append(("cap", tuple(((a + b) / 2).tolist()), (float(r), ln), color,
                                          _quat(np.column_stack([x, y, d]))))


def _box(lo, hi, color, collide=True, rot=None) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if rot is None:
        return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)
    return BoxShape(tuple((rot @ ((lo + hi) / 2)).tolist()), tuple((hi - lo).tolist()), color, collide=collide,
                    quat_xyzw=_quat(rot))


def _plate(p1, p2, width, normal, color, t=0.002, collide=True) -> BoxShape:
    """A plate whose face (on the ``normal`` side) runs along the line p1-p2, ``width`` wide."""
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    a = np.asarray(p1, float) - n * t / 2
    b = np.asarray(p2, float) - n * t / 2
    d = b - a
    ln = float(np.linalg.norm(d))
    d /= ln
    n = n - np.dot(n, d) * d
    n /= np.linalg.norm(n)
    tt = np.cross(d, n)
    return BoxShape(tuple(((a + b) / 2).tolist()), (t, width, ln), color, collide=collide,
                    quat_xyzw=_quat(np.column_stack([n, tt, d])))


def _ear_pt(a, c, z=0.0):
    """A point on her ear's side of the head: a toward her face, c toward her crown, z out of the head (up)."""
    return a * A_AX + c * CR_AX + np.array([0.0, 0.0, z])


def _head() -> Body:
    """The manikin's head with its fixed parts of the ear: the skin round the ear, the canal's front-lower half (the
    concha's funnel and the cartilaginous canal's fixed walls), the 7 mm bony canal and the eardrum at 25 mm. Fixed.
    Origin: the canal's opening."""
    sh = []
    r, f = R_OUT, R_OUT + LEAD
    # the skin round the ear (a 5 mm shell with the funnel's opening in it), under the outer ear
    for lo, hi in (((-0.045, -0.045), (-f, 0.050)), ((f, -0.045), (0.045, 0.050)), ((-f, -0.045), (f, -f)),
                   ((-f, f + 0.012), (f, 0.050))):
        sh.append(_box((lo[0], lo[1], -0.005), (hi[0], hi[1], 0.0), None))
    # the concha's funnel on the canal's front-lower side and its two sides (the back-upper side is on the ear)
    for nrm, p1, p2 in (((0, 1, 1), (0, -f, LEAD), (0, -r, 0)), ((1, 0, 1), (-f, 0, LEAD), (-r, 0, 0)),
                        ((-1, 0, 1), (f, 0, LEAD), (r, 0, 0))):
        sh.append(_plate(p1, p2, 2 * f, nrm, None, t=0.003))
    # the cartilaginous canal's fixed walls: the front-lower wall and the two sides (the moving wall slides between)
    t = 0.003
    sh.append(_box((-r - t, -r - t, -OUT_D), (r + t, -r, 0.0), None))
    sh.append(_box((-r - t, -r, -OUT_D), (-r, r + 0.004, 0.0), None))
    sh.append(_box((r, -r, -OUT_D), (r + t, r + 0.004, 0.0), None))
    # the step down to the bony canal, and the bony canal (7 mm), and the eardrum closing it
    b = R_BONE
    for lo, hi in (((-r, -r), (-b, r)), ((b, -r), (r, r)), ((-b, -r), (b, -b)), ((-b, b), (b, r))):
        sh.append(_box((lo[0], lo[1], -OUT_D - 0.002), (hi[0], hi[1], -OUT_D), None))
    for lo, hi in (((-b - t, -b - t), (-b, b + t)), ((b, -b - t), (b + t, b + t)), ((-b, -b - t), (b, -b)),
                   ((-b, b), (b, b + t))):
        sh.append(_box((lo[0], lo[1], -CANAL_D), (hi[0], hi[1], -OUT_D - 0.002), None))
    sh.append(_box((-b, -b, -CANAL_D - 0.002), (b, b, -CANAL_D), None))
    # drawn: the canal's dark opening and walls, the eardrum
    _ell("head", (0.0, 0.0, -0.0003), (R_OUT, R_OUT, 0.0002), CANAL_C)
    _cap("head", (0.0, 0.0, -OUT_D), (0.0, 0.0, -0.001), R_OUT - 0.0005, CANAL_C)
    _ell("head", (0.0, 0.0, -CANAL_D + 0.0005), (R_BONE, R_BONE, 0.0004), DRUM_C)
    # the concha's floor (front-lower half) round the opening
    _ell("head", _ear_pt(0.004, -0.004, 0.0005), (0.012, 0.010, 0.0012), SKIN_D, np.column_stack([A_AX, CR_AX, [0, 0, 1]]))
    # her head (lying on its right side), face, neck stump
    hrot = np.column_stack([A_AX, CR_AX, [0.0, 0.0, 1.0]])
    hc = _ear_pt(0.012, 0.0, -0.072)
    _ell("head", hc, (0.098, 0.106, 0.070), SKIN, hrot)
    _ell("head", hc + _ear_pt(0.093, -0.012, 0.0), (0.020, 0.013, 0.012), SKIN, hrot)  # nose
    _ell("head", hc + _ear_pt(0.060, -0.075, 0.0), (0.032, 0.030, 0.048), SKIN, hrot)  # jaw and chin
    _ell("head", hc + _ear_pt(0.070, 0.020, 0.040), (0.016, 0.010, 0.006), SKIN_D, hrot)  # brow
    _cap("head", hc + _ear_pt(0.000, -0.075, 0.0), hc + _ear_pt(-0.004, -0.100, 0.0), 0.045, SKIN)  # neck stump
    _ell("head", hc + _ear_pt(-0.004, -0.102, 0.0), (0.043, 0.004, 0.043), SKIN_D, hrot)  # its cut end
    return Body.build("head", pos=tuple(CANAL.tolist()), mass=3.0, kinematic=True, shapes=sh, color=SKIN,
                      friction=(0.9, 0.8))


def _ring() -> Body:
    """The gel head ring the head lies on, on the bench sheet. Fixed; drawn only."""
    hc = CANAL + _ear_pt(0.012, 0.0, -0.072)
    sh = [BoxShape((0.0, 0.0, 0.0005), (0.50, 0.42, 0.001), SHEET, collide=False)]
    _ell("ring", (hc[0] - 0.24, hc[1] - 0.10 + 0.0, 0.006), (0.09, 0.09, 0.008), GEL)
    return Body.build("ring", pos=(0.24, 0.10, TOP), mass=1.0, kinematic=True, shapes=sh, color=SHEET)


def _auricle() -> Body:
    """The left outer ear (auricle) with the cartilaginous canal's back-upper wall: one soft part that slides up and
    back (world +y) on a spring, 7 mm at most. At rest that wall closes the canal's opening to a 4 mm slit. Origin:
    the canal's opening (at rest)."""
    sh = []
    r = R_OUT
    # the moving wall of the cartilaginous canal, its face 1.5 mm past the axis at rest, and its bowl lip (45 degrees)
    sh.append(_box((-r + 0.0002, FOLD0, -OUT_D), (r - 0.0002, FOLD0 + 0.003, 0.0), LIP_C))
    sh.append(_plate((0.0, FOLD0, 0.0), (0.0, FOLD0 + LEAD, LEAD), 2 * r - 0.0004, (0, -1, 1), LIP_C, t=0.003))
    # the concha's back-upper floor and the antihelix up to the rim (solid, low)
    sh.append(_box((-0.012, FOLD0 + 0.003, 0.0), (0.012, RIM_C[1] - RIM_T / 2, 0.005), SKIN_D))
    # the helix rim's back-upper part: the grip, leaning out 20 degrees
    rot = _rx(-RIM_TILT)
    lo = np.array([-RIM_L / 2, -RIM_T / 2, 0.0])
    hi = np.array([RIM_L / 2, RIM_T / 2, RIM_H])
    sh.append(BoxShape(tuple((RIM_C + rot @ ((lo + hi) / 2)).tolist()), tuple((hi - lo).tolist()), SKIN,
                       quat_xyzw=_quat(rot)))
    # drawn: the rest of the helix, antihelix, tragus, antitragus and lobe round the concha
    pts = [(0.010, 0.016, 0.004), (0.000, 0.028, 0.008), (-0.013, 0.031, 0.012), (-0.024, 0.024, 0.015),
           (-0.031, 0.010, 0.015), (-0.031, -0.006, 0.013), (-0.025, -0.020, 0.010), (-0.014, -0.030, 0.007)]
    for p1, p2 in zip(pts[:-1], pts[1:]):
        _cap("auricle", _ear_pt(*p1), _ear_pt(*p2), 0.0045, SKIN)
    hrot = np.column_stack([A_AX, CR_AX, [0.0, 0.0, 1.0]])
    _ell("auricle", _ear_pt(-0.014, 0.004, 0.004), (0.010, 0.024, 0.006), SKIN_D, hrot)  # scapha and fossa
    _cap("auricle", _ear_pt(-0.018, -0.016, 0.006), _ear_pt(-0.012, 0.020, 0.007), 0.0032, SKIN)  # antihelix
    _cap("auricle", _ear_pt(-0.012, 0.014, 0.004), _ear_pt(0.006, 0.010, 0.002), 0.0026, SKIN)  # crus of the helix
    _ell("auricle", _ear_pt(-0.004, -0.002, 0.0008), (0.013, 0.014, 0.0012), SKIN_D, hrot)  # concha
    _ell("auricle", _ear_pt(-0.010, -0.034, 0.004), (0.012, 0.010, 0.005), SKIN, hrot)  # lobe
    _ell("auricle", _ear_pt(-0.006, -0.020, 0.003), (0.006, 0.004, 0.003), SKIN, hrot)  # antitragus
    _ell("head", _ear_pt(0.011, -0.002, 0.004), (0.004, 0.007, 0.005), SKIN, hrot)  # tragus (fixed, in front)
    return Body.build("auricle", pos=tuple(CANAL.tolist()), mass=AURICLE_MASS, shapes=sh, color=SKIN,
                      friction=(1.2, 1.0), solver_iterations=32, damping=(2.0, 2.0))


def _scope() -> Body:
    """The pocket LED otoscope, switched on: its head (30 mm wide, 51 mm deep with the tip cone that takes a
    speculum) on a 21 mm, 138 mm handle holding two AA cells; 118 g. Built nose-down, the handle along -y. Origin: the
    head's centre."""
    sh = [BoxShape((0.0, 0.0, 0.0), (HEAD_W, HEAD_DY, HEAD_H), SCOPE_BLACK)]
    n = 4
    for k in range(n):  # the tip cone, in steps
        z0 = -HEAD_H / 2 - NOSE_L * k / n
        rr = NOSE_R1 + (NOSE_R0 - NOSE_R1) * (k + 0.5) / n
        sh.append(CylShape((0.0, 0.0, z0 - NOSE_L / n / 2), rr, NOSE_L / n, SCOPE_BLACK))
    sh.append(CylShape((0.0, HANDLE_Y0 - HANDLE_L / 2, HANDLE_Z), HANDLE_R, HANDLE_L, SCOPE_BLACK, axis="Y"))
    sh.append(CylShape((0.0, HANDLE_Y0 - 0.004, HANDLE_Z), HANDLE_R + 0.0012, 0.008, SCOPE_GREY, axis="Y",
                       collide=False))  # the on/off and brightness dial at the head's base
    sh.append(BoxShape((0.0, HANDLE_Y0 - 0.060, HANDLE_Z + HANDLE_R + 0.001), (0.006, 0.050, 0.002), SCOPE_GREY,
                       collide=False))  # pocket clip
    sh.append(CylShape((0.0, 0.0, HEAD_H / 2 + 0.0005), 0.007, 0.001, LENS_C, collide=False))  # the viewing lens
    sh.append(CylShape((0.0, 0.0, NOSE_TIP - 0.0002), 0.0022, 0.0006, LED_C, collide=False))  # the lit LED
    com_y = (0.040 * 0.0 + 0.078 * (HANDLE_Y0 - 0.060)) / SCOPE_MASS
    return Body.build("scope", pos=tuple(REST0.tolist()), mass=SCOPE_MASS, shapes=sh, color=SCOPE_BLACK,
                      friction=(1.2, 1.0), solver_iterations=32, damping=(0.05, 0.2), com=(0.0, float(com_y), -0.008))


def _speculum(i: int) -> Body:
    """A disposable 4.25 mm adult ear speculum (polypropylene, black): a 27 mm cone from a 13 mm rim with a 16 mm
    flange to a 4.25 mm tip. Built tip-down. Origin: the rim's centre. It never collides with the scope: its inside
    is where the tip cone goes."""
    sh = [CylShape((0.0, 0.0, -0.00075), SPEC_R_FL, 0.0015, SPEC_C)]
    n = 6
    for k in range(n):
        rr = SPEC_R_RIM + (SPEC_R_TIP - SPEC_R_RIM) * (k + 0.5) / n
        sh.append(CylShape((0.0, 0.0, -SPEC_L * (k + 0.5) / n), rr, SPEC_L / n, SPEC_C))
    return Body.build(f"spec{i}", pos=tuple(SPEC0[i].tolist()), mass=SPEC_MASS, shapes=sh, color=SPEC_C,
                      friction=(0.8, 0.7), solver_iterations=32, damping=(0.05, 0.2), ignore=("scope",))


def _holder() -> Body:
    """The specula holder: a 70 x 30 x 60 mm block with three 15 mm pockets, each speculum hanging tip-down by its
    flange. Fixed. Origin: the block's footprint centre on the bench."""
    sh = []
    w, h, pk = 0.035, 0.015, 0.0075
    xs = [p[0] for p in POCKETS]
    edges = [-w] + [v for x in xs for v in (x - pk, x + pk)] + [w]
    for x0, x1 in zip(edges[0::2], edges[1::2]):  # the solid between the pockets
        sh.append(_box((x0, -h, 0.0), (x1, h, HOLDER_H), HOLDER_C))
    for x in xs:
        sh.append(_box((x - pk, -h, 0.0), (x + pk, -pk, HOLDER_H), HOLDER_C))
        sh.append(_box((x - pk, pk, 0.0), (x + pk, h, HOLDER_H), HOLDER_C))
        sh.append(_box((x - pk, -pk, 0.0), (x + pk, pk, HOLDER_H - SPEC_L - 0.003), HOLDER_C))  # the pocket's floor
    return Body.build("holder", pos=tuple(HOLDER0.tolist()), mass=0.3, kinematic=True, shapes=sh, color=HOLDER_C)


def _stand() -> Body:
    """The otoscope's stand: a base plate with two V-rests holding the handle level and two cheeks either side of the
    head's tip cone, the head hanging nose-down over the plate's end. Fixed. Origin: the base's centre under the head."""
    sh = [_box((-0.025, -0.165, 0.0), (0.025, 0.020, 0.006), STAND_C)]
    hz = REST0[2] + HANDLE_Z - TOP  # the handle's axis above the bench
    for yc in (-0.045, -0.140):
        for s in (-1, 1):  # two 45-degree faces under the handle
            p1 = np.array([s * HANDLE_R * 0.7071, yc, hz - HANDLE_R * 0.7071])
            sh.append(_plate(p1 - [s * 0.007, 0, 0.007], p1 + [s * 0.007, 0, 0.007], 0.012, (-s, 0, 1), STAND_C,
                             t=0.004))
        sh.append(_box((-0.016, yc - 0.006, 0.006), (0.016, yc + 0.006, hz - HANDLE_R - 0.006), STAND_C))
    for s in (-1, 1):  # two cheeks either side of the hanging head's tip cone keep it nose-down
        sh.append(_box((min(s * 0.012, s * 0.015), -0.012, 0.006), (max(s * 0.012, s * 0.015), 0.012, 0.048), STAND_C))
    return Body.build("stand", pos=(float(REST0[0]), float(REST0[1]), TOP), mass=0.4, kinematic=True, shapes=sh,
                      color=STAND_C)


bodykit.slider("ear_pull", "head", "auricle", tuple(CANAL.tolist()), axis="Y", limits=(0.0, PULL),
               stiffness=FOLD_K, damping=4.0, target=-FOLD_PRE, max_force=40.0)
OBJECTS = [_ring(), _head(), _auricle(), _stand(), _holder(), _scope()] + [_speculum(i) for i in range(3)]
SPECS = tuple(f"spec{i}" for i in range(3))

# -- extra visuals, contact sensing, and the specula's twist-on locks (authored with the bodies) ------------------------
_orig_spawn = bodykit._spawn_body
_LOCKS: dict[str, str] = {}
_LOCK_ATTR: dict[str, object] = {}
_VIEW: dict = {"env": None, "view": None, "failed": ""}


def _spawn_with_extras(prim_path, cfg, translation=None, orientation=None, **kwargs):
    prim = _orig_spawn(prim_path, cfg, translation, orientation, **kwargs)
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdPhysics

    stage = get_current_stage()
    name = cfg.body_name
    if name in ("head", "auricle"):  # their contacts are read: how hard the scope presses on her ear or head
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body")).CreateThresholdAttr(0.0)
        _VIEW["env"] = prim_path.rsplit("/", 1)[0]
    if name in SPECS:  # the twist-on lock: a fixed joint to the scope's tip cone, seated; switched off until locked
        parent = prim_path.rsplit("/", 1)[0]
        joint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/lock")
        joint.CreateBody0Rel().SetTargets([f"{parent}/scope/body"])
        joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(SPEC_SEAT)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateJointEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        _LOCKS[name] = f"{prim_path}/lock"
        _LOCK_ATTR.pop(name, None)
    items = _VISUALS.get(name)
    if items:
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from pxr import UsdGeom

        from yam_arena.station import linear_rgb

        looks = {}
        for i, it in enumerate(items):
            kind, centre, dims, color, q = it
            path = f"{prim_path}/body/v{i}"
            if kind == "ell":
                g = UsdGeom.Sphere.Define(stage, path)
                g.CreateRadiusAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(*centre))
                g.AddOrientOp().Set(Gf.Quatf(q[3], q[0], q[1], q[2]))
                g.AddScaleOp().Set(Gf.Vec3f(*dims))
            else:
                g = UsdGeom.Capsule.Define(stage, path)
                g.CreateRadiusAttr(dims[0])
                g.CreateHeightAttr(dims[1])
                g.CreateAxisAttr("Z")
                g.AddTranslateOp().Set(Gf.Vec3d(*centre))
                g.AddOrientOp().Set(Gf.Quatf(q[3], q[0], q[1], q[2]))
            if color not in looks:
                lp = f"{prim_path}/Looks/v{len(looks)}"
                surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.6)
                surface.func(lp, surface)
                looks[color] = lp
            bind_visual_material(path, looks[color], stage=stage)
    return prim


bodykit._spawn_body = _spawn_with_extras


def _set_lock(name: str, on: bool) -> None:
    if name not in _LOCKS:
        return
    if name not in _LOCK_ATTR:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        _LOCK_ATTR[name] = UsdPhysics.Joint(get_current_stage().GetPrimAtPath(_LOCKS[name])).GetJointEnabledAttr()
    _LOCK_ATTR[name].Set(bool(on))


def _contact_force() -> float:
    """The total contact force (N) between the scope with its specula and her head or ear, this step."""
    if _VIEW["view"] is None and not _VIEW["failed"] and _VIEW["env"]:
        try:
            from isaaclab_physx.physics import PhysxManager

            env = _VIEW["env"]
            parts = [f"{env}/{n}/body" for n in ("scope",) + SPECS]
            _VIEW["view"] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                [f"{env}/head/body", f"{env}/auricle/body"], filter_patterns=[parts, parts], max_contact_data_count=0)
        except Exception as e:  # noqa: BLE001
            _VIEW["failed"] = repr(e)[:300]
            print(f"[oto] contact view failed: {_VIEW['failed']}", file=sys.stderr, flush=True)
    if _VIEW["view"] is None:
        return 0.0
    m = _VIEW["view"].get_contact_force_matrix(dt=PHYSICS_DT)
    m = np.asarray(m.numpy() if hasattr(m, "numpy") and not hasattr(m, "detach") else as_numpy(m), float)
    return float(np.linalg.norm(m.reshape(-1, 3), axis=1).sum())


# -- followed after every control step --------------------------------------------------------------------------------


def _fresh() -> dict:
    return {"locked": None, "engaged": {}, "bare_in": False, "deep": False, "forced": False, "dropped": False,
            "why": "", "look_t": 0.0, "looked": False, "in_canal": False, "max_depth": 0.0, "max_force": 0.0,
            "over_t": 0.0, "tcps": None, "fingers": None, "s": {}, "pull": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("OTO_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    for n in SPECS:
        _set_lock(n, False)
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _turn_about(r0, r1, axis_local=(0.0, 0.0, 1.0)) -> float:
    """Degrees r1 is turned from r0 about the body's own z axis (positive anticlockwise seen from +z)."""
    rel = r0.T @ r1
    return math.degrees(math.atan2(rel[1, 0] - rel[0, 1], rel[0, 0] + rel[1, 1]))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    ps, rs = _pose(base, origin, "scope")
    nose = ps + rs @ np.array([0.0, 0.0, NOSE_TIP])
    axis = rs @ np.array([0.0, 0.0, 1.0])  # the scope's eyepiece direction (the tip cone points along -axis)
    # the twist-on fit: a speculum is engaged while the tip cone is 6-16 mm into it, on its axis; turning the scope 20
    # degrees clockwise (seen from the eyepiece) while engaged locks it on, seated
    if _EP["locked"] is None:
        for n in SPECS:
            pp, rp = _pose(base, origin, n)
            sax = rp @ np.array([0.0, 0.0, 1.0])
            rel = nose - pp
            depth = -float(np.dot(rel, sax))
            lat = float(np.linalg.norm(rel + depth * sax))
            ok = ENGAGE[0] <= depth <= ENGAGE[1] and lat <= ENGAGE_LAT and float(np.dot(axis, sax)) > math.cos(
                math.radians(10.0))
            if ok:
                if n not in _EP["engaged"]:
                    _EP["engaged"][n] = rs.copy()
                elif _turn_about(_EP["engaged"][n], rs) <= -LOCK_TURN:
                    _EP["locked"] = n
                    _set_lock(n, True)
                    break
            else:
                _EP["engaged"].pop(n, None)
    # where the locked speculum's tip is in her canal
    depth, inside = 0.0, False
    if _EP["locked"] is not None:
        pp, rp = _pose(base, origin, _EP["locked"])
        tip = pp + rp @ np.array([0.0, 0.0, -SPEC_L]) - CANAL
        inside = math.hypot(tip[0], tip[1]) < R_OUT + 0.001 and tip[2] < 0.0
        depth = -float(tip[2]) if inside else 0.0
    else:  # the bare tip cone in her canal (no speculum): never allowed
        tip = nose - CANAL
        if math.hypot(tip[0], tip[1]) < R_OUT + 0.002 and tip[2] < -0.001 and not _EP["bare_in"]:
            _EP["bare_in"], _EP["why"] = True, "the bare tip went into the canal without a speculum"
    _EP["max_depth"] = max(_EP["max_depth"], depth)
    if depth > BONY_DEPTH and not _EP["deep"]:
        _EP["deep"], _EP["why"] = True, f"speculum pushed {depth * 1000:.1f} mm in, into the bony canal"
    looking = LOOK_DEPTH <= depth <= BONY_DEPTH
    _EP["in_canal"] = _EP["in_canal"] or looking
    _EP["look_t"] = _EP["look_t"] + 0.05 if looking else 0.0
    if _EP["look_t"] >= LOOK_S - 1e-6:
        _EP["looked"] = True
    force = _contact_force()
    _EP["max_force"] = max(_EP["max_force"], force)
    _EP["over_t"] = _EP["over_t"] + 0.05 if force > FORCE_LIMIT else 0.0
    if _EP["over_t"] >= FORCE_HOLD_S - 1e-6 and not _EP["forced"]:
        _EP["forced"], _EP["why"] = True, f"pressed her ear with {force:.1f} N"
    if ps[2] < TOP + 0.045 and not _EP["dropped"]:
        _EP["dropped"], _EP["why"] = True, "the otoscope fell"
    pa, _ = _pose(base, origin, "auricle")
    _EP["pull"] = float(pa[1] - CANAL[1])
    _EP["tcps"] = _tcps(base, origin)
    _EP["fingers"] = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")])
                      for a in ("left_robot", "right_robot")]
    on_stand = (float(np.linalg.norm(ps - REST0)) < REST_WITHIN and float(axis[2]) > math.cos(math.radians(15.0))
                and not _near_tcp(ps + rs @ np.array([0.0, -GRIP_G, HANDLE_Z]), 0.04))
    _EP["s"] = {"on_stand": on_stand, "ear_free": _EP["pull"] < EAR_FREE, "depth": depth}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round(np.asarray(v) * 1000, 1).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            grip = rs.T @ (tr - ps)
            rim = CANAL + RIM_C + RIM_UP * RIM_H + [0.0, _EP["pull"], 0.0]
            print(f"[oto] t={_DEBUG['steps'] / 20:5.2f}s f={[round(v * 1000, 1) for v in _EP['fingers']]}"
                  f" scope={mm(ps - [0, 0, TOP])} ax={np.round(axis, 3).tolist()} gripR_in_scope={mm(grip)}"
                  f" nose={mm(nose - CANAL)} eng={ {k: round(_turn_about(v, rs), 1) for k, v in _EP['engaged'].items()} }"
                  f" lock={_EP['locked']} depth={depth * 1000:.1f} pull={_EP['pull'] * 1000:.1f}"
                  f" Rtcp={mm(tr - CANAL)} Ltcp={mm(tl - CANAL)} L-rim={mm(tl - rim)} wr={np.linalg.norm(tl - tr) * 1000:.0f} F={force:.2f} Fmax={_EP['max_force']:.2f}"
                  f" look={_EP['look_t']:.2f} s={_EP['s']} {_EP['why']} score={score(None)}", file=sys.stderr, flush=True)


def _near_tcp(point, within) -> bool:
    tcps, fj = _EP.get("tcps"), _EP.get("fingers")
    if tcps is None:
        return False
    for t, f in zip(tcps, fj):
        if float(np.linalg.norm(t - point)) < within and f > FINGER_OPEN * 0.15:
            return True
    return False


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if at any time the scope or a speculum pressed on her ear or head harder than 3 N (for 0.1 s), the
    speculum's tip went deeper than 13 mm, the bare tip cone (no speculum) went into her canal, or the otoscope fell.
    Otherwise: 1.0: a speculum locked on, kept 6-13 mm inside her canal for 1 s, then the otoscope back on its stand
    and let go, her ear let go (back at rest) and both arms back where they started; 0.75: the 1 s look done; 0.5:
    the speculum 6-13 mm inside her canal at some point; 0.25: a speculum locked on the scope; else 0.0."""
    s = _EP["s"]
    if not s or _EP["forced"] or _EP["deep"] or _EP["bare_in"] or _EP["dropped"]:
        return 0.0
    if _EP["locked"] is None:
        return 0.0
    if not _EP["in_canal"]:
        return 0.25
    if not _EP["looked"]:
        return 0.5
    return 1.0 if (s["on_stand"] and s["ear_free"] and s["depth"] == 0.0 and _arms_home()) else 0.75


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
    """Full score with the otoscope and her ear at rest: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("scope", "auricle"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand takes the otoscope by its handle (fingers across it, the hand leaning in from the robot's side), pushes
# its tip into a speculum in the holder and turns it 30 degrees clockwise to lock it on, and lifts it out. The left
# hand pinches the helix rim at the back-upper part of the ear and draws it up and back to its stop. The right brings
# the speculum down the canal's axis, 8 mm in, holds it there 2 s, and draws it out; the left lets the ear go; the
# scope goes back on its stand; arms home.

def _scope_rot(turn_deg=0.0):
    return _rz(math.radians(turn_deg))


G_S = _ry(math.radians(-float(os.environ.get("OTO_RLEAN", 35.0)))) @ grip_rotation(math.pi / 2)  # world, scope unturned
S_GRIP = np.array([0.0, -GRIP_G, HANDLE_Z + 0.0065])  # where the right TCP takes the handle (scope frame): above its axis
S_HELD = np.array([float(os.environ.get("OTO_SHX", 0.0)), -GRIP_G + float(os.environ.get("OTO_SHY", 0.0)),
                   HANDLE_Z + float(os.environ.get("OTO_SHZ", 0.0))])  # measured hold
G_E = _rx(-RIM_TILT) @ grip_rotation(0.0)  # the left grip on the rim: closing along y, leaning with the rim
E_OPEN = -(RIM_T / 2 + 0.006)
S_OPEN = -(HANDLE_R + 0.007)
LEAD_S, LEAD_STEPS = 3.0, 15
LOOK_AT = float(os.environ.get("OTO_DEPTH", 0.008))  # the speculum's tip this deep while looking
ENG_AT = float(os.environ.get("OTO_ENG", 0.010))  # the tip cone this far into the speculum to lock it
OVERPULL = float(os.environ.get("OTO_OVERPULL", 0.030))  # the ear hand aims this far past the ear's stop


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


def _joint_path(arm: str, q0, q1, label: str, finger, t=LEAD_S, n=LEAD_STEPS) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, n + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[arm])
        out.append(Waypoint(t / n, tuple(float(v) for v in p), finger, label, r))
    return out


def _cal(name, default):
    v = os.environ.get(name)
    return np.array([float(x) for x in v.split(",")]) if v else np.array(default, float)


# measured: where the held scope's tip ends up against where it was aimed (the arm's give, the scope's tilt in the
# grip), at the holder and over her ear; the hand aims that much the other way
CAL_HOLDER = _cal("OTO_CALH", (0.0061, 0.0034, 0.012))
CAL_EAR = _cal("OTO_CALE", (0.0057, 0.0032, 0.0144))
CAL_TURN = _cal("OTO_CALT", (0.0, -0.0061, 0.0))  # and, turned 30 degrees clockwise at the holder, this much more


def _hand_on_scope(origin, turn_deg=0.0, held=True, cal=(0.0, 0.0, 0.0)):
    """The right TCP pose holding the scope whose origin is at ``origin``, turned ``turn_deg`` about its axis."""
    r = _scope_rot(turn_deg)
    return np.asarray(origin, float) + r @ (S_HELD if held else S_GRIP) + np.asarray(cal, float), r @ G_S


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_CLOSED, np.asarray(r, float))
    qcur = {"left": np.array(READY, float), "right": np.array(READY, float)}
    one = os.environ.get("OTO_ONEHAND")  # checks only: "left" or "right" works alone, the other stays home

    def to(arm, t, pose, finger, label, n=1, joint=False):
        p1, r1 = np.asarray(pose[0], float), np.asarray(pose[1], float)
        if joint:
            q1 = _good_ik(arm, p1, r1)
            plan[arm].extend(_joint_path(arm, qcur[arm], q1, label, finger, t, max(n, 8)))
            qcur[arm] = q1
        else:
            p0 = cur[arm][0]
            for j in range(1, n + 1):
                plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, r1))
            qcur[arm] = _good_ik(arm, p1, r1)
        cur[arm] = (p1, finger, r1)

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

    up = np.array([0.0, 0.0, 1.0])
    # 1. the right hand takes the otoscope off its stand by the handle
    p, r = _hand_on_scope(REST0, held=False)
    to("right", LEAD_S, (p + up * 0.06, r), S_OPEN, "over the otoscope's handle", joint=True)
    to("right", 1.2, (p, r), S_OPEN, "down round the handle", n=2)
    to("right", 0.6, (p, r), FINGER_CLOSED, "take the handle")
    wait("right", 0.3)
    lift = REST0 + up * 0.06
    to("right", 1.2, _hand_on_scope(lift), FINGER_CLOSED, "lift it off the stand", n=2)
    # 2. push its tip into the middle speculum and turn it clockwise to lock it on
    sp = SPEC0[1]
    twist = float(os.environ.get("OTO_TWIST", 30.0))  # checks only: other turns
    nospec = bool(os.environ.get("OTO_NOSPEC"))  # checks only: skip the speculum
    over = sp + up * (0.030 - NOSE_TIP)  # the tip cone 30 mm above the speculum's rim
    eng = sp + up * (-NOSE_TIP - ENG_AT)
    if nospec:
        over, eng, twist = lift, lift, 0.0
    to("right", 2.0, _hand_on_scope(over, cal=CAL_HOLDER), FINGER_CLOSED, "over a speculum in the holder", joint=True)
    to("right", 1.5, _hand_on_scope(eng, cal=CAL_HOLDER), FINGER_CLOSED, "push the tip into the speculum", n=3)
    wait("right", 0.3)
    nt = 8
    for k in range(1, nt + 1):  # turn about the speculum's axis: the hand swings round it
        to("right", 2.4 / nt, _hand_on_scope(eng - up * 0.001, -twist * k / nt,
                                             cal=CAL_HOLDER + CAL_TURN * twist * k / nt / 30.0), FINGER_CLOSED,
           "turn it clockwise until the speculum locks")
    wait("right", 0.3)
    out = sp + up * (0.050 - NOSE_TIP)
    to("right", 1.5, _hand_on_scope(out, -twist, cal=CAL_HOLDER + CAL_TURN * twist / 30.0), FINGER_CLOSED, "lift it out with the speculum on", n=3)
    to("right", 1.2, _hand_on_scope(out, 0.0), FINGER_CLOSED, "turn it back square", n=3)
    # 3. as the speculum goes on, the left hand pinches the helix rim and draws the ear up and back to its stop
    rim_top = CANAL + RIM_C + RIM_UP * RIM_H
    ez = -(G_E @ np.array([0.0, 0.0, 1.0]))  # back along the approach
    pin = rim_top + ez * 0.004  # the TCP 4 mm above the rim's top: the fingers pinch it 9 mm deep
    wait("left", max(0.0, dur("right") - 6.6), "wait while the speculum goes on")  # the ear is drawn just as it ends
    to("left", LEAD_S, (pin + ez * 0.03, G_E), E_OPEN, "over the rim of her left ear", joint=True)
    to("left", 1.2, (pin, G_E), E_OPEN, "down round the helix rim", n=2)
    to("left", 0.6, (pin, G_E), FINGER_CLOSED, "pinch the rim")
    wait("left", 0.3)
    pulled = pin + np.array([0.0, PULL + OVERPULL, 0.0])
    to("left", 1.5, (pulled, G_E), FINGER_CLOSED, "draw the ear up and back: the canal opens", n=3)
    sync()
    # 4. the right brings the speculum down the canal's axis, in 8 mm, and holds it there
    org = lambda d: CANAL + up * (-SPEC_SEAT + SPEC_L - d)  # noqa: E731  the scope's origin for a tip d deep
    to("right", 2.5, _hand_on_scope(org(-0.035), cal=CAL_EAR), FINGER_CLOSED, "over her ear canal", joint=True)
    to("right", 1.2, _hand_on_scope(org(-0.008), cal=CAL_EAR), FINGER_CLOSED, "down to the opening", n=3)
    to("right", 1.5, _hand_on_scope(org(LOOK_AT), cal=CAL_EAR), FINGER_CLOSED, "in along the canal, 8 mm", n=5)
    wait("right", float(os.environ.get("OTO_LOOKT", 2.0)), "look: hold it there")
    if os.environ.get("OTO_EARLY"):  # checks only: the ear is let go with the speculum still in
        wait("left", dur("right") - dur("left") - 1.0)
        to("left", 0.4, (pin, G_E), E_OPEN, "let go of the ear (too early)")
    to("right", 1.5, _hand_on_scope(org(-0.025), cal=CAL_EAR), FINGER_CLOSED, "draw it back out", n=4)
    sync()
    # 5. the left lets the ear go; the right puts the otoscope back on its stand
    to("left", 0.8, (pin, G_E), FINGER_CLOSED, "ease the ear back")
    to("left", 0.6, (pin, G_E), E_OPEN, "let go of the ear")
    to("left", 1.0, (pin + ez * 0.03, G_E), E_OPEN, "rise clear")
    if os.environ.get("OTO_NORETURN"):  # checks only: the scope is kept in hand, held up
        wait("right", 4.0)
        sync()
        return _finish(plan, cur, qcur, one, dur)
    to("right", 2.5, _hand_on_scope(lift), FINGER_CLOSED, "carry the otoscope back over its stand", joint=True)
    to("right", 1.2, _hand_on_scope(REST0 + up * 0.010), FINGER_CLOSED, "set it on its stand", n=2)
    to("right", 0.6, (cur["right"][0], cur["right"][2]), S_OPEN, "let go")
    to("right", 1.0, (cur["right"][0] + up * 0.06, cur["right"][2]), S_OPEN, "rise clear")
    sync()
    return _finish(plan, cur, qcur, one, dur)


def _finish(plan, cur, qcur, one, dur):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    # 6. both arms go home and hold still (in a check that keeps the scope, only the left arm)
    keep = bool(os.environ.get("OTO_NORETURN"))
    for arm in (("left",) if keep else ("left", "right")):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, qcur[arm], np.array(READY, float), "back to where it started", f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    if one in ("left", "right"):  # checks only: the other arm stays home
        other = "right" if one == "left" else "left"
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[other])
        plan[other] = [Waypoint(dur(one), tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
    return plan
