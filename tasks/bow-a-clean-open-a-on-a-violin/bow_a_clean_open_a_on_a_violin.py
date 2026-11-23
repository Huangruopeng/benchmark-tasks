"""A full-size (4/4) student violin lies on its back on a felt cloth, its scroll to the robot's left, a clip-on tuner on
the scroll; its bow lies on the cloth to the right, hair down, the frog toward the robot.

Task: pick up the violin and play its open A string with the bow: one clean, steady note of at least two seconds, then
lay the bow and the violin back in their places on the cloth.

What the physics asks for: a bowed string speaks (Helmholtz motion) only while the hair touches that string alone,
between the bridge and the fingerboard (relative bow-bridge distance 1/30 to 1/6), moving along its length across the
string at 5 cm/s or more and pressing with a force inside the measured window for that distance and speed: above
0.69 mN / beta^2 and below 0.91 N s/m x speed / beta (Schoonderwaldt, Guettler and Askenfelt 2008, violin open string).
The bow's hair is a compliant ribbon (a sprung slide, 300 N/m, on the stick), so the force follows from how far the hand
lowers the bow. Which string sounds follows from the bow's tilt across the bridge's arc (adjacent strings 15 degrees
apart). The violin only plays held up: lying on the cloth it isn't being played, so one hand holds it up (here by the
fingerboard's edges over the body, near its balance point) while the other bows.

Run it:  inspect-robots run --embodiment yam_arena -E task=bow_a_clean_open_a_on_a_violin.py \\
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
from bodykit import plate, segment  # noqa: E402
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


def _frame(xc, zc) -> np.ndarray:
    xc = np.asarray(xc, float) / np.linalg.norm(xc)
    zc = np.asarray(zc, float) / np.linalg.norm(zc)
    return np.column_stack([xc, np.cross(zc, xc), zc])


# -- the cloth ---------------------------------------------------------------------------------------------------------
CL_X0, CL_X1, CL_Y0, CL_Y1, CL_H = 0.120, 0.900, -0.470, 0.340, 0.005
CLOTH_TOP = TOP + CL_H

# -- the violin (violin frame: origin at the neck root on the centre line, level with the top plate's edge; x toward
#    the tail, y toward the E string's side, z out of the top) -----------------------------------------------------------
# half outline of the body (mm): upper bout 168, C-bouts 112, lower bout 208 wide, 356 long, pointed corners
OUTLINE_MM = [(0.0, 0.0), (0.6, 12.2), (1.8, 24.5), (4.1, 36.5), (8.1, 48.2), (14.0, 59.1), (22.1, 68.4), (32.2, 75.3),
              (43.7, 79.8), (55.8, 82.6), (68.0, 83.8), (80.2, 83.9), (92.4, 82.4), (104.3, 78.7), (116.0, 75.2),
              (128.0, 77.0), (130.4, 71.0), (134.3, 65.7), (139.2, 61.5), (144.9, 58.4), (151.1, 56.7), (157.5, 56.0),
              (164.1, 56.0), (170.7, 56.2), (177.2, 57.0), (183.3, 58.7), (189.0, 61.7), (194.1, 65.9), (198.2, 71.0),
              (201.3, 76.8), (203.0, 83.0), (213.9, 93.3), (227.4, 99.2), (242.2, 102.3), (257.1, 103.8),
              (272.0, 103.9), (286.9, 102.2), (301.3, 98.2), (314.6, 91.3), (326.4, 82.1), (336.3, 70.8), (344.1, 58.0),
              (349.9, 44.2), (353.7, 29.8), (355.7, 14.9), (356.0, 0.0)]
BODY_L = 0.356
RIB = 0.030  # rib height
ARCH_TOP, ARCH_BACK = 0.015, 0.014  # the plates' arching
BACK_Z = -RIB - ARCH_BACK + 0.0  # -0.044: the back's lowest point
BRIDGE_X, NUT_X = 0.195, -0.130  # stop 195 mm, neck 130 mm (string length 327 mm along the string)
FB_END_X = 0.140  # the fingerboard: 270 mm from the nut
FB_TOP0, FB_SLOPE = 0.022, 0.1061  # its top: 22 mm above the top's edge at the neck root, rising 6.1 degrees
FB_T = 0.005
NUT_Z = 0.0087  # the strings at the nut
ARC_R, ARC_TOP = 0.042, 0.0484  # the bridge's top: 42 mm radius, its crown 48.4 mm above the top's edge
BRIDGE_Y = (-0.0165, -0.0055, 0.0055, 0.0165)  # G D A E at the bridge (11 mm apart)
NUT_Y = (-0.00825, -0.00275, 0.00275, 0.00825)  # and at the nut
STRING_NAMES = ("G", "D", "A", "E")
STR_T = 0.0012  # the strings' collision thickness (the A string itself is 0.7 mm)
GRIP_XV = float(os.environ.get("VLN_GRIPX", 0.070))  # the left hand holds the fingerboard 70 mm into the body, near the balance point
VIOLIN_MASS = 0.45
VIOLIN_COM = (0.105, 0.0, -0.010)


def _fb_top(x: float) -> float:
    return FB_TOP0 + FB_SLOPE * x


def _neck_back(x: float) -> float:
    """The neck's back (including the fingerboard the neck is 18 mm deep at the nut, 20.5 mm at the heel)."""
    f = (x - NUT_X) / (0.0 - NUT_X)
    return _fb_top(x) - (0.018 + 0.0025 * f)


def _string_ends(i: int):
    yb = BRIDGE_Y[i]
    zb = ARC_TOP - (ARC_R - math.sqrt(ARC_R ** 2 - yb ** 2))
    return np.array([NUT_X, NUT_Y[i], NUT_Z]), np.array([BRIDGE_X, yb, zb])


STRINGS = [_string_ends(i) for i in range(4)]
STRING_L = [float(np.linalg.norm(b - n)) for n, b in STRINGS]


def _string_at(i: int, x: float) -> np.ndarray:
    n, b = STRINGS[i]
    return n + (b - n) * (x - NUT_X) / (BRIDGE_X - NUT_X)


def _outline(inset: float = 0.0) -> np.ndarray:
    """The full body outline (m), counter-clockwise from the neck root, optionally inset."""
    h = np.array(OUTLINE_MM) / 1000.0
    full = np.vstack([h, (h[-2:0:-1] * [1.0, -1.0])])
    if inset > 0.0:
        out = []
        n = len(full)
        for k in range(n):
            t = full[(k + 1) % n] - full[k - 1]
            nrm = np.array([t[1], -t[0]])
            nrm /= np.linalg.norm(nrm) + 1e-12
            out.append(full[k] - nrm * inset)
        full = np.array(out)
    return full


def _half_width(x: float) -> float:
    h = np.array(OUTLINE_MM) / 1000.0
    return float(np.interp(x, h[:, 0], h[:, 1]))


def _arch(x: float, y: float, a: float) -> float:
    w = max(_half_width(min(max(x, 0.001), BODY_L - 0.001)), 1e-4)
    t = min(abs(y) / w, 1.0)
    e = math.sin(math.pi * min(max(x / BODY_L, 0.0), 1.0)) ** 0.35
    return a * (1.0 - t * t) ** 0.85 * e


VARNISH, VARNISH_D, SPRUCE = (0.55, 0.22, 0.06), (0.36, 0.12, 0.03), (0.70, 0.36, 0.10)
MAPLE_BACK, EBONY, BRIDGE_C = (0.58, 0.26, 0.08), (0.05, 0.04, 0.04), (0.88, 0.76, 0.55)
STR_C = {"G": (0.70, 0.68, 0.66), "D": (0.82, 0.80, 0.78), "A": (0.84, 0.84, 0.86), "E": (0.92, 0.92, 0.94)}
PURFLING, FHOLE = (0.08, 0.05, 0.03), (0.06, 0.03, 0.02)
CLOTH_C, CLOTH_D = (0.10, 0.28, 0.20), (0.08, 0.22, 0.16)
TUNER_C, TUNER_SCREEN = (0.08, 0.08, 0.09), (0.30, 0.85, 0.35)

# extra visual-only shapes drawn with the bodies (meshes, ellipsoids, capsules), by body name
_VISUALS: dict[str, list] = {}


def _mesh(body, pts, faces, color):
    _VISUALS.setdefault(body, []).append(("mesh", [tuple(map(float, p)) for p in pts], [tuple(f) for f in faces],
                                          color, None))


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


def _plate_mesh(body, outline, zfun, color, rings=7):
    """An arched plate over the outline: rings shrinking to the centre line, height zfun(x, y)."""
    n = len(outline)
    pts, faces = [], []
    for k in range(rings + 1):
        f = 1.0 - k / rings
        for (x, y) in outline:
            pts.append((x, y * f, zfun(x, y * f)))
    for k in range(rings):
        for i in range(n):
            a, b = k * n + i, k * n + (i + 1) % n
            faces.append((a, b, b + n, a + n))
    _mesh(body, pts, faces, color)


def _strip_mesh(body, loop_a, loop_b, color):
    """A closed band between two loops of 3D points."""
    n = len(loop_a)
    pts = [tuple(p) for p in loop_a] + [tuple(p) for p in loop_b]
    faces = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    _mesh(body, pts, faces, color)


def _violin_visuals() -> None:
    o = _outline()
    oi = _outline(0.0025)  # the ribs stand 2.5 mm in from the plates' edges
    _plate_mesh("violin", o, lambda x, y: 0.001 + _arch(x, y, ARCH_TOP), SPRUCE)  # the top (spruce under varnish)
    _plate_mesh("violin", o, lambda x, y: -RIB - 0.001 - _arch(x, y, ARCH_BACK), MAPLE_BACK)  # the back
    _strip_mesh("violin", [(x, y, -RIB) for x, y in oi], [(x, y, 0.0) for x, y in oi], VARNISH)  # the ribs
    for z in (0.0, -RIB - 0.001):  # the plates' edges
        _strip_mesh("violin", [(x, y, z) for x, y in o], [(x, y, z + 0.0012) for x, y in o], VARNISH_D)
    p1, p2 = _outline(0.0035), _outline(0.0047)  # the purfling round the top
    _strip_mesh("violin", [(x, y, 0.0016 + _arch(x, y, ARCH_TOP)) for x, y in p1],
                [(x, y, 0.0016 + _arch(x, y, ARCH_TOP)) for x, y in p2], PURFLING)
    # the f-holes, either side of the bridge
    for s in (-1.0, 1.0):
        pts = [(0.168, 0.031), (0.178, 0.034), (0.190, 0.037), (0.202, 0.040), (0.214, 0.043), (0.224, 0.046)]
        for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
            za = 0.0018 + _arch(xa, ya, ARCH_TOP)
            zb = 0.0018 + _arch(xb, yb, ARCH_TOP)
            _cap("violin", (xa, s * ya, za), (xb, s * yb, zb), 0.0013, FHOLE)
        for (x, y) in (pts[0], pts[-1]):
            _ell("violin", (x, s * (y - 0.002), 0.0018 + _arch(x, y, ARCH_TOP)), (0.003, 0.003, 0.0006), FHOLE)
    # the neck (rounded back), the heel and the button
    for x in (-0.118, -0.094, -0.070, -0.046, -0.024):
        top, back = _fb_top(x) - FB_T, _neck_back(x)
        hw = 0.0115 + 0.0020 * (x - NUT_X) / (0.0 - NUT_X)
        _ell("violin", (x, 0.0, (top + back) / 2 + 0.001), (0.016, hw, (top - back) / 2 + 0.001), VARNISH)
    _ell("violin", (-0.006, 0.0, -0.016), (0.012, 0.0125, 0.020), VARNISH)  # the heel
    # the fingerboard (ebony, tapered, cambered top)
    fb = []
    for x in (NUT_X, FB_END_X):
        w = 0.012 + 0.009 * (x - NUT_X) / (FB_END_X - NUT_X)
        for y, dz in ((-w, -0.0012), (w, -0.0012)):
            fb.append((x, y, _fb_top(x) + dz))
        for y in (-w, w):
            fb.append((x, y, _fb_top(x) - FB_T))
    # vertices: 0..3 at the nut (top-left, top-right, bottom-left, bottom-right), 4..7 at the end
    faces = [(0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5), (0, 2, 3, 1), (4, 5, 7, 6)]
    _mesh("violin", fb, faces, EBONY)
    for x in np.linspace(NUT_X, FB_END_X, 6)[:-1]:  # the crown of the board's camber
        x2 = x + (FB_END_X - NUT_X) / 5
        _cap("violin", (x + 0.002, 0.0, _fb_top(x) - 0.0012), (x2, 0.0, _fb_top(x2) - 0.0012), 0.0012, EBONY)
    # the nut, pegbox, pegs, scroll
    _cap("violin", (NUT_X - 0.002, -0.012, NUT_Z - 0.002), (NUT_X - 0.002, 0.012, NUT_Z - 0.002), 0.0025, EBONY)
    _ell("violin", (-0.205, 0.0, 0.006), (0.011, 0.0105, 0.015), VARNISH)  # the throat under the scroll
    for k, (x, s) in enumerate(((-0.145, 1.0), (-0.160, -1.0), (-0.175, 1.0), (-0.190, -1.0))):
        _cap("violin", (x, -0.020, 0.013), (x, 0.020, 0.013), 0.0032, EBONY)
        _ell("violin", (x, s * 0.030, 0.013), (0.009, 0.0035, 0.012), EBONY)  # the peg heads
    _ell("violin", (-0.219, 0.0, 0.014), (0.019, 0.0115, 0.020), VARNISH)  # the scroll
    for r, c in ((0.014, VARNISH_D), (0.009, VARNISH), (0.005, VARNISH_D)):
        for s in (-1.0, 1.0):
            _ell("violin", (-0.221, s * 0.0113, 0.015), (r, 0.0012, r), c)
    # the clip-on tuner on the scroll (black clamp, green display)
    _ell("violin", (-0.214, -0.004, 0.040), (0.013, 0.008, 0.012), TUNER_C)
    _ell("violin", (-0.214, -0.004, 0.051), (0.009, 0.0045, 0.0015), TUNER_SCREEN)
    _cap("violin", (-0.214, -0.004, 0.030), (-0.214, -0.004, 0.036), 0.003, TUNER_C)
    # the bridge (maple) with its arched top
    for k in range(9):
        y = -0.0205 + 0.041 * (k + 0.5) / 9
        ztop = ARC_TOP - (ARC_R - math.sqrt(ARC_R ** 2 - y ** 2)) - 0.0008
        zb = 0.001 + _arch(BRIDGE_X, y, ARCH_TOP)
        _cap("violin", (BRIDGE_X, y, zb + 0.001), (BRIDGE_X, y, ztop - 0.001), 0.0013, BRIDGE_C)
    # the tailpiece, fine tuner, tail gut, saddle, end button, chinrest
    _ell("violin", (0.300, 0.0, 0.024), (0.052, 0.018, 0.004), EBONY)
    _ell("violin", (0.262, 0.0165, 0.026), (0.006, 0.004, 0.005), (0.75, 0.75, 0.72))  # the E string's fine tuner
    _cap("violin", (0.350, 0.0, 0.010), (0.356, 0.0, 0.004), 0.0025, EBONY)
    _cap("violin", (0.356, 0.0, -0.014), (0.364, 0.0, -0.014), 0.0045, EBONY)
    _ell("violin", (0.325, -0.040, 0.020), (0.032, 0.026, 0.006), EBONY)
    # the strings: nut to bridge, and bridge to tailpiece
    for i, nm in enumerate(STRING_NAMES):
        n, b = STRINGS[i]
        _cap("violin", n, b, 0.00045 if nm == "E" else 0.0006, STR_C[nm])
        _cap("violin", b, (0.255, NUT_Y[i] * 2.6, 0.026), 0.0005, STR_C[nm])


def _violin_shapes() -> list:
    sh = []
    zb, zt = BACK_Z, 0.008
    sh.append(CylShape((0.082, 0.0, (zb + zt) / 2), 0.080, zt - zb, None))  # upper bout
    sh.append(CylShape((0.268, 0.0, (zb + zt) / 2), 0.088, zt - zb, None))  # lower bout
    sh.append(BoxShape((0.265, 0.0, (zb + zt) / 2), (0.070, 0.196, zt - zb), None))  # lower bout's widest part
    sh.append(BoxShape((0.165, 0.0, (zb + zt) / 2), (0.090, 0.108, zt - zb), None))  # the waist
    sh.append(BoxShape((0.185, 0.0, 0.006), (0.270, 0.080, 0.012), None))  # the arched top's crown
    # neck (the grip zone), heel, pegbox, scroll
    for x0, x1 in ((NUT_X, -0.070), (-0.070, -0.010)):
        xm = (x0 + x1) / 2
        top = _fb_top(xm) - FB_T
        back = _neck_back(xm)
        sh.append(BoxShape((xm, 0.0, (top + back) / 2), (x1 - x0, 0.0255, top - back), None))
    sh.append(BoxShape((-0.006, 0.0, -0.018), (0.012, 0.024, 0.030), None))  # the heel
    sh.append(BoxShape((-0.168, 0.0, 0.008), (0.070, 0.022, 0.028), None))  # pegbox
    sh.append(CylShape((-0.219, 0.0, 0.014), 0.019, 0.023, None, axis="Y"))  # scroll
    # the fingerboard: three tapered lengths
    for k in range(3):
        x0 = NUT_X + (FB_END_X - NUT_X) * k / 3
        x1 = NUT_X + (FB_END_X - NUT_X) * (k + 1) / 3
        w = 0.024 + 0.018 * ((x0 + x1) / 2 - NUT_X) / (FB_END_X - NUT_X)
        sh.append(plate((x0, 0.0, _fb_top(x0) - FB_T / 2), (x1, 0.0, _fb_top(x1) - FB_T / 2), w, FB_T,
                        (-FB_SLOPE, 0.0, 1.0), None))
    # the bridge, tailpiece, chinrest, end button
    sh.append(BoxShape((BRIDGE_X, 0.0, (0.015 + 0.043) / 2), (0.003, 0.041, 0.028), None))
    sh.append(BoxShape((0.300, 0.0, 0.022), (0.100, 0.034, 0.008), None))
    sh.append(BoxShape((0.325, -0.040, 0.018), (0.055, 0.045, 0.012), None))
    sh.append(CylShape((0.360, 0.0, -0.014), 0.0045, 0.010, None, axis="X"))
    # the strings, where the bow can reach them: from the fingerboard's end to the bridge
    for i in range(4):
        sh.append(segment(_string_at(i, 0.100), _string_at(i, BRIDGE_X - 0.002), STR_T, None))
    return sh


# where the violin lies: on its back on the cloth, along y, the scroll toward +y (the robot's left), the E side at +x
V_YAW = -math.pi / 2
V_ROT0 = _rz(V_YAW)
V_POS0 = np.array([0.400, 0.052, CLOTH_TOP - BACK_Z])
V_GRIP_LOCAL = np.array([GRIP_XV, 0.0, _fb_top(GRIP_XV) - 0.0005])  # the grip point: level with the fingerboard's top edges

# -- the bow (bow frame: origin at the middle of the frog; it points along +x, hair down) -----------------------------
FROG_W, FROG_L, FROG_H = 0.014, 0.046, 0.026
STICK_R0, STICK_R1 = 0.0042, 0.0029
HAIR_X0, HAIR_X1 = 0.025, 0.685  # the hair: 66 cm from the frog's ferrule to the head
HAIR_Z = -FROG_H / 2 + 0.0025  # the hair ribbon's middle
HAIR_W, HAIR_T = 0.010, 0.0025  # 10 mm wide
STICK_Z0, STICK_Z1 = FROG_H / 2 + STICK_R0 - 0.002, 0.010
CAMBER = 0.008
HEAD_X = 0.700
BOW_MASS, HAIR_MASS = 0.054, 0.006  # 60 g in all
BOW_COM_X = 0.205  # the bow balances 20.5 cm in front of the frog's middle
HAIR_K, HAIR_C = 300.0, 2.5  # the hair's give across its length: N/m (design value), N s/m
HAIR_TRAVEL = 0.008  # until the hair meets the stick
B_POS0 = np.array([0.160, -0.400, CLOTH_TOP + FROG_H / 2])
EBONY_B, SILVER, PEARL, STICK = (0.05, 0.04, 0.04), (0.78, 0.79, 0.82), (0.92, 0.92, 0.88), (0.38, 0.15, 0.07)
HAIR_C_LOOK, IVORY = (0.86, 0.84, 0.76), (0.94, 0.91, 0.82)

# -- the rules ---------------------------------------------------------------------------------------------------------
TOUCH_GAP = 0.0004  # the hair within 0.4 mm of a string: touching it
BETA_MIN, BETA_MAX = 1.0 / 30.0, 1.0 / 6.0  # the measured range of bow-bridge distances
V_MIN = 0.05  # m/s
C_LOWER = 0.00069  # N: F_min = C_LOWER / beta^2
C_UPPER_PER_V = 0.91  # N s/m: F_max = C_UPPER_PER_V * v / beta (91 mN at 10 cm/s)
SKEW_MAX = 15.0  # degrees from square to the string
CLEAR = 0.020  # the violin's lowest point this far above the cloth: held up
NOTE_S, HALF_S, GAP_S = 2.0, 0.5, 0.10
FALL_V = -0.5  # m/s downward: falling
HOME_WITHIN = 0.05
V_BACK_XY, V_BACK_DEG = 0.030, 10.0
B_BACK_XY, B_BACK_DEG = 0.040, 10.0
SUCCESS_HOLD_S = 1.0
PHYSICS_DT = 1.0 / 240.0
DT = 0.05


def _violin() -> Body:
    _violin_visuals()
    return Body.build("violin", pos=tuple(V_POS0.tolist()), yaw=V_YAW, mass=VIOLIN_MASS, shapes=_violin_shapes(),
                      color=VARNISH, friction=(1.2, 1.0), com=VIOLIN_COM, solver_iterations=32, damping=(0.05, 0.3))


def _bow() -> Body:
    """The bow's stick with its frog, screw and head (the hair is its own body, on a sprung slide)."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (FROG_L, FROG_W, FROG_H), EBONY_B),
              CylShape((0.0, FROG_W / 2 + 0.0003, 0.002), 0.004, 0.0006, PEARL, collide=False, axis="Y"),
              CylShape((0.0, -FROG_W / 2 - 0.0003, 0.002), 0.004, 0.0006, PEARL, collide=False, axis="Y"),
              BoxShape((FROG_L / 2 - 0.002, 0.0, -FROG_H / 2 + 0.003), (0.004, FROG_W + 0.001, 0.006), SILVER,
                       collide=False),
              CylShape((-FROG_L / 2 - 0.007, 0.0, STICK_Z0), 0.0045, 0.014, EBONY_B, axis="X"),
              CylShape((-FROG_L / 2 - 0.0005, 0.0, STICK_Z0), 0.0047, 0.002, SILVER, collide=False, axis="X")]
    n = 12
    for k in range(n):
        x0 = -FROG_L / 2 + (HEAD_X + FROG_L / 2) * k / n
        x1 = -FROG_L / 2 + (HEAD_X + FROG_L / 2) * (k + 1) / n
        z0 = STICK_Z0 + (STICK_Z1 - STICK_Z0) * k / n - CAMBER * math.sin(math.pi * k / n)
        z1 = STICK_Z0 + (STICK_Z1 - STICK_Z0) * (k + 1) / n - CAMBER * math.sin(math.pi * (k + 1) / n)
        r = STICK_R0 + (STICK_R1 - STICK_R0) * (k + 0.5) / n
        shapes.append(plate((x0, 0.0, z0), (x1, 0.0, z1), 2 * r, 2 * r, (0.0, 0.0, 1.0), STICK))
    head_top = STICK_Z1 + STICK_R1
    hb = HAIR_Z - HAIR_T / 2 - 0.0005
    shapes.append(BoxShape((HEAD_X, 0.0, (head_top + hb) / 2), (0.020, 0.009, head_top - hb), STICK))
    shapes.append(BoxShape((HEAD_X + 0.0105, 0.0, (head_top + hb) / 2), (0.001, 0.009, head_top - hb), IVORY,
                           collide=False))
    com = (BOW_COM_X * (BOW_MASS + HAIR_MASS) - 0.355 * HAIR_MASS) / BOW_MASS
    return Body.build("bow", pos=tuple(B_POS0.tolist()), mass=BOW_MASS, color=STICK, shapes=shapes,
                      friction=(1.2, 1.0), com=(float(com), 0.0, 0.006), solver_iterations=32)


def _hair() -> Body:
    """The horsehair ribbon, 10 mm wide, from the frog's ferrule to the head; it gives toward the stick on a spring."""
    xm, ln = (HAIR_X0 + HAIR_X1) / 2, HAIR_X1 - HAIR_X0
    shapes = [BoxShape((xm, 0.0, HAIR_Z), (ln, HAIR_W, HAIR_T), HAIR_C_LOOK)]
    return Body.build("hair", pos=tuple(B_POS0.tolist()), mass=HAIR_MASS, color=HAIR_C_LOOK, shapes=shapes,
                      friction=(0.35, 0.3), slick=True, ignore=("bow",), solver_iterations=32, damping=(0.0, 0.0))


def _cloth() -> Body:
    sh = [BoxShape((0.0, 0.0, CL_H / 2), (CL_X1 - CL_X0, CL_Y1 - CL_Y0, CL_H), CLOTH_C)]
    for s in (-1.0, 1.0):
        sh.append(BoxShape((s * ((CL_X1 - CL_X0) / 2 - 0.010), 0.0, CL_H + 0.0002), (0.002, CL_Y1 - CL_Y0 - 0.02,
                                                                                     0.0004), CLOTH_D, collide=False))
    return Body.build("cloth", pos=((CL_X0 + CL_X1) / 2, (CL_Y0 + CL_Y1) / 2, TOP), mass=1.0, kinematic=True,
                      color=CLOTH_C, shapes=sh, friction=(0.9, 0.8))


bodykit.slider("hair_give", "bow", "hair", tuple(B_POS0.tolist()), axis="Z", limits=(0.0, HAIR_TRAVEL),
               stiffness=HAIR_K, damping=HAIR_C, target=0.0, max_force=50.0)
OBJECTS = [_cloth(), _violin(), _bow(), _hair()]

# -- extra visuals and contact sensing (authored with the bodies) ----------------------------------------------------
_orig_spawn = bodykit._spawn_body
_VIEW: dict = {"env": None, "view": None, "failed": ""}


def _spawn_with_extras(prim_path, cfg, translation=None, orientation=None, **kwargs):
    prim = _orig_spawn(prim_path, cfg, translation, orientation, **kwargs)
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Vt

    stage = get_current_stage()
    name = cfg.body_name
    if name in ("hair", "violin"):  # their contact is read: how hard the hair presses on the strings
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body")).CreateThresholdAttr(0.0)
        _VIEW["env"] = prim_path.rsplit("/", 1)[0]
    items = _VISUALS.get(name)
    if items:
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from pxr import UsdGeom

        from yam_arena.station import linear_rgb

        looks = {}
        for i, it in enumerate(items):
            kind, a, b, color, q = it
            path = f"{prim_path}/body/v{i}"
            if kind == "mesh":
                g = UsdGeom.Mesh.Define(stage, path)
                g.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in a]))
                g.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in b]))
                g.CreateFaceVertexIndicesAttr(Vt.IntArray([int(v) for f in b for v in f]))
                g.CreateDoubleSidedAttr(True)
                g.CreateSubdivisionSchemeAttr("none")
            elif kind == "ell":
                g = UsdGeom.Sphere.Define(stage, path)
                g.CreateRadiusAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(*a))
                g.AddOrientOp().Set(Gf.Quatf(q[3], q[0], q[1], q[2]))
                g.AddScaleOp().Set(Gf.Vec3f(*b))
            else:
                g = UsdGeom.Capsule.Define(stage, path)
                g.CreateRadiusAttr(b[0])
                g.CreateHeightAttr(b[1])
                g.CreateAxisAttr("Z")
                g.AddTranslateOp().Set(Gf.Vec3d(*a))
                g.AddOrientOp().Set(Gf.Quatf(q[3], q[0], q[1], q[2]))
            if color not in looks:
                lp = f"{prim_path}/Looks/v{len(looks)}"
                surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
                surface.func(lp, surface)
                looks[color] = lp
            bind_visual_material(path, looks[color], stage=stage)
    return prim


bodykit._spawn_body = _spawn_with_extras


def _hair_contact_force() -> float:
    """The contact force (N) between the bow's hair and the violin, this step."""
    if _VIEW["view"] is None and not _VIEW["failed"] and _VIEW["env"]:
        try:
            from isaaclab_physx.physics import PhysxManager

            env = _VIEW["env"]
            _VIEW["view"] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                [f"{env}/hair/body"], filter_patterns=[[f"{env}/violin/body"]], max_contact_data_count=0)
        except Exception as e:  # noqa: BLE001
            _VIEW["failed"] = repr(e)[:300]
            print(f"[vln] contact view failed: {_VIEW['failed']}", file=sys.stderr, flush=True)
    if _VIEW["view"] is None:
        return -1.0
    m = _VIEW["view"].get_contact_force_matrix(dt=PHYSICS_DT)
    m = np.asarray(m.numpy() if hasattr(m, "numpy") and not hasattr(m, "detach") else as_numpy(m), float)
    return float(np.linalg.norm(m.reshape(-1, 3), axis=1).sum())


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"note_t": 0.0, "gap_t": 0.0, "best_t": 0.0, "played": False, "half": False, "touched_up": False,
            "fell": False, "struck": False, "why": "", "prev_s": None, "tcps": None, "fingers": None, "s": {},
            "prev_vz": None, "max_f": 0.0, "pv": None, "pb": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("VLN_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _vel(base, name):
    return as_numpy(base.scene[name].data.root_link_vel_w)[0]


def _grips(base):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        out.append(rotation(as_numpy(robot.data.body_link_pose_w)[0][i][3:7]))
    return out


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


_LOW_PTS = np.array([[0.02, 0.0, BACK_Z], [0.08, 0.075, BACK_Z], [0.08, -0.075, BACK_Z], [0.17, 0.05, BACK_Z],
                     [0.17, -0.05, BACK_Z], [0.27, 0.095, BACK_Z], [0.27, -0.095, BACK_Z], [0.35, 0.0, BACK_Z],
                     [-0.219, 0.0, -0.005], [0.364, 0.0, -0.019], [-0.168, 0.0, -0.006]])


def _box_dist(p, half) -> float:
    q = np.abs(p) - half
    return float(np.linalg.norm(np.maximum(q, 0.0)) + min(max(q[0], q[1], q[2]), 0.0))


_HAIR_HALF = np.array([(HAIR_X1 - HAIR_X0) / 2, HAIR_W / 2, HAIR_T / 2])
_HAIR_C = np.array([(HAIR_X0 + HAIR_X1) / 2, 0.0, HAIR_Z])


def _strings_vs_hair(pv, rv, ph, rh):
    """For each string: its gap to the hair ribbon (m), where it crosses the hair's middle line (violin frame) and how
    far along the hair that is (m from the bow's origin)."""
    out = []
    hn_w = ph + rh @ _HAIR_C
    hd_w = rh[:, 0]
    for i in range(4):
        n, b = STRINGS[i]
        d = (b - n) / np.linalg.norm(b - n)
        sn_w = pv + rv @ n
        sd_w = rv @ d
        w0 = sn_w - hn_w
        bb = float(sd_w @ hd_w)
        den = 1.0 - bb * bb
        if den < 1e-9:
            out.append((1.0, None, 0.0))
            continue
        t = (bb * float(w0 @ hd_w) - float(w0 @ sd_w)) / den
        u = (float(w0 @ hd_w) - bb * float(w0 @ sd_w)) / den
        best = 1.0
        for dt in np.linspace(-0.012, 0.012, 25):
            pt_v = n + d * (t + dt)
            if pt_v[0] > BRIDGE_X - 0.0015 or pt_v[0] < 0.100:
                continue
            ploc = rh.T @ (pv + rv @ pt_v - ph) - _HAIR_C
            best = min(best, _box_dist(ploc, _HAIR_HALF) - STR_T / 2)
        out.append((best, n + d * t, float(_HAIR_C[0] + u)))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pv, rv = _pose(base, origin, "violin")
    pb, rb = _pose(base, origin, "bow")
    ph, rh = _pose(base, origin, "hair")
    _EP["tcps"] = _tcps(base, origin)
    _EP["fingers"] = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")])
                      for a in ("left_robot", "right_robot")]
    low = float(min((pv + rv @ p)[2] for p in _LOW_PTS))
    up = low > CLOTH_TOP + CLEAR
    # falls: the violin or the bow dropping faster than 0.5 m/s
    vv, vb = _vel(base, "violin"), _vel(base, "bow")
    if (vv[2] < FALL_V or vb[2] < FALL_V) and not _EP["fell"]:
        _EP["fell"], _EP["why"] = True, f"{'violin' if vv[2] < FALL_V else 'bow'} fell ({min(vv[2], vb[2]):.2f} m/s)"
    # a gripper's fingertips inside the violin's body
    for k, t in enumerate(_EP["tcps"]):
        tip = rv.T @ (t - pv)
        inside = 0.005 < tip[0] < BODY_L - 0.005 and abs(tip[1]) < _half_width(tip[0]) - 0.003 and BACK_Z < tip[2] < 0.012
        if inside and not _EP["struck"]:
            _EP["struck"], _EP["why"] = True, f"{'left' if k == 0 else 'right'} gripper struck the violin's body"
    # the hair on the strings
    gaps = _strings_vs_hair(pv, rv, ph, rh)
    touch = [g[0] < TOUCH_GAP for g in gaps]
    p_slide = float((rb.T @ (ph - pb))[2])
    f_spring = HAIR_K * max(p_slide, 0.0)
    f_contact = _hair_contact_force()
    force = f_contact if f_contact >= 0.0 else f_spring
    _EP["max_f"] = max(_EP["max_f"], force if any(touch) else 0.0)
    if up and any(touch):
        _EP["touched_up"] = True
    sounding, beta, speed, skew = False, 0.0, 0.0, 90.0
    if touch[2]:
        pt = gaps[2][1]
        beta = float(np.linalg.norm(STRINGS[2][1] - pt)) / STRING_L[2]
        s_now = gaps[2][2]
        if _EP["prev_s"] is not None:
            speed = abs(s_now - _EP["prev_s"]) / DT
        _EP["prev_s"] = s_now
        d = STRINGS[2][1] - STRINGS[2][0]
        d = rv @ (d / np.linalg.norm(d))
        skew = math.degrees(math.asin(min(1.0, abs(float(rh[:, 0] @ d)))))
        if speed > 0.0 and beta > 0.0:
            fmin = C_LOWER / beta ** 2
            fmax = C_UPPER_PER_V * speed / beta
            sounding = (up and not (touch[0] or touch[1] or touch[3]) and BETA_MIN <= beta <= BETA_MAX
                        and speed >= V_MIN and skew <= SKEW_MAX and fmin <= force <= fmax)
    else:
        _EP["prev_s"] = None
    other = touch[0] or touch[1] or touch[3]
    if sounding:
        _EP["note_t"] += DT + _EP["gap_t"]
        _EP["gap_t"] = 0.0
    elif _EP["note_t"] > 0.0:
        _EP["gap_t"] += DT
        if _EP["gap_t"] > GAP_S + 1e-6 or other:
            _EP["note_t"], _EP["gap_t"] = 0.0, 0.0
    _EP["best_t"] = max(_EP["best_t"], _EP["note_t"])
    if _EP["note_t"] >= HALF_S - 1e-6:
        _EP["half"] = True
    if _EP["note_t"] >= NOTE_S - 1e-6:
        _EP["played"] = True
    # the end state: violin and bow back in their places on the cloth, let go
    lt, rt = _EP["tcps"]
    grip_w = pv + rv @ V_GRIP_LOCAL
    v_held = float(np.linalg.norm(lt - grip_w)) < 0.04 and _EP["fingers"][0] > -0.017
    yaw_v = math.degrees(math.atan2(rv[1, 0], rv[0, 0]) - V_YAW)
    yaw_v = (yaw_v + 180.0) % 360.0 - 180.0
    v_back = (float(np.linalg.norm((pv - V_POS0)[:2])) < V_BACK_XY and abs(yaw_v) < V_BACK_DEG
              and float(rv[2, 2]) > math.cos(math.radians(V_BACK_DEG)) and low < CLOTH_TOP + 0.004 and not v_held)
    b_held = float(np.linalg.norm(rt - pb)) < 0.04 and _EP["fingers"][1] > -0.0105
    yaw_b = (math.degrees(math.atan2(rb[1, 0], rb[0, 0])) + 180.0) % 360.0 - 180.0
    b_back = (float(np.linalg.norm((pb - B_POS0)[:2])) < B_BACK_XY and abs(yaw_b) < B_BACK_DEG
              and float(rb[2, 2]) > math.cos(math.radians(20.0)) and pb[2] < B_POS0[2] + 0.010 and not b_held)
    _EP["s"] = {"up": up, "v_back": v_back, "b_back": b_back}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 2 == 0:
            mm = lambda v: np.round(np.asarray(v) * 1000, 1).tolist()  # noqa: E731
            gl = rv.T @ (lt - pv)  # left TCP in the violin frame
            bl = rb.T @ (rt - pb)  # right TCP in the bow frame
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rv[2, 2])))))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(-rv[2, 0])))))  # tail down +
            roll = math.degrees(math.atan2(float(rv[2, 1]), float(rv[2, 2])))
            vyaw = math.degrees(math.atan2(float(rv[1, 0]), float(rv[0, 0])))
            a_pt = pv + rv @ _string_at(2, BRIDGE_X - 0.033)
            gls, grs = _grips(base)
            lrel = rv.T @ gls  # the left gripper in the violin frame
            rrel = rb.T @ grs
            bpitch = math.degrees(math.asin(max(-1.0, min(1.0, float(rb[2, 0])))))
            broll = math.degrees(math.atan2(float(rb[2, 1]), float(rb[2, 2])))
            print(f"[vln] t={_DEBUG['steps'] / 20:5.2f} f={[round(v * 1000, 1) for v in _EP['fingers']]}"
                  f" vpos={mm(pv - [0, 0, TOP])} tilt={tilt:.1f} pitch={pitch:.2f} roll={roll:.2f} yaw={vyaw:.2f} low={(low - CLOTH_TOP) * 1000:.0f}"
                  f" Lin_v={mm(gl)} Lax_v={np.round(lrel[:, 2], 3).tolist()} Lcl_v={np.round(lrel[:, 0], 3).tolist()} Ltcp={mm(lt - [0, 0, TOP])}"
                  f" Rin_b={mm(bl)} Rax_b={np.round(rrel[:, 2], 3).tolist()} bow={mm(pb - [0, 0, TOP])} bpitch={bpitch:.1f} broll={broll:.1f} A33={mm(a_pt - [0, 0, TOP])}"
                  f" gaps={[round(g[0] * 1000, 1) for g in gaps]} slide={p_slide * 1000:.2f} Fs={f_spring:.2f} Fc={f_contact:.2f}"
                  f" beta={beta:.3f} v={speed:.3f} skew={skew:.1f} snd={sounding} note={_EP['note_t']:.2f} best={_EP['best_t']:.2f}"
                  f" up={up} s={_EP['s']} {_EP['why']} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if at any time the violin or the bow fell, or a gripper's fingertips went into the violin's body.
    Otherwise: 1.0: a clean open A sounded for at least 2 s with the violin held up, and at the end the violin and the
    bow both lie back in their places on the cloth, let go, and both arms are back where they started; 0.75: the
    2-second note played, the end state not met; 0.5: a clean open A sounded for at least 0.5 s at some point; 0.25: at
    some point the bow's hair touched the strings while the violin was held up; else 0.0."""
    s = _EP["s"]
    if not s or _EP["fell"] or _EP["struck"]:
        return 0.0
    if _EP["played"]:
        return 1.0 if (s["v_back"] and s["b_back"] and _arms_home()) else 0.75
    if _EP["half"]:
        return 0.5
    return 0.25 if _EP["touched_up"] else 0.0


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
    """Full score with the violin and the bow at rest: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("violin", "bow"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the violin from above by the two edges of its fingerboard where the board runs out over the body
# (70 mm into the body, close to the violin's balance point), the strings running untouched between its fingers, and
# lifts it clear of the cloth. The right hand picks up the bow by the frog, rolls it 30 degrees (the stick toward the
# bridge) and lays the hair on the A string 29 mm from the bridge (beta 0.09), square to the string and tilted along
# the bridge's arc so it rests on the A alone, and draws a 3 s down-bow (27 cm at 9 cm/s) with the hair pressed
# about 0.3 N. It lifts the bow off and lays it back on the cloth; the left lays the violin back; both arms go home.

def _env(name, default):
    v = os.environ.get(name)
    return float(v) if v else float(default)


def _cal(name, default):
    v = os.environ.get(name)
    return np.array([float(x) for x in v.split(",")]) if v else np.array(default, float)


# the left hand takes the fingerboard from above, its fingers closing across the board's two edges (the strings run
# between the fingers, untouched), the hand leaning 30 degrees along the violin toward the scroll
L_ALONG = math.radians(_env("VLN_LALONG", 30.0))
G_L_PICK = _rx(-L_ALONG) @ grip_rotation(-math.pi / 2)  # closes along world x: across the fingerboard
PREPITCH = math.radians(_env("VLN_PREPITCH", 0.0))  # the hand tips the tail up this much against its droop
G_L_PLAY = _rx(-PREPITCH) @ G_L_PICK
L_OPEN = -0.0250  # the fingers 6 mm clear of the 37 mm fingerboard's edges each side
LIFT = _env("VLN_LIFT", 0.045)
SAG = _env("VLN_SAG", 0.035)  # the left arm sinks under the held violin (measured): it aims this much higher
G_R_PICK = grip_rotation(0.0)  # right: top-down, fingers across the frog
R_OPEN = -0.013
FROG_TCP = np.array([0.0, 0.0, FROG_H / 2 - 0.006])  # the right TCP in the bow frame: 6 mm below the frog's top
BOW_ROLL = math.radians(_env("VLN_ROLL", 30.0))
BETA_AIM = _env("VLN_BETA", 0.09)
S0 = _env("VLN_S0", 0.080)  # the hair's contact point from the frog's middle at the stroke's start
STROKE = _env("VLN_STROKE", 0.270)
STROKE_T = _env("VLN_STROKE_T", 3.0)
PRESS = _env("VLN_PRESS", 0.0022)  # how far the hair's edge is aimed below the string's top at the stroke's start
PRESS_END = _env("VLN_PRESS_END", 0.0056)  # and at its end (the arm gives a little more as the frog comes in)
TILT_ERR = math.radians(_env("VLN_TILT", 0.0))  # checks: tilt the bow off the A string's line (degrees)
LEAD_S, LEAD_STEPS = 3.0, 15
# measured: where the held violin and the bow sit against where they were aimed; the hands aim the other way
CAL_V = _cal("VLN_CALV", (0.0, 0.0, 0.0))  # the violin's A string point (world) in play, measured minus planned
CAL_B = _cal("VLN_CALB", (0.0, 0.0, 0.0))  # the right TCP in the bow frame, measured minus planned


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    from yam_arena import ik, station

    seeds = [(0, 1.6, 1.4, -1.0, 0, 0), (0.5, 1.6, 1.4, -1.0, 0, 0), (-0.5, 1.6, 1.4, -1.0, 0, 0),
             (0, 1.3, 1.0, -0.5, 0, 1.0), (0, 1.3, 1.0, -0.5, 0, -1.0), (0, 1.9, 1.7, -1.3, 0.5, 0),
             (0, 1.9, 1.7, -1.3, -0.5, 0), (0.7, 1.3, 1.7, -0.8, 0.4, 0.4), (-0.7, 1.3, 1.7, -0.8, -0.4, -0.4),
             (0, 1.0, 0.6, 0.0, 0, 0), (0, 1.6, 1.4, -1.0, 1.2, 0), (0, 1.6, 1.4, -1.0, -1.2, 0)]
    best = None
    for seed in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed, float),
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


# measured in a debug run: where the held violin sits while it is played (origin m above the table top's level, and
# roll, pitch, yaw in degrees); the bow is aimed at its A string there
V_PLAY = _cal("VLN_VPLAY", (0.3898, 0.0640, 0.1131, -3.48, 4.02, -87.47))


def violin_play_pose():
    """Where the violin is held in play: the measured pose if given, else lifted LIFT and level."""
    if np.any(V_PLAY != 0.0):
        r = _rz(math.radians(V_PLAY[5])) @ _ry(math.radians(V_PLAY[4])) @ _rx(math.radians(V_PLAY[3]))
        p = np.array([V_PLAY[0], V_PLAY[1], V_PLAY[2] + TOP])
        return p, r, p + r @ V_GRIP_LOCAL
    tcp = V_POS0 + V_ROT0 @ V_GRIP_LOCAL + np.array([0.0, 0.0, LIFT])
    return tcp - V_ROT0 @ V_GRIP_LOCAL, V_ROT0, tcp


def bow_pose_on_a(s_contact: float, press: float = None):
    """The bow pose with its hair's lower edge on the A string, at the point s_contact along the hair."""
    if press is None:
        press = PRESS + (PRESS_END - PRESS) * min(max((s_contact - S0) / STROKE, 0.0), 1.0)
    pv, rv, _ = violin_play_pose()
    xc = BRIDGE_X - BETA_AIM * STRING_L[2]
    a = pv + rv @ _string_at(2, xc) + CAL_V
    sd = rv @ ((STRINGS[2][1] - STRINGS[2][0]) / STRING_L[2])
    chord = rv @ (_string_at(3, xc) - _string_at(1, xc))  # D to E: parallel to the arc's tangent at A
    xb = chord - (chord @ sd) * sd
    xb /= np.linalg.norm(xb)
    nrm = np.cross(sd, xb)
    if nrm[2] < 0:
        nrm = -nrm
    rb0 = np.column_stack([xb, np.cross(nrm, xb), nrm])
    rb = rb0 @ _ry(-TILT_ERR) @ _rx(BOW_ROLL)
    q = np.array([s_contact, -HAIR_W / 2, HAIR_Z - HAIR_T / 2])  # the ribbon's lowest edge
    target = a + nrm * (STR_T / 2 - press)
    pb = target - rb @ q
    return pb, rb


def _tcp_on_bow(pb, rb):
    return pb + rb @ (FROG_TCP + CAL_B), rb @ G_R_PICK


def demo():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur, qcur = {}, {}
    for arm in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
        cur[arm] = (np.asarray(p, float), FINGER_CLOSED, np.asarray(r, float))
        qcur[arm] = np.array(READY, float)
    one = os.environ.get("VLN_ONEHAND")  # checks only: "left" or "right" works alone, the other stays home

    def to(arm, t, pose, finger, label, n=1, joint=False):
        p1, r1 = np.asarray(pose[0], float), np.asarray(pose[1], float)
        if joint:
            q1 = _good_ik(arm, p1, r1)
            plan[arm].extend(_joint_path(arm, qcur[arm], q1, label, finger, t, max(n, 8)))
            qcur[arm] = q1
        else:
            p0, r0 = cur[arm][0], cur[arm][2]
            for j in range(1, n + 1):
                f = j / n
                rr = _slerp(r0, r1, f)
                plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * f), finger, label, rr))
            qcur[arm] = _good_ik(arm, p1, r1)
        cur[arm] = (p1, finger, r1)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 1e-6:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    up = np.array([0.0, 0.0, 1.0])
    ex = np.array([1.0, 0.0, 0.0])
    # 1. the left takes the violin by the fingerboard's edges over the body and lifts it clear of the cloth
    g0 = V_POS0 + V_ROT0 @ V_GRIP_LOCAL + _cal("VLN_CALL", (0.0, 0.0, 0.0))
    back = -G_L_PICK[:, 2]  # back along the approach
    side = g0 + back * 0.05
    to("left", LEAD_S, (side, G_L_PICK), L_OPEN, "over the fingerboard where it runs over the body", joint=True)
    to("left", 1.2, (g0, G_L_PICK), L_OPEN, "down round the fingerboard's edges", n=3)
    to("left", 0.6, (g0, G_L_PICK), FINGER_CLOSED, "hold the fingerboard by its edges, the strings free")
    wait("left", 0.3)
    lifted = g0 + up * (LIFT + SAG)
    to("left", 2.5, (lifted, G_L_PLAY), FINGER_CLOSED, "lift the violin clear of the cloth", n=5)
    # 2. meanwhile the right picks up the bow by its frog
    fr0 = B_POS0 + FROG_TCP
    to("right", LEAD_S, (fr0 + up * 0.05, G_R_PICK), R_OPEN, "over the bow's frog", joint=True)
    to("right", 1.0, (fr0, G_R_PICK), R_OPEN, "down round the frog", n=2)
    to("right", 0.6, (fr0, G_R_PICK), FINGER_CLOSED, "grip the frog")
    wait("right", 0.3)
    to("right", 1.0, (fr0 + up * 0.08, G_R_PICK), FINGER_CLOSED, "lift the bow", n=2)
    sync("wait: the violin held up")
    # 3. the bow over the A string, down onto it, a 3 s down-bow
    pb_on, rb_on = bow_pose_on_a(S0)
    hover = _tcp_on_bow(pb_on + up * 0.020, rb_on)
    to("right", 2.5, hover, FINGER_CLOSED, "the bow over the A string, tilted to it", joint=True)
    to("right", 1.2, _tcp_on_bow(pb_on + up * 0.004, rb_on), FINGER_CLOSED, "lower it onto the string", n=3)
    to("right", 0.6, _tcp_on_bow(*bow_pose_on_a(S0)), FINGER_CLOSED, "settle the hair on the A string", n=2)
    nst = max(4, int(round(STROKE_T / 0.1)))
    for k in range(1, nst + 1):
        to("right", STROKE_T / nst, _tcp_on_bow(*bow_pose_on_a(S0 + STROKE * k / nst)), FINGER_CLOSED,
           "draw the bow: a down-bow on the open A")
    pb_end, rb_end = bow_pose_on_a(S0 + STROKE)
    to("right", 0.8, _tcp_on_bow(pb_end + up * 0.04, rb_end), FINGER_CLOSED, "lift the bow off the string", n=2)
    if os.environ.get("VLN_NORETURN"):  # checks only: both are kept in hand, held up
        sync()
        for arm in ("left", "right"):
            wait(arm, 4.0, "keep holding")
        return plan
    # 4. the bow back on the cloth
    to("right", 2.5, (fr0 + up * 0.08, G_R_PICK), FINGER_CLOSED, "carry the bow back over its place", joint=True)
    to("right", 1.2, (fr0 + up * 0.004, G_R_PICK), FINGER_CLOSED, "lay it down", n=2)
    to("right", 0.5, (fr0 + up * 0.004, G_R_PICK), R_OPEN, "let go")
    to("right", 0.8, (fr0 + up * 0.06, G_R_PICK), R_OPEN, "rise")
    sync("wait: the bow put down")
    # 5. the violin back on the cloth
    to("left", 2.0, (g0 + up * 0.01, G_L_PICK), FINGER_CLOSED, "lower the violin", n=5)
    to("left", 1.5, (g0 + up * 0.002, G_L_PICK), FINGER_CLOSED, "lay it on the cloth", n=3)
    to("left", 0.6, (g0 + up * 0.002, G_L_PICK), L_OPEN, "let go of the neck")
    to("left", 1.0, (side, G_L_PICK), L_OPEN, "rise off the neck", n=2)
    sync()
    for arm in ("left", "right"):
        plan[arm] += _joint_path(arm, qcur[arm], np.array(READY, float), "back to where it started", cur[arm][1])
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    if one in ("left", "right"):  # checks only: the other arm stays home
        other = "right" if one == "left" else "left"
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[other])
        plan[other] = [Waypoint(dur(one), tuple(float(v) for v in p), FINGER_CLOSED, "stay home", r)]
    return plan


def _slerp(r0, r1, f):
    rel = np.asarray(r0).T @ np.asarray(r1)
    ang = math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1.0) / 2.0)))
    if ang < 1e-6:
        return np.asarray(r1)
    ax = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]]) / (2 * math.sin(ang))
    a = ang * f
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.asarray(r0) @ (np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k)
