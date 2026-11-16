"""An end-of-line inspection bench in a luggage factory, covered with grey felt. A finished hard-shell cabin case
(55 x 40 x 20 cm overall, 3.5 kg, navy grooved polycarbonate, aluminium corner guards, four double spinner casters)
lies on its front, back panel up, its top end toward the robot's right. Its twin-tube aluminium pull handle is closed:
the black grip stands in a shallow recess in the top end panel, a finger gap behind it, a grey release button in the
middle of its end face; the tubes run inside the case along the back panel. The handle locks closed and at four
heights (36, 38, 40 and 42.5 in from the floor with the case standing: 364, 415, 466 and 530 mm out). It slides only
while the button is held in; let go between heights, the lock pin rides along the tube and clicks into the next hole
it meets. A small steel stand at the bench's front edge, between the arms, holds three red reject tags hung on a rod
by their moulded hooks; the case has a carry handle on its near side.

Some units have a worn lock: one height clicks but gives under a firm push along the rails, and the handle slides
back. Which unit this is depends on the trial's seed.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (LOOKS, Body, BoxShape, CylShape, MeshShape, SphereShape, allow_matrix_grips, as_numpy,  # noqa: E402
                     on_reset, on_step, rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- colours and finishes ---------------------------------------------------------------------------------------------
NAVY = (0.105, 0.150, 0.275)  # the shell (polycarbonate, glossy)
NAVY_RIB = (0.135, 0.185, 0.320)
ZIP = (0.060, 0.065, 0.075)
ALU = (0.560, 0.570, 0.590)  # anodised aluminium tubes
GUARD = (0.400, 0.410, 0.430)  # the corner guards
RUBBER = (0.045, 0.045, 0.050)
PLASTIC = (0.090, 0.090, 0.100)
HOUSING = (0.150, 0.150, 0.165)  # the handle's moulded housing in the top end
BUTTON = (0.550, 0.560, 0.580)
FELT = (0.600, 0.610, 0.605)
STEEL = (0.300, 0.310, 0.320)
TAG_RED = (0.800, 0.070, 0.060)
WHITE = (0.940, 0.940, 0.930)
LOOKS.update({NAVY: (0.28, 0.0), NAVY_RIB: (0.28, 0.0), ALU: (0.55, 0.35), GUARD: (0.50, 0.40), BUTTON: (0.35, 0.6),
              RUBBER: (0.85, 0.0), PLASTIC: (0.55, 0.0), HOUSING: (0.6, 0.0), FELT: (0.95, 0.0), ZIP: (0.75, 0.0),
              STEEL: (0.45, 0.5), TAG_RED: (0.45, 0.0)})

# -- the bench cover --------------------------------------------------------------------------------------------------
FELT_T = 0.003
FELT_X, FELT_Y = (0.05, 0.57), (-0.63, 0.63)
FELT_MU = (0.30, 0.27)  # felt on polycarbonate (estimate): 10 N slides the empty case
BENCH = TOP + FELT_T  # the felt's top
CASE_Z = BENCH + 0.001  # the case and its handle parts spawn 1 mm above the felt

# -- the case: Travelpro Platinum Elite Slim Carry-On Expandable Hardside Spinner 55 cm -------------------------------
# Maker: 55 x 40 x 20 cm overall (wheels and handles included), 3.5 kg, 39 L, 100% polycarbonate, PowerScope Lite
# handle with four stops at 36, 38, 40 and 42.5 in. The body's own size is estimated (overall minus wheels and grip).
L, W, D = 0.443, 0.366, 0.200  # body: length (top end to wheel end), width, depth
R_EDGE = np.array([0.045, 0.045, 0.032])  # rounding radii (x, y, z)
CX, Y0 = 0.31, -0.013  # the case's middle line (x) and its top end (y), world
CASE_MASS = 3.12  # shell, wheels and fixed parts; the handle's moving parts make up the rest of 3.5 kg
# In the case's frame: x across (+ = far side), y from the top end (0) toward the wheels, z up from the bench.
REC = (0.106, 0.012, 0.070)  # the handle housing's recess in the top end: half width, depth (y), height down from z=D
GRIP = (0.095, 0.020, 0.034)  # the grip: half length (x), thickness (y), height (z)
GRIP_Y = (-0.038, -0.018)  # its end face (with the button) and its inner face: it stands 18-38 mm off the top end
GRIP_Z = D - 0.035  # its middle
GAP = GRIP_Y[1], REC[1]  # the finger gap behind the grip (30 mm)
TUBE_X, TUBE_Z = 0.080, GRIP_Z  # the rails: 160 mm apart, inside along the back panel
MID_TUBE, INNER_TUBE = (0.022, 0.013), (0.019, 0.010)  # rectangular aluminium tubes (x, z)
STOPS_IN = (36.0, 38.0, 40.0, 42.5)  # the maker's handle heights (in, from the floor, case standing)
OVERALL_H = 0.550
STOPS = (0.0,) + tuple(round(h * 0.0254 - OVERALL_H, 4) for h in STOPS_IN)  # 0, 0.3644, 0.4152, 0.4660, 0.5295 m out
S_MAX = STOPS[-1]
STAGE_IN = 0.25  # the inner section runs out first, then the middle one
BUTTON_SIZE = (0.050, 0.016)  # x, z
BUTTON_TRAVEL = 0.004
CASTER_R, CASTER_W = 0.025, 0.012
SIDE_BAR = (0.022, 0.034, 0.110, 0.130)  # the side carry handle's bar: off the near side from/to (m), bottom/top z
SIDE_BAR_HALF = 0.056  # its half length (y), about the case's middle

# the handle's mechanism (estimates, see the README)
SLIDE_DAMPING = 25.0  # N s/m: the tubes' bushings (2.5 N at 0.1 m/s)
WORN_HOLD = 14.0  # N: what the worn lock holds before its pin is pushed out of its hole (the drive's force cap)
WORN_K = 6000.0  # N/m: the worn lock's stiffness while it holds
CAPTURE = 0.003  # m: a released pin drops into a hole this close (or one it crosses)
PIN_OUT, PIN_IN = 0.0025, 0.0010  # button travel (m) that pulls the pins out, and below which they spring back
SLIP_GAP = 0.004  # m past the worn hole: the pin is out of it

DEMO_SEED = 1415158438  # the reset seed of inspect-robots' default --seed 0: the demo's trial (variant 3)
N_VARIANTS = 5  # variant = reset seed mod 5: 0 good unit, 1-4 the lock at that height is worn


def variant_for(seed) -> int:
    return int(DEMO_SEED if seed is None else seed) % N_VARIANTS


# -- shell surface ----------------------------------------------------------------------------------------------------

def _axis_grid(h: float, r: float, step: float, extra=()) -> np.ndarray:
    """Coordinates along one axis of a face, from -h to h: fine in the rounded bands, ``step`` apart in the flat part."""
    band = 9
    t = np.concatenate([np.linspace(-h, -h + r, band, endpoint=False),
                        np.linspace(-h + r, h - r, max(2, int(math.ceil((2 * h - 2 * r) / step)) + 1)),
                        np.linspace(h - r, h, band + 1)[1:]])
    t = np.concatenate([t, np.asarray(extra, float)])
    t = np.unique(np.round(t[(t >= -h) & (t <= h)], 6))
    return t


def _rounded_box(half: np.ndarray, r: np.ndarray, step: float, extra: dict) -> tuple[np.ndarray, np.ndarray]:
    """Points and triangles of a box with rounded edges (elliptical, radii ``r``), centred on the origin: one grid per
    face, its cube points pushed onto the rounded surface. ``extra[axis]`` adds grid lines (centred coordinates)."""
    pts, tris = [], []
    for a in range(3):
        b, c = [k for k in range(3) if k != a]
        gb = _axis_grid(half[b], r[b], step, extra.get(b, ()))
        gc = _axis_grid(half[c], r[c], step, extra.get(c, ()))
        for s in (-1.0, 1.0):
            B, C = np.meshgrid(gb, gc, indexing="ij")
            p = np.zeros(B.shape + (3,))
            p[..., a], p[..., b], p[..., c] = s * half[a], B, C
            p = p.reshape(-1, 3)
            core = np.clip(p, -(half - r), half - r)
            d = (p - core) / r
            n = np.linalg.norm(d, axis=1, keepdims=True)
            q = core + r * d / np.maximum(n, 1e-12)
            base = sum(len(x) for x in pts)
            nb, nc = len(gb), len(gc)
            i = np.arange(nb - 1)[:, None] * nc + np.arange(nc - 1)[None, :]
            i = i.reshape(-1)
            t1 = np.stack([i, i + nc, i + nc + 1], axis=1)
            t2 = np.stack([i, i + nc + 1, i + 1], axis=1)
            t = np.concatenate([t1, t2]) + base
            m = (nb // 2) * nc + nc // 2  # a quad in the face's flat middle: wind the face outward
            v0, v1, v2 = q[m], q[m + nc], q[m + nc + 1]
            if np.dot(np.cross(v1 - v0, v2 - v0), np.eye(3)[a] * s) < 0:
                t = t[:, [0, 2, 1]]
            pts.append(q)
            tris.append(t)
    return np.concatenate(pts), np.concatenate(tris)


def _normals(p: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Area-weighted vertex normals, shared across coincident vertices (the face grids' seams)."""
    key = np.round(p * 1e5).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    fn = np.cross(p[t[:, 1]] - p[t[:, 0]], p[t[:, 2]] - p[t[:, 0]])
    acc = np.zeros((inv.max() + 1, 3))
    for k in range(3):
        np.add.at(acc, inv[t[:, k]], fn)
    n = acc[inv]
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def _subset(p, n, t, keep_tri, offset):
    """The triangles ``keep_tri`` as their own mesh, pushed ``offset`` out along the normals."""
    tt = t[keep_tri]
    used = np.unique(tt)
    remap = -np.ones(len(p), dtype=np.int64)
    remap[used] = np.arange(len(used))
    return p[used] + offset * n[used], remap[tt], n[used]


ZIP_Z = (0.080, 0.096)


def _shell_meshes() -> list[MeshShape]:
    """The case's shell (grooved back and front panels, the handle's recess in the top end), its zip band and its
    corner guards, in the case's frame."""
    half = np.array([W / 2, L / 2, D / 2])
    centre = np.array([0.0, L / 2, D / 2])
    rx_, ry_, rz_ = REC
    extra = {0: (-rx_, rx_, -rx_ + 0.0006, rx_ - 0.0006),
             1: (ry_ - L / 2, ry_ - 0.0006 - L / 2),
             2: tuple(z - D / 2 for z in (*ZIP_Z, D - rz_, D - rz_ + 0.0006))}
    p, t = _rounded_box(half, R_EDGE, 0.0035, extra)
    p = p + centre
    flat_top = np.abs(p[:, 2] - D) < 1e-7
    flat_bot = np.abs(p[:, 2]) < 1e-7
    # grooves along the case's length on both panels, fading out 75-105 mm from each end
    prof = np.zeros(len(p))
    for x0 in np.arange(-3, 4) * 0.044:
        u = np.abs(p[:, 0] - x0) / 0.0075
        prof = np.maximum(prof, np.where(u < 1.0, np.cos(u * math.pi / 2) ** 2, 0.0))
    fade = np.clip((np.minimum(p[:, 1], L - p[:, 1]) - 0.075) / 0.030, 0.0, 1.0)
    groove = 0.0022 * prof * fade
    p[flat_top, 2] -= groove[flat_top]
    p[flat_bot, 2] += groove[flat_bot]
    # the handle housing's shallow recess in the top end: the end panel steps back 12 mm there
    inside = (np.abs(p[:, 0]) < rx_ - 1e-4) & (p[:, 1] < ry_ - 1e-4) & (p[:, 2] > D - rz_ + 1e-4)
    p[inside, 1] = ry_
    n = _normals(p, t)
    rib = groove > 0.0011  # the groove bottoms a shade lighter: they catch the light like moulded grooves
    tri_rib = rib[t].all(axis=1) & (flat_top[t].all(axis=1) | flat_bot[t].all(axis=1))
    shell = MeshShape(p, t[~tri_rib], n, NAVY)
    ribs = MeshShape(*_subset(p, n, t, tri_rib, 0.0), NAVY_RIB)
    side = np.abs(n[:, 2]) < 0.35
    band = ((p[:, 2] >= ZIP_Z[0] - 1e-6) & (p[:, 2] <= ZIP_Z[1] + 1e-6) & side)[t].all(axis=1)
    zipm = MeshShape(*_subset(p, n, t, band, 0.0016), ZIP)
    rx, ry, rz = R_EDGE  # corner guards: the eight rounded corner caps, proud 2 mm
    corner = ((np.abs(p[:, 0]) >= W / 2 - rx - 1e-6) & ((p[:, 1] <= ry + 1e-6) | (p[:, 1] >= L - ry - 1e-6))
              & ((p[:, 2] <= rz + 1e-6) | (p[:, 2] >= D - rz - 1e-6)))[t].all(axis=1)
    guards = MeshShape(*_subset(p, n, t, corner, 0.0020), GUARD)
    return [shell, ribs, zipm, guards]


def _box(lo, hi, color, collide=False) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def side_bar_box() -> tuple[np.ndarray, np.ndarray]:
    """The side carry handle's bar, in the case's frame (lo, hi corners)."""
    f, t_, z0, z1 = SIDE_BAR
    return (np.array([-W / 2 - t_, L / 2 - SIDE_BAR_HALF, z0]), np.array([-W / 2 - f, L / 2 + SIDE_BAR_HALF, z1]))


def _case() -> Body:
    rx_, ry_, rz_ = REC
    shapes: list = _shell_meshes()
    # the handle's moulded housing lining the recess, with the rails' two openings
    shapes += [_box((-rx_, ry_ - 0.0015, D - rz_), (rx_, ry_, D - 0.010), HOUSING),
               _box((-rx_, 0.0, D - rz_ - 0.001), (rx_, ry_, D - rz_ + 0.001), HOUSING),
               _box((-rx_, 0.0, D - rz_), (-rx_ + 0.0012, ry_, D - 0.010), HOUSING),
               _box((rx_ - 0.0012, 0.0, D - rz_), (rx_, ry_, D - 0.010), HOUSING)]
    for s in (-1, 1):
        shapes.append(_box((s * TUBE_X - 0.016, ry_ - 0.0018, TUBE_Z - 0.012), (s * TUBE_X + 0.016, ry_ - 0.0014,
                                                                                 TUBE_Z + 0.012), RUBBER))
    # the top carry handle, on the top end toward the front panel (down)
    for x in (-0.062, 0.062):
        shapes.append(_box((x - 0.012, -0.012, 0.040), (x + 0.012, 0.004, 0.072), PLASTIC))
    shapes.append(_box((-0.070, -0.019, 0.046), (0.070, -0.008, 0.066), RUBBER))
    # the side carry handle on the near side: two mounts and the bar between them (22-34 mm off the side)
    lo, hi = side_bar_box()
    for y in (L / 2 - SIDE_BAR_HALF + 0.006, L / 2 + SIDE_BAR_HALF - 0.006):
        shapes.append(_box((-W / 2 - SIDE_BAR[1], y - 0.008, SIDE_BAR[2] - 0.004),
                           (-W / 2 + 0.004, y + 0.008, SIDE_BAR[3] + 0.004), PLASTIC, collide=True))
    shapes.append(_box(lo, hi, RUBBER, collide=True))
    # TSA combination lock on the near side, at the zip, and the two zip pullers beside it
    shapes += [_box((-W / 2 - 0.010, 0.060, 0.074), (-W / 2 + 0.004, 0.125, 0.102), PLASTIC),
               _box((-W / 2 - 0.012, 0.071, 0.081), (-W / 2 - 0.009, 0.103, 0.095), BUTTON),
               _box((-W / 2 - 0.006, 0.131, 0.084), (-W / 2 - 0.002, 0.153, 0.091), ALU),
               _box((-W / 2 - 0.006, 0.156, 0.084), (-W / 2 - 0.002, 0.178, 0.091), ALU)]
    # four double spinner casters on the wheel end (lying on its front, they point along +y)
    for x in (-W / 2 + 0.050, W / 2 - 0.050):
        for z in (0.046, D - 0.046):
            shapes.append(_box((x - 0.030, L - 0.012, z - 0.026), (x + 0.030, L + 0.016, z + 0.026), PLASTIC))
            shapes.append(CylShape((x, L + 0.022, z), 0.010, 0.014, ALU, collide=False, axis="Y"))
            shapes.append(_box((x - 0.018, L + 0.024, z - 0.010), (x + 0.018, L + 0.032, z + 0.010), PLASTIC))
            for dz in (-0.0085, 0.0085):
                shapes.append(CylShape((x, L + 0.042, z + dz), CASTER_R, CASTER_W * 0.62, RUBBER, collide=False))
                shapes.append(CylShape((x, L + 0.042, z + dz), 0.011, CASTER_W * 0.70, ALU, collide=False))
    # colliders: the shell without the housing's recess, and the casters
    shapes += [_box((-W / 2, 0.0, 0.0), (W / 2, L, D - rz_), None, True),
               _box((-W / 2, 0.0, D - rz_), (-rx_, L, D), None, True),
               _box((rx_, 0.0, D - rz_), (W / 2, L, D), None, True),
               _box((-rx_, ry_, D - rz_), (rx_, L, D), None, True)]
    for x in (-W / 2 + 0.050, W / 2 - 0.050):  # one collider round each caster (the end panel between them is free)
        for z in (0.046, D - 0.046):
            shapes.append(_box((x - 0.030, L, z - 0.027), (x + 0.030, L + 0.067, z + 0.027), None, True))
    return Body.build("case", pos=(CX, Y0, CASE_Z), mass=CASE_MASS, shapes=shapes, color=NAVY,
                      friction=(0.8, 0.7), com=(0.0, L / 2, D / 2), damping=(0.05, 0.1))


def _mid_tubes() -> Body:
    """The two middle tube sections (closed: inside the case, behind the housing)."""
    w, h = MID_TUBE
    y0 = REC[1] + 0.028
    shapes = [_box((s * TUBE_X - w / 2, y0, TUBE_Z - h / 2), (s * TUBE_X + w / 2, y0 + 0.300, TUBE_Z + h / 2), ALU,
                   collide=s > 0) for s in (-1, 1)]
    return Body.build("mid_tubes", pos=(CX, Y0, CASE_Z), mass=0.12, shapes=shapes, color=ALU, ignore=("case",))


def _grip() -> Body:
    """The grip on its two posts, and the two inner tube sections it carries."""
    gx, gt, gh = GRIP
    w, h = INNER_TUBE
    y0, y1 = GRIP_Y
    zc = GRIP_Z
    r = gt / 2
    shapes = [_box((-gx + r, y0, zc - gh / 2), (gx - r, y1, zc + gh / 2), RUBBER, collide=True),
              CylShape((-gx + r, (y0 + y1) / 2, zc), r, gh, RUBBER, axis="Z"),
              CylShape((gx - r, (y0 + y1) / 2, zc), r, gh, RUBBER, axis="Z"),
              # the grip's soft top, and the rim around the button on its end face
              _box((-gx + r, y0 + 0.003, zc + gh / 2), (gx - r, y1 - 0.003, zc + gh / 2 + 0.003), RUBBER),
              _box((-0.033, y0 - 0.0008, zc - 0.011), (0.033, y0, zc + 0.011), PLASTIC)]
    for s in (-1, 1):
        shapes.append(_box((s * TUBE_X - 0.012, y1, zc - 0.012), (s * TUBE_X + 0.012, REC[1] + 0.004, zc + 0.012),
                           PLASTIC, collide=True))
        shapes.append(_box((s * TUBE_X - w / 2, REC[1], TUBE_Z - h / 2), (s * TUBE_X + w / 2, REC[1] + 0.330,
                                                                           TUBE_Z + h / 2), ALU))
    return Body.build("grip", pos=(CX, Y0, CASE_Z), mass=0.22, shapes=shapes, color=RUBBER, friction=(1.0, 0.9),
                      ignore=("case", "mid_tubes"))


def _button() -> Body:
    bx, bz = BUTTON_SIZE
    y0 = GRIP_Y[0] - 0.0008
    shapes = [_box((-bx / 2, GRIP_Y[0] - BUTTON_TRAVEL, GRIP_Z - bz / 2), (bx / 2, GRIP_Y[0] + 0.002, GRIP_Z + bz / 2),
                   BUTTON, collide=True)]  # 4 mm proud: fully in when flush with the grip's end face
    return Body.build("button", pos=(CX, Y0, CASE_Z), mass=0.01, shapes=shapes, color=BUTTON, friction=(1.0, 0.9),
                      ignore=("case", "mid_tubes"))


def _felt() -> Body:
    o = np.array([(FELT_X[0] + FELT_X[1]) / 2, 0.0, TOP])
    shapes = [_box((FELT_X[0] - o[0], FELT_Y[0], 0.0), (FELT_X[1] - o[0], FELT_Y[1], FELT_T), FELT, collide=True)]
    return Body.build("bench_felt", pos=tuple(o.tolist()), mass=1.0, kinematic=True, shapes=shapes, color=FELT,
                      friction=FELT_MU, slick=True)


# -- the reject tags and their stand ----------------------------------------------------------------------------------
# A steel desk stand at the bench's front edge, between the arms, with a rod running sideways (along y); the tags hang
# on it by their hooks, cards facing the robot. The same hook fits over the case's side carry handle.
ROD_X, ROD_Y, ROD_Z, ROD_R = 0.072, (-0.135, 0.070), 0.130, 0.004  # rod line (x), its ends (y), height, radius
TAG = (0.003, 0.064, 0.080)  # the card: thickness (x), width (y), height (z)
HOOK_GAP = 0.038  # stem to lip: the bend is 33 mm wide inside, over the 8 mm rod or the 12 mm side-handle bar
CARD_TOP = -0.026  # the card's top, below the bend's inside top (the tag's origin)
KNOB = (0.011, 0.027, 0.006, 0.004, 0.026)  # the knob on top of the bend: x from/to, half width (y), z from/to
TAG_YS = (-0.098, -0.030, 0.038)
TAG_MASS = 0.022


def _rack() -> Body:
    o = np.array([ROD_X, (ROD_Y[0] + ROD_Y[1]) / 2, BENCH])
    post_y = ROD_Y[0] - 0.008 - o[1]
    shapes = [_box((-0.040, post_y - 0.040, 0.0), (0.040, post_y + 0.040, 0.008), STEEL, collide=True),
              CylShape((0.0, post_y, 0.008 + (ROD_Z - 0.002) / 2), 0.0065, ROD_Z - 0.002, STEEL),
              SphereShape((0.0, post_y, ROD_Z), 0.0085, STEEL),
              CylShape((0.0, (ROD_Y[1] - ROD_Y[0]) / 2 + post_y + 0.004, ROD_Z), ROD_R, ROD_Y[1] - ROD_Y[0], ALU,
                       axis="Y"),
              SphereShape((0.0, ROD_Y[1] - o[1], ROD_Z), 0.0055, STEEL)]
    for k in (-1, 1):  # rubber feet
        shapes.append(_box((-0.036, post_y + k * 0.030 - 0.006, -0.0005), (0.036, post_y + k * 0.030 + 0.006, 0.0),
                           RUBBER))
    return Body.build("tag_rack", pos=tuple(o.tolist()), mass=1.2, kinematic=True, shapes=shapes, color=STEEL)


def _tag_shapes() -> list:
    """A rigid red PVC reject tag with a moulded hook, in its own frame: the card in the y-z plane facing -x, the hook
    bending toward +x over a bar, a knob on top of the bend to hold it by; origin at the bend's inside top."""
    tt, tw, th = TAG
    g = HOOK_GAP
    ct = CARD_TOP
    kx0, kx1, ky, kz0, kz1 = KNOB
    return [_box((-tt / 2, -tw / 2, ct - th), (tt / 2, tw / 2, ct), TAG_RED, collide=True),
            _box((-0.003, -0.012, ct - 0.002), (0.003, 0.012, ct + 0.006), TAG_RED, collide=True),
            _box((-0.0025, -0.004, ct + 0.004), (0.0025, 0.004, 0.0), TAG_RED, collide=True),
            _box((-0.0025, -0.004, 0.0), (g + 0.0025, 0.004, 0.004), TAG_RED, collide=True),
            _box((g - 0.0025, -0.004, -0.014), (g + 0.0025, 0.004, 0.004), TAG_RED, collide=True),
            _box((kx0, -ky, kz0), (kx1, ky, kz1), TAG_RED, collide=True),
            # the printed white panel ("REJECT") on both faces, and the string hole's black grommet
            _box((-tt / 2 - 0.0004, -0.026, ct - 0.042), (-tt / 2, 0.026, ct - 0.014), WHITE),
            _box((tt / 2, -0.026, ct - 0.042), (tt / 2 + 0.0004, 0.026, ct - 0.014), WHITE),
            _box((-tt / 2 - 0.0005, -0.018, ct - 0.032), (-tt / 2 - 0.0004, 0.018, ct - 0.024), TAG_RED),
            _box((-tt / 2 - 0.0006, -0.004, ct - 0.070), (tt / 2 + 0.0006, 0.004, ct - 0.062), RUBBER)]


def _tag(name: str, y: float) -> Body:
    pos = (ROD_X - HOOK_GAP / 2, y, BENCH + ROD_Z + ROD_R + 0.0005)
    return Body.build(name, pos=pos, mass=TAG_MASS, shapes=_tag_shapes(), color=TAG_RED, friction=(0.9, 0.8),
                      damping=(0.5, 0.5), com=(0.002, 0.0, CARD_TOP - 0.035))


TAGS = [f"tag_{k}" for k in range(len(TAG_YS))]
OBJECTS = [_felt(), _case(), _mid_tubes(), _grip(), _button(), _rack()] + [
    _tag(n, y) for n, y in zip(TAGS, TAG_YS)]

# The handle: the grip (with the inner tubes) slides out of the case toward -y; it starts locked closed (limits shut
# about 0). The middle tubes follow it for looks (inner section first). The button slides into the grip (+y).
slider("handle_slide", "case", "grip", anchor=(CX, Y0 + 0.2, CASE_Z + TUBE_Z), axis="Y", limits=(-0.0005, 0.0005),
       stiffness=0.0, damping=SLIDE_DAMPING, target=0.0, max_force=WORN_HOLD)
slider("mid_slide", "case", "mid_tubes", anchor=(CX, Y0 + 0.2, CASE_Z + TUBE_Z), axis="Y", limits=(-0.30, 0.0),
       stiffness=3000.0, damping=60.0, target=0.0, max_force=80.0)
slider("button_slide", "grip", "button", anchor=(CX, Y0 + GRIP_Y[0], CASE_Z + GRIP_Z), axis="Y",
       limits=(0.0, BUTTON_TRAVEL), stiffness=750.0, damping=2.0, target=-0.0067, max_force=12.0)


# -- the trial's seed (which lock, if any, is worn) ------------------------------------------------------------------

def _catch_seed() -> None:
    """Wrap the embodiment's reset so the task file sees each trial's seed (kept on bodykit, shared by reloads)."""
    mod = sys.modules.get("yam_arena.inspect_embodiment")
    if mod is None:
        plain = bodykit._TARGETS["yam_arena.inspect_embodiment"]

        def patch(module):
            plain(module)
            _wrap(module)

        bodykit._TARGETS["yam_arena.inspect_embodiment"] = patch
    else:
        _wrap(mod)


def _wrap(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_suitcase_seed", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        bodykit._TRIAL_SEED = kwargs.get("seed", args[1] if len(args) > 1 else None)
        return plain(self, *args, **kwargs)

    cls.reset = reset
    cls._suitcase_seed = True


_catch_seed()


# -- the handle's pins: followed after every control step -------------------------------------------------------------

_LIMITS: dict = {}


def _handle_limits(lo: float, hi: float) -> None:
    """Set the handle slide's stops (m along its axis; extension is negative)."""
    if "attrs" not in _LIMITS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        joint = UsdPhysics.PrismaticJoint(get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS["handle_slide"]))
        _LIMITS["attrs"] = (joint.GetLowerLimitAttr(), joint.GetUpperLimitAttr())
    a_lo, a_hi = _LIMITS["attrs"]
    a_lo.Set(float(lo))
    a_hi.Set(float(hi))


def _fresh() -> dict:
    return {"s": 0.0, "s_prev": None, "button": 0.0, "pin_out": False, "lock": 0, "worn_hold": None,
            "locked_at": set(), "slipped": False, "tipped": False, "start": None, "case": None, "tags": {},
            "tcps": None, "variant": None, "mid": None, "steps": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SUITCASE_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _EP["variant"] = variant_for(getattr(bodykit, "_TRIAL_SEED", None))
    if "attrs" in _LIMITS or bodykit._SLIDER_PATHS.get("handle_slide"):
        try:
            _handle_limits(-0.0005, 0.0005)  # locked closed
        except Exception:  # noqa: BLE001 - the stage is not up yet at the very first reset
            pass


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


def _lock_at(k: int) -> None:
    s_k = STOPS[k]
    _handle_limits(-s_k - 0.0005, -s_k + 0.0005)
    _EP["lock"] = k
    if k > 0:
        _EP["locked_at"].add(k)


def _unlock() -> None:
    _handle_limits(-S_MAX, 0.0)
    if _EP["worn_hold"] is not None:
        set_slider_drive("handle_slide", stiffness=0.0)
    _EP["lock"] = None
    _EP["worn_hold"] = None


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    ep = _EP
    if ep["variant"] is None:
        ep["variant"] = variant_for(getattr(bodykit, "_TRIAL_SEED", None))
    pc, rc = _pose(base, origin, "case")
    pg, rg = _pose(base, origin, "grip")
    pb, _ = _pose(base, origin, "button")
    s = float(-(rc.T @ (pg - pc))[1])  # how far the handle is out (m)
    b = float((rg.T @ (pb - pg))[1])  # how far the button is pressed in (m)
    if ep["start"] is None:
        ep["start"] = (pc.copy(), rc.copy())
    ep["case"] = (pc, rc)
    ep["s"], ep["button"] = s, b
    ep["steps"] += 1
    if math.degrees(math.acos(max(-1.0, min(1.0, float(rc[2, 2]))))) > 30.0:
        ep["tipped"] = True
    # the pins: out while the button is held in, back once it is let up (hysteresis)
    if b >= PIN_OUT:
        ep["pin_out"] = True
    elif b < PIN_IN:
        ep["pin_out"] = False
    worn = ep["variant"]  # 0: none
    s_prev = s if ep["s_prev"] is None else ep["s_prev"]
    if ep["pin_out"]:
        if ep["lock"] is not None or ep["worn_hold"] is not None:
            _unlock()
    elif ep["lock"] is None:
        if ep["worn_hold"] is not None:  # the worn hole holds until a firm push forces the pin out of it
            k = ep["worn_hold"]
            if abs(s - STOPS[k]) > SLIP_GAP:
                ep["slipped"] = True
                ep["worn_hold"] = None
                set_slider_drive("handle_slide", stiffness=0.0)
        else:
            for k, s_k in enumerate(STOPS):
                if abs(s - s_k) <= CAPTURE or (s_prev - s_k) * (s - s_k) < 0.0:
                    if k == worn and k > 0:
                        if abs(s - s_k) <= CAPTURE + 0.002 or (s_prev - s_k) * (s - s_k) < 0.0:
                            set_slider_drive("handle_slide", target=-s_k, stiffness=WORN_K)
                            ep["worn_hold"] = k
                    else:
                        _lock_at(k)
                    break
    ep["s_prev"] = s
    m = max(0.0, s - STAGE_IN)  # the middle tubes follow (looks)
    if ep["mid"] is None or abs(m - ep["mid"]) > 0.0005:
        set_slider_drive("mid_slide", target=-m)
        ep["mid"] = m
    for name in TAGS:
        ep["tags"][name] = _pose(base, origin, name)
    ep["tcps"] = _tcps(base, origin)
    if _DEBUG["on"] and ep["steps"] % 5 == 0:
        st = _status()
        tl, tr = ep["tcps"]
        dpos = (pc - ep["start"][0]) * 1000
        print(f"[suitcase] t={ep['steps'] / 20:5.1f}s v={worn} s={s * 1000:6.1f}mm btn={b * 1000:4.1f}mm "
              f"pin_out={ep['pin_out']} lock={ep['lock']} worn_hold={ep['worn_hold']} slipped={ep['slipped']} "
              f"locked_at={sorted(ep['locked_at'])} case_d={np.round(dpos).astype(int).tolist()} "
              f"yaw={st['yaw']:.1f} grip={np.round(pg * 1000).astype(int).tolist()} "
              f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
              f"tags={st['tag_state']} score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring ----------------------------------------------------------------------------------------------------------
CLOSED_WITHIN = 0.003
CASE_MOVE, CASE_TURN = 0.050, 10.0
HOME_WITHIN = 0.05
TAG_CLEAR = 0.06  # no gripper TCP this close to a tag's knob


def _hook_point(p, r) -> np.ndarray:
    return p + r @ np.array([HOOK_GAP / 2, 0.0, 0.0])


def _tag_state(name: str) -> str:
    """'hung' (hanging by its hook from the case's side carry handle or from its closed grip, off the bench), 'on_case'
    (resting on the case some other way), or 'off'."""
    if name not in _EP["tags"] or _EP["case"] is None:
        return "off"
    p, r = _EP["tags"][name]
    pc, rc = _EP["case"]
    h = rc.T @ (_hook_point(p, r) - pc)  # the bend's middle, in the case's frame
    low = min(float((p + r @ np.array([0.0, s * TAG[1] / 2, CARD_TOP - TAG[2]]))[2]) for s in (-1, 1))
    off_bench = low > BENCH + 0.008
    lo, hi = side_bar_box()
    on_side = (abs(h[0] - (lo[0] + hi[0]) / 2) < 0.016 and lo[1] - 0.005 < h[1] < hi[1] + 0.005
               and hi[2] - 0.004 < h[2] < hi[2] + 0.014)
    on_grip = (_EP["s"] < 0.01 and abs(h[1] - sum(GRIP_Y) / 2) < 0.016 and abs(h[0]) < GRIP[0]
               and GRIP_Z + GRIP[2] / 2 - 0.004 < h[2] < GRIP_Z + GRIP[2] / 2 + 0.014)
    knob = p + r @ np.array([(KNOB[0] + KNOB[1]) / 2, 0.0, KNOB[4]])
    clear = _EP["tcps"] is None or all(float(np.linalg.norm(t - knob)) > TAG_CLEAR for t in _EP["tcps"])
    if (on_side or on_grip) and off_bench and clear:
        return "hung"
    hp = _hook_point(p, r)
    if abs(hp[0] - ROD_X) < 0.02 and abs(hp[2] - (BENCH + ROD_Z)) < 0.02 and ROD_Y[0] - 0.01 < hp[1] < ROD_Y[1] + 0.01:
        return "stand"
    c = rc.T @ (p + r @ np.array([0.0, 0.0, CARD_TOP - TAG[2] / 2]) - pc)
    near = -W / 2 - 0.05 < c[0] < W / 2 + 0.02 and -0.06 < c[1] < L + 0.07 and c[2] < D + 0.05
    return "on_case" if (near and off_bench) else "off"


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    home = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float) for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, home))


def _status() -> dict:
    ep = _EP
    variant = ep["variant"] if ep["variant"] is not None else variant_for(None)
    tags = {n: _tag_state(n) for n in TAGS}
    moved, yaw = 0.0, 0.0
    if ep["start"] is not None and ep["case"] is not None:
        (p0, r0), (p1, r1) = ep["start"], ep["case"]
        moved = float(np.linalg.norm((p1 - p0)[:2]))
        yaw = abs(math.degrees(math.atan2(float((r0.T @ r1)[1, 0]), float((r0.T @ r1)[0, 0]))))
    if variant == 0:
        checked = ep["locked_at"] >= {1, 2, 3, 4}
        verdict = all(v in ("off", "stand") for v in tags.values())
    else:
        checked = ep["slipped"]
        verdict = any(v == "hung" for v in tags.values())
    return {"variant": variant, "checked": checked, "verdict": verdict, "tag_state": tags,
            "closed": ep["s"] < CLOSED_WITHIN and ep["lock"] == 0, "moved": moved, "yaw": yaw,
            "in_place": moved < CASE_MOVE and yaw < CASE_TURN, "tipped": ep["tipped"], "home": _arms_home()}


def _points(st) -> float:
    if st["tipped"] or not st["checked"] or not st["verdict"]:
        return 0.0
    return 1.0 if (st["closed"] and st["in_place"] and st["home"]) else 0.5


def score(objects) -> float:
    """Judged at the end of the run (the case's tipping at any time). Checked: on a good unit, the handle was locked
    (pins in, button out) at each of its four heights at some point; on a unit with a worn lock, that lock was made to
    give at some point (pushed out of its hole). Verdict right: a good unit has no tag on it; a worn one has a tag
    hanging by its hook from the case (its side carry handle, or its closed grip), off the bench, no gripper within
    6 cm. 0.0 if the case ever tipped over (30 deg), if the handle was not checked, or if the verdict is wrong.
    Otherwise 1.0 when the handle is closed and locked, the case is within 50 mm and 10 deg of where it lay, and both
    arms are back where they started (each TCP within 5 cm); else 0.5."""
    return _points(_status())


def success(objects) -> bool:
    """Full score, with the case and the tags still."""
    return score(objects) >= 1.0 and objects["case"].resting() and all(objects[n].resting() for n in TAGS)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Written for the demo's trial (DEMO_SEED: the lock at 40 in is worn). The left hand braces the case at its wheel end,
# its closed fingers against the end panel between the casters. The right hand pinches the grip from above, one pad on
# the button and one in the finger gap, so the pinch holds the button in. At each height it draws the handle 25 mm
# past the stop, lets the button up, moves its pinch 45 mm along the grip beside the button and pushes the handle back
# toward the case, aiming 70 mm past the stop (about 20 N): the pin drops into its hole and the handle must hold. At 40 in it gives and
# slides back, so the inspection stops there: the right hand pinches the button again and pushes the handle all the
# way in, lets go (the pin clicks in closed) and goes back. The left hand lets go of the case, takes the nearest red
# tag off the stand by its knob, hangs it on the case's side carry handle and goes back.
# SUITCASE_PLAN=pass plays the good-unit plan (all four heights, no tag); SUITCASE_ONEHAND=1 leaves the left hand home.

LEAN = math.radians(-20.0)  # the right pinch leans so its fingertips point away from the robot (reach)
BX, DX = -0.005, 0.040  # the pinch on the button (5 mm toward the robot: the lean drifts it out), and beside it
OVER = 0.025  # draw this far past a height before letting the button up
PUSH = 0.070  # command the push this far past the height (the soft arm stalls ~4 cm short: about 20 N)
OVER_FULL = 0.012
PRE, EASE, SHUT = -0.022, -0.014, FINGER_CLOSED
TAG_PRE = -0.012
LEAD_S, LEAD_STEPS = 3.0, 15


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in pos), float(finger), label, rot)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.6, 0.0, 0.6), (0.8, 1.4), (0.5, 1.1), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4),
                                  (-0.6, 0.0, 0.6)):
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
        out.append(_w(LEAD_S / LEAD_STEPS, p, finger, label, r))
    return out


def _lead_in(arm, pos, rot, label, finger):
    from yam_arena.constants import READY

    return _sweep(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm, pos, rot, finger):
    from yam_arena.constants import READY

    return _sweep(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _world(p_case) -> np.ndarray:
    return np.array([CX, Y0, CASE_Z]) + np.asarray(p_case, float)


GRIP_YC = sum(GRIP_Y) / 2  # the grip's middle (case frame y), closed


def _grip_at(s: float, dx: float = BX, dz: float = 0.006) -> np.ndarray:
    """The right TCP pinching the grip when the handle is ``s`` out: ``dx`` along it, ``dz`` above its middle."""
    return _world((dx, GRIP_YC - s, GRIP_Z + dz))


def _right(plan: str) -> tuple[list[Waypoint], float]:
    rot = _rot_y(LEAN) @ grip_rotation(0.0)  # fingers close along y (across the grip)
    out = _lead_in("right", _grip_at(0.0) + [0.0, 0.0, 0.06], rot, "over the grip, pads open across it", PRE)
    out += [_w(1.2, _grip_at(0.0), PRE, "pads either side of the grip: one in the finger gap", rot),
            _w(0.6, _grip_at(0.0), SHUT, "pinch: the far pad holds the button in", rot),
            _w(0.3, _grip_at(0.0), SHUT, "settle", rot)]
    heights = (1, 2, 3, 4) if plan == "pass" else (1, 2, 3)
    worn = 0 if plan == "pass" else 3  # the plan's trial: the 40 in lock gives and the handle slides back to 38 in
    s_now = 0.0
    for k in heights:
        s_k = STOPS[k]
        s_end = STOPS[k - 1] if k == worn else s_k
        ease = "it gave: the handle slid back to the height below; ease off" if k == worn else "it held; ease off"
        s_pull = s_k + (OVER if k < 4 else OVER_FULL)
        t_pull = 1.6 + 6.0 * (s_pull - s_now)
        out += [_w(t_pull, _grip_at(s_pull), SHUT, f"draw the handle out past height {k}", rot),
                _w(0.4, _grip_at(s_pull), EASE, "let the button up: the pin rides along the tube", rot),
                _w(0.3, _grip_at(s_pull), PRE, "open", rot),
                _w(0.6, _grip_at(s_pull, DX), PRE, "move the pinch beside the button", rot),
                _w(0.5, _grip_at(s_pull, DX), SHUT, "pinch the grip beside the button", rot),
                _w(1.2, _grip_at(s_k - PUSH, DX), SHUT, f"push the handle back: the pin must catch at height {k}",
                   rot),
                _w(0.8, _grip_at(s_k - PUSH, DX), SHUT, "keep pushing: a good lock holds", rot),
                _w(0.6, _grip_at(s_end, DX), SHUT, ease, rot),
                _w(0.3, _grip_at(s_end, DX), PRE, "open", rot),
                _w(0.6, _grip_at(s_end), PRE, "back over the button", rot),
                _w(0.5, _grip_at(s_end), SHUT, "pinch the button in again", rot)]
        s_now = s_end
    t_in = 2.0 + 6.0 * s_now
    out += [_w(t_in, _grip_at(-0.012), SHUT, "push the handle all the way in, button held", rot),
            _w(0.4, _grip_at(-0.012), SHUT, "at its stop", rot)]
    done = sum(w.duration for w in out)
    out += [_w(0.5, _grip_at(-0.004), EASE, "let the button up: the pin clicks in closed", rot),
            _w(0.4, _grip_at(0.0), PRE, "open", rot),
            _w(1.0, _grip_at(0.0) + [0.0, 0.0, 0.07], PRE, "rise clear", rot)]
    out += _go_home("right", _grip_at(0.0) + [0.0, 0.0, 0.07], rot, FINGER_OPEN)
    return out, done


BRACE_ROT = _rot_y(LEAN) @ grip_rotation(math.pi / 2)  # fingers closed along x: their thin face pushes along y
BRACE_Z = 0.110  # the brace's TCP height above the case's bottom (between the casters)
BRACE_LEAN = 0.004  # just touching: the brace only stops the case moving away


def _brace_at(lean: float) -> np.ndarray:
    return _world((0.0, L + 0.012 - lean, BRACE_Z))


def _left(plan: str, release_at: float, total: float) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    above = _brace_at(-0.03) + [0.0, 0.0, 0.11]
    out = _lead_in("left", above, BRACE_ROT, "over the case's wheel end", SHUT)
    out += [_w(1.4, _brace_at(-0.03), SHUT, "down between the casters", BRACE_ROT),
            _w(1.0, _brace_at(BRACE_LEAN), SHUT, "brace the case: fingers against its wheel end", BRACE_ROT)]
    t = sum(w.duration for w in out)
    out.append(_w(max(release_at - t, 0.3), _brace_at(BRACE_LEAN), SHUT, "hold the case while the handle is tried",
                  BRACE_ROT))
    out += [_w(0.8, _brace_at(-0.03), SHUT, "let go of the case", BRACE_ROT),
            _w(1.2, above, SHUT, "rise clear of the case", BRACE_ROT)]
    last = above
    if plan != "pass":
        rot = _rot_y(LEAN) @ grip_rotation(0.0)  # fingers close along y, across the tag's knob
        knob_off = np.array([(KNOB[0] + KNOB[1]) / 2, 0.0, 0.018])
        tag0 = np.array([ROD_X - HOOK_GAP / 2, TAG_YS[-1], BENCH + ROD_Z + ROD_R + 0.0005])
        lo, hi = side_bar_box()
        bar_x, bar_top = (lo[0] + hi[0]) / 2, hi[2]
        hang = _world((bar_x - HOOK_GAP / 2, L / 2, bar_top + 0.002))
        pick = tag0 + knob_off
        place = hang + knob_off
        out += _go_home("left", above, BRACE_ROT, TAG_PRE)  # over the case, back by way of the start
        high = np.array([0.20, 0.15, TOP + 0.245])  # over the case's near half, clear of its top
        out += _lead_in("left", high, rot, "over the case toward the tag stand", TAG_PRE)
        out += [_w(1.6, pick + [0.0, 0.0, 0.034], TAG_PRE, "over the nearest reject tag", rot)]
        out += [
                _w(1.0, pick, TAG_PRE, "fingers either side of its knob", rot),
                _w(0.5, pick, SHUT, "take it by the knob", rot),
                _w(1.0, pick + [0.0, 0.0, 0.034], SHUT, "lift it off the rod", rot),
                _w(2.4, place + [0.0, 0.0, 0.020], SHUT, "over the case's side carry handle", rot),
                _w(1.2, place, SHUT, "hook it over the handle", rot),
                _w(0.5, place, TAG_PRE, "let go: it hangs on the case", rot),
                _w(0.8, place + [0.0, 0.0, 0.020], TAG_PRE, "rise clear", rot),
                _w(0.8, np.array([0.11, place[1], TOP + 0.205]), TAG_PRE, "up beside the case", rot),
                _w(1.2, np.array([0.20, 0.25, TOP + 0.245]), TAG_PRE, "over the case's edge", rot)]
        out += _go_home("left", np.array([0.20, 0.25, TOP + 0.245]), rot, SHUT)
    else:
        out += _go_home("left", last, BRACE_ROT, SHUT)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 1.0), out[-1].pos, out[-1].finger,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out


def _view_pose(arm: str, target, eye, roll_deg: float = 0.0):
    """The TCP pose that puts ``arm``'s wrist camera at ``eye`` looking at ``target`` (debug close-up views only)."""
    from yam_arena import station
    from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

    cam = station.wrist_camera(arm)  # in the gripper link's frame, OpenGL (looks along -z, +y up)
    rc, pc = quat_xyzw_to_matrix(np.array(cam.rot_xyzw)), np.array(cam.pos)
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    up = np.array([0.0, 0.0, 1.0]) - f[2] * f
    up /= np.linalg.norm(up)
    up = axis_angle_matrix(f, math.radians(roll_deg)) @ up
    want = np.column_stack([np.cross(up, -f), up, -f])
    rg = want @ rc.T
    origin = np.asarray(eye, float) - rg @ pc
    return origin + rg @ np.array(TCP_OFFSET), rg


def demo():
    view = os.environ.get("SUITCASE_VIEW", "")
    if view:  # debug: one arm's wrist camera to a view point, the other at rest
        from yam_arena import ik, station
        from yam_arena.constants import READY

        plans = {}
        for arm in ("left", "right"):
            p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
            wps = [Waypoint(3.0, tuple(float(v) for v in p), FINGER_OPEN, "wait", r)]
            if view.startswith(arm):
                eye = [float(v) for v in os.environ.get("SUITCASE_EYE", "0.05,-0.20,1.10").split(",")]
                tgt = [float(v) for v in os.environ.get("SUITCASE_TGT", "0.30,0.10,0.85").split(",")]
                tcp, rot = _view_pose(arm, tgt, eye, float(os.environ.get("SUITCASE_ROLL", "0")))
                wps = [_w(3.0, tcp, FINGER_OPEN, "to the view point", rot), _w(3.0, tcp, FINGER_OPEN, "hold", rot)]
            plans[arm] = wps
        return plans
    plan = os.environ.get("SUITCASE_PLAN", "")
    right, done = _right(plan)
    total = sum(w.duration for w in right)
    left = _left(plan, done + 0.9, total)
    total = max(total, sum(w.duration for w in left))
    right.append(_w(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, right[-1].finger,
                    "wait at the start", right[-1].yaw))
    if os.environ.get("SUITCASE_ONEHAND"):
        from yam_arena import ik, station
        from yam_arena.constants import READY

        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        left = [Waypoint(total, tuple(float(v) for v in p), FINGER_OPEN, "one-hand check: the left arm stays home", r)]
    return {"left": left, "right": right}
