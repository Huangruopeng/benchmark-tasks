"""A planning whiteboard lies on a desk: a white board in an aluminium frame, ruled into a grid of three by three
squares under three coloured column tabs, with a pen tray along its near edge. Three round push-pin magnets (red, blue
and yellow) stand in three of the squares (which three depends on the trial's seed). Old blue and black marker notes
cover part of the board, and a felt whiteboard eraser lies in the pen tray. The desk top is a smooth
laminate.

Task: take the three magnets off the board and lay them in its pen tray. Then wipe all the old marker off with the
eraser, holding the board still with your other hand, and put the eraser back in the pen tray. Then put each magnet
back in the square it was in, and take both arms back to where they started.

What it tests: memory. The magnets mark where things stand on the plan; once they are in the pen tray and the board is
wiped, nothing on the board shows which square each one came from, and the wiping comes in between. A policy that sees
only the current frame can only guess (one chance in nine for each magnet). The squares depend on the seed, so a
fixed habit does not pass either.

Why two hands: the board lies on a smooth laminate desk (it slides at about 0.4 N), while the eraser's felt grips the
board's surface; rubbing it with nothing holding the board just drags the whole board along, and no marker comes off.
One hand presses the board's frame down while the other wipes.

Run it:  inspect-robots run --embodiment yam_arena -E task=put_each_magnet_back_in_its_square_after_wiping_the_whiteboard.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     plate, rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the desk's laminate top, under the board and the eraser (fixed) ----------------------------------------------------
DX0, DX1, DY0, DY1, DESK_H = 0.095, 0.455, -0.245, 0.245, 0.006
DT = TOP + DESK_H  # the desk's top
# -- the whiteboard: origin at the middle of its underside --------------------------------------------------------------
BC_X, BC_Y = 0.295, 0.0
BX, BY = 0.270, 0.360  # the board, front to back (x) and across (y)
PANEL_H = 0.010  # the writing surface is this far above the desk
FW, FRAME_UP = 0.012, 0.004  # the aluminium frame: its width, and how far it stands proud of the surface
SURF = DT + PANEL_H
IN_X0, IN_X1 = BC_X - BX / 2 + FW, BC_X + BX / 2 - FW  # the writing surface inside the frame (0.172 .. 0.418)
IN_Y = BY / 2 - FW  # 0.168
HEAD_X0 = IN_X1 - 0.026  # the header strip with the column tabs, along the far side
ROW_X = [IN_X0 + (HEAD_X0 - IN_X0) * (k + 0.5) / 3 for k in range(3)]  # the squares' middles, near row first
COL_Y = [IN_Y - 2 * IN_Y * (k + 0.5) / 3 for k in range(3)]  # left column (+y) first
SQ_X, SQ_Y = (HEAD_X0 - IN_X0) / 3, 2 * IN_Y / 3  # a square's size (0.073 x 0.112)
# the pen tray along the near edge, part of the board
TRAY_X0, TRAY_X1 = BC_X - BX / 2 - 0.064, BC_X - BX / 2  # 0.096 .. 0.160
TRAY_X = (TRAY_X0 + TRAY_X1) / 2
TRAY_HALF_Y, TRAY_WALL_T, TRAY_WALL_H, TRAY_FLOOR = 0.150, 0.003, 0.014, 0.003
# -- the magnets: origin at the middle of the disc's underside ----------------------------------------------------------
MD_R, MD_H, KN_R, KN_H = 0.015, 0.007, 0.0065, 0.020
MAG_H = MD_H + KN_H
MAGNETS = ("red", "blue", "yellow")
MAG_COLOR = {"red": (0.80, 0.10, 0.10), "blue": (0.12, 0.30, 0.78), "yellow": (0.95, 0.78, 0.10)}
MAG_DARK = {"red": (0.55, 0.06, 0.06), "blue": (0.07, 0.18, 0.50), "yellow": (0.70, 0.55, 0.05)}
TRAY_SPOT = {"red": 0.112, "blue": 0.062, "yellow": 0.012}  # where the demo lays each magnet in the pen tray (y)
# -- the eraser: origin at the middle of its felt's underside, lying along y in the right part of the pen tray ---------
ER_L, ER_W, FELT_H, BLOCK_H, RIDGE_L, RIDGE_W, RIDGE_H = 0.110, 0.048, 0.008, 0.016, 0.090, 0.018, 0.022
ER_H = FELT_H + BLOCK_H + RIDGE_H  # 0.046
ER_START = (TRAY_X, -0.088)
# -- the old marker notes: short strokes in two lines an eraser's width apart, across the middle and right columns ----
BAND_X = (0.256, 0.314)
# -- the rules ---------------------------------------------------------------------------------------------------------
RUB_TO_ERASE = 0.060  # a mark is gone once the felt has rubbed 6 cm over it
STILL_MM, STILL_DEG = 0.0015, 0.75  # rubbing counts only in steps the board stayed this still
FEET_K, FEET_F = 1000.0, 1.2  # the board's rubber feet: they hold it with up to 1.2 N each way, then it slides
FEET_SLIP = FEET_F / FEET_K
FELT_TOUCH = 0.003  # the felt touches the board within this of its surface
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
DEMO_SEED = 1415158438  # the CLI's default seed (--seed 0); batch_run passes no seed, so demo and Dream runs share it

WHITE, ALU, ALU_D = (0.95, 0.95, 0.94), (0.74, 0.76, 0.79), (0.55, 0.57, 0.60)
GRID = (0.45, 0.62, 0.85)
LAMINATE, LAMINATE_D = (0.62, 0.50, 0.38), (0.52, 0.41, 0.30)
INK = ((0.10, 0.20, 0.62), (0.08, 0.08, 0.10))
FELT, ER_BODY, ER_RIDGE = (0.22, 0.22, 0.24), (0.15, 0.40, 0.72), (0.08, 0.08, 0.09)
TABS = ((0.25, 0.65, 0.30), (0.95, 0.62, 0.15), (0.55, 0.55, 0.58))  # the column tabs: green, amber, grey


def magnet_cells(seed) -> dict:
    """The square (0-8: row * 3 + column, near row and left column first) each magnet stands in at the start."""
    s = DEMO_SEED if seed is None else int(seed)
    cells = np.random.default_rng(s).permutation(9)[:3]
    return {m: int(c) for m, c in zip(MAGNETS, cells)}


def cell_xy(c: int) -> tuple[float, float]:
    return ROW_X[c // 3], COL_Y[c % 3]


def _seed():
    """The seed of the trial being reset or run (kept on the embodiment class, so every copy of this file sees it)."""
    mod = sys.modules.get("yam_arena.inspect_embodiment")
    return getattr(getattr(mod, "YamArenaEmbodiment", None), "_wb_seed", None)


# -- the marks: (x, y, length, angle in degrees, ink) in the world at the start, each a short stroke ------------------
def _marks() -> list[tuple]:
    rng = np.random.default_rng(11)
    out = []
    for b, xb in enumerate(BAND_X):
        y = -0.122
        while y < -0.020:
            ln = rng.uniform(0.014, 0.022)
            out.append((xb + rng.uniform(-0.006, 0.006), y + ln / 2, ln, 90.0 + rng.uniform(-35, 35),
                        INK[(len(out) // 3) % 2]))
            y += ln + rng.uniform(0.004, 0.010)
    return out


MARKS = _marks()


def _desk() -> Body:
    shapes = [BoxShape((0.0, 0.0, DESK_H / 2), (DX1 - DX0, DY1 - DY0, DESK_H), LAMINATE)]
    rng = np.random.default_rng(3)
    for _ in range(10):  # wood-grain streaks
        shapes.append(BoxShape((rng.uniform(-0.16, 0.16), rng.uniform(-0.23, 0.23), DESK_H + 0.0002),
                               (rng.uniform(0.08, 0.20), rng.uniform(0.001, 0.003), 0.0004), LAMINATE_D,
                               collide=False))
    return Body.build("desk", pos=((DX0 + DX1) / 2, (DY0 + DY1) / 2, TOP), mass=1.0, kinematic=True,
                      color=LAMINATE, shapes=shapes, friction=(0.07, 0.06), slick=True)


MARK_SHAPES: list[int] = []  # the board's shape index of each mark


RAIL_X = BC_X + BX / 2 + 0.001 + 0.007  # the desk's wooden back rail, just behind the board's far edge


def _rail() -> Body:
    """A low oak rail fixed along the desk's back, which the board stands against."""
    shapes = [BoxShape((0.0, 0.0, 0.010), (0.014, 0.40, 0.020), LAMINATE_D),
              BoxShape((-0.0071, 0.0, 0.019), (0.0002, 0.40, 0.002), LAMINATE, collide=False)]
    return Body.build("rail", pos=(RAIL_X, 0.0, DT), mass=1.0, kinematic=True, color=LAMINATE_D, shapes=shapes,
                      friction=(0.1, 0.1), slick=True)


def _carriage() -> Body:
    """A hidden body under the table between the board's two feet joints (one along x, one along y)."""
    return Body.build("feet", pos=(BC_X, BC_Y, TOP - 0.05), mass=0.05, color=ALU,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.01, 0.01, 0.01), None, collide=False)])


def _board() -> Body:
    hx, hy = BX / 2, BY / 2
    fh = PANEL_H + FRAME_UP
    shapes = [BoxShape((0.0, 0.0, PANEL_H / 2), (BX - 0.002, BY - 0.002, PANEL_H), WHITE)]
    for s in (-1.0, 1.0):  # the frame
        shapes.append(BoxShape((s * (hx - FW / 2), 0.0, fh / 2), (FW, BY, fh), ALU))
        shapes.append(BoxShape((0.0, s * (hy - FW / 2), fh / 2), (BX, FW, fh), ALU))
    for sx in (-1.0, 1.0):  # grey plastic corner caps
        for sy in (-1.0, 1.0):
            shapes.append(BoxShape((sx * (hx - 0.010), sy * (hy - 0.010), fh + 0.0004), (0.020, 0.020, 0.0008),
                                   (0.30, 0.31, 0.33), collide=False))
    z = PANEL_H + 0.0002
    x0, x1 = IN_X0 - BC_X, HEAD_X0 - BC_X
    for k in range(4):  # the grid: lines across and along
        x = x0 + (x1 - x0) * k / 3
        shapes.append(BoxShape((x, 0.0, z), (0.0016, 2 * IN_Y, 0.0004), GRID, collide=False))
        y = IN_Y - 2 * IN_Y * k / 3
        shapes.append(BoxShape(((x0 + x1) / 2, y, z), (x1 - x0, 0.0016, 0.0004), GRID, collide=False))
    for k in range(3):  # the column tabs in the header strip
        shapes.append(BoxShape((x1 + 0.013, COL_Y[k] - BC_Y, z), (0.016, SQ_Y - 0.030, 0.0005), TABS[k],
                               collide=False))
    for k in range(3):  # row markers: a small coloured dot in each row's left square corner
        shapes.append(CylShape((ROW_X[k] - BC_X + SQ_X / 2 - 0.010, IN_Y - 0.010, z), 0.004, 0.0005,
                               ((0.55, 0.25, 0.60), (0.15, 0.55, 0.55), (0.85, 0.40, 0.20))[k], collide=False))
    # the pen tray: floor, front lip and ends (its back is the frame)
    tx0, tx1 = TRAY_X0 - BC_X, TRAY_X1 - BC_X
    shapes.append(BoxShape(((tx0 + tx1) / 2, 0.0, TRAY_FLOOR / 2), (tx1 - tx0, 2 * TRAY_HALF_Y, TRAY_FLOOR), ALU))
    shapes.append(BoxShape((tx0 + TRAY_WALL_T / 2, 0.0, TRAY_WALL_H / 2), (TRAY_WALL_T, 2 * TRAY_HALF_Y, TRAY_WALL_H),
                           ALU))
    for s in (-1.0, 1.0):
        shapes.append(BoxShape(((tx0 + tx1) / 2, s * (TRAY_HALF_Y - TRAY_WALL_T / 2), TRAY_WALL_H / 2),
                               (tx1 - tx0, TRAY_WALL_T, TRAY_WALL_H), ALU_D))
    MARK_SHAPES.clear()
    for (mx, my, ln, ang, ink) in MARKS:  # the old marker notes
        MARK_SHAPES.append(len(shapes))
        shapes.append(BoxShape((mx - BC_X, my - BC_Y, z + 0.0002), (ln, 0.0022, 0.0004), ink, collide=False,
                               yaw=math.radians(ang)))
    return Body.build("board", pos=(BC_X, BC_Y, DT), mass=0.6, color=WHITE, shapes=shapes, friction=(0.9, 0.8),
                      com=(0.0, 0.0, PANEL_H / 2))


def _magnet(name: str) -> Body:
    """A round push-pin magnet: a disc with a knob on top, in one colour."""
    c = magnet_cells(None)[name]
    x, y = cell_xy(c)
    shapes = [CylShape((0.0, 0.0, MD_H / 2), MD_R, MD_H, MAG_COLOR[name]),
              CylShape((0.0, 0.0, MD_H + KN_H / 2), KN_R, KN_H, MAG_COLOR[name]),
              CylShape((0.0, 0.0, MD_H + 0.0003), MD_R - 0.0015, 0.0006, MAG_DARK[name], collide=False),
              CylShape((0.0, 0.0, MAG_H + 0.0003), KN_R - 0.0012, 0.0006, MAG_DARK[name], collide=False)]
    return Body.build(f"magnet_{name}", pos=(x, y, SURF), mass=0.012, color=MAG_COLOR[name], shapes=shapes,
                      friction=(1.0, 0.9), com=(0.0, 0.0, 0.006))


def _eraser() -> Body:
    """A whiteboard eraser: dark felt under a blue plastic block with a black handle ridge, lying along y."""
    shapes = [BoxShape((0.0, 0.0, FELT_H / 2), (ER_W, ER_L, FELT_H), FELT),
              BoxShape((0.0, 0.0, FELT_H + BLOCK_H / 2), (ER_W + 0.002, ER_L + 0.002, BLOCK_H), ER_BODY),
              BoxShape((0.0, 0.0, FELT_H + BLOCK_H + RIDGE_H / 2), (RIDGE_W, RIDGE_L, RIDGE_H), ER_RIDGE)]
    return Body.build("eraser", pos=(ER_START[0], ER_START[1], DT + TRAY_FLOOR), mass=0.06, color=ER_BODY,
                      shapes=shapes, friction=(1.0, 0.95), com=(0.0, 0.0, 0.016))


OBJECTS = [_desk(), _rail(), _carriage(), _board(), *[_magnet(m) for m in MAGNETS], _eraser()]
# the board's rubber feet on the desk: a sideways hold of up to FEET_F each way (no turning), moved along as it slips
slider("feet_x", "desk", "feet", anchor=(BC_X, BC_Y, TOP - 0.05), axis="X", limits=(-0.12, 0.12),
       stiffness=FEET_K, damping=5.0, target=0.0, max_force=FEET_F)
slider("feet_y", "feet", "board", anchor=(BC_X, BC_Y, TOP - 0.05), axis="Y", limits=(-0.12, 0.12),
       stiffness=FEET_K, damping=5.0, target=0.0, max_force=FEET_F)
_SPEC_BY_NAME = {o.name: o for o in OBJECTS}

# -- the trial's squares: the seed is caught as the embodiment resets, and each magnet is spawned (first reset) or put
# back (later resets) in its square for that seed ----------------------------------------------------------------------


def _magnet_start(name: str, seed) -> tuple[float, float, float]:
    x, y = cell_xy(magnet_cells(seed)[name.split("_", 1)[1]])
    return (x, y, SURF)


def _patch_scene(module) -> None:
    if getattr(module, "_wb_patched", False):
        return
    plain = module.task_object

    def task_object(spec):
        if isinstance(spec, Body) and spec.name.startswith("magnet_"):
            spec = dataclasses.replace(spec, pos=_magnet_start(spec.name, _seed()))
        return plain(spec)

    module.task_object, module._wb_patched = task_object, True


def _patch_reset(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_wb_seed_caught", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        type(self)._wb_seed = kwargs.get("seed")
        base = getattr(self, "_base", None)
        if base is not None:  # a later reset: move each magnet's default pose to its square for this seed
            try:
                for m in MAGNETS:
                    obj = base.scene[f"magnet_{m}"]
                    pose = obj.data.default_root_pose.torch
                    x, y, z = _magnet_start(f"magnet_{m}", kwargs.get("seed"))
                    pose[:, 0], pose[:, 1], pose[:, 2] = x, y, z
            except Exception as e:  # noqa: BLE001
                print(f"[wb] could not move the magnets' start: {e}", file=sys.stderr, flush=True)
        return plain(self, *args, **kwargs)

    cls.reset, cls._wb_seed_caught = reset, True


def _hook(name: str, patch_fn) -> None:
    if name in sys.modules:
        patch_fn(sys.modules[name])
        return
    plain = bodykit._TARGETS.get(name)

    def patch(module) -> None:  # bodykit patches the module first, then this wraps it
        if plain is not None:
            plain(module)
        patch_fn(module)

    bodykit._TARGETS[name] = patch


_hook("yam_arena.scene", _patch_scene)
_hook("yam_arena.inspect_embodiment", _patch_reset)

# the board's prim path, noted as it is spawned (to hide the marks as they are rubbed off)
_PATHS: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "board":
        _PATHS["board"] = prim_path


bodykit._author_sliders = _author_sliders


def _show_mark(i: int, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    if "board" not in _PATHS:
        return
    prim = get_current_stage().GetPrimAtPath(f"{_PATHS['board']}/body/s{MARK_SHAPES[i]}")
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"cells": magnet_cells(_seed()), "feet": [0.0, 0.0], "rub": [0.0] * len(MARKS), "gone": [False] * len(MARKS),
            "wiped": False,
            "prev_board": None, "prev_felt": None, "tcps": None, "s": {}, "hidden_any": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("WB_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    hid = _EP.get("hidden_any")
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if hid:
        for i in range(len(MARKS)):
            _show_mark(i, True)
    if _DEBUG["on"]:
        print(f"[wb] reset seed={_seed()} cells={_EP['cells']}", file=sys.stderr, flush=True)


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


def _in_tray(q) -> bool:
    """A magnet (its origin in the board's frame) lying in the pen tray."""
    return (TRAY_X0 - BC_X + 0.004 < q[0] < TRAY_X1 - BC_X - 0.004 and abs(q[1]) < TRAY_HALF_Y - 0.008
            and q[2] < TRAY_FLOOR + 0.006)


def _square_of(q) -> int | None:
    """The square a magnet (its origin in the board's frame) stands in, flat on the surface, or None."""
    if abs(q[2] - PANEL_H) > 0.005:
        return None
    x, y = q[0] + BC_X, q[1] + BC_Y
    for c in range(9):
        cx, cy = cell_xy(c)
        if abs(x - cx) < SQ_X / 2 - 0.004 and abs(y - cy) < SQ_Y / 2 - 0.004:
            return c
    return None


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "board")
    pe, re_ = _pose(base, origin, "eraser")
    for k, ax in ((0, "feet_x"), (1, "feet_y")):  # the feet: stick, or slip once pushed past FEET_F
        d = float(pb[k] - (BC_X, BC_Y)[k])
        t = _EP["feet"][k]
        if abs(d - t) > FEET_SLIP:
            t = d - math.copysign(FEET_SLIP, d - t)
            _EP["feet"][k] = t
            set_slider_drive(ax, target=t)
    mags = {}
    for m in MAGNETS:
        pm, rm = _pose(base, origin, f"magnet_{m}")
        q = rb.T @ (pm - pb)
        upright = float((rb.T @ rm)[2, 2]) > math.cos(math.radians(20.0))
        mags[m] = {"q": q, "tray": _in_tray(q), "square": _square_of(q) if upright else None}
    all_in_tray = all(v["tray"] for v in mags.values())
    # rubbing: the felt (its underside's middle, in the board's frame) on the surface, moving over the board, while the
    # board stays still and all three magnets lie in the pen tray
    felt = rb.T @ (pe - pb)
    ex = rb.T @ re_[:, 1]  # the eraser's long axis, in the board's frame
    prev_b, prev_f = _EP["prev_board"], _EP["prev_felt"]
    still = False
    if prev_b is not None:
        dp = float(np.linalg.norm(pb - prev_b[0]))
        dr = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(prev_b[1].T @ rb) - 1) / 2))))
        still = dp < STILL_MM and dr < STILL_DEG
    touching = abs(felt[2] - PANEL_H) < FELT_TOUCH and float(re_[2, 2]) > math.cos(math.radians(12.0))
    if still and touching and all_in_tray and prev_f is not None:
        step = float(np.linalg.norm((felt - prev_f)[:2]))
        if step > 0.0005:
            ey = np.array([-ex[1], ex[0]])
            exy = np.array([ex[0], ex[1]]) / max(1e-9, math.hypot(ex[0], ex[1]))
            for i, (mx, my, _ln, _a, _ink) in enumerate(MARKS):
                if _EP["gone"][i]:
                    continue
                d = np.array([mx - BC_X, my - BC_Y]) - felt[:2]
                if abs(float(d @ exy)) < ER_L / 2 - 0.004 and abs(float(d @ ey)) < ER_W / 2 - 0.004:
                    _EP["rub"][i] += step
                    if _EP["rub"][i] >= RUB_TO_ERASE:
                        _EP["gone"][i] = True
                        _EP["hidden_any"] = True
                        _show_mark(i, False)
    if all(_EP["gone"]):
        _EP["wiped"] = True
    _EP["prev_board"] = (pb.copy(), rb.copy())
    _EP["prev_felt"] = felt.copy()
    _EP["tcps"] = _tcps(base, origin)
    eraser_off = (TRAY_X0 - BC_X < felt[0] < TRAY_X1 - BC_X and abs(felt[1]) < TRAY_HALF_Y - ER_L / 2 + 0.010
                  and felt[2] < TRAY_FLOOR + 0.004)  # back in the pen tray
    right = [m for m in MAGNETS if mags[m]["square"] == _EP["cells"][m]]
    _EP["s"] = {"right": right, "eraser_off": eraser_off, "home": _arms_home(), "tray": all_in_tray}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round(np.asarray(v) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[x].data.joint_pos)[0][base.scene[x].joint_names.index("joint7")]) * 1000
                  for x in ("left_robot", "right_robot")]
            yaw = math.degrees(math.atan2(rb[1, 0], rb[0, 0]))
            ms = " ".join(f"{m[0]}={mm(v['q'] + [BC_X, BC_Y, 0])}{'T' if v['tray'] else ''}{v['square']}"
                          for m, v in mags.items())
            print(f"[wb] t={_DEBUG['steps'] / 20:5.1f}s board={mm(pb - [BC_X, BC_Y, DT])} yaw={yaw:.2f} still={still}"
                  f" felt={mm(felt + [BC_X, BC_Y, 0])} touch={touching} gone={sum(_EP['gone'])}/{len(MARKS)}"
                  f" rub={[round(r * 100) for r in _EP['rub']]} wiped={_EP['wiped']} {ms} right={right}"
                  f" eraser_off={eraser_off} home={_EP['s']['home']} fingers={[round(v, 1) for v in fj]}"
                  f" L={mm(tl)} R={mm(tr)} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the board wiped clean (all the old marker rubbed off while all three magnets lay in the pen tray and the
    board stayed still), and at the end every magnet back in the square it stood in at the start, the eraser back in the
    pen tray and both arms back where they started; 0.75: wiped, and at the end two or three magnets in their own squares;
    0.5: wiped, and at the end one magnet in its own square; 0.25: wiped; else 0.0."""
    s = _EP["s"]
    if not s or not _EP["wiped"]:
        return 0.0
    n = len(s["right"])
    if n == 3 and s["eraser_off"] and s["home"]:
        return 1.0
    return (0.25, 0.5, 0.75, 0.75)[n]


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
    """Full score with the board, the magnets and the eraser at rest: the trial ends there."""
    names = ["board", "eraser"] + [f"magnet_{m}" for m in MAGNETS]
    return score(objects) >= 1.0 and all(objects[n].resting() for n in names)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# For the demo's seed (red in the far left square, blue in the near middle one, yellow in the near right one). The
# left takes the red magnet to the pen tray, the right the blue and the yellow. The left presses the board's left frame
# down while the right picks up the eraser and rubs two bands, forth and back, then lays it back on the desk. Then each
# magnet goes back to its square, and both arms go home.

KNOB_FINGER, ERASER_FINGER = -0.016, -0.018
SETTLE = 0.4  # a pause over a knob before going down round it
KNOB_GRIP = 0.003  # the TCP this far below the knob's top
CARRY = 0.07  # carry height above the writing surface
RIDGE_GRIP = 0.008  # the TCP this far below the eraser ridge's top
PRESS = 0.004  # the eraser pressed this far into the board (the arm gives)
HOLD_PRESS = 0.014  # the holding hand's closed fingertips aim this far below the frame's top
LEAN = 0.010  # the holding hand leans this far against each rub (the arm gives under the load)
TIP = 0.013  # closed fingertips reach this far below the TCP
STROKE_Y = (-0.012, -0.100)  # the felt's middle, at the two ends of each rub
LEAD_S, LEAD_STEPS = 3.0, 15
OFF = {"left": np.zeros(3), "right": np.zeros(3)}  # measured grip offsets (TCP minus part), aimed against


def _rot(arm: str = "", along: str = "y") -> np.ndarray:
    return grip_rotation(0.0)  # straight down, the fingers closing along y


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (1.0, 1.8), (0.8, 1.6), (-1.2, -0.5), (0.0,), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if room > math.radians(25.0):
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


def demo():
    return _demo_plan()


def _demo_plan():
    from yam_arena.constants import READY

    from yam_arena import ik, station

    plan = {"left": [], "right": []}
    cur = {}
    ik_home = {}
    for a in ("left", "right"):
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES[a])
        ik_home[a] = np.asarray(p, float)
        cur[a] = (np.asarray(p, float), FINGER_OPEN, np.asarray(r, float))

    def go(arm, t, pos, finger, label, n=1, rot=None):
        rot = _rot() if rot is None else rot
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        p, f, r = cur[arm]
        plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def from_home(arm, pos, finger, label, rot=None):
        rot = _rot() if rot is None else rot
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def to_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)
        last = plan[arm][-1]
        cur[arm] = (np.asarray(last.pos, float), f, last.yaw)

    cells = magnet_cells(DEMO_SEED)
    on_board = {m: np.array([*cell_xy(cells[m]), SURF + MAG_H - KNOB_GRIP]) for m in MAGNETS}
    in_tray = {m: np.array([TRAY_X, TRAY_SPOT[m], DT + TRAY_FLOOR + MAG_H - KNOB_GRIP]) for m in MAGNETS}
    up = lambda p: np.array([p[0], p[1], SURF + CARRY])  # noqa: E731
    arm_of = {"red": "left", "blue": "right", "yellow": "right"}

    def move(m, a, b, label):
        arm = arm_of[m]
        a = a + OFF[arm]
        b = b + OFF[arm]
        if np.linalg.norm(np.asarray(cur[arm][0]) - ik_home[arm]) < 1e-4:
            from_home(arm, up(a), KNOB_FINGER, f"over the {m} magnet")
        else:
            go(arm, 1.4, up(a), KNOB_FINGER, f"over the {m} magnet", n=2)
        wait(arm, SETTLE, "settle over it")
        go(arm, 0.6, a + [0.0, 0.0, 0.015], KNOB_FINGER, f"down over the {m} magnet's knob")
        go(arm, 0.6, a, KNOB_FINGER, "down round it")
        go(arm, 0.6, a, FINGER_CLOSED, f"grip the {m} magnet")
        go(arm, 0.8, up(a), FINGER_CLOSED, "lift it")
        go(arm, 1.8, up(b), FINGER_CLOSED, label, n=3)
        go(arm, 1.0, b + [0.0, 0.0, 0.002], FINGER_CLOSED, "lower it")
        go(arm, 0.5, b + [0.0, 0.0, 0.002], KNOB_FINGER, "let go")
        go(arm, 0.6, up(b), KNOB_FINGER, "rise")

    # 1. the magnets into the pen tray, one hand at a time
    move("red", on_board["red"], in_tray["red"], "carry it to the pen tray")
    to_home("left")
    sync()
    move("blue", on_board["blue"], in_tray["blue"], "carry it to the pen tray")
    move("yellow", on_board["yellow"], in_tray["yellow"], "carry it to the pen tray")
    sync()
    # 2. the left presses the board's left frame down; the right wipes two bands with the eraser
    frame = np.array([0.205, BC_Y + BY / 2 - FW / 2, SURF + FRAME_UP])  # near the arm: it holds stiffer there
    from_home("left", frame + [0.0, 0.0, 0.05], FINGER_CLOSED, "over the board's left frame")
    nohold = bool(os.environ.get("WB_NOHOLD"))  # (a check: the left hovers over the frame instead)
    hold = frame + [0.0, 0.0, 0.03 if nohold else TIP - HOLD_PRESS]
    lean = 0.0 if nohold else LEAN
    go("left", 1.0, hold, FINGER_CLOSED, "press the frame down: hold the board still")
    rer = grip_rotation(math.pi / 2)  # the right's fingers close along x, across the eraser's ridge
    er = np.array([ER_START[0], ER_START[1], DT + TRAY_FLOOR + ER_H - RIDGE_GRIP]) + OFF["right"]
    to_home("right")
    from_home("right", up(er), ERASER_FINGER, "over the eraser in the pen tray", rot=rer)
    wait("right", SETTLE, "settle over it")
    go("right", 1.0, er, ERASER_FINGER, "down round its handle", rot=rer)
    go("right", 0.6, er, FINGER_CLOSED, "grip the eraser", rot=rer)
    go("right", 0.8, up(er), FINGER_CLOSED, "lift it", rot=rer)
    sync("wait: the board held")
    rub_z = SURF + ER_H - RIDGE_GRIP + 0.001 - PRESS  # (the eraser hangs about 1 mm low in the grip)
    for b, xb in enumerate(BAND_X):
        a0 = np.array([xb, STROKE_Y[0], rub_z]) + OFF["right"]
        a1 = np.array([xb, STROKE_Y[1] - 0.004, rub_z]) + OFF["right"]
        go("right", 1.4, up(a0), FINGER_CLOSED, f"over the old marker, line {b + 1}", n=2, rot=rer)
        go("right", 0.8, a0, FINGER_CLOSED, "eraser down on the board", rot=rer)
        sync("hold the board")
        for k in range(2):  # the left leans against each rub as it goes
            go("right", 2.0, a1, FINGER_CLOSED, f"rub across the marker ({k + 1})", n=4, rot=rer)
            go("left", 2.0, hold + [0.0, lean, 0.0], FINGER_CLOSED, "lean against the rub", n=4)
            go("right", 2.0, a0 + [0.0, 0.006, 0.0], FINGER_CLOSED, f"and back again ({k + 1})", n=4, rot=rer)
            go("left", 2.0, hold - [0.0, lean, 0.0], FINGER_CLOSED, "lean against the rub", n=4)
        go("right", 0.6, up(a0), FINGER_CLOSED, "lift the eraser", rot=rer)
        go("left", 0.6, hold, FINGER_CLOSED, "hold the board")
    go("right", 1.6, up(er), FINGER_CLOSED, "carry the eraser back to the pen tray", n=2, rot=rer)
    go("right", 1.0, er + [0.0, 0.0, 0.003], FINGER_CLOSED, "lay it in the tray", rot=rer)
    go("right", 0.5, er + [0.0, 0.0, 0.003], ERASER_FINGER, "let go", rot=rer)
    go("right", 0.6, up(er), ERASER_FINGER, "rise", rot=rer)
    wait("left", dur("right") - dur("left"), "hold the board")
    go("left", 0.8, frame + [0.0, 0.0, 0.05], FINGER_CLOSED, "let go of the board")
    to_home("left")
    to_home("right")
    sync("home")
    wait("left", 1.5, "both arms home: the board is wiped, nothing on it shows where the magnets were")
    wait("right", 1.5, "both arms home: the board is wiped, nothing on it shows where the magnets were")
    # 3. each magnet back in its square
    move("yellow", in_tray["yellow"], on_board["yellow"], "carry it back to its square")
    move("blue", in_tray["blue"], on_board["blue"], "carry it back to its square")
    to_home("right")
    sync()
    move("red", in_tray["red"], on_board["red"], "carry it back to its square")
    to_home("left")
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
