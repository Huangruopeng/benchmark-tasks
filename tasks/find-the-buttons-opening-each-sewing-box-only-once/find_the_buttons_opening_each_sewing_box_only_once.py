"""On a polished sideboard stand four identical pine sewing boxes with sliding lids, two on each side of the middle, each
lid with a small wooden knob at its outer end. One box holds a little clear box of buttons; the others hold thread
spools, balls of wool and a tape measure with tailor's chalk (which box holds what depends on the trial's seed). In
front of the sideboard lies a sewing tray with a folded shirt that has lost a button.

Task: find the buttons and put their box on the sewing tray beside the shirt. Open the boxes one at a time, sliding
each lid shut again before you open the next, and never open the same box twice; leave the boxes where they stand.
Hold each box down by its closed end while you slide its lid. Leave every lid shut and take both arms back to where
they started.

What it tests: memory of one's own search. With the lids shut the four boxes look the same, so after the first box or
two nothing in view says which boxes have already been looked into; a policy that acts on the current frame alone
opens a box it has already checked, and opening a box a second time is latched as a failure. Which box holds the
buttons depends on the seed, so a fixed habit (always the front right box, say) does not pass either.

Why two hands: a lid runs stiffly in its grooves (about 1.2 N to slide it), while the boxes stand on the polished
sideboard and slide at about 0.25 N. Pulled with nothing holding the box, the whole box comes along and the lid does not
open; one hand presses the box's closed end down while the other slides the lid by its knob.

Run it:  inspect-robots run --embodiment yam_arena -E task=find_the_buttons_opening_each_sewing_box_only_once.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the sideboard's polished top (the boxes stand on it; the sewing tray stands on the table in front of it) ----------
SX0, SX1, SY0, SY1, SB_H = 0.165, 0.395, -0.270, 0.270, 0.006
ST = TOP + SB_H
# -- the boxes: numbered 0 back left, 1 front left, 2 front right, 3 back right (left is +y, toward the left arm) ------
BL, BW, BH = 0.220, 0.080, 0.065  # long (y), wide (x), tall
WALL, FLOOR_T, END_T, OUT_T, LID_T = 0.006, 0.006, 0.024, 0.008, 0.008
INNER = 0.022  # each box's closed (inner) end is this far from the middle line
ROW_X = (0.335, 0.225, 0.225, 0.335)
SIGN = (1.0, 1.0, -1.0, -1.0)  # +1: the box on the left, its lid sliding out to the left
BOX_Y = tuple(s * (INNER + BL / 2) for s in SIGN)
BOX_MASS, LID_MASS = 0.45, 0.05
# the lid, along the box's outward axis u (u = 0 at the box's middle): from the closed end's block to the open end
LID_L = BL - END_T
LID_U = -BL / 2 + END_T + LID_L / 2
LID_W = BW - 2 * WALL - 0.001
KNOB_U = LID_U + LID_L / 2 - 0.018
KNOB_STEM, KNOB_HEAD, KNOB_R = 0.010, 0.012, 0.011
LID_OUT = 0.125  # the lid's stop (a peg in the groove)
LID_K, LID_F, LID_DAMP = 2000.0, 1.2, 2.0  # the lid's grooves: N/m, at most N (wood on wood), N s/m
SEAT = 0.003  # within this of shut, a lid is pulled shut (it drops into its seat)
INTERIOR_U0 = -BL / 2 + END_T  # the inside starts at the closed end's block
# -- the button box (clear plastic) and what the other boxes hold ------------------------------------------------------
BB_X, BB_U, BB_Z = 0.030, 0.050, 0.024  # across (x), along the box (u), tall
BB_AT_U = INTERIOR_U0 + 0.066  # its middle, along the box
# -- the sewing tray --------------------------------------------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_LX, TRAY_LY, TRAY_WALL, TRAY_FLOOR = 0.103, 0.0, 0.090, 0.220, 0.014, 0.004
SHIRT_Y = 0.048  # the folded shirt lies on the tray's left half; the button box goes on the right half
# -- the rules ---------------------------------------------------------------------------------------------------------
OPENED = 0.010  # a lid slid out more than this has opened its box
SHUT_AGAIN = 0.006  # an opened lid back within this of shut has shut its box again (opening it once more is a failure)
SHUT = 0.004  # at the end, every lid within this of shut
MOVE_MAX, TILT_MAX = 0.040, 15.0  # a box moved more than this (m) from its spot, or tipped more than this (deg)
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
DEMO_SEED = 1415158438  # the reset seed inspect-robots uses for --seed 0 (its default): the demo's trial
CONTENTS = ("spools", "wool", "tape")

PINE, PINE_D, PINE_L = (0.86, 0.70, 0.47), (0.66, 0.50, 0.30), (0.93, 0.80, 0.58)
WALNUT, WALNUT_L, FELT = (0.33, 0.20, 0.12), (0.42, 0.27, 0.16), (0.16, 0.30, 0.20)
KNOB_C = (0.42, 0.26, 0.14)
SHIRT, SHIRT_D, WHITE = (0.62, 0.74, 0.90), (0.48, 0.60, 0.78), (0.95, 0.95, 0.93)
CLEAR = (0.80, 0.86, 0.90)
BUTTON_COLS = ((0.92, 0.92, 0.88), (0.85, 0.20, 0.18), (0.20, 0.40, 0.80), (0.95, 0.75, 0.15), (0.15, 0.15, 0.15),
               (0.30, 0.65, 0.35))


def button_box_of(seed) -> int:
    """The box holding the buttons: seed mod 4 (box 2, front right, for the demo's seed)."""
    return int(DEMO_SEED if seed is None else seed) % 4


def contents_of(seed) -> list[str]:
    """What each box holds, 0-3."""
    s = int(DEMO_SEED if seed is None else seed)
    b, r = s % 4, (s // 4) % 3
    rest = [CONTENTS[(r + j) % 3] for j in range(3)]
    return [("buttons" if k == b else rest.pop(0)) for k in range(4)]


def _seed():
    return getattr(bodykit, "_SEWING_SEED", None)


def _catch_seed() -> None:
    """Wrap the embodiment's reset so the task file sees each trial's seed."""
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
    if cls is None or getattr(cls, "_sewing_seed", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        bodykit._SEWING_SEED = kwargs.get("seed", args[1] if len(args) > 1 else None)
        return plain(self, *args, **kwargs)

    cls.reset = reset
    cls._sewing_seed = True


_catch_seed()


# -- the bodies --------------------------------------------------------------------------------------------------------

def _sideboard() -> Body:
    shapes = [BoxShape((0.0, 0.0, SB_H / 2), (SX1 - SX0, SY1 - SY0, SB_H), WALNUT)]
    rng = np.random.default_rng(3)
    for _ in range(16):  # the walnut's figure, all of it on the top
        ln = rng.uniform(0.06, 0.20)
        shapes.append(BoxShape((rng.uniform(-0.10, 0.10), rng.uniform(-0.255 + ln / 2, 0.255 - ln / 2), SB_H + 0.0002),
                               (rng.uniform(0.002, 0.004), ln, 0.0004), WALNUT_L, collide=False,
                               yaw=rng.uniform(-0.03, 0.03)))
    shapes.append(BoxShape((-(SX1 - SX0) / 2 - 0.0006, 0.0, SB_H / 2), (0.001, SY1 - SY0, SB_H), WALNUT_L,
                           collide=False))  # the moulded front edge
    return Body.build("sideboard", pos=((SX0 + SX1) / 2, (SY0 + SY1) / 2, TOP), mass=1.0, kinematic=True,
                      color=WALNUT, shapes=shapes, friction=(0.05, 0.045), slick=True)


_DECO: dict[str, list[int]] = {c: [] for c in CONTENTS}  # each content's shapes in a box body (the same in all four)


def _box(k: int) -> Body:
    """The box: floor, two side walls, the closed end's block (full height; the lid's groove runs into it), the open
    end's low wall (the lid slides out over it), a felt lining and what it may hold (drawn only in the box that holds
    it). Origin: the middle of its footprint, on the sideboard."""
    s = SIGN[k]
    shapes = [BoxShape((0.0, 0.0, FLOOR_T / 2), (BW, BL, FLOOR_T), PINE)]
    for sx in (-1.0, 1.0):
        shapes.append(BoxShape((sx * (BW / 2 - WALL / 2), 0.0, BH / 2), (WALL, BL, BH), PINE))
        shapes.append(BoxShape((sx * (BW / 2 + 0.0004), 0.0, BH - 0.010), (0.0008, BL - 0.004, 0.0015), PINE_D,
                               collide=False))  # the groove line on the outside
    shapes.append(BoxShape((0.0, s * (-BL / 2 + END_T / 2), BH / 2), (BW - 2 * WALL, END_T, BH), PINE))
    shapes.append(BoxShape((0.0, s * (BL / 2 - OUT_T / 2), (BH - LID_T) / 2), (BW - 2 * WALL, OUT_T, BH - LID_T), PINE))
    shapes.append(BoxShape((0.0, s * (-BL / 2 - 0.0004), BH / 2), (BW - 0.012, 0.0008, BH - 0.012), PINE_L,
                           collide=False))  # a panel on the closed end
    shapes.append(BoxShape((0.0, s * (INTERIOR_U0 + (BL / 2 - OUT_T - INTERIOR_U0) / 2), FLOOR_T + 0.0003),
                           (BW - 2 * WALL - 0.002, BL / 2 - OUT_T - INTERIOR_U0 - 0.002, 0.0006), FELT,
                           collide=False))  # the felt lining
    z0 = FLOOR_T + 0.0006

    def deco(kind, shape):
        _DECO[kind].append(len(shapes)) if k == 0 else None
        shapes.append(shape)

    # thread spools: three lying across the box, red, navy, mustard
    for j, (u, col) in enumerate(((-0.036, (0.80, 0.16, 0.14)), (-0.010, (0.14, 0.24, 0.62)), (0.016, (0.92, 0.70, 0.12)))):
        uu = INTERIOR_U0 + 0.086 + u
        for xx in (-0.0165, 0.0165):
            deco("spools", CylShape((xx, s * uu, z0 + 0.0125), 0.0125, 0.003, (0.90, 0.84, 0.70), collide=False,
                                    axis="X"))
        deco("spools", CylShape((0.0, s * uu, z0 + 0.0125), 0.0105, 0.030, col, collide=False, axis="X"))
    # balls of wool: teal and mustard
    for u, x, col in ((-0.028, 0.008, (0.15, 0.55, 0.55)), (0.014, -0.008, (0.85, 0.62, 0.15))):
        uu = INTERIOR_U0 + 0.086 + u
        deco("wool", SphereShape((x, s * uu, z0 + 0.021), 0.021, col, collide=False))
        deco("wool", BoxShape((x, s * uu, z0 + 0.021), (0.043, 0.002, 0.030), tuple(c * 0.8 for c in col),
                              collide=False))  # a wound band
    # a yellow tape measure and a wedge of blue tailor's chalk
    uu = INTERIOR_U0 + 0.086 - 0.022
    deco("tape", CylShape((0.0, s * uu, z0 + 0.007), 0.024, 0.014, (0.95, 0.80, 0.10), collide=False))
    deco("tape", CylShape((0.0, s * uu, z0 + 0.0143), 0.010, 0.0006, (0.15, 0.15, 0.15), collide=False))
    deco("tape", BoxShape((0.0, s * (uu + 0.024), z0 + 0.004), (0.016, 0.006, 0.008), (0.85, 0.85, 0.80),
                          collide=False))  # the tape's end hook
    uu = INTERIOR_U0 + 0.086 + 0.022
    deco("tape", BoxShape((0.006, s * uu, z0 + 0.003), (0.034, 0.028, 0.006), (0.35, 0.50, 0.85), collide=False))
    return Body.build(f"box{k}", pos=(ROW_X[k], BOX_Y[k], ST), mass=BOX_MASS, color=PINE, shapes=shapes,
                      friction=(0.8, 0.7), com=(0.0, 0.0, 0.022))


def _lid(k: int) -> Body:
    s = SIGN[k]
    ku = KNOB_U - LID_U
    shapes = [BoxShape((0.0, 0.0, 0.0), (LID_W, LID_L, LID_T), PINE_L),
              BoxShape((0.0, s * (LID_L / 2 - 0.0004), 0.0), (LID_W - 0.004, 0.0008, LID_T - 0.002), PINE_D,
                       collide=False)]  # its end grain
    for j in range(5):  # the grain
        shapes.append(BoxShape((-0.026 + 0.013 * j + 0.003 * (j % 2), 0.0, LID_T / 2 + 0.0002),
                               (0.0012, LID_L - 0.02 - 0.01 * (j % 3), 0.0004), PINE, collide=False))
    shapes += [CylShape((0.0, s * ku, LID_T / 2 + KNOB_STEM / 2), 0.006, KNOB_STEM, KNOB_C),
               CylShape((0.0, s * ku, LID_T / 2 + KNOB_STEM + KNOB_HEAD / 2), KNOB_R, KNOB_HEAD, KNOB_C),
               CylShape((0.0, s * ku, LID_T / 2 + 0.0004), 0.010, 0.0008, PINE_D, collide=False)]
    return Body.build(f"lid{k}", pos=(ROW_X[k], BOX_Y[k] + s * LID_U, ST + BH - LID_T / 2), mass=LID_MASS,
                      color=PINE_L, shapes=shapes, friction=(0.9, 0.8))


def _bb_pos(k: int) -> tuple[float, float, float]:
    return (ROW_X[k], BOX_Y[k] + SIGN[k] * BB_AT_U, ST + FLOOR_T + 0.0006 + BB_Z / 2)


def _buttons() -> Body:
    """A small clear plastic box of buttons (its long side along y), drawn by its edges and the buttons inside."""
    hx, hy, hz = BB_X / 2, BB_U / 2, BB_Z / 2
    shapes = [BoxShape((0.0, 0.0, 0.0), (BB_X, BB_U, BB_Z), None),  # collision only: the clear box
              BoxShape((0.0, 0.0, -hz + 0.0008), (BB_X - 0.002, BB_U - 0.002, 0.0016), CLEAR, collide=False)]
    for sx in (-1.0, 1.0):  # the clear box's edges
        for sz in (-1.0, 1.0):
            shapes.append(BoxShape((sx * (hx - 0.0008), 0.0, sz * (hz - 0.0008)), (0.0016, BB_U, 0.0016), CLEAR,
                                   collide=False))
        for sy in (-1.0, 1.0):
            shapes.append(BoxShape((sx * (hx - 0.0008), sy * (hy - 0.0008), 0.0), (0.0016, 0.0016, BB_Z), CLEAR,
                                   collide=False))
    for sy in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            shapes.append(BoxShape((0.0, sy * (hy - 0.0008), sz * (hz - 0.0008)), (BB_X, 0.0016, 0.0016), CLEAR,
                                   collide=False))
    shapes.append(BoxShape((0.0, 0.0, 0.0), (0.0016, BB_U - 0.003, BB_Z - 0.003), CLEAR, collide=False))  # divider
    rng = np.random.default_rng(11)
    for j in range(14):  # the buttons, two layers
        x = (-1 if j % 2 else 1) * rng.uniform(0.004, 0.009)
        y = rng.uniform(-hy + 0.007, hy - 0.007)
        z = -hz + 0.004 + (0.004 if j > 7 else 0.0) + rng.uniform(0.0, 0.002)
        r = rng.uniform(0.0055, 0.0072)
        shapes.append(CylShape((x, y, z), r, 0.0025, BUTTON_COLS[j % len(BUTTON_COLS)], collide=False))
    return Body.build("buttons", pos=_bb_pos(button_box_of(None)), mass=0.06, color=CLEAR, shapes=shapes,
                      friction=(1.1, 1.0))


def _tray() -> Body:
    """A shallow beech sewing tray with a folded light-blue shirt (a button missing from its placket) on its left
    half, and a needle threaded with white cotton."""
    shapes = [BoxShape((0.0, 0.0, TRAY_FLOOR / 2), (TRAY_LX, TRAY_LY, TRAY_FLOOR), PINE_L)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (TRAY_LX / 2 - 0.003), 0.0, TRAY_WALL / 2), (0.006, TRAY_LY, TRAY_WALL), PINE_D))
        shapes.append(BoxShape((0.0, s * (TRAY_LY / 2 - 0.003), TRAY_WALL / 2), (TRAY_LX, 0.006, TRAY_WALL), PINE_D))
    zf = TRAY_FLOOR
    shapes += [BoxShape((0.0, SHIRT_Y, zf + 0.008), (0.072, 0.086, 0.016), SHIRT, collide=False),
               BoxShape((0.030, SHIRT_Y, zf + 0.0165), (0.012, 0.050, 0.002), SHIRT_D, collide=False),  # collar
               BoxShape((0.0, SHIRT_Y, zf + 0.0162), (0.064, 0.012, 0.0006), SHIRT_D, collide=False)]  # placket
    for j, x in enumerate((0.020, 0.004, -0.012, -0.028)):
        if j == 2:  # the missing button: a loose thread
            shapes.append(BoxShape((x, SHIRT_Y + 0.003, zf + 0.0167), (0.001, 0.010, 0.0005), WHITE, collide=False))
            continue
        shapes.append(CylShape((x, SHIRT_Y, zf + 0.0168), 0.0035, 0.0008, WHITE, collide=False))
    shapes.append(plate(np.array([-0.032, SHIRT_Y - 0.052, zf + 0.0012]), np.array([0.010, SHIRT_Y - 0.050, zf + 0.0012]),
                        0.0012, 0.0012, [0.0, 0.0, 1.0], (0.75, 0.76, 0.78), collide=False))  # the needle
    shapes.append(plate(np.array([0.010, SHIRT_Y - 0.050, zf + 0.0012]), np.array([0.032, SHIRT_Y - 0.030, zf + 0.0012]),
                        0.0008, 0.0008, [0.0, 0.0, 1.0], WHITE, collide=False))  # its thread
    return Body.build("sewing_tray", pos=(TRAY_X, TRAY_Y, TOP), mass=0.3, kinematic=True, color=PINE_L, shapes=shapes,
                      friction=(0.9, 0.8))


OBJECTS = [_sideboard(), *[_box(k) for k in range(4)], *[_lid(k) for k in range(4)], _buttons(), _tray()]
for _k in range(4):
    _lim = (0.0, LID_OUT) if SIGN[_k] > 0 else (-LID_OUT, 0.0)
    slider(f"groove{_k}", f"box{_k}", f"lid{_k}", anchor=(ROW_X[_k], BOX_Y[_k] + SIGN[_k] * LID_U, ST + BH - LID_T / 2),
           axis="Y", limits=_lim, stiffness=LID_K, damping=LID_DAMP, target=0.0, max_force=LID_F)

_PATHS: dict[str, str] = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: note where each box is, and draw only what the box holds."""
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name.startswith("box"):
        _PATHS[spec.name] = prim_path
        _show_contents(stage)


bodykit._author_sliders = _author_sliders


def _show_contents(stage=None) -> None:
    from pxr import UsdGeom

    if stage is None:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
    holds = contents_of(_seed())
    for k in range(4):
        path = _PATHS.get(f"box{k}")
        if path is None:
            continue
        for kind, idx in _DECO.items():
            for i in idx:
                prim = stage.GetPrimAtPath(f"{path}/body/s{i}")
                if prim:
                    img = UsdGeom.Imageable(prim)
                    img.MakeVisible() if holds[k] == kind else img.MakeInvisible()


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"b": button_box_of(_seed()), "placed": False, "state": [0, 0, 0, 0], "wrong": "", "found": False,
            "tcps": None, "s": {}, "start": None, "steps": 0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SEWING_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    if _PATHS:
        _show_contents()
    if _DEBUG["on"]:
        print(f"[sewing] reset seed={_seed()} buttons in box {_EP['b']} holds={contents_of(_seed())}",
              file=sys.stderr, flush=True)


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


def _put(base, origin, name, pos) -> None:
    """Move a body to ``pos``, unrotated and at rest."""
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)
    asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=asset.device))


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    if _EP["steps"] == 0 and not _EP["placed"]:  # the buttons go in the trial's box
        if _EP["b"] != button_box_of(None):
            _put(base, origin, "buttons", _bb_pos(_EP["b"]))
        _EP["placed"] = True
    _EP["steps"] += 1
    outs, moved = [], []
    for k in range(4):
        pb, rb = _pose(base, origin, f"box{k}")
        pl, _rl = _pose(base, origin, f"lid{k}")
        d = float((rb.T @ (pl - pb))[1]) - SIGN[k] * LID_U  # the lid's slide along y, from shut
        o = SIGN[k] * d
        outs.append(o)
        set_slider_drive(f"groove{k}", target=0.0 if o < SEAT else d)  # the grooves hold the lid where it is
        moved.append((math.hypot(pb[0] - ROW_X[k], pb[1] - BOX_Y[k]), _tilt(rb)))
    st = _EP["state"]
    for k, o in enumerate(outs):
        if o > OPENED:
            if st[k] == 2 and not _EP["wrong"]:
                _EP["wrong"] = f"box {k} opened twice"
            st[k] = 1
        elif st[k] == 1 and o < SHUT_AGAIN:
            st[k] = 2
    if sum(o > OPENED for o in outs) >= 2 and not _EP["wrong"]:
        _EP["wrong"] = "two boxes open at once"
    for k, (dm, tl) in enumerate(moved):
        if (dm > MOVE_MAX or tl > TILT_MAX) and not _EP["wrong"]:
            _EP["wrong"] = f"box {k} moved {dm * 100:.1f} cm / tipped {tl:.0f} deg"
    if st[_EP["b"]] >= 1:
        _EP["found"] = True
    pbb, rbb = _pose(base, origin, "buttons")
    q = pbb - np.array([TRAY_X, TRAY_Y, TOP])
    on_tray = (abs(q[0]) < TRAY_LX / 2 - 0.006 and abs(q[1]) < TRAY_LY / 2 - 0.006 and q[2] < TRAY_FLOOR + BB_Z / 2 + 0.02
               and _tilt(rbb) < 35.0)
    _EP["tcps"] = _tcps(base, origin)
    home = _arms_home()
    all_shut = all(o < SHUT for o in outs)
    _EP["s"] = {"outs": outs, "all_shut": all_shut, "on_tray": on_tray, "home": home}
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        mm = lambda v: np.round((np.asarray(v) - [0, 0, ST]) * 1000).astype(int).tolist()  # noqa: E731
        tl, tr = _EP["tcps"]
        fj = [float(as_numpy(base.scene[x].data.joint_pos)[0][base.scene[x].joint_names.index("joint7")]) * 1000
              for x in ("left_robot", "right_robot")]
        print(f"[sewing] t={_EP['steps'] / 20:5.1f}s outs={[round(o * 1000) for o in outs]} state={st}"
              f" moved={[round(m * 1000) for m, _ in moved]} tilt={[round(t) for _, t in moved]} bb={mm(pbb)}"
              f" tray={on_tray} found={_EP['found']} wrong='{_EP['wrong']}' home={home}"
              f" fingers={[round(v, 1) for v in fj]} L={mm(tl)} R={mm(tr)} score={score(None)}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if a box was opened a second time, two boxes stood open at once, or a box was moved more than 4 cm or
    tipped more than 15 degrees (latched). Otherwise 1.0: at the end the button box on the sewing tray, every lid
    shut and both arms back where they started; 0.75: the button box on the tray and every lid shut at the end; 0.5:
    the button box on the tray at the end; 0.25: the box holding the buttons opened at some point; else 0.0."""
    s = _EP["s"]
    if not s or _EP["wrong"]:
        return 0.0
    if s["on_tray"]:
        if s["all_shut"]:
            return 1.0 if s["home"] else 0.75
        return 0.5
    return 0.25 if _EP["found"] else 0.0


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
    """Full score with the boxes, their lids and the button box at rest: the trial ends there."""
    names = ["buttons"] + [f"box{k}" for k in range(4)] + [f"lid{k}" for k in range(4)]
    return score(objects) >= 1.0 and all(objects[n].resting() for n in names)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Written for the demo's trial (DEMO_SEED: the buttons in box 2, front right). It looks in box 0 (back left), then
# box 1 (front left), then box 2: for a box on the left the right hand presses the box's closed end down while the
# left slides the lid out by its knob, pauses, and slides it shut again; for a box on the right, the other way round.
# In box 2 it finds the buttons: the left lets go and rises, the right lets go of the knob, lifts the button box out
# and sets it on the sewing tray, then takes the knob again; the left presses again and the right slides the lid
# shut. Both arms go home.

DEMO_ORDER = (0, 1, 2)
KNOB_FINGER, BB_FINGER = -0.018, -0.021
PRESS_U = INTERIOR_U0 - END_T / 2 - 0.0035  # the pressing fingers' middle, along the box (clear of the lid's end)
PRESS_DEPTH = 0.010  # aim the closed fingertips this far into the box's top
TIP = 0.013  # the closed fingertips this far below the TCP
PULL_OVER, PUSH_PAST = 0.015, 0.012
LOOK_S = 1.2
BRACE = 0.008
LEAD_S, LEAD_STEPS = 3.0, 15


def _rot(arm: str) -> np.ndarray:
    """Straight down, the fingers closing along x (across the boxes)."""
    return grip_rotation(-math.pi / 2 if arm == "left" else math.pi / 2)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.0, 1.8), (0.8, 1.6), (-1.2, -0.5), (-0.5, 0.0, 0.5),
                                  (-0.4, 0.4)):
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
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    home = {}

    def go(arm, t, pos, finger, label, n=1):
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label,
                                      _rot(arm)))
        cur[arm] = (p1, finger, _rot(arm))

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

    def from_home(arm, pos, finger, label):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, _rot(arm)), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, _rot(arm))
        home[arm] = False

    def to_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)
        last = plan[arm][-1]
        cur[arm] = (np.asarray(last.pos, float), f, last.yaw)
        home[arm] = True

    def knob(k, out):
        return np.array([ROW_X[k], BOX_Y[k] + SIGN[k] * (KNOB_U + out), ST + BH + KNOB_STEM + KNOB_HEAD / 2])

    def press_at(k):
        return np.array([ROW_X[k], BOX_Y[k] + SIGN[k] * PRESS_U, ST + BH + TIP - PRESS_DEPTH])

    up = np.array([0.0, 0.0, 0.06])
    pup = np.array([0.0, 0.0, 0.03])  # the pressing hand stays low (the far arm reaches no higher over the boxes)

    def reach(arm, pos, finger, label, t=1.8):
        if home.get(arm, True):
            from_home(arm, pos, finger, label)
        else:
            p = cur[arm][0]
            hi = max(p[2], pos[2]) + 0.005
            go(arm, t * 0.4, [p[0], p[1], hi], cur[arm][1], "rise", n=1)
            go(arm, t * 0.6, [pos[0], pos[1], hi], finger, label, n=2)
            go(arm, 0.6, pos, finger, label)

    def press(k, holder, label):
        reach(holder, press_at(k) + pup, FINGER_CLOSED, f"over the closed end of box {k}")
        go(holder, 1.0, press_at(k), FINGER_CLOSED, f"press box {k} down by its closed end ({label})")

    def slide(k, puller, holder, a, b, label):
        """The puller, holding the knob, slides the lid from ``a`` to ``b`` out; the holder leans against it."""
        s = SIGN[k]
        lean = -s if b > a else s
        go(puller, 2.6, knob(k, b), FINGER_CLOSED, label, n=5)
        go(holder, 2.6, press_at(k) + [0.0, lean * BRACE, 0.0], FINGER_CLOSED, "hold the box down, leaning", n=5)
        wait(puller, 0.3, "hold")
        go(holder, 0.3, press_at(k), FINGER_CLOSED, "ease off")

    def look(k, puller, holder, take_out=False):
        reach(puller, knob(k, 0.0) + up, KNOB_FINGER, f"over the knob of box {k}")
        press(k, holder, "1")
        go(puller, 1.0, knob(k, 0.0), KNOB_FINGER, "down round the knob")
        go(puller, 0.6, knob(k, 0.0), FINGER_CLOSED, "grip the knob")
        sync(f"wait: box {k} held down, its knob gripped")
        slide(k, puller, holder, 0.0, LID_OUT + PULL_OVER, f"slide box {k}'s lid open")
        sync()
        if not take_out:
            wait(puller, LOOK_S, f"look into box {k}")
            wait(holder, LOOK_S, f"look into box {k}")
            slide(k, puller, holder, LID_OUT, -PUSH_PAST, f"no buttons: slide box {k}'s lid shut")
            sync()
            go(puller, 0.5, knob(k, -PUSH_PAST), KNOB_FINGER, "let go of the knob")
            go(holder, 0.5, press_at(k) + [0.0, 0.0, 0.01], FINGER_CLOSED, "lift off")
            go(puller, 0.6, knob(k, -PUSH_PAST) + up, KNOB_FINGER, "rise")
            go(holder, 0.6, press_at(k) + pup, FINGER_CLOSED, "rise")
            sync()
            return
        # the buttons: the holder lets go and rises; the puller lifts the button box out onto the sewing tray
        bb = np.array(_bb_pos(k))
        bb_tcp = np.array([bb[0], bb[1], bb[2] + BB_Z / 2 - 0.007])
        carry_z = ST + BH + 0.05
        tray_at = np.array([TRAY_X, TRAY_Y - 0.050, TOP + TRAY_FLOOR + BB_Z - 0.007 + 0.008])
        go(holder, 0.6, press_at(k) + pup, FINGER_CLOSED, "let go of the box and rise")
        to_home(holder, "out of the way while the other hand takes the buttons")
        go(puller, 0.5, knob(k, LID_OUT), KNOB_FINGER, "let go of the knob")
        go(puller, 0.6, knob(k, LID_OUT) + up, KNOB_FINGER, "rise")
        go(puller, 1.6, [bb_tcp[0], bb_tcp[1], carry_z], BB_FINGER, "over the button box", n=2)
        go(puller, 1.2, bb_tcp, BB_FINGER, "down round it")
        go(puller, 0.6, bb_tcp, FINGER_CLOSED, "take the button box")
        go(puller, 1.0, [bb_tcp[0], bb_tcp[1], carry_z], FINGER_CLOSED, "lift it out")
        go(puller, 2.0, [tray_at[0], tray_at[1], carry_z], FINGER_CLOSED, "carry it over the sewing tray", n=3)
        go(puller, 1.0, tray_at, FINGER_CLOSED, "lower it onto the tray, beside the shirt")
        go(puller, 0.5, tray_at, BB_FINGER, "let go")
        go(puller, 0.6, tray_at + [0.0, 0.0, 0.06], BB_FINGER, "rise")
        reach(puller, knob(k, LID_OUT) + up, KNOB_FINGER, f"back over the knob of box {k}", t=2.2)
        sync("the other hand comes back once the button box is on the tray")
        reach(holder, press_at(k) + pup, FINGER_CLOSED, f"back over the closed end of box {k}")
        go(holder, 1.0, press_at(k), FINGER_CLOSED, f"press box {k} down again")
        sync("wait: box held down")
        go(puller, 1.0, knob(k, LID_OUT), KNOB_FINGER, "down round the knob")
        go(puller, 0.6, knob(k, LID_OUT), FINGER_CLOSED, "grip the knob")
        sync("wait: knob gripped")
        slide(k, puller, holder, LID_OUT, -PUSH_PAST, f"slide box {k}'s lid shut")
        sync()
        go(puller, 0.5, knob(k, -PUSH_PAST), KNOB_FINGER, "let go of the knob")
        go(holder, 0.5, press_at(k) + [0.0, 0.0, 0.01], FINGER_CLOSED, "lift off")
        go(puller, 0.6, knob(k, -PUSH_PAST) + up, KNOB_FINGER, "rise")
        go(holder, 0.6, press_at(k) + pup, FINGER_CLOSED, "rise")
        sync()

    for k in DEMO_ORDER:
        puller, holder = ("left", "right") if SIGN[k] > 0 else ("right", "left")
        look(k, puller, holder, take_out=(k == button_box_of(None)))
    for arm in ("left", "right"):
        to_home(arm)
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
