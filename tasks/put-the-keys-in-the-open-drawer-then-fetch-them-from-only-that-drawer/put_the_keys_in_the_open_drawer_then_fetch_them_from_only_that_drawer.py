"""On a hall table stands a low wooden chest of four identical drawers side by side, on a polished marble top, with a
brass carry handle on top near each end. One of the drawers stands open a little (which one depends on the trial's seed). A bunch of keys (a car key fob, a
steel ring and a brass house key) lies at the front right; a navy glasses case lies at the front left, a leather valet
tray stands beside the chest on the left and a white key dish beside it on the right.

Task: put the keys away in the drawer that is standing open and push it shut. Then put the glasses case in the tray and
take both arms back to where they started. Then take the keys back out, opening only the drawer you put them in (once
the keys are put away, never open any other drawer), put them in the key dish, push the drawer shut and take both arms
back to where they started.

What it tests: memory. Once the drawer is shut the four drawers look the same, and the glasses case and the trip back
home come in between, so nothing in view says where the keys are; a policy that sees only the current frame can only
guess (one chance in four) when it comes to fetch them. The open drawer depends on the seed, so a fixed habit (always the
left drawer, say) does not pass either.

Why two hands: the drawers run stiffly (about 1.8 N, felt-lined runners with a magnetic catch), while the chest stands on
furniture glides on the polished table and slides at about 0.6 N. Pulled or pushed with nothing holding the chest, the
whole chest comes along and the drawer does not move in it; one hand holds the chest by a carry handle while the other
works the drawer.

Run it:  inspect-robots run --embodiment yam_arena -E task=put_the_keys_in_the_open_drawer_then_fetch_them_from_only_that_drawer.py \\
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

# -- the polished marble top the chest stands on (the keys, the case, the tray and the dish stand on the table) --------
WX0, WX1, WY0, WY1, WT_H = 0.130, 0.470, -0.250, 0.250, 0.006
CT = TOP + WT_H
# -- the chest: four drawers side by side, numbered 0-3 from the left (+y) ----------------------------------------------
FACE_X = 0.290  # the drawer fronts' outer faces (shut)
CH_D, CH_W = 0.142, 0.460  # deep (x), wide (y)
CH_X = FACE_X + CH_D / 2  # the chest's origin: the middle of its footprint, on the table
BOT_T, OPEN_H, TOPB_T = 0.008, 0.060, 0.010  # bottom board, the drawer openings' height, top board
SIDE_T, DIV_T = 0.010, 0.008
DW = (CH_W - 2 * SIDE_T - 3 * DIV_T) / 4  # an opening's width (0.104)
DRAWER_Y = [CH_W / 2 - SIDE_T - DW / 2 - i * (DW + DIV_T) for i in range(4)]
HANDLE_Z = BOT_T + OPEN_H + TOPB_T + 0.022  # the carry handles' bars, above the chest's bottom
HANDLE_Y, HANDLE_POST_X, HANDLE_R = 0.140, 0.035, 0.006  # a handle near each end, its bar running front to back
# -- a drawer: origin at the middle of its front's outer face -----------------------------------------------------------
DR_Z = CT + BOT_T + OPEN_H / 2  # the drawer fronts' middle height
FRONT_T, FRONT_H = 0.012, 0.058
BOX_D = 0.118  # the drawer box behind its front
BOX_W = DW - 0.008
WALL_T, BOX_H = 0.006, 0.042
FLOOR_Z = -OPEN_H / 2 + 0.008  # the top of the drawer's floor, from the drawer's origin
IN_X0, IN_X1 = FRONT_T, FRONT_T + BOX_D - WALL_T  # its inside, along x from the face
IN_Y = BOX_W / 2 - WALL_T
KNOB_STEM, KNOB_HEAD, KNOB_R = 0.022, 0.016, 0.0125
KNOB_X = -(KNOB_STEM + KNOB_HEAD / 2)  # the knob's head, from the face
OUT_MAX = 0.125  # the runners' stop
AJAR = 0.040  # the open drawer's runner target at the start (it settles about 2.8 cm out: its runner's force is capped)
AJAR_SEEN = 0.028
DRAWER_MASS, CHEST_MASS = 0.20, 0.60
RUN_K, RUN_F, RUN_DAMP = 2000.0, 1.8, 2.0  # the runners: N/m, at most N (felt runners and catch), N s/m
CATCH = 0.012  # within this of shut the magnetic catch pulls the drawer shut
# -- the keys: origin at the middle of the fob ---------------------------------------------------------------------------
FOB_L, FOB_W, FOB_H = 0.018, 0.022, 0.015  # the fob's straight part; its front end is rounded (radius FOB_W / 2)
RING_X, RING_R = 0.017, 0.008
BOW_X, BOW_R = 0.029, 0.007
BLADE_X0, BLADE_X1 = 0.036, 0.052
KEYS_START = (0.215, -0.290)
# -- the glasses case, the tray, the dish -------------------------------------------------------------------------------
CASE_L, CASE_W, CASE_H = 0.155, 0.058, 0.036
CASE_START = (0.222, 0.295)
TRAY_X, TRAY_Y, TRAY_L, TRAY_W, TRAY_WALL, TRAY_FLOOR = 0.400, 0.300, 0.200, 0.090, 0.018, 0.004
DISH_X, DISH_Y, DISH_R, DISH_H, DISH_FLOOR = 0.400, -0.300, 0.048, 0.022, 0.006
# -- the rules ---------------------------------------------------------------------------------------------------------
SHUT = 0.004  # a drawer within this of shut is shut
PEEK = 0.008  # a drawer out more than this has been opened (a crack too narrow to see the keys through)
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

OAK, MARBLE, VEIN = (0.72, 0.55, 0.36), (0.86, 0.86, 0.84), (0.60, 0.61, 0.64)
CHEST, CHEST_D, GAP = (0.47, 0.30, 0.18), (0.36, 0.22, 0.13), (0.10, 0.07, 0.05)
BRASS, BRASS_D, STEEL, BLACK, GREY = (0.82, 0.67, 0.32), (0.62, 0.48, 0.20), (0.74, 0.75, 0.78), (0.06, 0.06, 0.07), \
    (0.35, 0.36, 0.38)
NAVY, NAVY_D, LEATHER, LEATHER_D = (0.10, 0.14, 0.32), (0.06, 0.08, 0.20), (0.55, 0.33, 0.18), (0.40, 0.22, 0.11)
CERAMIC, GLAZE = (0.94, 0.94, 0.92), (0.22, 0.38, 0.62)

def open_drawer(seed) -> int:
    """The drawer standing open at the start (0-3 from the left): seed mod 4; drawer 2 without a seed."""
    return 2 if seed is None else int(seed) % 4


def _seed():
    """The seed of the trial being reset or run (kept on the embodiment class, so every copy of this file sees it)."""
    mod = sys.modules.get("yam_arena.inspect_embodiment")
    return getattr(getattr(mod, "YamArenaEmbodiment", None), "_keys_seed", None)


def _top() -> Body:
    """The polished marble top under the chest: the chest's glides slide on it."""
    shapes = [BoxShape((0.0, 0.0, WT_H / 2), (WX1 - WX0, WY1 - WY0, WT_H), MARBLE)]
    rng = np.random.default_rng(5)
    for _ in range(14):  # grey veins
        shapes.append(BoxShape((rng.uniform(-0.15, 0.15), rng.uniform(-0.23, 0.23), WT_H + 0.0002),
                               (rng.uniform(0.04, 0.12), rng.uniform(0.001, 0.003), 0.0004), VEIN, collide=False,
                               yaw=rng.uniform(-0.6, 0.6)))
    return Body.build("marble_top", pos=((WX0 + WX1) / 2, (WY0 + WY1) / 2, TOP), mass=1.0, kinematic=True,
                      color=MARBLE, shapes=shapes, friction=(0.045, 0.04), slick=True)


def _chest() -> Body:
    """The carcass (bottom, top, ends, dividers, back) and the brass carry handle; it slides on its glides."""
    zc = BOT_T + OPEN_H / 2
    shapes = [BoxShape((0.0, 0.0, BOT_T / 2), (CH_D, CH_W, BOT_T), CHEST),
              BoxShape((-0.003, 0.0, BOT_T + OPEN_H + TOPB_T / 2), (CH_D + 0.006, CH_W + 0.012, TOPB_T), CHEST),
              BoxShape((-CH_D / 2 - 0.0035, 0.0, BOT_T + OPEN_H + 0.002), (0.0012, CH_W + 0.012, 0.003), CHEST_D,
                       collide=False),  # the top board's moulded front edge
              BoxShape((CH_D / 2 - 0.005, 0.0, zc), (0.010, CH_W - 2 * SIDE_T, OPEN_H), CHEST)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((0.0, s * (CH_W / 2 - SIDE_T / 2), zc), (CH_D, SIDE_T, OPEN_H), CHEST))
    for k in range(3):
        y = CH_W / 2 - SIDE_T - (k + 1) * DW - k * DIV_T - DIV_T / 2
        shapes.append(BoxShape((0.0, y, zc), (CH_D, DIV_T, OPEN_H), CHEST))
    for y in DRAWER_Y:  # the dark gaps round each drawer front
        shapes.append(BoxShape((-CH_D / 2 + 0.002, y, zc), (0.004, DW, OPEN_H), GAP, collide=False))
    shapes.append(BoxShape((-CH_D / 2 - 0.0006, 0.0, 0.004), (0.001, CH_W, 0.006), CHEST_D, collide=False))  # plinth
    top = BOT_T + OPEN_H + TOPB_T
    for sy in (-1.0, 1.0):  # a handle near each end: two posts on round plates, and the bar between them
        for sx in (-1.0, 1.0):
            shapes.append(CylShape((sx * HANDLE_POST_X, sy * HANDLE_Y, top + 0.0012), 0.011, 0.0024, BRASS_D,
                                   collide=False))
            shapes.append(BoxShape((sx * HANDLE_POST_X, sy * HANDLE_Y, top + (HANDLE_Z - top) / 2),
                                   (0.010, 0.012, HANDLE_Z - top), BRASS))
        shapes.append(CylShape((0.0, sy * HANDLE_Y, HANDLE_Z), HANDLE_R, 2 * HANDLE_POST_X + 0.010, BRASS, axis="X"))
    return Body.build("chest", pos=(CH_X, 0.0, CT), mass=CHEST_MASS, color=CHEST, shapes=shapes,
                      friction=(0.9, 0.8), com=(0.01, 0.0, 0.030))


def _drawer(i: int) -> Body:
    zf = FLOOR_Z
    shapes = [BoxShape((FRONT_T / 2, 0.0, 0.0), (FRONT_T, DW - 0.003, FRONT_H), CHEST),
              BoxShape((-0.0006, 0.0, 0.0), (0.001, DW - 0.016, FRONT_H - 0.014), CHEST_D, collide=False),  # bevel
              BoxShape((FRONT_T + BOX_D / 2, 0.0, zf - 0.003), (BOX_D, BOX_W, 0.006), OAK),  # floor
              BoxShape((FRONT_T + BOX_D - WALL_T / 2, 0.0, zf + BOX_H / 2), (WALL_T, BOX_W, BOX_H), OAK)]  # back
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((FRONT_T + BOX_D / 2, s * (BOX_W / 2 - WALL_T / 2), zf + BOX_H / 2),
                               (BOX_D, WALL_T, BOX_H), OAK))
    shapes.append(BoxShape((FRONT_T + BOX_D / 2, 0.0, zf + 0.0003), (BOX_D - 0.004, BOX_W - 2 * WALL_T, 0.0006),
                           (0.30, 0.16, 0.18), collide=False))  # a felt lining
    shapes += [CylShape((-KNOB_STEM / 2, 0.0, 0.0), 0.0055, KNOB_STEM, BRASS_D, axis="X"),
               CylShape((KNOB_X, 0.0, 0.0), KNOB_R, KNOB_HEAD, BRASS, axis="X"),
               CylShape((-0.0004, 0.0, 0.0), 0.011, 0.0008, BRASS_D, collide=False, axis="X")]  # the knob's rose
    return Body.build(f"drawer{i}", pos=(FACE_X, DRAWER_Y[i], DR_Z), mass=DRAWER_MASS, color=CHEST, shapes=shapes,
                      friction=(0.8, 0.7), com=(FRONT_T + BOX_D / 2, 0.0, zf + 0.01))


def _keys() -> Body:
    """A car key fob (black, three buttons), a steel split ring and a brass house key, lying flat."""
    zb = -FOB_H / 2  # their undersides
    shapes = [BoxShape((0.0, 0.0, 0.0), (FOB_L, FOB_W, FOB_H), BLACK),
              CylShape((-FOB_L / 2, 0.0, 0.0), FOB_W / 2, FOB_H, BLACK),
              BoxShape((FOB_L / 2 + 0.002, 0.0, zb + 0.004), (0.006, 0.008, 0.006), GREY),  # the loop for the ring
              CylShape((RING_X, 0.0, zb + 0.0015), RING_R, 0.003, None),  # the ring's collision disc
              CylShape((BOW_X, 0.0, zb + 0.00125), BOW_R, 0.0025, BRASS),
              CylShape((BOW_X, 0.0, zb + 0.0026), 0.0025, 0.0004, BRASS_D, collide=False),  # the bow's hole
              BoxShape(((BLADE_X0 + BLADE_X1) / 2, 0.0, zb + 0.00125), (BLADE_X1 - BLADE_X0, 0.007, 0.0025), BRASS)]
    for k in range(3):  # the fob's buttons
        shapes.append(CylShape((-0.011 + 0.008 * k, 0.0, FOB_H / 2 + 0.0004), 0.003, 0.0008,
                               (0.75, 0.12, 0.10) if k == 0 else GREY, collide=False))
    for k in range(16):  # the split ring, drawn
        a0, a1 = 2 * math.pi * k / 16, 2 * math.pi * (k + 1) / 16
        p0 = np.array([RING_X + RING_R * math.cos(a0), RING_R * math.sin(a0), zb + 0.0016])
        p1 = np.array([RING_X + RING_R * math.cos(a1), RING_R * math.sin(a1), zb + 0.0016])
        shapes.append(plate(p0, p1, 0.0024, 0.0024, [0.0, 0.0, 1.0], STEEL, collide=False))
    for k in range(4):  # the key's teeth
        shapes.append(BoxShape((BLADE_X0 + 0.003 + 0.0035 * k, -0.0038, zb + 0.00125), (0.0018, 0.0016, 0.0024),
                               BRASS_D, collide=False))
    return Body.build("keys", pos=(KEYS_START[0], KEYS_START[1], TOP + FOB_H / 2), mass=0.045, color=BLACK,
                      shapes=shapes, friction=(0.9, 0.8), com=(0.004, 0.0, 0.0))


def _case() -> Body:
    """A navy hard-shell glasses case with rounded ends, its seam and its hinge."""
    core = CASE_L - CASE_W
    shapes = [BoxShape((0.0, 0.0, 0.0), (core, CASE_W, CASE_H), NAVY)]
    for s in (-1.0, 1.0):
        shapes.append(CylShape((s * core / 2, 0.0, 0.0), CASE_W / 2, CASE_H, NAVY))
    shapes += [BoxShape((0.0, 0.0, 0.002), (core + 0.002, CASE_W + 0.0012, 0.0016), NAVY_D, collide=False),
               BoxShape((0.0, CASE_W / 2 + 0.0004, 0.004), (0.030, 0.001, 0.005), STEEL, collide=False)]
    return Body.build("glasses_case", pos=(CASE_START[0], CASE_START[1], TOP + CASE_H / 2), mass=0.07, color=NAVY,
                      shapes=shapes, friction=(0.8, 0.7))


def _tray() -> Body:
    shapes = [BoxShape((0.0, 0.0, TRAY_FLOOR / 2), (TRAY_L, TRAY_W, TRAY_FLOOR), LEATHER)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (TRAY_L / 2 - 0.003), 0.0, TRAY_WALL / 2), (0.006, TRAY_W, TRAY_WALL), LEATHER_D))
        shapes.append(BoxShape((0.0, s * (TRAY_W / 2 - 0.003), TRAY_WALL / 2), (TRAY_L, 0.006, TRAY_WALL), LEATHER_D))
    shapes.append(BoxShape((0.0, 0.0, TRAY_FLOOR + 0.0002), (TRAY_L - 0.020, TRAY_W - 0.020, 0.0004),
                           (0.62, 0.40, 0.24), collide=False))  # the stitched inner panel
    return Body.build("tray", pos=(TRAY_X, TRAY_Y, TOP), mass=0.2, kinematic=True, color=LEATHER, shapes=shapes,
                      friction=(0.9, 0.8))


def _dish() -> Body:
    shapes = [CylShape((0.0, 0.0, DISH_FLOOR / 2), DISH_R - 0.004, DISH_FLOOR, CERAMIC)]
    n = 24
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        nrm = np.array([math.cos(a), math.sin(a), 0.0])
        r = DISH_R - 0.003
        shapes.append(plate(r * nrm + [0, 0, DISH_FLOOR - 0.001], r * nrm + [0, 0, DISH_H],
                            2 * DISH_R * math.tan(math.pi / n) + 0.001, 0.006, nrm, CERAMIC))
    for k in range(n):  # the blue glazed rim
        a = 2 * math.pi * (k + 0.5) / n
        nrm = np.array([math.cos(a), math.sin(a), 0.0])
        shapes.append(plate((DISH_R - 0.003) * nrm + [0, 0, DISH_H - 0.004], (DISH_R - 0.003) * nrm + [0, 0, DISH_H + 0.0005],
                            2 * DISH_R * math.tan(math.pi / n) + 0.001, 0.0064, nrm, GLAZE, collide=False))
    return Body.build("dish", pos=(DISH_X, DISH_Y, TOP), mass=0.3, kinematic=True, color=CERAMIC, shapes=shapes,
                      friction=(0.7, 0.6))


OBJECTS = [_top(), _chest(), *[_drawer(i) for i in range(4)], _keys(), _case(), _tray(), _dish()]
for _i in range(4):
    slider(f"runner{_i}", "chest", f"drawer{_i}", anchor=(FACE_X, DRAWER_Y[_i], DR_Z), axis="X",
           limits=(-OUT_MAX, 0.0), stiffness=RUN_K, damping=RUN_DAMP, target=0.0, max_force=RUN_F)


# -- the trial's open drawer: the seed is caught as the embodiment resets, and the open drawer's runner is set to hold it
# out before the arms and objects settle: as the runner is authored (the first reset builds the scene) and after
# bodykit puts every runner back to its authored target (later resets). bodykit looks both functions up in its own
# module's globals, so the hooks go there (whichever copy of bodykit patched the class) -------------------------------

def _patch_reset(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_keys_seed_caught", False):
        return
    plain = cls.reset
    g = plain.__globals__  # the globals of the bodykit copy whose reset wrapper this is
    plain_sliders_reset = g.get("_sliders_reset")

    def sliders_reset() -> None:
        if plain_sliders_reset is not None:
            plain_sliders_reset()
        a = open_drawer(_seed())
        if f"runner{a}" in g.get("_SLIDER_PATHS", {}):
            g["set_slider_drive"](f"runner{a}", target=-AJAR)

    if plain_sliders_reset is not None:
        g["_sliders_reset"] = sliders_reset
    plain_author = g.get("_author_sliders")

    def author_sliders(stage, prim_path: str, spec) -> None:
        """The first reset builds the scene after bodykit has reset the runners: set the open drawer's target as its
        runner is authored."""
        plain_author(stage, prim_path, spec)
        a = open_drawer(_seed())
        if spec.name == f"drawer{a}":
            from pxr import UsdPhysics

            drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/runner{a}"), "linear")
            drive.GetTargetPositionAttr().Set(-AJAR)

    if plain_author is not None:
        g["_author_sliders"] = author_sliders

    def reset(self, *args, **kwargs):
        type(self)._keys_seed = kwargs.get("seed")
        return plain(self, *args, **kwargs)

    cls.reset, cls._keys_seed_caught = reset, True


def _hook_reset() -> None:
    name = "yam_arena.inspect_embodiment"
    if name in sys.modules:
        _patch_reset(sys.modules[name])
        return
    plain = bodykit._TARGETS.get(name)

    def patch(module) -> None:  # bodykit patches the class first, then this wraps it
        if plain is not None:
            plain(module)
        _patch_reset(module)

    bodykit._TARGETS[name] = patch


_hook_reset()


# -- followed after every control step -----------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"a": open_drawer(_seed()), "hid": False, "wrong": False, "glasses": False, "interlude": False,
            "was_in_tray": True, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("KEYS_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if _DEBUG["on"]:
        print(f"[keys] reset seed={_seed()} open drawer={_EP['a']} module={__name__} bodykit={id(bodykit)}",
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


def _in_drawer(q) -> bool:
    """The keys' fob (in a drawer's frame) inside that drawer's box."""
    return IN_X0 < q[0] < IN_X1 and abs(q[1]) < IN_Y and FLOOR_Z - 0.01 < q[2] < FLOOR_Z + 0.03


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pc, rc = _pose(base, origin, "chest")
    pk, rk = _pose(base, origin, "keys")
    pg, rg = _pose(base, origin, "glasses_case")
    outs, inside = [], []
    for i in range(4):
        pd, rd = _pose(base, origin, f"drawer{i}")
        out = -float((rc.T @ (pd - pc))[0] + CH_D / 2)
        outs.append(out)
        inside.append(_in_drawer(rd.T @ (pk - pd)))
        # the runners: felt friction (the drive holds the drawer where it is, up to RUN_F) and the catch near shut
        set_slider_drive(f"runner{i}", target=0.0 if out < CATCH else -out)
    a = _EP["a"]
    all_shut = all(o < SHUT for o in outs)
    hidden = all_shut and inside[a]
    if hidden:
        _EP["hid"] = True
    if _EP["hid"] and any(o > PEEK for i, o in enumerate(outs) if i != a):
        _EP["wrong"] = True
    q = pg - np.array([TRAY_X, TRAY_Y, TOP])
    in_tray = (abs(q[0]) < TRAY_L / 2 - 0.006 and abs(q[1]) < TRAY_W / 2 - 0.006
               and q[2] < TRAY_FLOOR + CASE_H / 2 + 0.006 and float(rg[2, 2]) > math.cos(math.radians(25.0)))
    if in_tray and not _EP["was_in_tray"] and hidden:
        _EP["glasses"] = True  # put in the tray while the keys were put away
    if not hidden:
        _EP["glasses"] = _EP["glasses"] and _EP["interlude"]
    _EP["was_in_tray"] = in_tray
    _EP["tcps"] = _tcps(base, origin)
    home = _arms_home()
    if _EP["glasses"] and hidden and in_tray and home:
        _EP["interlude"] = True
    kd = pk - np.array([DISH_X, DISH_Y, TOP])
    in_dish = math.hypot(kd[0], kd[1]) < DISH_R - 0.008 and kd[2] < DISH_H + 0.01
    _EP["s"] = {"outs": outs, "all_shut": all_shut, "hidden": hidden, "in_tray": in_tray, "in_dish": in_dish,
                "home": home}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, CT]) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[x].data.joint_pos)[0][base.scene[x].joint_names.index("joint7")]) * 1000
                  for x in ("left_robot", "right_robot")]
            yaw = math.degrees(math.atan2(rc[1, 0], rc[0, 0]))
            print(f"[keys] t={_DEBUG['steps'] / 20:5.1f}s outs={[round(o * 1000) for o in outs]} chest={mm(pc)}"
                  f" yaw={yaw:.1f} keys={mm(pk)} in={inside} hidden={hidden} hid={_EP['hid']} wrong={_EP['wrong']}"
                  f" case={mm(pg)} tray={in_tray} glasses={_EP['glasses']} home={home} interlude={_EP['interlude']}"
                  f" dish={in_dish} fingers={[round(v, 1) for v in fj]} L={mm(tl)} R={mm(tr)} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """0.0 if, once the keys were put away, a drawer other than theirs was opened (more than 8 mm). Otherwise 1.0: the
    keys put away and fetched as told, and at the end the keys in the key dish, the glasses case in the tray, all four
    drawers shut and both arms back where they started; 0.75: at the end the keys in the key dish (after the
    interlude); 0.5: at some point, with the keys shut in the drawer that stood open, the glasses case put in the tray
    and then both arms back where they started; 0.25: at some point the keys shut in the drawer that stood open, all
    drawers shut; else 0.0."""
    s = _EP["s"]
    if not s or _EP["wrong"]:
        return 0.0
    if _EP["interlude"]:
        if s["in_dish"]:
            return 1.0 if s["in_tray"] and s["all_shut"] and s["home"] else 0.75
        return 0.5
    return 0.25 if _EP["hid"] else 0.0


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
    """Full score with the chest, its drawers, the keys and the glasses case at rest: the trial ends there."""
    names = ["chest", "keys", "glasses_case"] + [f"drawer{i}" for i in range(4)]
    return score(objects) >= 1.0 and all(objects[n].resting() for n in names)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# For the demo's seed (the default: drawer 2, the third from the left, stands open). The right hand holds the chest by
# its right carry handle, the one nearest the drawer, leaning against the pull, while the left pulls the open drawer
# out to its stop by the knob. The right lets go, picks up the keys by the fob and lays them in the drawer, then holds
# the chest again while the left pushes the drawer shut with its closed fingers against the front, beside the knob. The left puts the glasses case in the tray; both arms go home
# and wait. Then the same again the other way: the right holds, the left pulls the same drawer out, the right takes the
# keys out and drops them in the key dish, holds again, the left pushes the drawer shut; both arms go home.

DEMO_DRAWER = 2
HANDLE_FINGER, KNOB_FINGER, FOB_FINGER, CASE_FINGER = -0.016, -0.022, -0.020, -0.036
FOB_TCP_UP = 0.0145  # the TCP this far above the fob's underside
PUSH_PAST = 0.012  # push the drawer front this much past shut: the arm gives, and the catch does the rest
PUSH_Y, PUSH_Z = 0.028, 0.040  # the closed fingers push the drawer front this far beside the knob, the TCP this high
FINGER_FRONT = 0.010  # the closed fingers' front faces, ahead of the TCP
PULL_OVER = 0.010  # pull the knob this much past the stop, so the drawer surely reaches it
BRACE = 0.012  # the holding hand leans this far against the pull or the push
KEYS_IN_X = 0.035  # the fob's middle this far behind the drawer front's inner face
LEAD_S, LEAD_STEPS = 3.0, 15


def _rot(kind: str = "") -> np.ndarray:
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
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}

    def go(arm, t, pos, finger, label, n=1):
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, _rot()))
        cur[arm] = (p1, finger, _rot())

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
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, _rot()), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, _rot())

    def to_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)
        last = plan[arm][-1]
        cur[arm] = (np.asarray(last.pos, float), f, last.yaw)

    y_d = DRAWER_Y[DEMO_DRAWER]
    knob_shut = np.array([FACE_X + KNOB_X, y_d, DR_Z])
    knob_open = knob_shut - [OUT_MAX, 0.0, 0.0]
    up = np.array([0.0, 0.0, 0.05])
    handle = np.array([CH_X, -HANDLE_Y, CT + HANDLE_Z])  # the right-hand carry handle, nearest the drawer
    handle_up = handle + [0.0, 0.0, 0.04]
    park = np.array([0.24, 0.13, CT + 0.12])  # the left waits here, clear of the right's work over the drawer
    floor_top = DR_Z + FLOOR_Z
    keys_in = np.array([FACE_X - OUT_MAX + FRONT_T + KEYS_IN_X, y_d, floor_top + FOB_TCP_UP])
    keys_start = np.array([KEYS_START[0], KEYS_START[1], TOP + FOB_TCP_UP])
    carry_z = CT + 0.135  # above the chest's carry handles
    case_at = np.array([CASE_START[0], CASE_START[1], TOP + 0.014])
    case_in = np.array([TRAY_X, TRAY_Y, TOP + TRAY_FLOOR + 0.005 + 0.014])
    dish_drop = np.array([DISH_X, DISH_Y, TOP + 0.055])

    def hold(label):
        go("right", 1.2, handle, HANDLE_FINGER, f"down round the chest's right carry handle ({label})")
        go("right", 0.6, handle, FINGER_CLOSED, "hold the chest by its handle")

    def let_go_of_chest():
        go("right", 0.5, handle, HANDLE_FINGER, "let go of the handle")
        go("right", 0.6, handle_up, HANDLE_FINGER, "rise")

    def work_drawer(start, end, sign, label):
        """The left grips the knob at ``start`` and pulls it to ``end`` while the right leans against the pull."""
        go("left", 1.2, start, KNOB_FINGER, "down round the drawer's knob")
        go("left", 0.6, start, FINGER_CLOSED, "grip the knob")
        sync("wait: the chest held, the knob gripped")
        go("left", 2.4, end, FINGER_CLOSED, label, n=4)
        go("right", 2.4, handle + [sign * BRACE, 0.0, 0.0], FINGER_CLOSED, "lean against it", n=4)
        wait("left", 0.4, "hold")
        wait("right", 0.4, "hold")
        go("left", 0.5, end, KNOB_FINGER, "let go of the knob")
        go("right", 0.5, handle, FINGER_CLOSED, "ease off")
        go("left", 0.6, end + up, KNOB_FINGER, "rise")
        wait("right", 0.6, "hold")

    def push_shut():
        """The left's closed fingers push the drawer's front shut beside the knob while the right leans against it."""
        y_p = y_d + PUSH_Y
        before = np.array([FACE_X - OUT_MAX - 0.030 - FINGER_FRONT, y_p, CT + PUSH_Z])
        done = np.array([FACE_X + PUSH_PAST - FINGER_FRONT, y_p, CT + PUSH_Z])
        go("left", 1.2, before + up, FINGER_CLOSED, "over the open drawer's front, beside the knob", n=2)
        go("left", 1.0, before, FINGER_CLOSED, "down in front of it")
        sync("wait: the chest held")
        go("left", 2.8, done, FINGER_CLOSED, "push the drawer shut", n=4)
        go("right", 2.8, handle - [BRACE, 0.0, 0.0], FINGER_CLOSED, "lean against the push", n=4)
        wait("left", 0.4, "hold")
        wait("right", 0.4, "hold")
        go("left", 0.5, done - [0.02, 0.0, 0.0], FINGER_CLOSED, "back off")
        go("right", 0.5, handle, FINGER_CLOSED, "ease off")
        go("left", 0.6, done - [0.02, 0.0, 0.0] + up, FINGER_CLOSED, "rise")
        wait("right", 0.6, "hold")

    # 1. the right holds the chest; the left pulls the open drawer out to its stop
    from_home("right", handle_up, HANDLE_FINGER, "over the chest's right carry handle")
    hold("1")
    from_home("left", knob_shut - [AJAR_SEEN, 0.0, 0.0] + up, KNOB_FINGER, "over the open drawer's knob")
    work_drawer(knob_shut - [AJAR_SEEN, 0.0, 0.0], knob_open - [PULL_OVER, 0.0, 0.0], 1.0,
                "pull the drawer out to its stop")
    # 2. the keys into the drawer (the left waits clear)
    go("left", 1.2, park, KNOB_FINGER, "out of the way", n=2)
    let_go_of_chest()
    go("right", 1.8, keys_start + [0.0, 0.0, 0.06], FOB_FINGER, "over the keys' fob", n=3)
    go("right", 1.0, keys_start, FOB_FINGER, "down round the fob")
    go("right", 0.6, keys_start, FINGER_CLOSED, "pick up the keys")
    go("right", 1.0, [keys_start[0], keys_start[1], carry_z], FINGER_CLOSED, "lift them")
    go("right", 1.8, [keys_in[0], keys_in[1], carry_z], FINGER_CLOSED, "carry them over the open drawer", n=3)
    go("right", 1.0, keys_in + [0.0, 0.0, 0.006], FINGER_CLOSED, "lower them into the drawer")
    go("right", 0.5, keys_in + [0.0, 0.0, 0.006], FOB_FINGER, "let go")
    go("right", 0.6, [keys_in[0], keys_in[1], carry_z], FOB_FINGER, "rise")
    sync()
    go("right", 1.6, handle_up, HANDLE_FINGER, "back over the handle", n=2)
    # 3. hold again; the left pushes the drawer shut
    hold("2")
    push_shut()
    let_go_of_chest()
    sync()
    # 4. the glasses case into the tray; both arms home and wait
    to_home("right")
    case_up = TOP + 0.13 - case_at[2]  # carried well above the chest's top and handles
    go("left", 1.8, case_at + [0.0, 0.0, 0.08], CASE_FINGER, "over the glasses case", n=3)
    go("left", 1.0, case_at, CASE_FINGER, "down round it")
    go("left", 0.6, case_at, FINGER_CLOSED, "pick it up")
    go("left", 1.0, case_at + [0.0, 0.0, case_up], FINGER_CLOSED, "lift it")
    go("left", 2.0, case_in + [0.0, 0.0, case_up], FINGER_CLOSED, "carry it over the tray", n=3)
    go("left", 1.2, case_in, FINGER_CLOSED, "lower it into the tray")
    go("left", 0.5, case_in, CASE_FINGER, "let go")
    go("left", 0.6, case_in + [0.0, 0.0, 0.06], CASE_FINGER, "rise")
    to_home("left")
    sync("home")
    wait("left", 1.5, "both arms home: nothing in view shows where the keys are")
    wait("right", 1.5, "both arms home: nothing in view shows where the keys are")
    # 5. hold the chest again; the left pulls the same drawer out
    from_home("right", handle_up, HANDLE_FINGER, "over the chest's right carry handle")
    hold("3")
    from_home("left", knob_shut + up, KNOB_FINGER, "over the knob of the drawer the keys are in")
    work_drawer(knob_shut, knob_open - [PULL_OVER, 0.0, 0.0], 1.0, "pull the drawer out to its stop")
    # 6. the right takes the keys out into the dish
    go("left", 1.2, park, KNOB_FINGER, "out of the way", n=2)
    let_go_of_chest()
    go("right", 1.6, [keys_in[0], keys_in[1], carry_z], FOB_FINGER, "over the keys", n=2)
    go("right", 1.2, keys_in, FOB_FINGER, "down round the fob")
    go("right", 0.6, keys_in, FINGER_CLOSED, "take the keys")
    go("right", 1.0, [keys_in[0], keys_in[1], carry_z], FINGER_CLOSED, "lift them out")
    go("right", 1.4, [keys_in[0] + 0.01, dish_drop[1], carry_z], FINGER_CLOSED,
       "carry them round the front of the chest", n=2)
    go("right", 1.4, [dish_drop[0], dish_drop[1], carry_z], FINGER_CLOSED, "then back over the key dish", n=2)
    go("right", 0.8, dish_drop, FINGER_CLOSED, "lower them")
    go("right", 0.5, dish_drop, FOB_FINGER, "drop them in the dish")
    go("right", 0.6, [dish_drop[0], dish_drop[1], carry_z], FOB_FINGER, "rise")
    sync()
    go("right", 1.4, handle_up, HANDLE_FINGER, "back over the handle (from the dish, along the chest's right end)",
       n=2)
    # 7. hold again; the left pushes the drawer shut; both arms home
    hold("4")
    push_shut()
    let_go_of_chest()
    sync()
    for arm in ("left", "right"):
        to_home(arm)
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
