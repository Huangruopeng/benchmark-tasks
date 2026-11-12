"""A robot's camera cable has a frayed jacket. It is plugged into one port of a 7-port USB hub that lies loose on a
slick white desk riser, among a few other cables; the rest of the ports are empty. The robot's software finds each
camera by the port it is plugged into, so the new cable must go into the same port. The new cable's plug is parked
upright in a grey cable dock to the right of the hub, and a blue bin for scrap stands on the desk in front.

Task: hold the hub down with one hand while the other pulls the frayed cable's plug straight up out of its port, and
drop it in the bin; then take the new cable's plug out of the dock and push it all the way into the port the frayed one
came out of; then take both arms back to where they started. A plug put into any other port, even part way, fails the
trial, and so does taking the new plug out of its dock before the frayed one is in the bin (no leaving the old plug by
the port as a marker). Which port the camera uses (and which other ports hold cables) depends on the trial's seed:
once the frayed plug is out, its port looks like the other empty ones, so the policy has to remember it.

Run it:  inspect-robots run --embodiment yam_arena -E task=swap_a_frayed_camera_cable_into_the_same_usb_port.py \\
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

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the desk riser (slick white laminate) ---------------------------------------------------------------------------
RISER_X, RISER_Y = (0.230, 0.400), (-0.360, 0.045)
RISER_TOP = TOP + 0.060

# -- the hub (7 ports on its top face, numbered 0-6 from the left as the arms see it) --------------------------------
HX, HD, HH, FEET = 0.258, 0.056, 0.024, 0.002  # its front face is flush with the riser's front edge
HUB_BOT = RISER_TOP + FEET
HUB_TOP = HUB_BOT + HH
Y_LEFT, BLANK, PITCH, N_PORTS = 0.070, 0.050, 0.036, 7  # its left end overhangs the riser by 2.5 cm
PORT_Y = tuple(Y_LEFT - BLANK - 0.018 - j * PITCH for j in range(N_PORTS))
Y_RIGHT = PORT_Y[-1] - 0.018 - 0.014
HY = (Y_LEFT + Y_RIGHT) / 2
HL = Y_LEFT - Y_RIGHT
PORT_W, PORT_T, PORT_D = 0.0200, 0.0110, 0.0170  # a port's opening (along y, along x) and depth
SEAT = HUB_TOP - PORT_D  # a port's floor
HUB_MASS = 0.10

# -- the plugs (USB-A, right-angled: the cable leaves the overmould toward the front) -----------------------------------
CON = (0.0075, 0.0165, PORT_D)  # the steel connector (x, y, z)
OVER = (0.012, 0.022, 0.030)  # the black overmould above it
CAB_R = 0.003
PLUG_MASS = 0.018
DOCK_Y = -0.300  # the new cable's dock
DOCK_W = 0.040

# -- the latch: a USB plug's spring tabs hold it in its port (about 3 N), as a stick-slip along the port --------------
K_L, SLIP_L, D_L = 2000.0, 0.0015, 4.0
X_OUT = PORT_D + 0.002  # pulled this far up, the connector is out of the port
CATCH_R, CATCH_TILT, CATCH_X = 0.009, math.radians(12.0), (0.004, 0.020)  # generous: a plug set on the port's mouth
SEATED_X = 0.003
WRONG_X = PORT_D - 0.005  # a plug sunk 5 mm or more into a port

# -- the bin ----------------------------------------------------------------------------------------------------------
BIN_C, BIN_IN, BIN_WALL = np.array([0.120, -0.190]), (0.160, 0.160), 0.035

LAMINATE, EDGE = (0.92, 0.92, 0.90), (0.70, 0.70, 0.68)
HUB_C, TRIM = (0.15, 0.16, 0.18), (0.72, 0.74, 0.77)
BLACK, STEEL, BLUE = (0.05, 0.05, 0.06), (0.72, 0.73, 0.75), (0.12, 0.28, 0.70)
USB3, LED_OFF, LED_ON = (0.10, 0.35, 0.85), (0.10, 0.12, 0.16), (0.30, 0.60, 1.00)
COPPER, GREY = (0.85, 0.48, 0.22), (0.45, 0.46, 0.48)
RED, GREEN, WHITE, YELLOW = (0.95, 0.12, 0.08), (0.12, 0.90, 0.25), (0.92, 0.92, 0.92), (0.92, 0.78, 0.15)
CABLE_COLORS = (BLACK, GREY, WHITE)

# -- scoring ----------------------------------------------------------------------------------------------------------
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0

# -- the trial's layout: which port the camera uses and which other ports hold cables (by seed) -----------------------
LAYOUTS = [(k, o) for k in range(N_PORTS) for o in itertools.combinations([j for j in range(N_PORTS) if j != k], 3)]
DEMO_SEED = 1415158438  # the seed a default run resets with (inspect-robots run --seed 0): camera in port 0
LAYOUT_SHIFT = 73


def layout(seed) -> tuple[int, tuple[int, ...]]:
    """The camera's port and the three other occupied ports for a trial's seed (140 layouts)."""
    return LAYOUTS[((DEMO_SEED if seed is None else int(seed)) + LAYOUT_SHIFT) % len(LAYOUTS)]


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _plug_shapes(cable, frayed: bool, collide: bool, at=(0.0, 0.0, 0.0), hang_collide: bool = False) -> list:
    """A right-angled USB-A plug, origin at the connector's bottom middle: the steel connector (in the port when
    seated), the overmould, and the cable leaving its front face, lying across the hub's top, over its front edge and
    down to the desk."""
    x, y, z = at
    top = PORT_D  # the hub's top, above the origin of a seated plug
    s = [BoxShape((x, y, z + CON[2] / 2), CON, STEEL, collide=collide),
         BoxShape((x + 0.0018, y, z + CON[2] / 2), (0.0025, CON[1] - 0.004, CON[2] - 0.002), USB3, collide=False),
         BoxShape((x, y, z + top + OVER[2] / 2), OVER, BLACK, collide=collide),
         BoxShape((x, y, z + top + OVER[2] + 0.0002), (OVER[0] - 0.003, OVER[1] - 0.004, 0.0004), (0.20, 0.20, 0.22),
                  collide=False),
         CylShape((x - OVER[0] / 2 - 0.003, y, z + top + CAB_R), 0.0045, 0.006, BLACK, collide=False, axis="X")]
    run0, run1 = x - OVER[0] / 2 - 0.006, x - HD / 2 - 0.006  # from the overmould to just past the hub's front
    zc = z + top + CAB_R
    if frayed:  # a stretch of the jacket split open, its braid and wires showing
        f0, f1 = run0 - 0.004, run0 - 0.016
        s += [BoxShape(((run0 + f0) / 2, y, zc), (run0 - f0, 2 * CAB_R, 2 * CAB_R), cable, collide=False),
              BoxShape(((f0 + f1) / 2, y, zc - 0.0005), (f0 - f1, 0.0042, 0.0042), COPPER, collide=False),
              BoxShape(((f0 + f1) / 2, y + 0.0028, zc + 0.0006), (f0 - f1 - 0.002, 0.0012, 0.0012), RED,
                       collide=False),
              BoxShape(((f0 + f1) / 2, y - 0.0026, zc + 0.0010), (f0 - f1 - 0.003, 0.0012, 0.0012), WHITE,
                       collide=False),
              BoxShape(((f0 + f1) / 2 + 0.001, y + 0.0005, zc + 0.0022), (f0 - f1 - 0.004, 0.0012, 0.0012), GREEN,
                       collide=False),
              BoxShape((f0 - 0.0015, y + 0.0042, zc + 0.0012), (0.004, 0.0015, 0.0045), cable, yaw=0.5,
                       collide=False),
              BoxShape((f1 + 0.0015, y - 0.0042, zc + 0.0012), (0.004, 0.0015, 0.0045), cable, yaw=-0.5,
                       collide=False),
              BoxShape(((f1 + run1) / 2, y, zc), (f1 - run1, 2 * CAB_R, 2 * CAB_R), cable, collide=False)]
    else:
        s.append(BoxShape(((run0 + run1) / 2, y, zc), (run0 - run1, 2 * CAB_R, 2 * CAB_R), cable, collide=False))
    bottom = z + (TOP + 0.002) - SEAT  # the desk, below a seated plug's origin
    s.append(CylShape((run1, y, (zc + bottom) / 2), CAB_R, zc - bottom, cable, collide=hang_collide))
    s.append(CylShape((run1, y, zc), CAB_R, 2 * CAB_R, cable, collide=False, axis="Y"))  # the bend over the edge
    return s


_HUB_IDX: dict[str, int] = {}


def _hub(k: int, others) -> Body:
    """The hub (origin at its middle on the riser's top): body, ports, port lights, the other cables, and the hub's
    own lead to the computer."""
    s = []
    for sx in (-1.0, 1.0):  # rubber feet
        for sy in (-1.0, 1.0):
            s.append(BoxShape((sx * (HD / 2 - 0.008), sy * (HL / 2 - 0.015), FEET / 2), (0.010, 0.016, FEET), BLACK))
    floor = SEAT - RISER_TOP
    top = HUB_TOP - RISER_TOP
    s.append(BoxShape((0.0, 0.0, (FEET + floor) / 2), (HD, HL, floor - FEET), HUB_C))
    # the upper part: solid between the ports, slotted at each port
    edges = [Y_LEFT]
    for py in PORT_Y:
        edges += [py + PORT_W / 2, py - PORT_W / 2]
    edges.append(Y_RIGHT)
    zc, zh = (floor + top) / 2, top - floor
    for a, b in zip(edges[0::2], edges[1::2]):  # solid stretches (left end, between ports, right end)
        s.append(BoxShape((0.0, (a + b) / 2 - HY, zc), (HD, a - b, zh), HUB_C))
    for py in PORT_Y:  # beside each port: the strips in front of and behind its slot
        w = (HD - PORT_T) / 2
        for sx in (-1.0, 1.0):
            s.append(BoxShape((sx * (PORT_T / 2 + w / 2), py - HY, zc), (w, PORT_W, zh), HUB_C))
        s.append(BoxShape((0.0, py - HY, floor + 0.0004), (PORT_T, PORT_W, 0.0008), BLACK, collide=False))
        s.append(BoxShape((0.0018, py - HY, floor + 0.0045), (0.0025, PORT_W - 0.005, 0.008), USB3, collide=False))
        s.append(BoxShape((0.0, py - HY, top + 0.0002), (PORT_T + 0.003, PORT_W + 0.003, 0.0004), BLACK,
                          collide=False))
        s.append(BoxShape((-0.020, py - HY, top + 0.0003), (0.004, 0.006, 0.0006), LED_OFF, collide=False))
        s.append(BoxShape((0.018, py - HY, top + 0.0003), (0.006, 0.0015, 0.0006), WHITE, collide=False))
    s += [BoxShape((-HD / 2 + 0.0015, 0.0, top + 0.0003), (0.003, HL - 0.004, 0.0006), TRIM, collide=False),
          BoxShape((HD / 2 - 0.0015, 0.0, top + 0.0003), (0.003, HL - 0.004, 0.0006), TRIM, collide=False),
          BoxShape((0.0, Y_LEFT - HY - 0.022, top + 0.0003), (0.010, 0.010, 0.0006), (0.20, 0.55, 0.95),
                   collide=False)]  # the hub's power light
    # the hub's own lead: out of its right end and down off the back of the riser
    s += [CylShape((0.012, -HL / 2 - 0.010, zc), CAB_R + 0.0005, 0.020, BLACK, collide=False, axis="Y"),
          BoxShape((0.012 + 0.07, -HL / 2 - 0.020, zc), (0.14, 2 * CAB_R, 2 * CAB_R), BLACK, collide=False)]
    # lit lights: the other occupied ports (always) and the camera's port (while its cable is in)
    for j in others:
        s.append(BoxShape((-0.020, PORT_Y[j] - HY, top + 0.0007), (0.0045, 0.0065, 0.0008), LED_ON, collide=False))
    _HUB_IDX["led_k"] = len(s)
    s.append(BoxShape((-0.020, PORT_Y[k] - HY, top + 0.0007), (0.0045, 0.0065, 0.0008), LED_ON, collide=False))
    # the other cables, plugged in (part of the hub: they are not to be touched)
    for n, j in enumerate(others):
        s += _plug_shapes(CABLE_COLORS[n % len(CABLE_COLORS)], False, True, at=(0.0, PORT_Y[j] - HY, floor))
    return Body.build("hub", pos=(HX, HY, RISER_TOP), mass=HUB_MASS, shapes=s, color=HUB_C, friction=(0.5, 0.4),
                      com=(0.0, 0.0, 0.012))


def _riser() -> Body:
    """The white laminate desk riser (slick), with a status box for the robot's computer on its back corner."""
    (x0, x1), (y0, y1) = RISER_X, RISER_Y
    h = RISER_TOP - TOP
    s = [BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, h / 2), (x1 - x0, y1 - y0, h), LAMINATE),
         BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, 0.004), (x1 - x0 + 0.002, y1 - y0 + 0.002, 0.008), EDGE,
                  collide=False)]
    return Body.build("riser", pos=(0.0, 0.0, TOP), mass=3.0, kinematic=True, shapes=s, color=LAMINATE,
                      friction=(0.08, 0.06), slick=True)


_PC_IDX: dict[str, int] = {}


def _pc() -> Body:
    """The robot computer's status box on the riser's back corner: a CAMERA lamp (green while the camera is on its
    port, red otherwise)."""
    s = [BoxShape((0.0, 0.0, 0.016), (0.060, 0.070, 0.032), (0.12, 0.12, 0.13)),
         BoxShape((-0.0305, 0.0, 0.016), (0.001, 0.060, 0.022), (0.03, 0.03, 0.035), collide=False),
         CylShape((0.0, 0.010, 0.0325), 0.009, 0.002, (0.20, 0.20, 0.22), collide=False),
         BoxShape((0.0, -0.016, 0.0325), (0.020, 0.018, 0.001), (0.05, 0.10, 0.18), collide=False)]
    _PC_IDX["green"] = len(s)
    s.append(CylShape((0.0, 0.010, 0.0340), 0.0075, 0.002, GREEN, collide=False))
    _PC_IDX["red"] = len(s)
    s.append(CylShape((0.0, 0.010, 0.0342), 0.0075, 0.002, RED, collide=False))
    return Body.build("pc_status", pos=(0.355, 0.000, RISER_TOP), mass=0.5, kinematic=True, shapes=s,
                      color=(0.12, 0.12, 0.13))


def _dock() -> Body:
    """The grey cable dock the new cable's plug is parked in: a block with one socket, flush with the riser's front
    edge; origin at its middle on the riser."""
    floor, top = SEAT - RISER_TOP, HUB_TOP - RISER_TOP
    g, w = (0.55, 0.56, 0.58), (HD - PORT_T) / 2
    s = [BoxShape((0.0, 0.0, floor / 2), (HD, DOCK_W, floor), g),
         BoxShape((0.0, (DOCK_W / 2 + PORT_W / 2) / 2, (floor + top) / 2), (HD, DOCK_W / 2 - PORT_W / 2, top - floor),
                  g),
         BoxShape((0.0, -(DOCK_W / 2 + PORT_W / 2) / 2, (floor + top) / 2), (HD, DOCK_W / 2 - PORT_W / 2,
                                                                             top - floor), g)]
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * (PORT_T / 2 + w / 2), 0.0, (floor + top) / 2), (w, PORT_W, top - floor), g))
    return Body.build("dock", pos=(HX, DOCK_Y, RISER_TOP), mass=0.2, kinematic=True, shapes=s, color=g)


def _plug(name: str, xy, frayed: bool) -> Body:
    return Body.build(name, pos=(float(xy[0]), float(xy[1]), SEAT), mass=PLUG_MASS,
                      shapes=_plug_shapes(BLACK, frayed, True, hang_collide=True), color=BLACK, friction=(1.2, 1.0),
                      com=(0.0, 0.0, 0.026), solver_iterations=32)


def _bin() -> Body:
    w, (lx, ly), hgt = 0.004, BIN_IN, BIN_WALL
    s = [BoxShape((0.0, 0.0, -w / 2), (lx + 2 * w, ly + 2 * w, w), BLUE)]
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * (lx / 2 + w / 2), 0.0, hgt / 2), (w, ly + 2 * w, hgt), BLUE))
    for sy in (-1.0, 1.0):
        s.append(BoxShape((0.0, sy * (ly / 2 + w / 2), hgt / 2), (lx, w, hgt), BLUE))
    s.append(BoxShape((-lx / 2 - w - 0.0003, 0.0, hgt * 0.6), (0.0006, 0.08, 0.012), YELLOW, collide=False))
    return Body.build("bin", pos=(BIN_C[0], BIN_C[1], TOP + w), mass=0.5, kinematic=True, shapes=s, color=BLUE)


def _port_rel(j: int) -> tuple[float, float, float]:
    """Port j's floor in the hub's frame."""
    return (0.0, PORT_Y[j] - HY, SEAT - RISER_TOP)


_K0, _O0 = layout(None)
OBJECTS = [_riser(), _hub(_K0, _O0), _pc(), _dock(), _plug("old_plug", (HX, PORT_Y[_K0]), frayed=True),
           _plug("new_plug", (HX, DOCK_Y), frayed=False), _bin()]
OLD_LATCH = thread("old_latch", "hub", "old_plug", anchor0=_port_rel(_K0), axis0=(0.0, 0.0, 1.0),
                   anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 0.0, 1.0), slide_limits=(-0.001, 0.030), turn_damping=0.5,
                   slide_stiffness=K_L, slide_damping=D_L, slide_max_force=10.0, play=0.0015, tilt_deg=4.0,
                   always_on=True)
NEW_LATCH = thread("new_latch", "hub", "new_plug", anchor0=_port_rel(_K0), axis0=(0.0, 0.0, 1.0),
                   anchor1=(0.0, 0.0, 0.0), axis1=(0.0, 0.0, 1.0), slide_limits=(-0.001, 0.030), turn_damping=0.5,
                   slide_stiffness=K_L, slide_damping=D_L, slide_max_force=10.0, play=0.0015, tilt_deg=4.0)

# -- setting the trial's layout before the scene is built ---------------------------------------------------------------
_LAY = {"k": _K0, "others": _O0, "built": False}


def _seed():
    """The seed of the trial being reset (kept on the embodiment class, so every copy of this file sees it)."""
    mod = sys.modules.get("yam_arena.inspect_embodiment")
    return getattr(getattr(mod, "YamArenaEmbodiment", None), "_usb_seed", None)


def _apply_layout() -> None:
    """Put the camera's plug, the latches and the other cables where this trial's seed puts them. The scene is built
    at the first reset, after the seed is known; later resets keep that layout."""
    k, others = layout(_seed())
    if _LAY["built"]:
        if (k, others) != (_LAY["k"], _LAY["others"]):
            print(f"[usb] the scene was built for port {_LAY['k']}; seed {_seed()} would put the camera in port {k}",
                  file=sys.stderr, flush=True)
        return
    hub = _hub(k, others)
    object.__setattr__(OBJECTS[1], "shapes", hub.shapes)
    object.__setattr__(OBJECTS[1], "size", hub.size)
    object.__setattr__(OBJECTS[4], "pos", (HX, PORT_Y[k], SEAT))
    for t in bodykit._THREADS:  # every copy's latches, so whichever is authored last agrees
        if t.name in ("old_latch", "new_latch"):
            object.__setattr__(t, "anchor0", _port_rel(k))
    _LAY.update(k=k, others=others)


_arena_object_plain = bodykit._arena_object


def _arena_object(spec):
    if any(spec is o for o in OBJECTS):
        _apply_layout()
        _LAY["built"] = True
    return _arena_object_plain(spec)


bodykit._arena_object = _arena_object


def _patch_reset(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_usb_seed_caught", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        type(self)._usb_seed = kwargs.get("seed")
        return plain(self, *args, **kwargs)

    cls.reset, cls._usb_seed_caught = reset, True


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

# -- lights, and the latches' authoring (collisions kept on, the camera's plug held where it starts) ------------------
_PATHS: dict[str, str] = {}
_LIT: dict[str, bool] = {}


def _show(body: str, idx: int, on: bool, stage=None) -> None:
    from pxr import UsdGeom

    if body not in _PATHS:
        return
    if stage is None:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
    prim = stage.GetPrimAtPath(f"{_PATHS[body]}/body/s{idx}")
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


def _camera_on(on: bool, stage=None) -> None:
    """The camera's port light and the computer's CAMERA lamp."""
    if _LIT.get("on") == on and stage is None:
        return
    _show("hub", _HUB_IDX["led_k"], on, stage)
    _show("pc_status", _PC_IDX["green"], on, stage)
    _show("pc_status", _PC_IDX["red"], not on, stage)
    _LIT["on"] = on


_author_threads_plain = bodykit._author_threads


def _author_threads(stage, prim_path: str, spec) -> None:
    from pxr import UsdPhysics

    _author_threads_plain(stage, prim_path, spec)
    for name in ("old_latch", "new_latch"):
        prim = stage.GetPrimAtPath(f"{prim_path}/{name}")
        if prim and prim.IsValid():
            UsdPhysics.Joint(prim).GetCollisionEnabledAttr().Set(True)  # the plug still meets the port's walls
            if name == "old_latch":
                UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr().Set(0.0)  # seated from the start


bodykit._author_threads = _author_threads

_author_hinges_plain = bodykit._author_hinges


def _author_hinges(stage, prim_path: str, spec) -> None:
    _author_hinges_plain(stage, prim_path, spec)
    if spec.name in ("hub", "pc_status"):
        _PATHS[spec.name] = prim_path
        if "hub" in _PATHS and "pc_status" in _PATHS:
            _LIT.pop("on", None)
            _camera_on(True, stage)


bodykit._author_hinges = _author_hinges


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"old_on": True, "old_hold": 0.0, "unplugged": False, "new_on": False, "new_hold": 0.0, "new_x": 1.0,
            "wrong": False, "early": False, "old_in_bin": False, "tcps": None, "seen": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("USB_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _LIT.pop("on", None)
    try:
        set_slide_target("old_latch", 0.0)
        set_thread_on("old_latch", True)
        set_thread_on("new_latch", False)
        _camera_on(True)
    except Exception:  # before the scene exists
        pass
    if _DEBUG["on"]:
        print(f"[usb] reset seed={_seed()} camera port={_LAY['k']} others={_LAY['others']}", file=sys.stderr,
              flush=True)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _in_port(h_pos, h_rot, p_pos, p_rot, j: int) -> tuple[float, float, float]:
    """A plug against port j: its origin's height above the port's floor (x), how far off the port's axis, its tilt."""
    up = h_rot @ np.array([0.0, 0.0, 1.0])
    seat = h_pos + h_rot @ np.array(_port_rel(j))
    v = p_pos - seat
    x = float(v @ up)
    tilt = math.acos(max(-1.0, min(1.0, float((p_rot @ np.array([0.0, 0.0, 1.0])) @ up))))
    return x, float(np.linalg.norm(v - x * up)), tilt


def _in_bin(pos) -> bool:
    return (abs(pos[0] - BIN_C[0]) < BIN_IN[0] / 2 and abs(pos[1] - BIN_C[1]) < BIN_IN[1] / 2
            and pos[2] < TOP + BIN_WALL + 0.02)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    k = _LAY["k"]
    hp = as_numpy(base.scene["hub"].data.root_link_pose_w)[0]
    h_pos, h_rot = hp[:3] - origin, rotation(hp[3:7])
    op = as_numpy(base.scene["old_plug"].data.root_link_pose_w)[0]
    o_pos, o_rot = op[:3] - origin, rotation(op[3:7])
    npz = as_numpy(base.scene["new_plug"].data.root_link_pose_w)[0]
    n_pos, n_rot = npz[:3] - origin, rotation(npz[3:7])
    _EP["tcps"] = _tcps(base, origin)
    _EP["seen"] = True
    # the camera's old plug: held by its latch until pulled out
    if _EP["old_on"]:
        x, _, _ = _in_port(h_pos, h_rot, o_pos, o_rot, k)
        if x > X_OUT:
            set_thread_on("old_latch", False)
            _EP.update(old_on=False, unplugged=True)
        elif abs(x - _EP["old_hold"]) > SLIP_L:  # pulled (or pushed) past the latch's grip: it slides, and stays
            _EP["old_hold"] = x
            set_slide_target("old_latch", x)
    _EP["old_in_bin"] = _in_bin(o_pos)
    # the new plug: lifted out of its dock before the old one is in the bin -> fails
    if not _EP["old_in_bin"] and n_pos[2] > SEAT + 0.020 and not _EP["new_on"]:
        _EP["early"] = True
    x, off, tilt = _in_port(h_pos, h_rot, n_pos, n_rot, k)
    _EP["new_x"] = x
    if not _EP["new_on"]:
        if off <= CATCH_R and tilt <= CATCH_TILT and CATCH_X[0] <= x <= CATCH_X[1]:  # started into the camera's port
            set_slide_target("new_latch", x)
            set_thread_on("new_latch", True)
            _EP.update(new_on=True, new_hold=x)
    elif x > X_OUT:
        set_thread_on("new_latch", False)
        _EP["new_on"] = False
    elif abs(x - _EP["new_hold"]) > SLIP_L:
        _EP["new_hold"] = x
        set_slide_target("new_latch", x)
    # any plug sunk into another port fails the trial
    for name, pos, rot, on in (("old", o_pos, o_rot, _EP["old_on"]), ("new", n_pos, n_rot, _EP["new_on"])):
        if on:
            continue
        for j in range(N_PORTS):
            if j == k or j in _LAY["others"]:
                continue
            xj, offj, _ = _in_port(h_pos, h_rot, pos, rot, j)
            if offj <= 0.008 and xj <= WRONG_X:
                _EP["wrong"] = True
    _camera_on(bool(_EP["old_on"] or _seated()))
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, tr = _EP["tcps"]
            xo, _, _ = _in_port(h_pos, h_rot, o_pos, o_rot, k)
            print(f"[usb] t={_DEBUG['steps'] / 20:5.1f}s old_on={_EP['old_on']} xo={xo * 1000:6.1f} "
                  f"hold={_EP['old_hold'] * 1000:5.1f} out={_EP['unplugged']} bin={_EP['old_in_bin']} "
                  f"new_on={_EP['new_on']} xn={x * 1000:6.1f} off={off * 1000:5.1f} wrong={_EP['wrong']} "
                  f"early={_EP['early']} hub={np.round(h_pos * 1000).astype(int).tolist()} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"R-old={np.round((tr - o_pos) * 1000).astype(int).tolist()} "
                  f"R-new={np.round((tr - n_pos) * 1000).astype(int).tolist()}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _seated() -> bool:
    return bool(_EP["seen"] and _EP["new_on"] and _EP["new_x"] <= SEATED_X)


def _status(objects) -> dict:
    return {"out": _EP["unplugged"], "in_bin": _in_bin(np.asarray(objects["old_plug"].pos, float)),
            "caught": bool(_EP["new_on"]), "seated": _seated(), "bad": bool(_EP["wrong"] or _EP["early"]),
            "home": _arms_home()}


def _points(st) -> float:
    if st["bad"]:
        return 0.0
    if st["out"] and st["in_bin"] and st["seated"] and st["home"]:
        return 1.0
    if st["out"] and st["in_bin"] and st["caught"]:
        return 0.5
    return 0.25 if st["out"] else 0.0


def score(objects) -> float:
    """Judged at the end, except "pulled out", "another port" and "too early", which count at any time. Pulled out:
    the frayed cable's plug came right out of the camera's port at some point. Another port: either plug sunk 5 mm or
    more into any port but the camera's. Too early: the new plug lifted 2 cm out of its dock while the frayed plug was
    not in the bin. 0.0 if another port or too early. 1.0: pulled out, the frayed plug lies in the blue bin, the new
    plug is pushed all the way into the camera's port (within 3 mm of its floor, held by the latch), and both arms are
    back where they started (each gripper's fingertip centre within 5 cm of its start); 0.5: pulled out, the frayed
    plug is in the bin and the new plug has caught in the camera's port; 0.25: pulled out; else 0.0."""
    return _points(_status(objects))


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
    """Full score with the hub and both plugs still."""
    return (score(objects) >= 1.0 and objects["hub"].resting() and objects["new_plug"].resting()
            and objects["old_plug"].resting())


# -- scripted demo (for the demo seed: the camera in port 3) --------------------------------------------------------
# The left hand comes in from its side along the hub, tilted a little down, and closes its fingers on the front and
# back of the hub's left end, holding it down on the riser. The right hand comes straight down on the frayed cable's
# plug, fingers across its overmould, and pulls it straight up out of the port against its latch, carries it high
# over the bin and lets it drop in. It takes the new plug out of the dock the same way, carries it over the camera's
# port, lowers it in and pushes it home until the latch holds it. Both hands let go and go back to where they started.

SIDE_DOWN = math.radians(20.0)
L_GRIP = _rx(SIDE_DOWN) @ np.column_stack([[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]])  # pointing -y,
# tilted down; the fingers close along x on the hub's front and back faces, at its left end (next to port 0)
L_HOLD = np.array([HX, Y_LEFT - 0.025, HUB_BOT + HH / 2])
SIDE_IN = 0.07
R_PLUG = grip_rotation(math.pi / 2)  # the fingers close along x, across the overmould's thin way
PLUG_IN_HAND = 0.034  # the right's TCP this far above a plug's origin (13 mm below its top)
PRE_PLUG = -(OVER[0] / 2 + 0.007)
L_AIM = np.array([0.0, 0.0, -0.005])  # aim corrections (measured; the left bears down on the hub a little)
R_AIM = np.array([0.0, 0.0, 0.0])
PULL_UP = 0.050  # aimed this far above seated (the latch holds the arm back about 1-2 cm)
PUSH_PAST = 0.020  # pushed this far below seated (the arm stops short against the latch)
HIGH = TOP + 0.170
LEAD_S, LEAD_STEPS = 3.0, 15
_IK_CACHE: dict = {}


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    from yam_arena import ik, station

    key = (arm, tuple(np.round(np.asarray(pos, float), 5)), tuple(np.round(np.asarray(rot, float), 5).ravel()))
    if key in _IK_CACHE:
        return _IK_CACHE[key]
    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    _IK_CACHE[key] = best[0]
    return best[0]


def _sweep(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _plug_tcp(y: float, x: float = 0.0) -> np.ndarray:
    """The right's TCP gripping a plug whose origin is x above a port's floor (or the dock's), at y."""
    return np.array([HX, y, SEAT + x + PLUG_IN_HAND]) + R_AIM


def _right(t_left_holds: float) -> tuple[list[Waypoint], dict]:
    from yam_arena.constants import READY

    ev = {}
    k = layout(DEMO_SEED)[0]
    yk = PORT_Y[k]
    g = R_PLUG
    p0 = _plug_tcp(yk)
    out = _sweep("right", np.array(READY, float), _good_ik("right", p0 + [0.0, 0.0, 0.04], g),
                 "over the frayed cable's plug", PRE_PLUG)
    out += [_w(1.0, p0, PRE_PLUG, "down round its overmould", g),
            _w(0.6, p0, FINGER_CLOSED, "grip it", g)]
    if t_left_holds + 0.5 > _dur(out):
        out.append(_w(t_left_holds + 0.5 - _dur(out), p0, FINGER_CLOSED, "wait for the hub to be held", g))
    up = _plug_tcp(yk, PULL_UP)
    drop = np.array([BIN_C[0], BIN_C[1], HIGH])
    out += [_w(1.6, up, FINGER_CLOSED, "pull it straight up out of the port", g),
            _w(0.8, up + [-0.04, 0.0, 0.03], FINGER_CLOSED, "lift it clear, toward the front", g),
            _w(2.4, drop, FINGER_CLOSED, "carry it over the bin", g),
            _w(0.5, drop, FINGER_OPEN, "let it drop in", g),
            _w(0.6, drop + [0.0, 0.0, 0.01], FINGER_OPEN, "rise", g)]
    # the new plug, out of the dock and into the same port
    d0 = _plug_tcp(DOCK_Y)
    out += [_w(2.0, d0 + [0.0, 0.0, 0.04], PRE_PLUG, "over the new plug in its dock", g),
            _w(1.0, d0, PRE_PLUG, "down round its overmould", g),
            _w(0.6, d0, FINGER_CLOSED, "grip it", g),
            _w(1.4, _plug_tcp(DOCK_Y, PULL_UP), FINGER_CLOSED, "pull it up out of the dock", g),
            _w(0.8, _plug_tcp(DOCK_Y, PULL_UP + 0.02) + [-0.04, 0.0, 0.0], FINGER_CLOSED, "lift it clear, toward the front", g),
            _w(2.8, _plug_tcp(yk, PULL_UP + 0.02) + [-0.04, 0.0, 0.0], FINGER_CLOSED, "carry it along to the camera's port", g),
            _w(1.0, _plug_tcp(yk, PULL_UP + 0.02), FINGER_CLOSED, "over the port", g),
            _w(1.2, _plug_tcp(yk, 0.012), FINGER_CLOSED, "lower it into the port", g),
            _w(1.4, _plug_tcp(yk, -PUSH_PAST), FINGER_CLOSED, "push it all the way in", g),
            _w(0.5, _plug_tcp(yk, -PUSH_PAST), FINGER_CLOSED, "hold it there", g),
            _w(0.5, _plug_tcp(yk, 0.0), FINGER_OPEN, "let go", g),
            _w(0.8, _plug_tcp(yk, 0.04), FINGER_OPEN, "rise", g)]
    ev["done"] = _dur(out)
    out += _sweep("right", _good_ik("right", _plug_tcp(yk, 0.04), g), np.array(READY, float),
                  "back to where it started", FINGER_OPEN)
    return out, ev


def _left(done: float | None, total: float) -> tuple[list[Waypoint], dict]:
    from yam_arena.constants import READY

    times = {}
    hold = L_HOLD + L_AIM
    out_pt = hold - SIDE_IN * L_GRIP[:, 2]
    q_out = _good_ik("left", out_pt, L_GRIP)
    out = _sweep("left", np.array(READY, float), q_out, "beside the hub's left end", FINGER_OPEN)
    out += [_w(1.4, hold, FINGER_OPEN, "slide the fingers round the hub's end", L_GRIP),
            _w(0.6, hold, FINGER_CLOSED, "grip it", L_GRIP)]
    times["holds"] = _dur(out)
    if done is None:
        return out, times
    out.append(_w(max(done - _dur(out), 0.3), hold, FINGER_CLOSED, "hold the hub down", L_GRIP))
    out += [_w(0.5, hold, FINGER_OPEN, "let go", L_GRIP),
            _w(0.9, out_pt, FINGER_OPEN, "draw back", L_GRIP)]
    out += _sweep("left", q_out, np.array(READY, float), "back to where it started", FINGER_OPEN)
    if total > _dur(out):
        out.append(_w(total - _dur(out), out[-1].pos, FINGER_OPEN,
                      "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, times


def demo():
    _, t0 = _left(None, 0.0)
    right, ev = _right(t0["holds"])
    left, _ = _left(ev["done"], 0.0)
    total = max(_dur(right), _dur(left)) + 3.0
    right.append(_w(total - _dur(right), right[-1].pos, FINGER_OPEN,
                    "hold still: the trial ends once the full score has held 1 s", right[-1].yaw))
    left, _ = _left(ev["done"], total)
    return {"left": left, "right": right}
