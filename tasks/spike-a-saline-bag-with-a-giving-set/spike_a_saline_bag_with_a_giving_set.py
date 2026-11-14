"""On the station table lies a smooth stainless-steel tray, and on it a full 500 ml bag of 0.9 % saline, lying flat, its
two ports toward the right arm: the administration (spike) port, still closed by its blue tamper-evident protector,
and the medication port. Beside the bag lies a gravity infusion giving set: a spike under its white protective cap, a
flange, a clear (empty) drip chamber and a short length of its line with the roller clamp.

Task: spike the saline bag with the giving set, keeping the spike sterile, and leave the bag on the tray. The port's
protector has to be pulled off (its seal tears at about 10 N), the spike's cap pulled off (about 4 N), and the
uncapped spike pushed straight into the port, through the port's diaphragm (it gives at about 20 N) and on until the
flange meets the port's mouth. The full bag slides on the tray at about 2.5 N, so it has to be held while the
protector is pulled and the spike pushed; the giving set has to be held while its cap is pulled. The uncapped spike
must touch nothing but the port's opened end (the part its protector covered): any other touch (the tray, the bag,
the protector or the cap, a gripper) contaminates it, and the trial scores 0. Priming the drip chamber is not part of
the task: no liquid is simulated.

Run it:  inspect-robots run --embodiment yam_arena -E task=spike_a_saline_bag_with_a_giving_set.py \\
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step,  # noqa: E402
                     rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tray: smooth stainless steel ----------------------------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_T = (0.16, 0.50), (-0.30, 0.24), 0.004
TT = TOP + TRAY_T
TRAY_MU = (0.45, 0.40)  # soft PVC film on smooth stainless steel (static, sliding)

# -- the bag: a full 500 ml PVC bag of 0.9 % saline, lying flat, its ports toward -y -----------------------------------
# Bag frame: origin on the tray under the middle of the filled pillow; y along the bag (ports at -y), z up.
BAG_XY = (0.314, 0.095)
PW, PL, PT = 0.115, 0.185, 0.030  # the filled pillow: wide (x), long (y), thick; its long sides rounded (radius PT/2)
SEAL_SIDE, SEAL_TOP, SEAL_BOT, SEAL_T = 0.0075, 0.025, 0.015, 0.001  # welded seams round it, at its mid-plane
BAG_W, BAG_L = PW + 2 * SEAL_SIDE, PL + SEAL_TOP + SEAL_BOT  # 13.0 x 22.5 cm outline
PILLOW_ML = ((PW - PT) * PL * PT + math.pi * (PT / 2) ** 2 * PL) * 1e6  # 602 ml inside the pillow
FILL_ML, AIR_ML = 540.0, 30.0  # saline (500 ml nominal; makers overfill) and the air left in the bag
SALINE_G = FILL_ML * 1.0046  # 0.9 % NaCl, 1.0046 g/ml
FILM_G = 27.0  # the PVC bag, its ports and the protector
BAG_TOTAL = (SALINE_G + FILM_G) / 1000.0  # 0.570 kg
PORT_MASS, PROT_MASS = 0.002, 0.003
BAG_MASS = BAG_TOTAL - PORT_MASS - PROT_MASS
AXIS_Z = PT / 2  # the ports' axis: the bag's mid-plane
PORT_X, MED_X = -0.018, 0.018  # the administration (spike) port and the medication port, either side of the middle
PORT_R = 0.004  # the ports: 8 mm tubes
Y_SEAL = -PL / 2 - SEAL_BOT  # the bottom seam's edge (-0.1075)
PORT_BASE = 0.010  # the port's first 10 mm (part of the bag body); the next 26 mm, under the protector, is sterile
PORT_END = 0.026
Y_MOUTH = Y_SEAL - PORT_BASE - PORT_END  # -0.1435: the spike port's mouth
MOUTH = np.array([PORT_X, Y_MOUTH, AXIS_Z])  # in the bag's frame
PORT_AT_B = np.array([PORT_X, Y_SEAL - PORT_BASE - PORT_END / 2, AXIS_Z])  # the sterile end's middle, bag frame

# the protector: a blue sleeve over the port's sterile end (closed 2 mm past the mouth) with a flat pull tab
PROT_R, PROT_L = 0.0055, 0.025
PROT_AT_B = np.array([PORT_X, Y_MOUTH - 0.002 + PROT_L / 2, AXIS_Z])  # the sleeve's middle (it runs -0.1205..-0.1455)
K_SEAL, SEAL_GIVE = 2500.0, 0.004  # the tamper seal tears at 2500 N/m x 4 mm = 10 N of pull
SLIP_PROT = 0.0004  # then the sleeve slides off against about 1 N
PROT_OFF = 0.0225  # slid this far it has cleared the mouth: free


def _w2b(p) -> np.ndarray:
    return np.asarray(p, float) + np.array([BAG_XY[0], BAG_XY[1], TT])


# -- the giving set: lying on the tray, its spike toward +y ------------------------------------------------------------
# Set frame: origin on the axis at the flange's front face; the spike points +y.
SET_AT = np.array([0.41, -0.095, TT + 0.010])  # its axis 10 mm above the tray (the drip chamber's radius)
SPIKE_R, SPIKE_LEN = 0.0028, 0.024  # the spike: 5.6 mm across, 24 mm from the flange to its point (the usual size)
FL_R, FL_T = 0.008, 0.0025  # the flange (finger guard)
HUB_R, HUB_Y = 0.005, (-0.016, -FL_T)
CH_R, CH_Y = 0.010, (-0.076, -0.016)  # the drip chamber: 20 mm across, 60 mm long, clear
TUBE_R = 0.00205  # 4.1 mm line, cut to 10 cm
SET_MASS, SPIKE_MASS = 0.021, 0.004
CAP_R, CAP_L = 0.005, 0.030  # the spike's protective cap: 10 mm across, 30 mm long, seated on the flange
K_CAP, CAP_GIVE = 2000.0, 0.002  # the cap's push fit lets go at 2000 N/m x 2 mm = 4 N of pull
SLIP_CAP = 0.00025  # then it slides off against about 0.5 N
CAP_OFF = SPIKE_LEN + 0.005  # slid this far it has cleared the point: free
CAP_MASS = 0.002

# -- the spike in the port --------------------------------------------------------------------------------------------
SEAT = SPIKE_LEN  # the point this deep in the port: the flange on the mouth
DIAPH = 0.010  # the port's diaphragm, this far in from the mouth
K_SPIKE, D_SPIKE = 4000.0, 3.0
SLIP_SPIKE = 0.00075  # the port's bore grips the spike: about 3 N to slide it (4000 N/m x 0.75 mm)
PIERCE_GIVE = 0.005  # the diaphragm gives at 4000 N/m x 5 mm = 20 N
CAPTURE_R, CAPTURE_DEG, CAPTURE_IN = 0.0035, 12.0, 0.006  # a point this close to the axis, this square, enters

# -- scoring -----------------------------------------------------------------------------------------------------------
SEATED_WITHIN = 0.0015  # the flange within 1.5 mm of the mouth
BAG_Z_BAND = (-0.003, 0.010)  # the bag's underside this close to the tray's top
BAG_TILT = 15.0
TOUCH_N = 0.01  # a contact force above this is a touch
SUCCESS_HOLD_S = 1.0

STEEL, BAGC, SEAM, WHITE, BLUE, NAVY, CLEAR, DARK, TUBE, RUBBER, CAPC = (0.62, 0.64, 0.66), (0.80, 0.88, 0.95), \
    (0.72, 0.80, 0.88), (0.96, 0.96, 0.96), (0.10, 0.35, 0.85), (0.10, 0.15, 0.40), (0.85, 0.88, 0.92), \
    (0.12, 0.12, 0.13), (0.78, 0.86, 0.92), (0.55, 0.55, 0.52), (0.93, 0.93, 0.90)


def _tray() -> Body:
    return Body.build("tray", pos=((TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2, TOP), mass=2.0,
                      shapes=[BoxShape((0.0, 0.0, TRAY_T / 2), (TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], TRAY_T),
                                       STEEL)],
                      kinematic=True, friction=TRAY_MU, slick=True, color=STEEL)


def _bag() -> Body:
    """The full bag: a pillow with rounded long sides inside its welded seams, the hanger hole in the top seam, and
    the ports' bases at the bottom seam. Origin on the tray under the pillow's middle."""
    yc = (SEAL_TOP - SEAL_BOT) / 2
    shapes = [BoxShape((0.0, 0.0, PT / 2), (PW - PT, PL, PT), BAGC),  # the filled pillow
              CylShape(((PW - PT) / 2, 0.0, PT / 2), PT / 2, PL, BAGC, axis="Y"),  # its rounded sides
              CylShape((-(PW - PT) / 2, 0.0, PT / 2), PT / 2, PL, BAGC, axis="Y"),
              BoxShape((0.0, yc, AXIS_Z), (BAG_W, BAG_L, SEAL_T), SEAM),  # the welded seams
              CylShape((0.0, PL / 2 + SEAL_TOP * 0.55, AXIS_Z), 0.006, SEAL_T + 0.0006, DARK, collide=False),  # hanger
              BoxShape((0.0, -PL / 2 - SEAL_BOT / 2, AXIS_Z), (0.046, SEAL_BOT, 0.009), SEAM),  # the ports' base
              CylShape((PORT_X, Y_SEAL - PORT_BASE / 2 + 0.002, AXIS_Z), PORT_R, PORT_BASE + 0.004, WHITE, axis="Y"),
              CylShape((MED_X, Y_SEAL - 0.006, AXIS_Z), PORT_R, 0.016, WHITE, axis="Y"),  # medication port
              CylShape((MED_X, Y_SEAL - 0.017, AXIS_Z), 0.005, 0.006, RUBBER, axis="Y"),  # its rubber plug
              CylShape((MED_X, Y_SEAL - 0.0202, AXIS_Z), 0.0035, 0.0006, DARK, axis="Y", collide=False),
              # the printed label and its volume scale, on top
              BoxShape((0.0, 0.010, PT + 0.0002), (0.070, 0.105, 0.0004), WHITE, collide=False),
              BoxShape((0.0, 0.045, PT + 0.0004), (0.054, 0.012, 0.0004), BLUE, collide=False),
              BoxShape((0.0, 0.025, PT + 0.0004), (0.046, 0.004, 0.0004), NAVY, collide=False),
              BoxShape((0.0, 0.014, PT + 0.0004), (0.052, 0.003, 0.0004), NAVY, collide=False),
              BoxShape((0.0, 0.004, PT + 0.0004), (0.034, 0.003, 0.0004), NAVY, collide=False),
              BoxShape((0.0, -0.020, PT + 0.0004), (0.040, 0.003, 0.0004), NAVY, collide=False)]
    for k in range(5):  # the scale: a mark every 100 ml down one side
        shapes.append(BoxShape((0.044, 0.070 - k * 0.030, PT + 0.0002), (0.010 if k % 2 == 0 else 0.006, 0.0012,
                                                                         0.0004), NAVY, collide=False))
    return Body.build("bag", pos=(BAG_XY[0], BAG_XY[1], TT), mass=BAG_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=BAGC, com=(0.0, 0.0, PT / 2), solver_iterations=32)


def _port() -> Body:
    """The spike port's sterile end, under the protector: welded to the bag. The giving set (spike, flange) passes into
    it: its bore and diaphragm are the spike-in joint below. Origin in its middle, on its axis."""
    return Body.build("port", pos=tuple(_w2b(PORT_AT_B).tolist()), mass=PORT_MASS, friction=(1.0, 0.9), color=WHITE,
                      shapes=[CylShape((0.0, 0.0, 0.0), PORT_R, PORT_END, WHITE, axis="Y"),
                              CylShape((0.0, -PORT_END / 2 - 0.0002, 0.0), SPIKE_R, 0.0006, DARK, axis="Y",
                                       collide=False)])  # the bore


def _protector() -> Body:
    """The tamper-evident protector: a blue sleeve closed at its end, with a flat pull tab. Origin in the sleeve's
    middle, on its axis."""
    return Body.build("protector", pos=tuple(_w2b(PROT_AT_B).tolist()), mass=PROT_MASS, friction=(1.0, 0.9),
                      color=BLUE, ignore=("port",), shapes=[
                          CylShape((0.0, 0.0, 0.0), PROT_R, PROT_L, BLUE, axis="Y"),
                          BoxShape((0.0, -PROT_L / 2 - 0.006, 0.0), (0.016, 0.012, 0.002), BLUE),  # the pull tab
                          CylShape((0.0, PROT_L / 2 - 0.004, 0.0), PROT_R + 0.0004, 0.0012, NAVY, axis="Y",
                                   collide=False)])  # its seal line


def _giving_set() -> Body:
    """The giving set without its spike: the flange, the hub with its air vent, the clear drip chamber (empty), the
    outlet and a 10 cm stub of the line with its roller clamp. Origin on the axis at the flange's front face."""
    shapes = [CylShape((0.0, -FL_T / 2, 0.0), FL_R, FL_T, WHITE, axis="Y"),  # the flange
              CylShape((0.0, (HUB_Y[0] + HUB_Y[1]) / 2, 0.0), HUB_R, HUB_Y[1] - HUB_Y[0], WHITE, axis="Y"),
              CylShape((HUB_R + 0.002, -0.009, 0.0), 0.0025, 0.004, WHITE, axis="X", collide=False),  # air vent
              CylShape((HUB_R + 0.0042, -0.009, 0.0), 0.0028, 0.0008, BLUE, axis="X", collide=False),
              CylShape((0.0, (CH_Y[0] + CH_Y[1]) / 2, 0.0), CH_R, CH_Y[1] - CH_Y[0], None, axis="Y"),  # chamber
              CylShape((0.0, CH_Y[1] - 0.0015, 0.0), CH_R + 0.0003, 0.003, CLEAR, axis="Y", collide=False),
              CylShape((0.0, CH_Y[0] + 0.0015, 0.0), CH_R + 0.0003, 0.003, CLEAR, axis="Y", collide=False),
              CylShape((0.0, CH_Y[1] - 0.008, 0.0), 0.0015, 0.010, WHITE, axis="Y", collide=False),  # drop former
              CylShape((0.0, CH_Y[0] - 0.004, 0.0), 0.003, 0.008, WHITE, axis="Y"),  # the outlet
              CylShape((0.0, CH_Y[0] - 0.058, 0.0), TUBE_R, 0.100, TUBE, axis="Y"),  # the line
              BoxShape((0.0, CH_Y[0] - 0.060, -0.003), (0.016, 0.045, 0.014), BLUE),  # the roller clamp
              CylShape((0.0, CH_Y[0] - 0.060, 0.0045), 0.006, 0.010, WHITE, axis="X", collide=False)]  # its roller
    for x, z in ((CH_R, 0.0), (-CH_R, 0.0), (0.0, CH_R), (0.0, -CH_R)):  # faint seams down the clear chamber
        shapes.append(BoxShape((x * 0.98, (CH_Y[0] + CH_Y[1]) / 2, z * 0.98),
                               (0.0008 if x else 0.0012, CH_Y[1] - CH_Y[0] - 0.004, 0.0012 if x else 0.0008),
                               CLEAR, collide=False))
    return Body.build("giving_set", pos=tuple(SET_AT.tolist()), mass=SET_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=WHITE, com=(0.0, -0.055, -0.0005), ignore=("port",), solver_iterations=32)


def _spike() -> Body:
    """The sterile spike: its own body, welded to the set, so that what it touches can be told apart."""
    return Body.build("spike", pos=tuple(SET_AT.tolist()), mass=SPIKE_MASS, friction=(0.5, 0.4), color=WHITE,
                      ignore=("port",), solver_iterations=32, contact_offset=0.001, shapes=[
                          CylShape((0.0, (SPIKE_LEN - 0.005) / 2, 0.0), SPIKE_R, SPIKE_LEN - 0.005, WHITE, axis="Y"),
                          CylShape((0.0, SPIKE_LEN - 0.0035, 0.0), 0.0017, 0.003, WHITE, axis="Y"),  # its point
                          CylShape((0.0, SPIKE_LEN - 0.001, 0.0), 0.0008, 0.002, WHITE, axis="Y")])


def _spike_cap() -> Body:
    """The spike's protective cap, pushed onto the flange, with two fins. Origin on the axis at its open end."""
    return Body.build("spike_cap", pos=tuple((SET_AT + [0.0, 0.0002, 0.0]).tolist()), mass=CAP_MASS,
                      friction=(1.0, 0.9), color=CAPC, ignore=("giving_set",), solver_iterations=32, shapes=[
                          CylShape((0.0, CAP_L / 2, 0.0), CAP_R, CAP_L, CAPC, axis="Y"),
                          BoxShape((0.0, 0.006, 0.0), (0.016, 0.008, 0.0015), CAPC)])  # fins


OBJECTS = [_tray(), _bag(), _port(), _protector(), _giving_set(), _spike(), _spike_cap()]
glue("port_weld", "bag", "port", anchor=tuple(_w2b(PORT_AT_B).tolist()))
glue("spike_on_set", "giving_set", "spike", anchor=tuple(SET_AT.tolist()))
# the protector's tamper seal and sleeve: a slide along the port (0 = on; pulled off toward -y it goes negative)
thread("seal", "bag", "protector", anchor0=tuple(PROT_AT_B.tolist()), axis0=(0.0, 1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, 1.0, 0.0), slide_limits=(-0.030, 0.0), turn_damping=0.05, slide_stiffness=K_SEAL,
       slide_damping=2.0, slide_max_force=40.0, always_on=True)
# the spike's cap on the spike: a slide along it (0 = on; pulled off toward +y it goes negative)
thread("cap_fit", "spike", "spike_cap", anchor0=(0.0, 0.0002, 0.0), axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
       axis1=(0.0, -1.0, 0.0), slide_limits=(-0.040, 0.0), turn_damping=0.01, slide_stiffness=K_CAP,
       slide_damping=1.0, slide_max_force=20.0, always_on=True)
# the spike in the port: switched on when an uncapped point enters the mouth square on; the slide is its depth
thread("spike_in", "bag", "giving_set", anchor0=tuple(MOUTH.tolist()), axis0=(0.0, 1.0, 0.0),
       anchor1=(0.0, SPIKE_LEN, 0.0), axis1=(0.0, 1.0, 0.0), slide_limits=(-0.05, SEAT), turn_damping=0.002,
       slide_stiffness=K_SPIKE, slide_damping=D_SPIKE, slide_max_force=60.0, play=0.0008, tilt_deg=3.0)

# -- contact reports: the spike against everything it may touch; the bag and the set against the grippers --------------
_J: dict = {}
_author_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    from pxr import PhysxSchema

    if spec.name in ("spike", "bag", "port", "giving_set"):
        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body"))
        api.CreateThresholdAttr(0.0)
        _J[spec.name] = f"{prim_path}/body"
        _J["env"] = prim_path.rsplit("/", 1)[0]


bodykit._author_sliders = _author_sliders


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


_CT = {"views": None, "failed": "", "names": [], "spike_hits": [], "physics_hook": False, "armed": False,
       "samples": 0}


def _views():
    """Two PhysX rigid-contact views, made once: (1) the spike against the bag, the tray, the protector, its own cap
    and every link of both arms; (2) the bag, the port and the giving set against every arm link."""
    if _CT["views"] is None and not _CT["failed"] and "env" in _J:
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            env = _J["env"]
            links = [f"{env}/{arm}{p}" for arm in ("LeftArm", "RightArm") for p in asset_link_paths().values()]
            others = [f"{env}/{o}/body" for o in ("bag", "tray", "protector", "spike_cap")]
            sim = PhysxManager.get_physics_sim_view()
            spike = sim.create_rigid_contact_view([_J["spike"]], filter_patterns=[others + links],
                                                  max_contact_data_count=0)
            held = sim.create_rigid_contact_view([_J["bag"], _J["port"], _J["giving_set"]],
                                                 filter_patterns=[links] * 3, max_contact_data_count=0)
            _CT["names"] = ["bag", "tray", "protector", "spike cap"] + [
                ("left " if "LeftArm" in p else "right ") + p.rsplit("/", 1)[-1] for p in links]
            _CT["views"] = (spike, held, len(others) + len(links), len(links))
        except Exception as e:  # noqa: BLE001
            _CT["failed"] = repr(e)[:300]
    return _CT["views"]


N_WORLD = 3  # the first three filters (the bag, the tray, the protector) count from the moment the cap starts to come
# off; the cap itself and the grippers only once it is fully off (until then the cap is between them and the spike: a
# hard squeeze on the 10 mm cap can press a finger through its 2.2 mm wall in PhysX, which no real squeeze does)


def _sample_spike(_dt=None) -> None:
    """After every physics step (240 Hz): once the cap has started to come off, a contact force on the spike from the
    bag, the tray or the protector is a touch; once the cap is off, so is one from the cap or either gripper."""
    if not _CT["armed"]:
        return
    v = _views()
    if v is None:
        return
    try:
        m = _np(v[0].get_contact_force_matrix(dt=1.0 / 240.0)).reshape(v[2], 3)
    except Exception as e:  # noqa: BLE001
        _CT["failed"] = repr(e)[:300]
        return
    _CT["samples"] += 1
    f = np.linalg.norm(m, axis=1)
    if not _EP["cap_free"]:
        f[N_WORLD:] = 0.0
    for j in np.nonzero(f > TOUCH_N)[0]:
        if len(_CT["spike_hits"]) < 20:
            _CT["spike_hits"].append(f"{_CT['names'][j]} ({f[j]:.2f} N)")
        _EP["contaminated"] = True


def _hook_physics() -> None:
    """Run _sample_spike after every physics step, not only every control step (a 50 ms graze would be missed)."""
    if _CT["physics_hook"]:
        return
    _CT["physics_hook"] = True
    try:
        from isaaclab_physx.physics import PhysxManager

        step = PhysxManager.step

        def stepped(*args, **kwargs):
            out = step(*args, **kwargs)
            _sample_spike()
            return out

        PhysxManager.step = stepped
    except Exception as e:  # noqa: BLE001
        _CT["failed"] = "hook: " + repr(e)[:300]


def _held() -> list[bool]:
    """Whether any gripper link touches the bag (or its port) or the giving set, now."""
    v = _views()
    if v is None:
        return [True, True]
    m = _np(v[1].get_contact_force_matrix(dt=1.0 / 240.0)).reshape(3, v[3], 3)
    f = np.linalg.norm(m, axis=2).max(axis=1)
    return [bool(f[0] > TOUCH_N or f[1] > TOUCH_N), bool(f[2] > TOUCH_N)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": False, "hold": 0.0, "pierced": False, "torn": False, "prot_hold": 0.0, "prot_free": False,
            "cap_loose": False, "cap_hold": 0.0, "cap_free": False, "contaminated": False, "s": {}, "tcps": None,
            "steps": 0, "arm_speed": 1.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("IVBAG_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _CT["spike_hits"], _CT["armed"], _CT["samples"] = [], False, 0
    for name in ("seal", "cap_fit"):  # the protector and the cap back on, held
        try:
            set_thread_on(name, True)
            set_slide_target(name, 0.0)
        except Exception:  # noqa: BLE001
            pass


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _hands(base, origin) -> tuple[list[np.ndarray], float]:
    """Each gripper's TCP (left, right), and the fastest joint speed of either arm (rad/s)."""
    tcps, speed = [], 0.0
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        names = list(robot.body_names)
        i = next(k for k, n in enumerate(names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
        speed = max(speed, float(np.abs(as_numpy(robot.data.joint_vel)[0]).max()))
    return tcps, speed


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _hook_physics()
    _EP["steps"] += 1
    b_pos, b_rot = _pose(base, origin, "bag")
    g_pos, g_rot = _pose(base, origin, "giving_set")
    p_pos, _p_rot = _pose(base, origin, "protector")
    k_pos, _k_rot = _pose(base, origin, "spike_cap")
    s_pos, s_rot = _pose(base, origin, "spike")
    # the protector: its slide along the port (negative = pulled off)
    prot = float(b_rot[:, 1] @ (p_pos - (b_pos + b_rot @ PROT_AT_B)))
    if not _EP["torn"]:
        if prot < -SEAL_GIVE:  # the tamper seal tears
            _EP["torn"], _EP["prot_hold"] = True, prot
            set_slide_target("seal", prot)
    elif not _EP["prot_free"]:
        if prot < -PROT_OFF:  # off the port: free
            _EP["prot_free"] = True
            set_thread_on("seal", False)
        elif abs(prot - _EP["prot_hold"]) > SLIP_PROT:
            _EP["prot_hold"] = prot
            set_slide_target("seal", prot)
    # the spike's cap: its slide along the spike (negative = pulled off)
    cap = -float(s_rot[:, 1] @ (k_pos - s_pos))
    if not _EP["cap_loose"]:
        if cap < -CAP_GIVE:
            _EP["cap_loose"], _EP["cap_hold"] = True, cap
            set_slide_target("cap_fit", cap)
    elif not _EP["cap_free"]:
        if cap < -CAP_OFF:
            _EP["cap_free"] = True
            set_thread_on("cap_fit", False)
        elif abs(cap - _EP["cap_hold"]) > SLIP_CAP:
            _EP["cap_hold"] = cap
            set_slide_target("cap_fit", cap)
    if _EP["cap_loose"]:
        _CT["armed"] = True  # the spike is bare from here on
    # the spike's point against the port's mouth (bag frame)
    tip = g_pos + g_rot @ np.array([0.0, SPIKE_LEN, 0.0])
    q = b_rot.T @ (tip - b_pos) - MOUTH
    off, depth = math.hypot(q[0], q[2]), float(q[1])
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(b_rot[:, 1] @ g_rot[:, 1])))))
    if not _EP["on"]:
        if _EP["cap_free"] and off <= CAPTURE_R and 0.0 <= depth <= CAPTURE_IN and angle <= CAPTURE_DEG:
            _EP["on"], _EP["hold"] = True, depth
            set_slide_target("spike_in", depth)
            set_thread_on("spike_in", True)
    else:
        if depth < -0.003:  # drawn back out of the port
            _EP["on"] = False
            set_thread_on("spike_in", False)
        elif not _EP["pierced"] and depth >= DIAPH - SLIP_SPIKE:  # against the diaphragm: it bulges, then gives
            if depth - DIAPH > PIERCE_GIVE:
                _EP["pierced"], _EP["hold"] = True, depth
                set_slide_target("spike_in", depth)
            elif _EP["hold"] != DIAPH:
                _EP["hold"] = DIAPH
                set_slide_target("spike_in", DIAPH)
        elif abs(depth - _EP["hold"]) > SLIP_SPIKE:  # pushed (or pulled) past the bore's grip: it slides, and stays
            _EP["hold"] = depth if _EP["pierced"] else min(depth, DIAPH)
            set_slide_target("spike_in", _EP["hold"])
    # a bare spike on the table (outside the tray): a touch too
    if _CT["armed"]:
        for pt in (s_pos + s_rot @ np.array([0.0, y, 0.0]) for y in (0.0, SPIKE_LEN)):
            outside = not (TRAY_X[0] <= pt[0] <= TRAY_X[1] and TRAY_Y[0] <= pt[1] <= TRAY_Y[1])
            if outside and pt[2] < TOP + SPIKE_R + 0.001:
                _EP["contaminated"] = True
                if len(_CT["spike_hits"]) < 20:
                    _CT["spike_hits"].append("table")
    if not _CT["physics_hook"] or _CT["failed"]:
        _sample_spike()
    held = _held()
    bag_z = float(b_pos[2] - TT)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(b_rot[2, 2])))))
    on_tray = (TRAY_X[0] <= b_pos[0] <= TRAY_X[1] and TRAY_Y[0] <= b_pos[1] <= TRAY_Y[1]
               and BAG_Z_BAND[0] <= bag_z <= BAG_Z_BAND[1] and tilt <= BAG_TILT)
    _EP["s"] = {"depth": depth, "on_tray": on_tray, "held_bag": held[0], "held_set": held[1]}
    _EP["tcps"], _EP["arm_speed"] = _hands(base, origin)
    if _DEBUG["on"] and _EP["steps"] % 10 == 0:
        t = _EP["tcps"]
        f_seal = K_SEAL * (_EP["prot_hold"] - prot) if _EP["torn"] else -K_SEAL * prot
        f_cap = K_CAP * (_EP["cap_hold"] - cap) if _EP["cap_loose"] else -K_CAP * cap
        f_spk = K_SPIKE * (depth - _EP["hold"]) if _EP["on"] else 0.0
        print(f"[ivbag] t={_EP['steps'] / 20:5.1f}s F seal/cap/spike={f_seal:5.1f}/{f_cap:5.1f}/{f_spk:5.1f}N "
              f"setpitch={math.degrees(math.asin(max(-1.0, min(1.0, float(g_rot[2, 1]))))):5.1f} "
              f"setyaw={math.degrees(math.atan2(-g_rot[0, 1], g_rot[1, 1])):5.1f} "
              f"bagyaw={math.degrees(math.atan2(-b_rot[0, 1], b_rot[1, 1])):5.1f} "
              f"prot={prot * 1000:5.1f}mm torn={_EP['torn']} "
              f"free={_EP['prot_free']} cap={cap * 1000:5.1f}mm capfree={_EP['cap_free']} "
              f"depth={depth * 1000:6.1f}mm off={off * 1000:4.1f}mm ang={angle:4.1f} "
              f"q={np.round(q * 1000, 1).tolist()} "
              f"on={_EP['on']} pierced={_EP['pierced']} contam={_EP['contaminated']} hits={_CT['spike_hits'][:3]} "
              f"held={held} tray={on_tray} bag={np.round(b_pos * 1000).astype(int).tolist()} bagz={bag_z * 1000:.1f} "
              f"set={np.round(g_pos * 1000).astype(int).tolist()} "
              f"L-bag={np.round((t[0] - b_pos) * 1000).astype(int).tolist()} "
              f"R-set={np.round((t[1] - g_pos) * 1000).astype(int).tolist()} "
              f"R-prot={np.round((t[1] - p_pos) * 1000).astype(int).tolist()} "
              f"L-cap={np.round((t[0] - k_pos) * 1000).astype(int).tolist()} "
              f"armv={_EP['arm_speed']:.3f} samples={_CT['samples']} fail={_CT['failed'][:120]} "
              f"score={_points(_status())}",
              file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    depth = float(st.get("depth", -1.0))
    return {"contaminated": _EP["contaminated"], "torn": _EP["torn"],
            "pierced": _EP["on"] and _EP["pierced"] and depth > DIAPH + 0.001,
            "seated": _EP["on"] and depth >= SEAT - SEATED_WITHIN,
            "left": bool(st.get("on_tray")) and not st.get("held_bag", True) and not st.get("held_set", True)}


def _points(s) -> float:
    if s["contaminated"]:
        return 0.0
    if s["seated"] and s["left"]:
        return 1.0
    if s["seated"]:
        return 0.75
    if s["pierced"]:
        return 0.5
    return 0.25 if s["torn"] else 0.0


def score(objects) -> float:
    """0.0 if the spike, once its cap has started to come off, ever touched anything but the port's opened end (the
    26 mm its protector covered; latched). Otherwise, at the end: 1.0 with the spike in the port up to its flange (the
    flange within 1.5 mm of the port's mouth), the bag lying on the tray, and neither gripper touching the bag or the
    giving set; 0.75 with the spike in up to its flange; 0.5 with the spike through the port's diaphragm (more than
    11 mm in); 0.25 with the port's protector torn off; else 0.0."""
    return _points(_status())


ARMS_STILL = 0.05  # rad/s: both arms at rest


def success(objects) -> bool:
    """Full score (which has both grippers off the bag and the set) with the bag, the giving set, the protector, the
    spike's cap and both arms still."""
    return (score(objects) >= 1.0 and _EP.get("arm_speed", 1.0) < ARMS_STILL
            and all(objects[n].resting() for n in ("bag", "giving_set", "protector", "spike_cap")))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the bag and presses it flat on the tray, in line with the spike port. The right takes the
# protector by its sleeve, pulls it straight off the port (tearing its seal) and drops it at the front of the tray. It
# takes the giving set by its drip chamber and holds it up level, the spike toward the left. The left lets go of the
# bag, takes the spike's cap by its end, pulls it straight off and drops it on the tray, and presses the bag down
# again. The right lines the bare spike up with the port and pushes it straight in, through the diaphragm and on until
# the flange meets the port's mouth. It lets go and rises; the left lets go of the bag, and both arms go back to where
# they started.

PRESS = grip_rotation(0.0)  # the left, straight down, fingers closed (along y)
GX = grip_rotation(math.pi / 2)  # the right, straight down, fingers closing along x
GXL = grip_rotation(-math.pi / 2)  # the left, straight down, fingers closing along x
SIDE_L = np.column_stack([(-1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, -1.0, 0.0)])  # the left, pointing -y, level,
# fingers closing along x: it takes the spike's cap end on and pulls it straight back along its own axis
PRESS_AT = np.array([BAG_XY[0] + PORT_X, BAG_XY[1] + 0.035, TT + PT])  # on the bag's top, in line with the port
PRESS_DOWN = 0.060  # aim this far below the touch: about 25 N
TIP_PAST = 0.0128  # the closed fingertips reach this far past the TCP
PROT_UP = 0.003  # the right TCP this far above the protector's axis
PROT_OPEN = -0.012
PROT_PULL = 0.060  # pull this far (the seal gives 4 mm, the sleeve slides 22 mm, the arm lags)
DROP_PROT = np.array([0.22, -0.22, TT + 0.05])
CH_UP = 0.004  # the right TCP this far above the chamber's axis
CH_OPEN = -0.016
GRIP_Y = -0.052  # the right takes the chamber this far behind the flange (set frame)
CAP_TAKE = 0.028  # the left's TCP this far ahead of the flange: its fingers over the cap's last 13 mm
CAP_AT_L = np.array([0.31, -0.04, TT + 0.12])  # the left's TCP on the cap, the cap's axis there
UNCAP_AT = CAP_AT_L + [0.0, GRIP_Y - CAP_TAKE, CH_UP]  # the right's TCP while the cap comes off
CAP_OPEN = -0.012
CAP_SQUEEZE = -0.0035  # a gentle grip on the 10 mm cap (a full one presses through its wall)
CAP_PULL = 0.055
DROP_CAP = np.array([0.21, 0.03, TT + 0.05])
LINE_BACK = 0.012  # the spike's point lined up this far short of the mouth
PUSH_PAST = 0.130  # aim the push this far past the seat: the arm is soft (about 190 N/m along y here)
EASE = 0.025  # then ease off to this far past it (the held bag gives up to 15 mm), still pushing lightly
SET_OFF = np.array([0.0, 0.0, 0.0])  # where the point ends up off the mouth if aimed straight at it (measured)
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
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


def _joint_path(arm: str, q0, q1, label: str, finger, total: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(total / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _move(arm: str, p0, r0, p1, r1, finger, label: str, total: float) -> list[Waypoint]:
    """A move that also turns the grip a lot: interpolated in joint space between the two IK solutions."""
    return _joint_path(arm, _good_ik(arm, p0, r0), _good_ik(arm, p1, r1), label, finger, total)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _right_part1() -> tuple[list[Waypoint], dict]:
    """Up to holding the giving set at the uncapping pose. Returns the waypoints and the times of its events."""
    prot = _w2b(PROT_AT_B) + [0.0, -0.003, PROT_UP]  # take the sleeve 3 mm out from its middle
    out = _lead_in("right", prot + [0.0, 0.0, 0.05], GX, "over the protector", PROT_OPEN)
    ev = {"over_prot": _dur(out)}
    out += [_w(0.4, prot + [0.0, 0.0, 0.05], PROT_OPEN, "wait for the bag to be held", GX)]
    ev["wait_idx"] = len(out) - 1
    out += [_w(0.9, prot, PROT_OPEN, "down round the protector's sleeve", GX),
            _w(0.5, prot, FINGER_CLOSED, "take it", GX),
            _w(2.0, prot + [0.0, -PROT_PULL, 0.0], FINGER_CLOSED, "pull it straight off the port: the seal tears", GX),
            _w(0.8, prot + [0.0, -PROT_PULL, 0.04], FINGER_CLOSED, "lift it clear", GX),
            _w(1.4, DROP_PROT, FINGER_CLOSED, "carry it to the front of the tray", GX),
            _w(0.4, DROP_PROT, PROT_OPEN, "drop it", GX)]
    grip = SET_AT + [0.0, GRIP_Y, CH_UP]
    out += [_w(1.4, grip + [0.0, 0.0, 0.05], CH_OPEN, "over the drip chamber", GX),
            _w(0.9, grip, CH_OPEN, "down round it", GX),
            _w(0.5, grip, FINGER_CLOSED, "take the giving set", GX),
            _w(1.0, grip + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift it", GX),
            _w(1.6, UNCAP_AT, FINGER_CLOSED, "hold it up level, the spike toward the left", GX)]
    ev["at_uncap"] = _dur(out)
    return out, ev


def demo():
    from yam_arena.constants import READY  # noqa: F401

    # -- the left: press the bag; take the spike's cap off; press again; let go and go home
    press = PRESS_AT + [0.0, 0.0, TIP_PAST - PRESS_DOWN]
    above = PRESS_AT + [0.0, 0.0, TIP_PAST + 0.04]
    left = _lead_in("left", above, PRESS, "over the bag, in line with its spike port", FINGER_CLOSED)
    left += [_w(1.0, PRESS_AT + [0.0, 0.0, TIP_PAST + 0.003], FINGER_CLOSED, "down onto the bag", PRESS),
             _w(0.6, press, FINGER_CLOSED, "press it flat on the tray", PRESS)]
    pressed_at = _dur(left)
    right, ev = _right_part1()
    # the right waits over the protector until the bag is held
    wait = right[ev["wait_idx"]]
    right[ev["wait_idx"]] = _w(max(pressed_at - ev["over_prot"] + 0.2, 0.4), wait.pos, wait.finger, wait.label,
                               wait.yaw)
    t_uncap = _dur(right)
    # the left holds the bag until the protector is off, then goes for the cap once the right holds the set up
    take = CAP_AT_L
    behind = take + [0.0, 0.05, 0.0]
    off = take + [0.0, CAP_PULL, 0.0]
    high = PRESS_AT + [0.0, 0.0, TIP_PAST + 0.10]
    left += [_w(max(t_uncap - _dur(left) - 3.6, 0.3), press, FINGER_CLOSED, "hold the bag still", PRESS),
             _w(0.4, above, FINGER_CLOSED, "let go of the bag", PRESS),
             _w(0.6, high, FINGER_CLOSED, "rise", PRESS),
             *_move("left", high, PRESS, behind, SIDE_L, CAP_OPEN, "level with the spike's cap, behind it", 2.2),
             _w(1.0, take, CAP_OPEN, "fingers round the cap's end", SIDE_L),
             _w(0.5, take, CAP_SQUEEZE, "take it, gently", SIDE_L),
             _w(1.5, off, CAP_SQUEEZE, "pull it straight off the spike", SIDE_L),
             *_move("left", off, SIDE_L, DROP_CAP, GXL, CAP_SQUEEZE, "carry it aside", 2.0),
             _w(0.4, DROP_CAP, CAP_OPEN, "drop it on the tray", GXL),
             *_move("left", DROP_CAP, GXL, high, PRESS, FINGER_CLOSED, "back over the bag", 1.8),
             _w(0.6, above, FINGER_CLOSED, "down toward it", PRESS),
             _w(0.8, PRESS_AT + [0.0, 0.0, TIP_PAST + 0.003], FINGER_CLOSED, "down onto the bag", PRESS),
             _w(0.6, press, FINGER_CLOSED, "press it flat on the tray again", PRESS)]
    pressed_again = _dur(left)
    # the right: once the cap is off, line the spike up and push it in
    mouth = _w2b(MOUTH)
    line = mouth + [0.0, -LINE_BACK - SPIKE_LEN + GRIP_Y, CH_UP] - SET_OFF  # the TCP with the point 12 mm short
    seated = line + [0.0, LINE_BACK + SEAT + PUSH_PAST, 0.0]
    eased = line + [0.0, LINE_BACK + SEAT + EASE, 0.0]
    right += [_w(max(pressed_again - t_uncap - 1.2, 0.3), UNCAP_AT, FINGER_CLOSED, "hold the set while the cap comes "
                 "off", GX),
              _w(1.6, line + [0.0, -0.03, 0.03], FINGER_CLOSED, "bring the bare spike over to the port", GX),
              _w(1.0, line, FINGER_CLOSED, "the spike in line with the port, its point at the mouth", GX),
              _w(3.5, seated, FINGER_CLOSED, "push it straight in, through the diaphragm, up to the flange", GX),
              _w(0.8, seated, FINGER_CLOSED, "hold", GX),
              _w(0.8, eased, FINGER_CLOSED, "ease off", GX),
              _w(0.5, eased, CH_OPEN, "let go", GX)]
    up = eased + [0.0, -0.01, 0.05]
    right.append(_w(0.8, up, CH_OPEN, "rise", GX))
    done = _dur(right)
    right += _go_home("right", up, GX, CH_OPEN)
    # the left lets go once the right has let go
    lift = above + [0.0, 0.0, 0.03]
    left += [_w(max(done - _dur(left), 0.3), press, FINGER_CLOSED, "hold the bag still", PRESS),
             _w(0.6, above, FINGER_CLOSED, "let go of the bag", PRESS),
             _w(0.6, lift, FINGER_CLOSED, "rise", PRESS)]
    left += _go_home("left", lift, PRESS, FINGER_CLOSED)
    total = max(_dur(left), _dur(right)) + 3.0
    left.append(_w(max(total - _dur(left), 0.5), left[-1].pos, FINGER_CLOSED,
                   "hold still: the trial ends once the full score has held 1 s", left[-1].yaw))
    right.append(_w(max(total - _dur(right), 0.5), right[-1].pos, CH_OPEN, "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
