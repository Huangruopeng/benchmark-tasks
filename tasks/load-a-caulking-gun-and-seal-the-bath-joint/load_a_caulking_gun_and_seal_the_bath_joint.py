"""On the station table lies an empty caulking gun, the kind used to seal baths and sinks: a red steel frame with a
pistol grip, an open half-round cradle for a sealant cartridge and a front yoke with a notch for the cartridge's
nozzle. It lies level along the table, its nozzle end toward the left, on its grip and on a wooden rest block under its
front. Its steel plunger rod comes out through the back of the frame and ends in a hook; the rod is part way in. On top
of the frame, behind the cradle, is the gun's thumb release lever. To the left of the gun, along the table's left
edge, is the white rim of a bath against a tiled wall; the joint where the rim meets the tiles is not sealed yet. In
front of the gun, on a cardboard tray, lies a new white cartridge of silicone sealant, its nozzle already cut.

Task: load the gun and seal the bath joint. With one hand press the release lever down and keep it pressed, which
also holds the gun still, while the other hand pulls the plunger rod back by its hook, all the way out. Let go of the
lever, lay the cartridge in the cradle with its nozzle through the yoke, and push the rod back in until its plate
meets the cartridge. Then lift the gun with both hands, put the nozzle's tip into the joint between the bath rim and
the tiles and run it along the joint from one end to the other, laying a bead of sealant. Put the gun back on its grip
and rest block, and take both arms back to where they started. The rod is held by a ratchet: it slides back only while
the release lever is held down, so one hand alone pulls the whole gun along the table instead of the rod out of it.

Run it:  inspect-robots run --embodiment yam_arena -E task=load_a_caulking_gun_and_seal_the_bath_joint.py \\
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
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, plate, rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the gun. Its frame: origin on the cartridge axis at the back plate's inner face, +y toward the nozzle end ------
GX, GY = 0.300, 0.020
AXIS_Z = TOP + 0.090  # the axis this high with the grip's foot on the table
GUN0 = np.array([GX, GY, AXIS_Z])
R_TROUGH = 0.028  # the cradle's strips sit on this radius (their inner faces 1.5 mm inside it)
YOKE_Y0, YOKE_T = 0.236, 0.004  # the front yoke's inner face and thickness
GRIP_FOOT = -0.090
GUN_MASS = 0.40
# -- the plunger rod: its origin at the push plate's front face, which sits S (m) in front of the back plate ---------
ROD_BACK = 0.276  # from the plate's face back to the rod's end, where the hook starts
S0 = 0.080  # where the plate starts (the rod part way in)
S_MAX = 0.226
HOOK_GRIP = (0.0, -0.294, 0.040)  # the hand's TCP on the hook's top bar, rod frame
ROD_MASS = 0.10
# -- the ratchet -------------------------------------------------------------------------------------------------
LOCK_K, LOCK_D, FREE_D = 3000.0, 40.0, 12.0  # N/m and N s/m; FREE_D while the lever is held down
LOCK_FOLLOW = 0.001
# -- the release lever: hinged about x behind the back plate, its pad pressed down -------------------------------
PIVOT = np.array([GX, GY - 0.030, AXIS_Z + 0.034])
RELEASE_DEG = -6.0  # pressed at least this far: the rod is free
# -- the cartridge: its origin at the middle of its tube, nozzle along +y ----------------------------------------
TUBE_R, TUBE_L = 0.0245, 0.205
SHOULDER = 0.006
NOZZLE = ((0.009, 0.030), (0.007, 0.030), (0.0052, 0.030), (0.0035, 0.030))
TIP = TUBE_L / 2 + SHOULDER + sum(n[1] for n in NOZZLE)  # the nozzle's tip, from the middle
SEAT_Y = YOKE_Y0 - TUBE_L / 2 - SHOULDER  # the middle, seated (shoulder against the yoke)
SEAT_Z = -(R_TROUGH - 0.0015 - TUBE_R)  # the cartridge's axis a little below the gun's, lying in the cradle
TUBE_MASS = 0.30
TRAY = np.array([0.170, 0.130])
TUBE0 = np.array([TRAY[0], TRAY[1], TOP + 0.036])
# -- the bath rim and the tiled wall ------------------------------------------------------------------------------
BX0, BX1 = 0.140, 0.480
RIM_Y0, WALL_Y = 0.390, 0.432  # the rim's front face; the wall's face (the joint runs along it)
RIM_Z = TOP + 0.100
JOINT_X0, JOINT_X1 = 0.185, 0.377
N_BEAD = 12
SEG = (JOINT_X1 - JOINT_X0) / N_BEAD
BEAD_R = 0.0045
BEAD_TOL = 0.015  # the nozzle's tip within this of the joint (sideways and up/down)
HIDE = (0.300, 0.410, TOP + 0.050)  # inside the rim, where the unlaid bead waits

RED, RED_DARK, ZINC, STEEL, RUBBER = (0.72, 0.10, 0.08), (0.50, 0.07, 0.06), (0.80, 0.82, 0.84), (0.62, 0.64, 0.67), \
    (0.07, 0.07, 0.07)
WHITE, LABEL, LABEL2, NOZ = (0.95, 0.95, 0.93), (0.10, 0.34, 0.70), (0.98, 0.80, 0.15), (0.90, 0.90, 0.86)
ACRYLIC, TILE, GROUT, SILICONE = (0.97, 0.97, 0.96), (0.86, 0.91, 0.93), (0.58, 0.60, 0.60), (0.99, 0.99, 0.99)
WOOD, WOOD_DARK, CARD, CARD_DARK = (0.74, 0.57, 0.36), (0.56, 0.41, 0.24), (0.68, 0.52, 0.33), (0.52, 0.38, 0.22)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _strip(a_deg: float, y0: float, y1: float, width: float, color, collide=True) -> BoxShape:
    """One flat strip of the half-round cradle, ``a_deg`` round from the bottom (toward +x)."""
    a = math.radians(a_deg)
    rc = R_TROUGH
    c = (rc * math.sin(a), (y0 + y1) / 2, -rc * math.cos(a))
    return BoxShape(c, (width, y1 - y0, 0.003), color, collide=collide,
                    quat_xyzw=(0.0, -math.sin(a / 2), 0.0, math.cos(a / 2)))  # tangent: rising toward +x for a > 0


def _gun() -> Body:
    s = []
    w = 2 * R_TROUGH * math.tan(math.radians(33.4 / 2))
    for a in (-33.4, 0.0, 33.4):  # the cradle: three strips, up to 50 degrees round each side
        s.append(_strip(a, 0.000, YOKE_Y0, w, RED))
    for y in (0.060, 0.120, 0.180):  # pressed stiffening ribs
        s.append(_strip(0.0, y - 0.004, y + 0.004, 0.012, RED_DARK, collide=False))
    # the back plate (the rod passes through a hole in it)
    s.append(BoxShape((0.0, -0.002, 0.0), (0.064, 0.004, 0.064), RED))
    s.append(CylShape((0.0, -0.012, 0.0), 0.011, 0.016, RED_DARK, axis="Y"))  # the rod's bearing boss
    s.append(BoxShape((0.0, -0.018, 0.022), (0.030, 0.026, 0.004), RED_DARK))  # the release lever's bracket
    # the front yoke, with a V notch the nozzle drops into
    yc = YOKE_Y0 + YOKE_T / 2
    s.append(BoxShape((0.0, yc, -0.0215), (0.064, YOKE_T, 0.021), RED))
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * 0.0215, yc, 0.000), (0.021, YOKE_T, 0.022), RED))
        s.append(BoxShape((sx * 0.0240, yc, 0.0215), (0.016, YOKE_T, 0.021), RED))
    # the pistol grip, black rubber over steel, and the trigger in front of it
    s.append(BoxShape((0.0, -0.016, -0.060), (0.024, 0.028, 0.060), RUBBER))
    s.append(BoxShape((0.0, -0.016, GRIP_FOOT + 0.003), (0.028, 0.032, 0.006), RUBBER))
    for z in (-0.045, -0.060, -0.075):
        s.append(BoxShape((0.0, -0.030 - 0.0006, z), (0.020, 0.0012, 0.004), (0.15, 0.15, 0.15), collide=False))
    s.append(BoxShape((0.0, 0.010, -0.058), (0.012, 0.008, 0.050), RED_DARK, collide=False))  # the trigger
    s.append(BoxShape((0.0, 0.004, -0.032), (0.014, 0.020, 0.006), RED_DARK, collide=False))
    return Body.build("gun", pos=tuple(GUN0.tolist()), mass=GUN_MASS, shapes=s, friction=(0.8, 0.7), color=RED,
                      com=(0.0, 0.040, -0.045))


def _release() -> Body:
    """The thumb release lever: a pressed-steel lever pivoted behind the back plate, its ribbed pad forward."""
    s = [CylShape((0.0, 0.0, 0.0), 0.004, 0.034, ZINC, axis="X"),
         BoxShape((0.0, 0.014, 0.004), (0.030, 0.024, 0.004), ZINC)]
    for y in (0.008, 0.014, 0.020):
        s.append(BoxShape((0.0, y, 0.0063), (0.026, 0.0015, 0.0006), STEEL, collide=False))
    return Body.build("release", pos=tuple(PIVOT.tolist()), mass=0.015, shapes=s, friction=(1.0, 0.9), color=ZINC)


def _rod() -> Body:
    """The plunger rod: the push plate, the rod and the D-shaped hook at its back end."""
    rb = -ROD_BACK
    s = [CylShape((0.0, -0.0025, 0.0), 0.021, 0.005, ZINC, axis="Y"),
         CylShape((0.0, -0.007, 0.0), 0.008, 0.004, STEEL, axis="Y"),
         CylShape((0.0, rb / 2, 0.0), 0.004, -rb, ZINC, axis="Y"),
         BoxShape((0.0, rb - 0.022, 0.000), (0.010, 0.044, 0.008), ZINC),  # the hook's bottom bar
         BoxShape((0.0, rb - 0.040, 0.018), (0.010, 0.008, 0.044), ZINC),  # its back
         BoxShape((0.0, rb - 0.022, 0.036), (0.010, 0.044, 0.008), ZINC),  # its top bar (taken hold of)
         BoxShape((0.0, rb - 0.003, 0.028), (0.010, 0.006, 0.010), ZINC)]  # its lip
    return Body.build("rod", pos=(GX, GY + S0, AXIS_Z), mass=ROD_MASS, shapes=s, friction=(1.2, 1.1), color=ZINC,
                      com=(0.0, -0.10, 0.0))


def _tube() -> Body:
    """A 300 ml cartridge of white silicone: the tube, its blue and yellow label, the shoulder and the cut nozzle."""
    s = [CylShape((0.0, 0.0, 0.0), TUBE_R, TUBE_L, WHITE, axis="Y"),
         CylShape((0.0, 0.010, 0.0), TUBE_R + 0.0004, 0.120, LABEL, collide=False, axis="Y"),
         CylShape((0.0, 0.030, 0.0), TUBE_R + 0.0007, 0.020, LABEL2, collide=False, axis="Y"),
         CylShape((0.0, -0.040, 0.0), TUBE_R + 0.0007, 0.006, WHITE, collide=False, axis="Y"),
         CylShape((0.0, -TUBE_L / 2 + 0.002, 0.0), TUBE_R - 0.004, 0.0035, (0.80, 0.80, 0.78), collide=False,
                  axis="Y"),
         CylShape((0.0, TUBE_L / 2 + SHOULDER / 2, 0.0), 0.016, SHOULDER, WHITE, axis="Y")]
    y = TUBE_L / 2 + SHOULDER
    for r, ln in NOZZLE:
        s.append(CylShape((0.0, y + ln / 2, 0.0), r, ln, NOZ, axis="Y"))
        y += ln
    return Body.build("tube", pos=tuple(TUBE0.tolist()), mass=TUBE_MASS, shapes=s, friction=(1.0, 0.9), color=WHITE)


def _tray() -> Body:
    """A cardboard tray with two notched saddles the cartridge lies on."""
    s = [BoxShape((0.0, 0.0, 0.002), (0.100, 0.340, 0.004), CARD)]
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * 0.049, 0.0, 0.0125), (0.002, 0.340, 0.025), CARD_DARK))
    for sy in (-0.150, 0.150):
        s.append(BoxShape((0.0, sy * 1.12, 0.0125), (0.100, 0.002, 0.025), CARD_DARK))
    for y in (-0.060, 0.060):
        for sx in (-1.0, 1.0):
            s.append(BoxShape((sx * 0.026, y, 0.010), (0.024, 0.016, 0.012), CARD_DARK))
    return Body.build("tray", pos=(TRAY[0], TRAY[1] + 0.005, TOP), mass=1.0, kinematic=True, shapes=s, color=CARD)


def _rest() -> Body:
    h = AXIS_Z - R_TROUGH - 0.0015 - TOP
    s = [BoxShape((0.0, 0.0, h / 2), (0.060, 0.040, h), WOOD),
         BoxShape((0.0, 0.0201, h / 2), (0.056, 0.0004, h - 0.006), WOOD_DARK, collide=False)]
    return Body.build("rest", pos=(GX, YOKE_Y0 - 0.026, TOP), mass=1.0, kinematic=True, shapes=s, color=WOOD,
                      friction=(0.8, 0.7))


def _bath() -> Body:
    """The white acrylic rim of a bath along the table's left edge, against a tiled wall."""
    cx = (BX0 + BX1) / 2
    lx = BX1 - BX0
    h = RIM_Z - TOP
    s = [BoxShape((0.0, (RIM_Y0 + WALL_Y) / 2, h / 2), (lx, WALL_Y - RIM_Y0, h), ACRYLIC),
         BoxShape((0.0, RIM_Y0 + 0.004, h - 0.004), (lx, 0.008, 0.008), (0.93, 0.93, 0.92), collide=False)]
    wall_h = 0.300
    s.append(BoxShape((0.0, WALL_Y + 0.010, wall_h / 2), (lx + 0.04, 0.020, wall_h), TILE))
    for k in range(1, 4):  # grout lines between 75 mm tiles, on the wall's face
        z = h + 0.075 * k
        s.append(BoxShape((0.0, WALL_Y - 0.0004, z), (lx + 0.04, 0.0008, 0.003), GROUT, collide=False))
    for k in range(6):
        x = -lx / 2 - 0.02 + 0.075 * (k + 0.5) + 0.0175
        s.append(BoxShape((x, WALL_Y - 0.0004, (h + wall_h) / 2), (0.003, 0.0008, wall_h - h), GROUT, collide=False))
    return Body.build("bath", pos=(cx, 0.0, TOP), mass=10.0, kinematic=True, shapes=s, color=ACRYLIC,
                      friction=(0.6, 0.5))


def _beads() -> list[Body]:
    return [Body.build(f"bead{k}", pos=HIDE, mass=0.001, kinematic=True, color=SILICONE,
                       shapes=[CylShape((0.0, 0.0, 0.0), BEAD_R, SEG + 0.002, SILICONE, collide=False, axis="X")])
            for k in range(N_BEAD)]


hinge("release_pivot", "gun", "release", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(-12.0, 1.0),
      spring=0.003, damping=2e-4, target_deg=0.0)
slider("rod_slide", "gun", "rod", anchor=(GX, GY + S0, AXIS_Z), axis="Y", limits=(-S0, S_MAX - S0),
       stiffness=LOCK_K, damping=LOCK_D, target=0.0, max_force=400.0)

OBJECTS = [_bath(), _rest(), _tray(), _gun(), _release(), _rod(), _tube()] + _beads()


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"target": 0.0, "free": False, "laid": [False] * N_BEAD, "tcps": None, "s": {}, "place": []}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CAULK_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


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


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def lever_deg(rg, rl) -> float:
    rel = rg.T @ rl
    return math.degrees(math.atan2(rel[2, 1], rel[1, 1]))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pg, rg = _pose(base, origin, "gun")
    _, rl = _pose(base, origin, "release")
    pr, _ = _pose(base, origin, "rod")
    pt, rt = _pose(base, origin, "tube")
    lever = lever_deg(rg, rl)
    s = float((rg.T @ (pr - pg))[1])  # the plate's face, in front of the back plate
    free = lever <= RELEASE_DEG
    # the ratchet: free while the lever is held down; otherwise the rod may go in but not come back out
    rel = s - S0  # the slide's own coordinate (0 where the rod started)
    if free:
        if not _EP["free"]:
            set_slider_drive("rod_slide", stiffness=0.0, damping=FREE_D)
    elif _EP["free"]:  # the lever was just let go: the rod is locked where it is
        _EP["target"] = rel
        set_slider_drive("rod_slide", target=rel, stiffness=LOCK_K, damping=LOCK_D)
    elif rel > _EP["target"] + LOCK_FOLLOW:  # pushed in: the ratchet follows
        _EP["target"] = rel
        set_slider_drive("rod_slide", target=rel)
    _EP["free"] = free
    # the cartridge in the gun
    v = rg.T @ (pt - pg)
    along = float((rg.T @ rt)[:, 1][1])  # the cartridge's nozzle direction along the gun's axis (cos)
    seated = (abs(v[0]) <= 0.008 and SEAT_Z - 0.010 <= v[2] <= SEAT_Z + 0.008 and SEAT_Y - 0.030 <= v[1] <= SEAT_Y + 0.006
              and along >= math.cos(math.radians(8.0)))
    back = float(v[1]) - TUBE_L / 2
    home = seated and s >= back - 0.004
    tip = pt + rt @ np.array([0.0, TIP, 0.0])
    near = abs(tip[1] - (WALL_Y - 0.004)) <= BEAD_TOL and abs(tip[2] - (RIM_Z + 0.004)) <= BEAD_TOL
    if home and near:
        for k in range(N_BEAD):
            x0 = JOINT_X0 + k * SEG
            if not _EP["laid"][k] and x0 - 0.002 <= tip[0] <= x0 + SEG + 0.002:
                _EP["laid"][k] = True
                _place(base, origin, f"bead{k}", (x0 + SEG / 2, WALL_Y - BEAD_R, RIM_Z + BEAD_R - 0.0005))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rg[2, 2])))))
    yaw = math.degrees(math.atan2(rg[1, 0], rg[0, 0]))
    on_rest = float(np.linalg.norm(pg - GUN0)) <= 0.025 and tilt <= 6.0 and abs(yaw) <= 8.0
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"seated": seated, "home": home, "on_rest": on_rest, "beads": sum(_EP["laid"])}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = _EP["tcps"]
            print(f"[caulk] t={_DEBUG['steps'] / 20:5.1f}s lever={lever:5.1f} free={free} s={s * 1000:6.1f}"
                  f" tgt={(_EP['target'] + S0) * 1000:6.1f} tube=({v[0] * 1000:.0f},{v[1] * 1000:.0f},{v[2] * 1000:.0f})"
                  f" along={along:.3f} seated={seated} home={home} tip={np.round((tip - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" near={near} beads={sum(_EP['laid'])} gun={np.round((pg - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" tilt={tilt:4.1f} yaw={yaw:5.1f} rest={on_rest}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} score={score(None)}"
                  f" tube-L={np.round((pt - tl) * 1000).astype(int).tolist()}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """At the end: 1.0 with the cartridge seated in the gun nozzle first, the rod pushed in against it, a bead along at
    least 10 of the joint's 12 lengths, the gun back on its grip and rest block and both arms back where they started;
    0.5 with the cartridge seated, the rod in against it and a bead along at least 6 lengths; 0.25 with the cartridge
    seated in the gun; else 0.0. (A bead once laid stays laid.)"""
    s = _EP["s"]
    if not s.get("seated"):
        return 0.0
    if s.get("home") and s.get("beads", 0) >= 10 and s.get("on_rest") and _arms_home():
        return 1.0
    if s.get("home") and s.get("beads", 0) >= 6:
        return 0.5
    return 0.25


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
    """Full score with the gun, its rod and the cartridge still: the trial ends there."""
    return (score(objects) >= 1.0 and objects["gun"].resting() and objects["rod"].resting()
            and objects["tube"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand presses the release lever down with closed fingers; the right takes the rod's hook and pulls the rod
# all the way back. The left lets go of the lever, takes the cartridge off its tray, carries it over the gun and lowers
# it into the cradle, nozzle through the yoke, and lets it drop the last few millimetres. The right pushes the rod back
# in until its plate meets the cartridge and pushes it against the yoke. The left takes the yoke's near ear; both
# hands lift the gun, carry it to the joint's right end with the nozzle's tip in the joint and draw it along to the
# left end, laying the bead; then carry the gun back, set it down on its grip and rest block, let go and go home.

LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.035
PRESS_DZ = 0.005  # the closed fingertips aimed this far below the lever pad's top
PAD = np.array([GX, GY - 0.015, AXIS_Z + 0.034 + 0.006])  # the pad's top, at rest
HOOK_FINGER = -0.012
TUBE_FINGER = -0.0305
TUBE_DX = 0.017  # the cartridge sits this far toward -x of the left TCP once taken hold of (measured)
YOKE_FINGER = -0.010
EAR = np.array([-0.0240, YOKE_Y0 + YOKE_T / 2, 0.032 + 0.004])  # the left's TCP on the yoke's near ear, gun frame
AIM_DZ = 0.006  # aimed this much higher: the arms sag with the gun


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


RL_ROT = grip_rotation(0.0)  # the left pressing the lever (closed fingers side by side along y)
RH_ROT = grip_rotation(math.pi / 2)  # the right on the hook: fingers closing across it, along x
LT_ROT = grip_rotation(-math.pi / 2)  # the left across the cartridge, closing along x
LY_ROT = grip_rotation(0.0)  # the left on the yoke's ear, closing along y


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    L, R = plan["left"], plan["right"]

    def hook_at(s):
        return GUN0 + np.array([HOOK_GRIP[0], S0 + (s - S0) + HOOK_GRIP[1], HOOK_GRIP[2]])

    # -- the left presses the lever; the right takes the hook and pulls the rod back -------------------------------
    press = PAD + [0.0, 0.0, 0.013 - PRESS_DZ - 0.006]
    near = press + [0.0, 0.040, 0.025]  # (straight over the lever is out of the left's reach)
    L += _joint_path("left", np.array(READY, float), _good_ik("left", near, RL_ROT), "over the release lever",
                     FINGER_CLOSED)
    L.append(_w(1.0, press + [0, 0, 0.012], FINGER_CLOSED, "down onto the release lever", RL_ROT))
    L.append(_w(0.8, press, FINGER_CLOSED, "press the release lever down and hold the gun", RL_ROT))
    h0 = hook_at(S0)
    R += _joint_path("right", np.array(READY, float), _good_ik("right", h0 + [0, 0, OVER], RH_ROT),
                     "over the rod's hook", HOOK_FINGER)
    R.append(_w(1.0, h0 + [0, 0, OVER], HOOK_FINGER, "over the rod's hook", RH_ROT))
    R.append(_w(0.8, h0, HOOK_FINGER, "down round the hook's top bar", RH_ROT))
    R.append(_w(0.5, h0, FINGER_CLOSED, "take hold of the hook", RH_ROT))
    h1 = hook_at(-0.012)
    for k in range(1, 5):
        R.append(_w(0.6, h0 + (h1 - h0) * k / 4, FINGER_CLOSED, "pull the rod back, all the way", RH_ROT))
    R.append(_w(0.5, h1, FINGER_CLOSED, "pull the rod back, all the way", RH_ROT))
    t_pull_end = sum(w.duration for w in R)
    t_left = sum(w.duration for w in L)
    L.append(_w(t_pull_end - t_left, press, FINGER_CLOSED, "keep the lever pressed", RL_ROT))
    L.append(_w(0.6, press + [0, 0.040, 0.025], FINGER_CLOSED, "let go of the lever", RL_ROT))

    # -- the left lays the cartridge in the cradle ---------------------------------------------------------------
    grab = TUBE0.copy()
    L.append(_w(1.4, grab + [0, 0, OVER + 0.04], TUBE_FINGER, "over the cartridge", LT_ROT))
    L.append(_w(1.0, grab + [0, 0, 0.012], TUBE_FINGER, "down round the cartridge", LT_ROT))
    L.append(_w(0.6, grab, TUBE_FINGER, "down round the cartridge", LT_ROT))
    L.append(_w(0.6, grab, FINGER_CLOSED, "take hold of the cartridge", LT_ROT))
    up = grab + [0, 0, 0.13]
    L.append(_w(1.0, up, FINGER_CLOSED, "lift it off the tray", LT_ROT))
    drop_y = GY + SEAT_Y - 0.006
    over = np.array([GX + TUBE_DX, drop_y, up[2]])
    L.append(_w(1.6, over, FINGER_CLOSED, "carry it over the gun's cradle", LT_ROT))
    seat = np.array([GX + TUBE_DX, drop_y, AXIS_Z + SEAT_Z + 0.003 + 0.004])
    L.append(_w(1.2, seat + [0, 0, 0.03], FINGER_CLOSED, "lower it into the cradle, nozzle through the yoke", LT_ROT))
    L.append(_w(0.8, seat, FINGER_CLOSED, "lower it into the cradle, nozzle through the yoke", LT_ROT))
    L.append(_w(0.6, seat, TUBE_FINGER, "let go of it", LT_ROT))
    L.append(_w(0.8, seat + [0, 0, 0.07], TUBE_FINGER, "rise clear", LT_ROT))
    t_laid = sum(w.duration for w in L)
    t_right = sum(w.duration for w in R)
    R.append(_w(t_laid - t_right, h1, FINGER_CLOSED, "hold the hook while the cartridge goes in", RH_ROT))
    # -- the right pushes the rod back in until the plate meets the cartridge -----------------------------------
    h2 = hook_at(0.040)
    for k in range(1, 4):
        R.append(_w(0.6, h1 + (h2 - h1) * k / 3, FINGER_CLOSED, "push the rod in until the plate meets the cartridge",
                    RH_ROT))
    R.append(_w(0.6, h2, FINGER_CLOSED, "push the rod in until the plate meets the cartridge", RH_ROT))
    h_seat = hook_at(0.025)
    R.append(_w(0.5, h_seat, FINGER_CLOSED, "hold the hook", RH_ROT))
    # -- the left takes the yoke's ear --------------------------------------------------------------------------
    ear0 = GUN0 + EAR
    L.append(_w(1.4, ear0 + [0, 0, OVER], YOKE_FINGER, "over the gun's front yoke", LY_ROT))
    t_left = sum(w.duration for w in L)
    t_right = sum(w.duration for w in R)
    if t_right > t_left:
        L.append(_w(t_right - t_left, ear0 + [0, 0, OVER], YOKE_FINGER, "over the gun's front yoke", LY_ROT))
    else:
        R.append(_w(t_left - t_right, h_seat, FINGER_CLOSED, "hold the hook", RH_ROT))
    L.append(_w(1.0, ear0, YOKE_FINGER, "down round the yoke's ear", LY_ROT))
    L.append(_w(0.6, ear0, FINGER_CLOSED, "take hold of the yoke", LY_ROT))
    R.append(_w(1.6, h_seat, FINGER_CLOSED, "hold the hook", RH_ROT))

    # -- both hands carry the gun: its origin's path -------------------------------------------------------------
    hook_local = np.array([HOOK_GRIP[0], 0.025 + HOOK_GRIP[1], HOOK_GRIP[2]])

    def carry(t, o, label, n=1):
        o = np.asarray(o, float)
        start = carry.o
        for k in range(1, n + 1):
            p = start + (o - start) * k / n
            L.append(_w(t / n, p + EAR + [0, 0, AIM_DZ], FINGER_CLOSED, label, LY_ROT))
            R.append(_w(t / n, p + hook_local + [0, 0, AIM_DZ], FINGER_CLOSED, label, RH_ROT))
        carry.o = o

    carry.o = GUN0.copy()
    tip_local = np.array([0.0, SEAT_Y + TIP, SEAT_Z])
    tip_z = RIM_Z + 0.014  # (the arms sag ~7 mm: the tip runs ~6 mm over the rim, clear of it)
    dy = (WALL_Y - 0.005) - (GY + tip_local[1])
    lift = GUN0 + [0, 0, 0.040]
    carry(1.0, lift, "lift the gun off its rest", n=2)
    o_start = np.array([JOINT_X0 - 0.006 - 0.0 + 0.0, GY + dy, tip_z + 0.020 - tip_local[2]])
    o_start[0] = JOINT_X0 - 0.006
    carry(1.8, o_start, "carry it to the joint's right end", n=4)
    o_low = o_start.copy()
    o_low[2] = tip_z - tip_local[2]
    carry(0.8, o_low, "the nozzle's tip into the joint", n=2)
    o_end = o_low.copy()
    o_end[0] = JOINT_X1 + 0.018  # (the arms lag ~1-2 cm)
    carry(5.0, o_end, "draw the nozzle along the joint, laying the bead", n=12)
    carry(0.6, o_end + [-0.020, 0, 0.012], "lift the nozzle out of the joint", n=1)
    carry(1.8, lift, "carry the gun back over its rest", n=4)
    carry(1.2, GUN0 + [0, 0, -0.004], "set it down on its grip and rest block", n=3)
    L.append(_w(0.6, carry.o + EAR, YOKE_FINGER, "let go of the yoke", LY_ROT))
    R.append(_w(0.6, carry.o + hook_local, HOOK_FINGER, "let go of the hook", RH_ROT))
    L.append(_w(0.8, GUN0 + EAR + [0, 0, OVER], YOKE_FINGER, "rise clear", LY_ROT))
    R.append(_w(0.8, GUN0 + hook_local + [0, 0, OVER], HOOK_FINGER, "rise clear", RH_ROT))
    L += _joint_path("left", _good_ik("left", GUN0 + EAR + [0, 0, OVER], LY_ROT), np.array(READY, float),
                     "back to where it started", YOKE_FINGER)
    R += _joint_path("right", _good_ik("right", GUN0 + hook_local + [0, 0, OVER], RH_ROT), np.array(READY, float),
                     "back to where it started", HOOK_FINGER)
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(_w(3.0, last.pos, last.finger, "hold still: the trial ends once the full score has held 1 s",
                            last.yaw))
    return plan
