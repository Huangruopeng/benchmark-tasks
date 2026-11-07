"""On the station table stands a small tin of paint with its steel lid on. The tin is light, and its wire bail lies
folded flat to one side, a black plastic grip in the middle. Three white tabs, hinged on the lid's rim over white
arrows on the tin, hook their lips in under the rolled bead round the tin's rim, so the lid cannot be lifted until all
three have been pried up. A steel lid lifter, a claw on a short handle, stands in a wooden holder.

Task: open the tin. One hand holds the tin down by its bail's grip, because a tab takes more of a pull than it takes to
tip the light tin up: hooked and pulled up with nothing holding the tin, the tab tips the whole tin up instead of
turning. The other hand takes the lid lifter and pries up each of the three tabs in turn, hooking its claw under the
tab's lip and lifting until the tab turns up and its lip swings out from under the bead. Once all three are up, the lid
is loose: the bail is let go, the lifter goes back on its holder, the lid is lifted off by its knob and set down on the
table beside the tin, showing the paint, and both arms go back to where they started. The tin must never tip over.

Run it:  inspect-robots run --embodiment yam_arena -E task=pry_the_lid_off_a_paint_tin.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step,  # noqa: E402
                     rotation, segment, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tin: a 9 cm steel tin, 10 cm tall, nearly full of paint; light and smooth-bottomed --------------------------
TX, TY = 0.30, -0.03  # its axis
R_W, WALL_T, H = 0.045, 0.004, 0.100  # outer radius, wall, height
HT = TOP + H  # the rim's top (world z)
TIN_MASS = 0.40
BAIL_R = 0.085  # the folded bail's grip: this far from the axis, toward +y (the left arm's side)
EAR_Z = H - 0.012  # the bail's ears and grip, above the tin's base
GRIP_R = 0.009  # the bail grip's radius

# -- the lid: a steel disc on the rim with a plug in the mouth and a knob; three tabs on its rim ---------------------
LID_R = R_W - 0.002
KNOB = (0.014, 0.014, 0.024)
MARKS = (225.0, 262.5, 300.0)  # the tabs' angles (degrees about the tin's axis, from +x: near-right, right, far-right)
TAB_W = 0.020  # along the rim
BEAD_OUT, BEAD_H = 0.004, 0.006  # the tin's rolled bead round its rim: out from the wall, tall
HINGE_R, HINGE_Z = R_W + 0.002, 0.004  # each tab's hinge: over the bead, this far above the rim's top
TAB_HOLD = 0.030  # N m per degree: a tab holds where it last stopped ...
SLIP_DEG = 2.0  # ... until turned this far past it (about 10 N at its lip; then the task file moves its hold)
PRIED_DEG = 18.0  # a tab turned up this far: its hook has swung out from under the bead (pried)

# -- the lid lifter: a square handle, a steel shank down from it and a claw at its foot; origin the handle's middle ----
HANDLE = (0.016, 0.016, 0.030)
SHANK = (0.005, 0.005, 0.032)
CLAW = (0.012, 0.008, 0.004)  # out from the shank (along the lifter's x), across, thick
CLAW_TIP = np.array([0.0025 + CLAW[0], 0.0, -HANDLE[2] / 2 - SHANK[2] + CLAW[2] / 2])  # the claw's tip, top face
HOLDER_AT = np.array([0.17, -0.20])  # the lifter's holder
HOLDER_H = 0.045  # the holder's top: the handle stands on it, the shank and claw hang in its slot
LIFTER_Z = TOP + HOLDER_H + HANDLE[2] / 2

STEEL, DARK, BLACK, WHITE, WOOD, LABEL, PAINT, LID_C = (0.72, 0.73, 0.75), (0.25, 0.25, 0.27), (0.08, 0.08, 0.08), \
    (0.96, 0.96, 0.94), (0.70, 0.52, 0.33), (0.10, 0.22, 0.50), (0.12, 0.62, 0.55), (0.62, 0.63, 0.66)

# -- scoring ----------------------------------------------------------------------------------------------------------
TIPPED = math.radians(25.0)  # the tin tilted more than this, ever: tipped over (paint spilled)
LID_AWAY = 0.08  # the lid off: its middle at least this far from the tin's axis, down on the table
LEVER_AWAY = 0.10
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


def _u(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0])


def _tin() -> Body:
    """The tin with its folded bail; origin at its base's middle."""
    shapes = [CylShape((0.0, 0.0, 0.002), R_W, 0.004, STEEL)]
    n = 20
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = R_W - WALL_T / 2
        shapes.append(BoxShape((rc * math.cos(a), rc * math.sin(a), H / 2), (WALL_T, 2 * math.pi * R_W / n * 1.12, H),
                               STEEL, yaw=a))
    for i in range(n):  # the rolled bead round the rim, which the lid's tabs hook under
        a = 2 * math.pi * i / n
        rc = R_W + (BEAD_OUT - 0.0005) / 2
        shapes.append(BoxShape((rc * math.cos(a), rc * math.sin(a), H - BEAD_H / 2),
                               (BEAD_OUT + 0.0005, 2 * math.pi * (R_W + BEAD_OUT) / n * 1.12, BEAD_H), STEEL, yaw=a))
    shapes += [CylShape((0.0, 0.0, 0.045), R_W + 0.0009, 0.062, LABEL, collide=False),  # the label
               CylShape((0.0, 0.0, 0.050), R_W + 0.0010, 0.006, WHITE, collide=False),  # a white band on it
               CylShape((0.0, 0.0, (0.004 + H - 0.014) / 2), R_W - WALL_T - 0.0003, H - 0.018, PAINT,
                        collide=False)]  # the paint, 1.4 cm below the rim
    for deg in MARKS:  # a white arrow under each tab, pointing up at it
        u = _u(deg)
        a = math.radians(deg)
        shapes += [BoxShape(tuple((u * (R_W + 0.0012) + [0.0, 0.0, H - 0.010]).tolist()), (0.0006, 0.004, 0.010), WHITE,
                            yaw=a, collide=False),
                   BoxShape(tuple((u * (R_W + 0.0012) + [0.0, 0.0, H - 0.0045]).tolist()), (0.0006, 0.010, 0.003),
                            WHITE, yaw=a, collide=False)]
    gz = EAR_Z
    for s in (-1.0, 1.0):  # the ears on the near and far sides, and the bail's wire from them to its grip
        shapes.append(BoxShape((s * (R_W + 0.003), 0.0, gz), (0.007, 0.012, 0.016), STEEL))
        pts = [(s * (R_W + 0.005), 0.0, gz), (s * (R_W + 0.002), BAIL_R * 0.55, gz), (s * 0.034, BAIL_R, gz),
               (s * 0.0, BAIL_R, gz)]
        for p, q in zip(pts[:-1], pts[1:]):
            shapes.append(segment(p, q, 0.003, DARK, collide=False))
    shapes.append(CylShape((0.0, BAIL_R, gz), GRIP_R, 0.060, BLACK, axis="X"))  # the bail's grip
    return Body.build("tin", pos=(TX, TY, TOP), mass=TIN_MASS, shapes=shapes, friction=(0.7, 0.6),
                      color=STEEL, com=(0.0, 0.0, 0.035))


def _lid() -> Body:
    """The lid; origin on the tin's axis at the rim's top."""
    shapes = [CylShape((0.0, 0.0, 0.001), LID_R, 0.002, LID_C),
              CylShape((0.0, 0.0, -0.003), R_W - WALL_T - 0.003, 0.004, LID_C),  # its plug, in the tin's mouth
              CylShape((0.0, 0.0, 0.0021), LID_R - 0.006, 0.0004, (0.55, 0.56, 0.59), collide=False),
              BoxShape((0.0, 0.0, 0.002 + KNOB[2] / 2), KNOB, BLACK)]
    return Body.build("lid", pos=(TX, TY, HT), mass=0.05, shapes=shapes, friction=(1.0, 0.9), color=LID_C)


def _tab(k: int) -> Body:
    """Tab k: a white steel hook on the lid's rim, hinged over the tin's bead; its lip reaches in under the bead, so the
    lid cannot be lifted until the tab is turned up and the lip swings out. Origin on the hinge line."""
    deg = MARKS[k]
    u, a = _u(deg), math.radians(deg)

    def box(r0, r1, z0, z1):  # radial and height extents, from the hinge
        return BoxShape(tuple((u * (r0 + r1) / 2 + [0.0, 0.0, (z0 + z1) / 2]).tolist()), (r1 - r0, TAB_W, z1 - z0),
                        WHITE, yaw=a)

    lip_in = R_W + 0.0015 - HINGE_R
    shapes = [box(-0.010, 0.008, -0.002, 0.002),  # the arm, over the lid's edge and out over the bead
              box(0.004, 0.008, -0.014, 0.002),  # the hook, down outside the bead
              box(lip_in, 0.006, -0.014, -0.0105)]  # its lip, in under the bead
    return Body.build(f"tab_{k + 1}", pos=tuple((np.array([TX, TY, HT + HINGE_Z]) + u * HINGE_R).tolist()), mass=0.006,
                      shapes=shapes, friction=(0.8, 0.7), color=WHITE)


def _rest() -> Body:
    """The lifter's wooden holder: two blocks with a slot between them; the handle stands on top, the shank and claw
    hang in the slot."""
    h = HOLDER_H
    return Body.build("rest", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.3, kinematic=True, color=WOOD,
                      shapes=[BoxShape((0.0, s * 0.0125, h / 2), (0.05, 0.016, h), WOOD) for s in (-1.0, 1.0)]
                      + [BoxShape((0.0, 0.0, 0.004), (0.05, 0.05, 0.008), WOOD)])


def _lever() -> Body:
    """The lid lifter, claw pointing +x."""
    sz = -HANDLE[2] / 2 - SHANK[2] / 2
    shapes = [BoxShape((0.0, 0.0, 0.0), HANDLE, (0.15, 0.15, 0.16)),
              BoxShape((0.0, 0.0, sz), SHANK, STEEL),
              BoxShape((0.0025 + CLAW[0] / 2, 0.0, -HANDLE[2] / 2 - SHANK[2] + CLAW[2] / 2), CLAW, STEEL)]
    return Body.build("lever", pos=(HOLDER_AT[0], HOLDER_AT[1], LIFTER_Z), mass=0.05, shapes=shapes,
                      friction=(1.0, 0.9), color=STEEL, com=(0.0, 0.0, -0.01))


OBJECTS = [_tin(), _lid(), *[_tab(k) for k in range(3)], _rest(), _lever()]
for _k, _deg in enumerate(MARKS):  # each tab's hinge, along the rim (the task file turns the joint's frame to match)
    hinge(f"tab_{_k + 1}_hinge", "lid", f"tab_{_k + 1}",
          anchor=tuple((np.array([TX, TY, HT + HINGE_Z]) + _u(_deg) * HINGE_R).tolist()), axis="X",
          limits_deg=(-1.0, 50.0), spring=TAB_HOLD, damping=0.0005, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hold": [0.0] * 3, "turned": [0.0] * 3, "loose": False, "tipped": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TIN_DEBUG")), "steps": 0}


def _hinge_frames() -> None:
    """Turn each tab's hinge to run along the rim (its x axis along the rim, outward-up positive), and rebuild it."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    for k, deg in enumerate(MARKS):
        joint = UsdPhysics.RevoluteJoint(get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS[f"tab_{k + 1}_hinge"]))
        h = math.radians(deg - 90.0) / 2
        q = Gf.Quatf(math.cos(h), 0.0, 0.0, math.sin(h))
        joint.GetLocalRot0Attr().Set(q)
        joint.GetLocalRot1Attr().Set(q)
        enabled = joint.GetJointEnabledAttr() or joint.CreateJointEnabledAttr(True)
        enabled.Set(False)
        enabled.Set(True)


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _hinge_frames()


def _tcps(base, origin) -> list[np.ndarray]:
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


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    tin_p, tin_r = _pose(base, origin, "tin")
    lid_p, lid_r = _pose(base, origin, "lid")
    lev_p, lev_r = _pose(base, origin, "lever")
    tilt = math.acos(max(-1.0, min(1.0, float(tin_r[2, 2]))))
    if tilt > TIPPED:
        _EP["tipped"] = True
    for k, deg in enumerate(MARKS):
        _, tr = _pose(base, origin, f"tab_{k + 1}")
        u = _u(deg)
        v = (lid_r.T @ tr) @ u  # the tab's outward direction, in the lid's frame
        turned = math.degrees(math.atan2(float(v[2]), float(v @ u)))  # turned up (outer end up), degrees
        _EP["turned"][k] = turned
        if turned - _EP["hold"][k] > SLIP_DEG:  # turned up past its hold: it gives, and stays
            _EP["hold"][k] = turned
            set_hinge_drive(f"tab_{k + 1}_hinge", target_deg=turned)
    pried = [h >= PRIED_DEG for h in _EP["hold"]]
    _EP["loose"] = all(pried)
    _EP["tcps"] = tcps = _tcps(base, origin)
    axis_xy = tin_p[:2]
    knob = lid_p + lid_r @ np.array([0.0, 0.0, 0.002 + KNOB[2]])
    lid_off = (_EP["loose"] and float(np.linalg.norm(lid_p[:2] - axis_xy)) >= LID_AWAY and lid_p[2] < TOP + 0.03
               and float(lid_r[2, 2]) > math.cos(math.radians(30.0)))
    lever_down = float(np.linalg.norm(lev_p[:2] - axis_xy)) >= LEVER_AWAY and lev_p[2] < TOP + 0.10
    clear = all(float(np.linalg.norm(t - lev_p)) >= LET_GO and float(np.linalg.norm(t - knob)) >= LET_GO for t in tcps)
    _EP["s"] = {"pried": sum(pried), "lid_off": lid_off, "lever_down": lever_down, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[tin] t={_DEBUG['steps'] / 20:5.1f}s turned={[round(v, 1) for v in _EP['turned']]} "
                  f"hold={[round(v, 1) for v in _EP['hold']]} loose={_EP['loose']} "
                  f"tin={np.round(tin_p * 1000).astype(int).tolist()} tilt={math.degrees(tilt):.1f} pitch={math.degrees(math.asin(max(-1.0, min(1.0, float(lev_r[2, 0]))))):.1f} "
                  f"lid={np.round(lid_p * 1000).astype(int).tolist()} lever={np.round(lev_p * 1000).astype(int).tolist()}"
                  f" lid_off={lid_off} lever_down={lever_down} clear={clear} home={_arms_home()} "
                  f"R={np.round(tr * 1000).astype(int).tolist()} L={np.round(tl * 1000).astype(int).tolist()} "
                  f"score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"pried": st.get("pried", 0), "lid_off": bool(st.get("lid_off")), "lever_down": bool(st.get("lever_down")),
            "clear": bool(st.get("clear")), "tipped": _EP["tipped"], "home": _arms_home()}


def _points(s) -> float:
    if s["tipped"]:
        return 0.0
    if s["pried"] == 3 and s["lid_off"] and s["lever_down"] and s["clear"] and s["home"]:
        return 1.0
    if s["pried"] == 3:
        return 0.5
    return 0.25 if s["pried"] >= 1 else 0.0


def score(objects) -> float:
    """0.0 if the tin has ever tilted more than 25 degrees (tipped over). Otherwise 1.0 with all three tabs pried up
    (each turned up at least 18 degrees, which swings its hook out from under the tin's bead and frees the lid), the lid off (its middle at least 8 cm from the tin's axis, down
    on the table, right way up within 30 degrees), the lid lifter put down (at least 10 cm from the tin's axis, its
    handle's middle lower than 10 cm above the table), each gripper's TCP at least 4 cm from the lifter's handle and
    from the lid's knob, and both arms back where they started (each TCP within 5 cm of its start); 0.5 with all three
    tabs pried up; 0.25 with at least one; else 0.0."""
    return _points(_status())


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
    """Full score with the tin, the lid and the lid lifter still."""
    return (score(objects) >= 1.0 and objects["tin"].resting() and objects["lid"].resting()
            and objects["lever"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the bail's grip, fingers closing across it, and holds the tin down. The right hand
# takes the lid lifter out of its holder by its handle and, for each tab in turn, lowers its claw just outside the tab,
# below the rim, slides the claw in under the tab's lip and lifts: the tab turns up. Once all three tabs are up the left
# lets go of the bail and goes home; the right puts the lifter back on its holder, lifts the lid off by its knob, sets
# it down on the table beside the tin and goes home.

HOLD_GRIP = grip_rotation(0.0)  # left: straight down, fingers closing along y, across the bail's grip
HOLD_OPEN = -0.016  # the left's fingers opened only a little wider than the grip
PRESS_DOWN = 0.008  # while the tabs are pried, the left aims this far below the grip: it holds the tin down
GRIP_DZ = 0.0  # TCP aimed at the lifter's origin (the handle's middle) when taking it (the arm stops ~8 mm high)
HANG_DZ = 0.018  # ... and once held (the handle settles 1 cm lower in the fingers)
LIFTER_X_OFF = 0.005  # the TCP this far ahead of the lifter's origin, toward its claw
CLAW_LOW = -0.017  # the claw's top this far below the rim's top as it goes under a tab's hook
CLAW_UP = 0.004  # ... and lifted to this far above it: the hook turns up and swings out
OUT = 0.022  # the claw's tip this far outside the tin's wall as it comes down (clear of the tab)
UNDER = 0.007  # ... and this far outside it, under the hook's lip (outside the bead)
LID_PUT = np.array([0.20, -0.10])  # where the lid is set down
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
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _claw_tcp(r: float, dz: float, deg: float) -> np.ndarray:
    """The right TCP that puts the lifter's claw tip at distance ``r`` from the tin's axis toward the tab at ``deg``,
    pointing in, its top ``dz`` above the rim's top."""
    u = _u(deg)
    yaw = math.radians(deg) + math.pi
    tip = np.array([TX, TY, HT + dz]) + u * r
    return tip - _rot_z(yaw) @ CLAW_TIP + _rot_z(yaw) @ np.array([LIFTER_X_OFF, 0.0, HANG_DZ])


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    grip = np.array([TX, TY + BAIL_R, TOP + EAR_Z])
    out = _lead_in("left", grip + [0.0, 0.0, 0.05], HOLD_GRIP, "over the bail's grip", HOLD_OPEN)
    out += [_w(1.0, grip, HOLD_OPEN, "down round it", HOLD_GRIP),
            _w(0.6, grip, FINGER_CLOSED, "hold the tin by its bail", HOLD_GRIP)]
    held_at = sum(w.duration for w in out)
    up = grip + [0.0, 0.0, 0.06]
    out += [_w(0.5, grip - [0.0, 0.0, PRESS_DOWN], FINGER_CLOSED, "hold the tin down", HOLD_GRIP),
            _w(max(release_at - held_at - 1.0, 0.3), grip - [0.0, 0.0, PRESS_DOWN], FINGER_CLOSED,
               "hold it down while the tabs are pried", HOLD_GRIP),
            _w(0.5, grip, FINGER_CLOSED, "ease off", HOLD_GRIP),
            _w(0.5, grip, FINGER_OPEN, "let go", HOLD_GRIP),
            _w(0.8, up, FINGER_OPEN, "rise", HOLD_GRIP)]
    out += _go_home("left", up, HOLD_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    take = np.array([HOLDER_AT[0], HOLDER_AT[1], LIFTER_Z + GRIP_DZ])
    back = np.array([HOLDER_AT[0] + LIFTER_X_OFF, HOLDER_AT[1], LIFTER_Z + HANG_DZ + 0.003])
    g0 = grip_rotation(0.0)
    out = _lead_in("right", take + [0.0, 0.0, 0.05], g0, "over the lid lifter's handle", FINGER_OPEN)
    out += [_w(0.8, take, FINGER_OPEN, "down round the handle", g0),
            _w(0.5, take, FINGER_CLOSED, "take the lifter", g0),
            _w(1.0, take + [0.0, 0.0, 0.08], FINGER_CLOSED, "lift it out of its holder", g0)]
    t = sum(w.duration for w in out)
    if start > t + 0.5:
        out.append(_w(start + 0.5 - t, out[-1].pos, FINGER_CLOSED, "wait for the tin to be held down", g0))
    for k, deg in enumerate(MARKS):
        g = grip_rotation(math.radians(deg) + math.pi)  # the claw pointing at the tin's axis
        out += [_w(1.6 if k == 0 else 1.2, _claw_tcp(R_W + OUT, 0.015, deg), FINGER_CLOSED,
                   f"over tab {k + 1}, the claw pointing in", g),
                _w(0.8, _claw_tcp(R_W + OUT, CLAW_LOW, deg), FINGER_CLOSED, "down beside the tin, below the rim", g),
                _w(0.7, _claw_tcp(R_W + UNDER, CLAW_LOW, deg), FINGER_CLOSED, f"the claw in under tab {k + 1}", g),
                _w(1.4, _claw_tcp(R_W + UNDER, CLAW_UP, deg), FINGER_CLOSED, "lift: the tab bends up", g),
                _w(0.5, _claw_tcp(R_W + OUT, CLAW_UP, deg), FINGER_CLOSED, "draw the claw out", g),
                _w(0.5, _claw_tcp(R_W + OUT, 0.015, deg), FINGER_CLOSED, "up", g)]
    pried_at = sum(w.duration for w in out)
    out += [_w(1.6, back + [0.0, 0.0, 0.08], FINGER_CLOSED, "back over the holder", g0),
            _w(0.6, back + [0.0, 0.0, 0.08], FINGER_CLOSED, "steady", g0),
            _w(1.4, back, FINGER_CLOSED, "put the lifter back in its holder", g0),
            _w(0.4, back, FINGER_OPEN, "let go", g0),
            _w(0.6, back + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", g0)]
    knob = np.array([TX, TY, HT + 0.002 + KNOB[2] - 0.008])
    gk = grip_rotation(math.pi / 2)  # fingers closing along x: clear of the turned-up tabs
    ko = -(KNOB[0] / 2 + 0.006)  # opened only a little wider than the knob
    put = np.array([LID_PUT[0], LID_PUT[1], TOP + 0.005 + 0.002 + KNOB[2] - 0.008 + 0.002])
    out += [_w(1.4, knob + [0.0, 0.0, 0.03], ko, "over the lid's knob", gk),
            _w(0.7, knob, ko, "down round it", gk),
            _w(0.5, knob, FINGER_CLOSED, "take the knob", gk)]
    out += [_w(1.0, knob + [0.0, 0.0, 0.035], FINGER_CLOSED, "lift the lid off: the paint shows", gk)]
    lifted_at = sum(w.duration for w in out)
    out += [_w(1.5, put + [0.0, 0.0, 0.10], FINGER_CLOSED, "over the table beside the tin", gk),
            _w(0.9, put, FINGER_CLOSED, "set the lid down", gk),
            _w(0.4, put, ko, "let go", gk),
            _w(0.6, put + [0.0, 0.0, 0.05], ko, "rise", gk)]
    out += _go_home("right", put + [0.0, 0.0, 0.05], gk, FINGER_OPEN)
    return out, pried_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, pried_at = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(pried_at + 0.3, total)  # the lid is loose: the tin needs no holding to lift it off
    return {"left": left, "right": right}
