"""On the station table lies an open steel equipment chassis, its right end propped on a wooden block so that it slopes
gently down to the left. A screw has been dropped inside and slid down under the circuit board that covers the left
half of the floor, where nothing can reach it. A magnetic pick-up tool (a rod with a magnet on its bent-down tip)
stands on a holder in front of the chassis, and a magnetic parts dish waits to its right.

Task: get the screw out and into the dish. One hand lifts the chassis's left end by its handle, tipping the chassis
the other way, so the screw slides out from under the board to the open right end, and holds it there (let go and the
chassis drops back and the screw slides back under the board). The other hand takes the pick-up tool, picks the
screw up with its magnet, lifts it out and touches it into the dish, which takes it. Then the tool goes back on its
holder, the chassis is set back down on its block, and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=fish_a_dropped_screw_out_of_a_chassis.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step,  # noqa: E402
                     rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the chassis: an open steel box, long along y; its right end on a block, so it slopes down to the left -----------
CX, CY = 0.30, 0.06
LEN, WID, WALL_H, T = 0.26, 0.16, 0.030, 0.004  # along y, along x, wall height, sheet thickness (as modelled)
FOOT, FH = 0.014, 0.008  # rubber feet, square, and their height
FOOT_B = LEN / 2 - 0.02  # the feet's distance from the middle, along the chassis
BLOCK_H = 0.023  # the block under the right end
PHI0 = -math.asin(BLOCK_H / (2 * FOOT_B))  # the chassis's slope at rest (about x; negative: the left end lower)
Z0 = TOP + FOOT_B * math.sin(-PHI0) + FH * math.cos(PHI0)  # its origin (the floor's underside, middle) above the table
GAP = 0.012  # the board's standoffs: the gap over the floor
BOARD_B = (0.0, LEN / 2 - T)  # the board covers the floor from the middle to the left end
CHASSIS_MASS = 1.2
HANDLE_UP = 0.030  # the handle's bar, above the left wall's top

# -- the screw ------------------------------------------------------------------------------------------------------
SCREW_B = LEN / 2 - T - 0.012  # where it lies at the start: under the board, against the left end

# -- the pick-up tool: a square handle on a steel rod, a magnet at its foot; it stands in a slotted holder -------------
WAND_AT = np.array([0.20, -0.20])  # its holder
T_HANDLE = (0.016, 0.040, 0.030)  # across the fingers, across the slot (it rests on both blocks), tall
T_ROD = (0.006, 0.006, 0.055)
MAG_R, MAG_H = 0.007, 0.012
TIP_DOWN = T_HANDLE[2] / 2 + T_ROD[2] + MAG_H  # the magnet's face, below the tool's origin (the handle's middle)
STAND_TOP = TOP + T_ROD[2] + MAG_H + 0.004  # the holder's top: the handle stands on it, the rod hangs in its slot
WAND_Z = STAND_TOP + T_HANDLE[2] / 2

# -- the magnetic parts dish -------------------------------------------------------------------------------------------
DISH_AT = np.array([0.30, -0.17])
DISH_R, DISH_H = 0.045, 0.020

DARK, STEEL, GREEN, BLACK, ZINC, WOOD, RED, CHIP = (0.30, 0.31, 0.33), (0.62, 0.63, 0.66), (0.10, 0.42, 0.20), \
    (0.07, 0.07, 0.07), (0.86, 0.84, 0.70), (0.70, 0.52, 0.33), (0.80, 0.12, 0.10), (0.12, 0.12, 0.13)

# -- scoring ----------------------------------------------------------------------------------------------------------
PICK_R = 0.020  # the magnet takes the screw within this distance of its face
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _quat(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


R0 = _rot_x(PHI0)
Q0 = _quat(R0)
ORIGIN = np.array([CX, CY, Z0])  # the chassis's origin, world


def _sloped(a, b, c, size, color, collide=True) -> BoxShape:
    """A box in the chassis's own (sloped) frame: across a, along b, up c."""
    return BoxShape(tuple((R0 @ np.array([a, b, c])).tolist()), tuple(size), color, collide=collide, quat_xyzw=Q0)


def _chassis() -> Body:
    shapes = [_sloped(0.0, 0.0, T / 2, (WID, LEN, T), DARK),
              *[_sloped(s * (WID / 2 - T / 2), 0.0, WALL_H / 2, (T, LEN, WALL_H), STEEL) for s in (-1.0, 1.0)],
              *[_sloped(0.0, s * (LEN / 2 - T / 2), WALL_H / 2, (WID - 2 * T, T, WALL_H), STEEL) for s in (-1.0, 1.0)]]
    b0, b1 = BOARD_B
    bw = WID - 2 * T - 0.006
    shapes.append(_sloped(0.0, (b0 + b1) / 2, T + GAP + 0.001, (bw, b1 - b0, 0.002), GREEN))  # the circuit board
    for sa in (-1.0, 1.0):
        for sb in (b0 + 0.012, b1 - 0.012):
            shapes.append(_sloped(sa * (bw / 2 - 0.008), sb, T + GAP / 2, (0.005, 0.005, GAP), STEEL))  # standoffs
    for a, b, size, col in ((-0.03, 0.03, (0.030, 0.030, 0.004), CHIP), (0.035, 0.05, (0.020, 0.040, 0.003), CHIP),
                            (0.0, 0.095, (0.050, 0.020, 0.012), STEEL), (-0.045, 0.085, (0.010, 0.010, 0.012), BLACK),
                            (0.045, 0.10, (0.010, 0.010, 0.012), BLACK)):
        shapes.append(_sloped(a, b, T + GAP + 0.002 + size[2] / 2, size, col, collide=False))  # parts on the board
    for sa in (-1.0, 1.0):
        for sb in (-1.0, 1.0):
            shapes.append(_sloped(sa * (WID / 2 - 0.02), sb * FOOT_B, -FH / 2, (FOOT, FOOT, FH), BLACK))
    hb = LEN / 2 - T / 2  # the handle: two posts and a bar along a, over the left end's wall
    for sa in (-1.0, 1.0):
        shapes.append(_sloped(sa * 0.045, hb, WALL_H + HANDLE_UP / 2, (0.008, 0.008, HANDLE_UP + 0.008), BLACK))
    shapes.append(_sloped(0.0, hb, WALL_H + HANDLE_UP, (0.100, 0.016, 0.016), BLACK))
    return Body.build("chassis", pos=tuple(ORIGIN.tolist()), mass=CHASSIS_MASS, shapes=shapes, friction=(0.9, 0.8),
                      color=STEEL, com=tuple((R0 @ np.array([0.0, 0.0, 0.02])).tolist()))


def _block() -> Body:
    y = CY - FOOT_B * math.cos(PHI0)
    return Body.build("block", pos=(CX, y, TOP), mass=0.5, kinematic=True, color=WOOD,
                      shapes=[BoxShape((0.0, 0.0, BLOCK_H / 2), (0.20, 0.045, BLOCK_H), WOOD)])


def _screw_at() -> np.ndarray:
    return ORIGIN + R0 @ np.array([0.0, SCREW_B, T])


def _screw() -> Body:
    """A small steel screw lying on its side across the chassis; origin under its middle, on the floor."""
    shapes = [BoxShape(tuple((R0 @ np.array([-0.002, 0.0, 0.002])).tolist()), (0.014, 0.004, 0.004), ZINC,
                       quat_xyzw=Q0),
              BoxShape(tuple((R0 @ np.array([0.0065, 0.0, 0.003])).tolist()), (0.0035, 0.008, 0.006), ZINC,
                       quat_xyzw=Q0)]
    return Body.build("screw", pos=tuple(_screw_at().tolist()), mass=0.0025, shapes=shapes, friction=(0.06, 0.05),
                      slick=True, color=ZINC, contact_offset=0.001)


def _stand() -> Body:
    """The tool's wooden holder: a pocket 2.8 by 3 cm; the handle rests across its top, the rod hangs in it."""
    h = STAND_TOP - TOP
    return Body.build("stand", pos=(WAND_AT[0], WAND_AT[1], TOP), mass=0.5, kinematic=True, color=WOOD,
                      shapes=[BoxShape((0.0, s * 0.0215, h / 2), (0.06, 0.015, h), WOOD) for s in (-1.0, 1.0)]
                      + [BoxShape((s * 0.0225, 0.0, (h - 0.003) / 2), (0.015, 0.058, h - 0.003), WOOD)
                         for s in (-1.0, 1.0)]  # its ends, lower than the handle: the slot is a pocket
                      + [BoxShape((0.0, 0.0, 0.004), (0.06, 0.058, 0.008), WOOD)])


def _wand() -> Body:
    """The pick-up tool, upright; origin at its handle's middle."""
    shapes = [BoxShape((0.0, 0.0, 0.0), T_HANDLE, BLACK),
              BoxShape((0.0, 0.0, -T_HANDLE[2] / 2 - T_ROD[2] / 2), T_ROD, STEEL),
              CylShape((0.0, 0.0, -TIP_DOWN + MAG_H / 2), MAG_R, MAG_H, RED)]
    return Body.build("wand", pos=(WAND_AT[0], WAND_AT[1], WAND_Z), mass=0.06, shapes=shapes, friction=(1.0, 0.9),
                      color=BLACK, com=(0.0, 0.0, -0.02))


def _dish() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), DISH_R, 0.004, DARK)]
    n = 18
    for i in range(n):
        a = 2 * math.pi * i / n
        shapes.append(BoxShape(((DISH_R - 0.002) * math.cos(a), (DISH_R - 0.002) * math.sin(a), DISH_H / 2),
                               (0.004, 2 * math.pi * DISH_R / n * 1.1, DISH_H), DARK, yaw=a))
    shapes.append(CylShape((0.0, 0.0, 0.0042), 0.012, 0.0004, RED, collide=False))
    return Body.build("dish", pos=(DISH_AT[0], DISH_AT[1], TOP), mass=0.4, kinematic=True, shapes=shapes, color=DARK)


OBJECTS = [_block(), _chassis(), _stand(), _wand(), _screw(), _dish()]
glue("magnet", "wand", "screw", anchor=tuple(_screw_at().tolist()))  # off until the magnet takes the screw


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"held": False, "ever_held": False, "in_dish": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("SCREW_DEBUG")), "steps": 0}


def _quat_wxyz(m: np.ndarray):
    x, y, z, w = _quat(m)
    return (w, x, y, z)


def _magnet_on(rel_pos, rel_rot) -> None:
    """Hold the screw where it is, against the tool."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS["magnet"]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_pos]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(rel_rot)))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    set_glue_on("magnet", True)


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_glue_on("magnet", False)


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
    ch_p, ch_r = _pose(base, origin, "chassis")
    w_p, w_r = _pose(base, origin, "wand")
    s_p, s_r = _pose(base, origin, "screw")
    tip = w_p + w_r @ np.array([0.0, 0.0, -TIP_DOWN])
    if not _EP["held"] and not _EP["in_dish"] and float(np.linalg.norm(tip - s_p)) < PICK_R:
        _EP["held"] = _EP["ever_held"] = True  # the magnet takes it
        _magnet_on(w_r.T @ (s_p - w_p), w_r.T @ s_r)
    if (_EP["held"] and float(np.linalg.norm(s_p[:2] - DISH_AT)) < DISH_R - 0.012
            and s_p[2] < TOP + DISH_H + 0.015):  # touched into the dish: the dish's stronger magnet takes it
        _EP["held"] = False
        _EP["in_dish"] = True
        set_glue_on("magnet", False)
    _EP["tcps"] = tcps = _tcps(base, origin)
    tilt = math.atan2(float(ch_r[2, 1]), float(ch_r[1, 1]))  # the chassis turned about x from where it started
    down = abs(tilt) < math.radians(2.0) and float(np.linalg.norm(ch_p - ORIGIN)) < 0.02
    in_dish = (_EP["in_dish"] and float(np.linalg.norm(s_p[:2] - DISH_AT)) < DISH_R - 0.004
               and s_p[2] < TOP + DISH_H)
    on_stand = float(np.linalg.norm(w_p - np.array([WAND_AT[0], WAND_AT[1], WAND_Z]))) < 0.03
    handle = ch_p + ch_r @ (R0 @ np.array([0.0, LEN / 2 - T / 2, WALL_H + HANDLE_UP]))
    clear = all(float(np.linalg.norm(t - w_p)) >= LET_GO and float(np.linalg.norm(t - handle)) >= LET_GO for t in tcps)
    _EP["s"] = {"in_dish": in_dish, "on_stand": on_stand, "down": down, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = tcps
            print(f"[screw] t={_DEBUG['steps'] / 20:5.1f}s tilt={math.degrees(tilt):5.1f} "
                  f"screw={np.round(s_p * 1000).astype(int).tolist()} tip={np.round(tip * 1000).astype(int).tolist()} "
                  f"d={float(np.linalg.norm(tip - s_p)) * 1000:.0f}mm held={_EP['held']} in_dish={in_dish} "
                  f"wand={np.round(w_p * 1000).astype(int).tolist()} on_stand={on_stand} down={down} clear={clear} "
                  f"home={_arms_home()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} score={_points(_status())}", file=sys.stderr,
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"ever_held": _EP["ever_held"], "in_dish": bool(st.get("in_dish")), "on_stand": bool(st.get("on_stand")),
            "down": bool(st.get("down")), "clear": bool(st.get("clear")), "home": _arms_home()}


def _points(s) -> float:
    if s["in_dish"] and s["on_stand"] and s["down"] and s["clear"] and s["home"]:
        return 1.0
    if s["in_dish"]:
        return 0.5
    return 0.25 if s["ever_held"] else 0.0


def score(objects) -> float:
    """1.0 with the screw in the parts dish (its middle within 4.1 cm of the dish's middle, below the dish's rim), the
    pick-up tool back in its holder (within 3 cm of where it started), the chassis back down on its block (within 2
    degrees and 2 cm of where it started), each gripper's TCP at least 4 cm from the tool's handle and from the
    chassis's handle, and both arms back where they started (each TCP within 5 cm of its start); 0.5 with the screw in
    the dish; 0.25 if the tool's magnet has picked the screw up at some point; else 0.0."""
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
    """Full score with the screw, the tool and the chassis still."""
    return (score(objects) >= 1.0 and objects["screw"].resting() and objects["wand"].resting()
            and objects["chassis"].resting())


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the chassis's handle, fingers closing across its bar, and lifts the left end
# along its arc until the chassis slopes 9 degrees down to the right: the screw slides out from under the board to
# the right end. The right hand takes the pick-up tool out of its holder by its handle, brings the magnet down on the
# screw, lifts it straight up clear of the walls, carries it over the parts dish and lowers it in, and the dish takes
# it; then it puts the tool back in its holder. The left lowers the chassis back onto its block and lets go, and both arms go back to where they started.

HOLD_GRIP = grip_rotation(0.0)  # left: straight down, fingers closing along y, across the handle's bar
HOLD_OPEN = -0.016
WAND_GRIP = grip_rotation(math.pi / 2)  # right: straight down, fingers closing along x, across the tool's handle
PICK_DZ = 0.0  # TCP aimed at the tool's origin when taking it (the arm stops ~8 mm high: 7 mm under the handle's top)
HANG_DZ = 0.015  # ... once held (the handle settles lower in the fingers)
PHI1 = math.radians(9.0)  # the chassis's slope while held up (right end lower)
CARRY_Z = TOP + 0.086  # the magnet's height while carrying the screw: clear of the chassis's walls
LEAD_S, LEAD_STEPS = 3.0, 15


def _pivot() -> np.ndarray:
    """Where the chassis turns when its left end is lifted: under its right feet's outer edges, on the block."""
    return ORIGIN + R0 @ np.array([0.0, -(FOOT_B + FOOT / 2), -FH])


def _tilted(p_world, phi) -> np.ndarray:
    """A point of the chassis, as it would be with the chassis turned about the pivot to slope ``phi``."""
    piv = _pivot()
    return piv + _rot_x(phi - PHI0) @ (np.asarray(p_world, float) - piv)


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


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    bar = ORIGIN + R0 @ np.array([0.0, LEN / 2 - T / 2, WALL_H + HANDLE_UP])
    out = _lead_in("left", bar + [0.0, 0.0, 0.05], HOLD_GRIP, "over the chassis's handle", HOLD_OPEN)
    out += [_w(1.0, bar, HOLD_OPEN, "down round its bar", HOLD_GRIP),
            _w(0.6, bar, FINGER_CLOSED, "take the handle", HOLD_GRIP)]
    steps = 5
    for k in range(1, steps + 1):
        out.append(_w(0.45, _tilted(bar, PHI0 + (PHI1 - PHI0) * k / steps), FINGER_CLOSED,
                      "lift the left end: the screw slides out to the right end", HOLD_GRIP))
    tilted_at = sum(w.duration for w in out)
    up = _tilted(bar, PHI1)
    out.append(_w(max(release_at - tilted_at, 0.3), up, FINGER_CLOSED, "hold the chassis tipped", HOLD_GRIP))
    for k in range(1, steps + 1):
        out.append(_w(0.35, _tilted(bar, PHI1 + (PHI0 - PHI1) * k / steps), FINGER_CLOSED,
                      "set it back down on its block", HOLD_GRIP))
    out += [_w(0.5, bar, FINGER_OPEN, "let go", HOLD_GRIP),
            _w(0.8, bar + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", HOLD_GRIP)]
    out += _go_home("left", bar + [0.0, 0.0, 0.05], HOLD_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, tilted_at


def _tool_tcp(tip) -> np.ndarray:
    """The right TCP that puts the magnet's face at ``tip`` (the tool hanging straight down)."""
    return np.asarray(tip, float) + [0.0, 0.0, TIP_DOWN + HANG_DZ]


def _right(tilted_at: float) -> tuple[list[Waypoint], float]:
    stand = np.array([WAND_AT[0], WAND_AT[1], WAND_Z])
    g = WAND_GRIP
    out = _lead_in("right", stand + [0.0, 0.0, PICK_DZ + 0.05], g, "over the pick-up tool's handle", FINGER_OPEN)
    out += [_w(0.8, stand + [0.0, 0.0, PICK_DZ], FINGER_OPEN, "down round the handle", g),
            _w(0.5, stand + [0.0, 0.0, PICK_DZ], FINGER_CLOSED, "take the tool", g),
            _w(1.0, stand + [0.0, 0.0, HANG_DZ + 0.075], FINGER_CLOSED, "lift it out of its holder", g)]
    t = sum(w.duration for w in out)
    wait = tilted_at + 1.0 - t
    if wait > 0:
        out.append(_w(wait, out[-1].pos, FINGER_CLOSED, "wait for the screw to slide out", g))
    screw = _tilted(ORIGIN + R0 @ np.array([0.0, -(LEN / 2 - T - 0.022), T + 0.004]), PHI1)  # 1.5 cm in from where it
    # lies against the right end (the magnet takes it within 2 cm; this keeps the magnet clear of the end wall)
    over = _tool_tcp(screw + [0.0, 0.0, 0.035])
    on = _tool_tcp(screw + [0.0, 0.0, 0.002])
    out += [_w(1.6, over, FINGER_CLOSED, "the magnet over the screw", g),
            _w(1.0, on, FINGER_CLOSED, "down onto it: the magnet takes it", g),
            _w(0.5, on, FINGER_CLOSED, "hold", g),
            _w(1.2, _tool_tcp(np.array([screw[0], screw[1], CARRY_Z])), FINGER_CLOSED,
               "lift it straight up, clear of the walls", g)]
    dish = np.array([DISH_AT[0], DISH_AT[1], TOP + 0.004 + 0.012])
    out += [_w(1.8, _tool_tcp(np.array([DISH_AT[0], DISH_AT[1], CARRY_Z])), FINGER_CLOSED, "over the parts dish", g),
            _w(0.8, _tool_tcp(dish), FINGER_CLOSED, "touch the screw into the dish: it takes it", g),
            _w(0.4, _tool_tcp(dish), FINGER_CLOSED, "hold", g)]
    out_at = sum(w.duration for w in out)
    out += [
            _w(0.7, _tool_tcp(dish + [0.0, 0.0, 0.06]), FINGER_CLOSED, "lift the tool away", g),
            _w(1.6, stand + [0.0, 0.0, HANG_DZ + 0.075], FINGER_CLOSED, "back over its holder", g),
            _w(0.8, stand + [0.0, 0.0, HANG_DZ + 0.075], FINGER_CLOSED, "steady", g),
            _w(1.6, stand + [0.0, 0.0, HANG_DZ + 0.003], FINGER_CLOSED, "put it back in its holder", g),
            _w(0.4, stand + [0.0, 0.0, HANG_DZ + 0.002], FINGER_OPEN, "let go", g),
            _w(0.6, stand + [0.0, 0.0, HANG_DZ + 0.05], FINGER_OPEN, "rise", g)]
    out += _go_home("right", stand + [0.0, 0.0, HANG_DZ + 0.05], g, FINGER_OPEN)
    return out, out_at


def demo():
    _, tilted_at = _left(0.0, 0.0)
    right, out_at = _right(tilted_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(out_at + 0.5, total)
    return {"left": left, "right": right}
