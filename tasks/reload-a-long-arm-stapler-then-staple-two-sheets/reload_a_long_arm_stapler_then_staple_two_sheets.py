"""On the table stands a long-arm stapler (the kind used to staple booklets), empty, its nose to the left: a long grey
base with a paper rest and a paper guide by its nose, a black magazine housing over the front part of the base, and a
sprung top arm hinged on the housing's back end. Its staple magazine is a channel that slides out of the back of the
housing, toward the right arm, against a spring: pulled out, the channel's front part comes out into the open; let go,
it springs back in. On a tray to the left lies a fresh strip of staples; on the paper rest lie two sheets of paper.

Task: reload it, then staple the two sheets. One hand pulls the magazine out by its tab and holds it there; the other
lays the strip of staples in the open channel. The magazine is let slide back in, carrying the strip into the housing.
Then a hand slides the two sheets in under the stapler's head against the paper guide, presses the top arm down to
staple them together, pulls the stapled sheets back out from under the head, and both arms go back to where they
started. Let go of the magazine, and it springs shut before a strip can go in; an empty stapler staples nothing.

Run it:  inspect-robots run --embodiment yam_arena -E task=reload_a_long_arm_stapler_then_staple_two_sheets.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_glue_on, slider)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stapler (fixed on the table): a long base along y, its nose toward +y ----------------------------------------
SX = 0.30  # the stapler's middle line (x)
BASE_Y = (-0.19, 0.30)
BASE_W, BASE_H = 0.060, 0.015
REST_X, REST_Y = (0.12, SX - BASE_W / 2), (0.17, 0.30)  # the paper rest, flush with the base, on the arms' side
FENCE_X = 0.318  # the paper guide's face (a fence along y on the base, under the housing)
FENCE_H = 0.008
THROAT = 0.014  # the gap between the base's top and the housing's underside (the sheets go in there)
HB = BASE_H + THROAT  # the housing's underside (above the table)
HOUSE_Y = (0.04, 0.290)  # the magazine housing, from its open back end to its nose
HOUSE_W = 0.045
HOUSE_FLOOR, HOUSE_IN, HOUSE_SKIN = 0.003, 0.017, 0.003  # floor, inside height, walls and top
HOUSE_TOP = HB + HOUSE_FLOOR + HOUSE_IN + HOUSE_SKIN
STAPLE_AT = np.array([SX, 0.265])  # where a staple goes in (x, y): under the driver, over the anvil

# -- the top arm: hinged on the housing's back end, sprung up; pressed down 4 degrees it lies flat on the housing ------
PIVOT = np.array([SX, 0.050, TOP + HOUSE_TOP + 0.003])
LEVER_L, LEVER_W, LEVER_T = 0.232, 0.040, 0.010
LEVER_TILT = 4.0  # degrees, nose up, at rest
PAD_AT = 0.205  # the press pad's middle, along the arm from the pivot
STAPLE_PRESS = 3.0  # pressed at least this far (degrees) with a loaded, shut magazine and sheets under the head: stapled

# -- the magazine: a channel on a slide, sprung shut; its tab stands up at its back end ---------------------------------
CH_L = 0.22
CH_IN, CH_WALL, CH_H = 0.020, 0.003, 0.012
CH_FLOOR = 0.003
SHUT_FRONT = 0.262  # shut, the channel's front end (y); its tab then stands just behind the housing
MAG_Z = HB + HOUSE_FLOOR + 0.0005  # the magazine's underside (above the table)
STROKE = 0.20
SPRING, DRAG = 20.0, 6.0  # N/m (4 N pulled right out), N s/m
TAB = (0.012, 0.016, 0.032)

# -- the strip of staples on its tray, and the two sheets on the paper rest -------------------------------------------
STRIP = (0.012, 0.070, 0.008)
STRIP_MASS = 0.03
TRAY_AT = np.array([0.20, 0.09])
TRAY_T = 0.008
SHEET = (0.100, 0.070, 0.003)  # across (x), along (y), thick: stiff paper
SHEET_MASS = 0.008
TOP_SHEET_AT = np.array([0.215, 0.252])  # its middle (x, y), out from under the head
UNDER = np.array([-0.003, -0.003])  # the lower sheet lies this far offset, its edges showing

STEEL, DARK, BLACK, BASE_C, TRAY_C, RED, CHROME, PAPER, PAPER2, INK, REST_C = \
    (0.75, 0.76, 0.78), (0.20, 0.20, 0.22), (0.06, 0.06, 0.07), (0.34, 0.35, 0.37), (0.85, 0.80, 0.70), \
    (0.72, 0.10, 0.09), (0.86, 0.87, 0.89), (0.96, 0.96, 0.94), (0.97, 0.92, 0.70), (0.35, 0.40, 0.55), \
    (0.55, 0.56, 0.58)

# -- scoring ----------------------------------------------------------------------------------------------------------
SHUT = 0.005
IN_SIDE = 0.008
IN_LEVEL = 15.0
UNDER_MARGIN = 0.005  # a sheet is under the head with the staple point at least this far inside it
FLAT = 10.0  # degrees: a sheet lying flat
OUT_X = SX - HOUSE_W / 2 - 0.003  # out from under the head: each sheet's edge short of this (x)
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _quat_xyzw(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return (float((m[2, 1] - m[1, 2]) / (4 * w)), float((m[0, 2] - m[2, 0]) / (4 * w)),
            float((m[1, 0] - m[0, 1]) / (4 * w)), float(w))


def _stapler() -> Body:
    """Origin on the table under the middle line, at y = 0."""
    yb0, yb1 = BASE_Y
    yh0, yh1 = HOUSE_Y
    hl, hm = yh1 - yh0, (yh0 + yh1) / 2
    rx, ry = REST_X, REST_Y
    # the base's back part, and its front part with the paper rest as one plate (no seam for a sheet's edge to catch)
    shapes = [BoxShape((0.0, (yb0 + ry[0]) / 2, BASE_H / 2), (BASE_W, ry[0] - yb0, BASE_H), BASE_C),
              BoxShape(((rx[0] + SX + BASE_W / 2) / 2 - SX, (ry[0] + ry[1]) / 2, BASE_H / 2),
                       (SX + BASE_W / 2 - rx[0], ry[1] - ry[0], BASE_H), BASE_C),
              BoxShape(((rx[0] + rx[1]) / 2 - SX - 0.004, (ry[0] + ry[1]) / 2, BASE_H + 0.0002),
                       (rx[1] - rx[0] - 0.008, ry[1] - ry[0] - 0.008, 0.0004), REST_C, collide=False),
              BoxShape((FENCE_X + 0.003 - SX, (ry[0] + ry[1]) / 2, BASE_H + FENCE_H / 2), (0.006, ry[1] - ry[0],
                                                                                        FENCE_H), DARK),
              # the post under the housing's back end
              BoxShape((0.0, yh0 + 0.018, BASE_H + THROAT / 2), (HOUSE_W - 0.006, 0.036, THROAT), BLACK),
              # the housing: floor, two walls, top and a closed nose; its back end open for the magazine
              BoxShape((0.0, hm, HB + HOUSE_FLOOR / 2), (HOUSE_W, hl, HOUSE_FLOOR), BLACK),
              *[BoxShape((s * (HOUSE_W - HOUSE_SKIN) / 2, hm, HB + HOUSE_FLOOR + HOUSE_IN / 2),
                         (HOUSE_SKIN, hl, HOUSE_IN), BLACK) for s in (-1.0, 1.0)],
              BoxShape((0.0, hm, HOUSE_TOP - HOUSE_SKIN / 2), (HOUSE_W, hl, HOUSE_SKIN), BLACK),
              BoxShape((0.0, yh1 - 0.004, HB + HOUSE_FLOOR + HOUSE_IN / 2), (HOUSE_W, 0.008, HOUSE_IN), BLACK),
              # the hinge's cheeks at the back of the housing's top
              *[BoxShape((s * (HOUSE_W / 2 - 0.002), PIVOT[1], HOUSE_TOP + 0.003), (0.004, 0.016, 0.010), CHROME)
                for s in (-1.0, 1.0)],
              # markings: the anvil under the driver, a ruler along the paper guide, a label on the housing
              BoxShape((STAPLE_AT[0] - SX, STAPLE_AT[1], BASE_H + 0.0004), (0.010, 0.022, 0.0008), CHROME,
                       collide=False),
              *[BoxShape((FENCE_X - 0.006 - SX, ry[0] + 0.01 * (k + 1), BASE_H + 0.0003), (0.006 if k % 5 == 4 else
                                                                                         0.003, 0.0008, 0.0006),
                         BLACK, collide=False) for k in range(11)],
              BoxShape((0.0, hm - 0.04, HOUSE_TOP + 0.0004), (0.030, 0.06, 0.0008), DARK, collide=False)]
    return Body.build("stapler", pos=(SX, 0.0, TOP), mass=2.0, shapes=shapes, kinematic=True, color=BLACK,
                      friction=(0.12, 0.10), slick=True)


def _magazine() -> Body:
    """Origin at the middle of its floor's underside at its front end."""
    w = CH_IN + 2 * CH_WALL
    shapes = [BoxShape((0.0, -CH_L / 2, CH_FLOOR / 2), (w, CH_L, CH_FLOOR), STEEL),
              *[BoxShape((s * (CH_IN / 2 + CH_WALL / 2), -CH_L / 2, CH_FLOOR + CH_H / 2), (CH_WALL, CH_L, CH_H), STEEL)
                for s in (-1.0, 1.0)],
              BoxShape((0.0, -0.004, CH_FLOOR + CH_H / 2), (CH_IN, 0.008, CH_H), STEEL),  # its front stop
              BoxShape((0.0, -CH_L - TAB[1] / 2, TAB[2] / 2), TAB, RED)]  # the tab at its back end
    return Body.build("magazine", pos=(SX, SHUT_FRONT, TOP + MAG_Z), mass=0.08, shapes=shapes,
                      friction=(0.3, 0.2), color=STEEL, com=(0.0, -CH_L / 2, CH_FLOOR))


def _lever() -> Body:
    """The top arm, origin on the hinge line; it rises 4 degrees toward its nose."""
    r = _rx(LEVER_TILT)
    q = _quat_xyzw(r)

    def at(y, z):
        return tuple(float(v) for v in r @ np.array([0.0, y, z]))

    shapes = [BoxShape(at(LEVER_L / 2, LEVER_T / 2), (LEVER_W, LEVER_L, LEVER_T), BLACK, quat_xyzw=q),
              BoxShape(at(LEVER_L - 0.007, -0.004), (LEVER_W, 0.014, 0.018), BLACK, quat_xyzw=q),  # the head
              BoxShape(at(PAD_AT, LEVER_T + 0.003), (LEVER_W - 0.006, 0.050, 0.006), RED, quat_xyzw=q),  # the pad
              BoxShape(at(0.09, LEVER_T + 0.0005), (0.006, 0.08, 0.001), CHROME, quat_xyzw=q, collide=False),
              CylShape((0.0, 0.0, 0.0), 0.005, LEVER_W + 0.010, CHROME, axis="X")]
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=0.15, shapes=shapes, color=BLACK, friction=(0.8, 0.7))


def _tray() -> Body:
    x, y = TRAY_AT
    return Body.build("tray", pos=(float(x), float(y), TOP), mass=0.2, kinematic=True, color=TRAY_C,
                      friction=(0.6, 0.5), shapes=[BoxShape((0.0, 0.0, TRAY_T / 2), (0.06, 0.12, TRAY_T), TRAY_C)])


def _strip() -> Body:
    shapes = [BoxShape((0.0, 0.0, STRIP[2] / 2), STRIP, STEEL)]
    for k in range(13):  # the staples' crowns, shown
        shapes.append(BoxShape((0.0, -STRIP[1] / 2 + 0.005 * (k + 1), STRIP[2] + 0.0002), (STRIP[0], 0.0006, 0.0004),
                               DARK, collide=False))
    x, y = TRAY_AT
    return Body.build("strip", pos=(float(x), float(y), TOP + TRAY_T), mass=STRIP_MASS, shapes=shapes,
                      friction=(0.8, 0.7), color=STEEL)


def _sheet(name: str, xy, color, lines: bool) -> Body:
    """Origin at the sheet's middle."""
    shapes = [BoxShape((0.0, 0.0, 0.0), SHEET, color)]
    if lines:  # printed lines, shown
        for k in range(6):
            shapes.append(BoxShape((-0.035 + 0.012 * k, 0.0, SHEET[2] / 2 + 0.0002), (0.0012, 0.050, 0.0003), INK,
                                   collide=False))
    z = TOP + BASE_H + SHEET[2] / 2 + (SHEET[2] if name == "sheet_top" else 0.0) + 0.0003
    return Body.build(name, pos=(float(xy[0]), float(xy[1]), z), mass=SHEET_MASS, shapes=shapes, color=color,
                      friction=(0.9, 0.8), ignore=("pin",) if name == "sheet_top" else ())


LOW_AT = TOP_SHEET_AT + UNDER
SHEET_LOW = _sheet("sheet_low", LOW_AT, PAPER2, False)
# the staple: a body hidden inside the lower sheet, held to it; stapling moves it onto the top sheet over the staple
# point and holds the top sheet to it too
PIN_START = np.array(SHEET_LOW.pos)
PIN = Body.build("pin", pos=tuple(PIN_START.tolist()), mass=0.001, color=DARK, ignore=("sheet_low",),
                 shapes=[BoxShape((0.0, 0.0, 0.0), (0.002, 0.014, 0.0008), DARK)])

OBJECTS = [_stapler(), _magazine(), _lever(), _tray(), _strip(), SHEET_LOW, PIN,
           _sheet("sheet_top", TOP_SHEET_AT, PAPER, True)]
slider("magazine_slide", "stapler", "magazine", anchor=(SX, SHUT_FRONT, TOP + MAG_Z), axis="Y",
       limits=(-STROKE, 0.0), stiffness=SPRING, damping=DRAG, target=0.0, max_force=50.0)
# the spring holds the arm up against its own weight (it rests at its start, clear of its stops); pressed fully down it
# pushes back with about 3 N at the pad
hinge("press", "stapler", "lever", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(-LEVER_TILT - 0.5, 2.0),
      spring=0.12, damping=0.012, target_deg=1.35)
glue("pin_in", "sheet_low", "pin", anchor=tuple(PIN_START.tolist()))
glue("staple", "pin", "sheet_top", anchor=tuple(PIN_START.tolist()))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}, "stapled": False}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STAPLER_DEBUG")), "steps": 0}


def _quat_wxyz(m: np.ndarray):
    x, y, z, w = _quat_xyzw(m)
    return w, x, y, z


def _set_frames(name: str, pos0, rot0, pos1, rot1) -> None:
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[name]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos0]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(rot0)))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in pos1]))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(*_quat_wxyz(rot1)))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _set_frames("pin_in", np.zeros(3), np.eye(3), np.zeros(3), np.eye(3))  # the pin back inside the lower sheet
    set_glue_on("pin_in", False)
    set_glue_on("pin_in", True)
    set_glue_on("staple", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return t[:3] - origin, rotation(t[3:7])


def _inside(p_xy, pos, rot, margin: float) -> bool:
    """Is the point (x, y) over the sheet at least ``margin`` in from its edges?"""
    v = rot.T @ (np.array([p_xy[0], p_xy[1], pos[2]]) - pos)
    return abs(float(v[0])) <= SHEET[0] / 2 - margin and abs(float(v[1])) <= SHEET[1] / 2 - margin


def _tilt(rot) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def _staple(lo, hi) -> None:
    """Drive a staple through both sheets at the staple point: the pin moves onto the top sheet there, and both sheets
    are held to it as they lie now."""
    (pl, rl), (pt, rt) = lo, hi
    at = np.array([STAPLE_AT[0], STAPLE_AT[1], 0.0])
    at[2] = float((pt + rt @ np.array([0.0, 0.0, SHEET[2] / 2 + 0.0004]))[2])
    # the staple point on the top sheet's top face, straight above (x, y)
    v = rt.T @ (at - pt)
    v[2] = SHEET[2] / 2 + 0.0004
    pin = pt + rt @ v
    _set_frames("pin_in", rl.T @ (pin - pl), rl.T @ rt, np.zeros(3), np.eye(3))
    set_glue_on("pin_in", False)
    set_glue_on("pin_in", True)
    _set_frames("staple", np.zeros(3), np.eye(3), v, np.eye(3))
    set_glue_on("staple", True)
    _EP["stapled"] = True


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    m_pos, m_rot = _pose(base, origin, "magazine")
    s_pos, s_rot = _pose(base, origin, "strip")
    l_pos, l_rot = _pose(base, origin, "lever")
    lo, hi = _pose(base, origin, "sheet_low"), _pose(base, origin, "sheet_top")
    out = SHUT_FRONT - float(m_pos[1])
    v = m_rot.T @ (s_pos - m_pos)
    along = math.degrees(math.acos(min(1.0, abs(float((m_rot.T @ s_rot)[1, 1])))))
    in_ch = (abs(float(v[0])) <= IN_SIDE and -CH_L <= float(v[1]) <= 0.0 and float(v[2]) <= CH_FLOOR + CH_H
             and along <= IN_LEVEL)
    loaded = in_ch and out <= SHUT
    pressed = -math.degrees(math.atan2(float(l_rot[2, 1]), float(l_rot[1, 1])))  # pressed down from rest (degrees)
    rest_z = TOP + BASE_H + SHEET[2] / 2
    flat = [(_tilt(r) <= FLAT and abs(float(p[2]) - rest_z - k * SHEET[2]) <= 0.004) for k, (p, r) in enumerate((lo, hi))]
    under = all(_inside(STAPLE_AT, p, r, UNDER_MARGIN) for p, r in (lo, hi))
    if not _EP["stapled"] and loaded and pressed >= STAPLE_PRESS and under and all(flat):
        _staple(lo, hi)
    corners = [p + r @ np.array([sx * SHEET[0] / 2, sy * SHEET[1] / 2, 0.0])
               for p, r in (lo, hi) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]
    clear_of_head = all(float(c[0]) <= OUT_X for c in corners)
    _EP["tcps"] = _tcps(base, origin)
    tab = m_pos + m_rot @ np.array([0.0, -CH_L - TAB[1] / 2, TAB[2]])
    pad = l_pos + l_rot @ (_rx(LEVER_TILT) @ np.array([0.0, PAD_AT, LEVER_T + 0.006]))
    strip_top = s_pos + s_rot @ np.array([0.0, 0.0, STRIP[2]])
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (tab, strip_top, pad, hi[0]))
    _EP["s"] = {"out": out, "in": in_ch, "loaded": loaded, "pressed": pressed, "under": under, "flat": all(flat),
                "clear_of_head": clear_of_head, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[stapler] t={_DEBUG['steps'] / 20:5.1f}s out={out * 1000:6.1f}mm in={in_ch} pressed={pressed:4.1f} "
                  f"low={np.round(lo[0][:2] * 1000).astype(int).tolist()} top={np.round(hi[0] * 1000).astype(int).tolist()} "
                  f"tilt={_tilt(hi[1]):4.1f} under={under} flat={flat} stapled={_EP['stapled']} "
                  f"clear_head={clear_of_head} clear={clear} L={np.round(tl * 1000).astype(int).tolist()} "
                  f"R-tab={np.round((tr - tab) * 1000).astype(int).tolist()} pad={np.round(pad * 1000).astype(int).tolist()} "
                  f"score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    s = _EP["s"]
    return {"loaded": bool(s.get("loaded")), "stapled": bool(_EP.get("stapled")),
            "out": bool(s.get("clear_of_head")) and bool(s.get("flat")), "clear": bool(s.get("clear")),
            "home": _arms_home()}


def _points(s) -> float:
    if s["loaded"] and s["stapled"] and s["out"] and s["clear"] and s["home"]:
        return 1.0
    if s["loaded"] and s["stapled"]:
        return 0.5
    return 0.25 if s["loaded"] else 0.0


def score(objects) -> float:
    """1.0 with the stapler loaded (the strip of staples lying in the magazine's channel and the magazine slid back
    shut), the two sheets stapled together, both sheets pulled back out from under the head (every corner of each short
    of the housing) and lying flat on the paper rest, both hands let go (each TCP at least 4 cm from the magazine's tab,
    the strip, the arm's press pad and the top sheet) and both arms back where they started (each TCP within 5 cm of its
    start); 0.5 with the stapler loaded and the sheets stapled together; 0.25 with the stapler loaded; else 0.0. A
    staple goes in only when the arm is pressed down with the stapler loaded and both sheets lying flat on the base with
    the staple point at least 5 mm inside each."""
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
    """Full score with the magazine, the strip, the arm and both sheets still."""
    names = ("magazine", "strip", "lever", "sheet_low", "sheet_top")
    full = score(objects) >= 1.0
    if full and _DEBUG["on"] and any(not objects[n].resting() for n in names):
        print("[stapler] moving:", [(n, round(float(np.linalg.norm(objects[n].lin_vel)), 4),
                                     round(float(np.linalg.norm(objects[n].ang_vel)), 3)) for n in names
                                    if not objects[n].resting()], file=sys.stderr, flush=True)
    return full and all(objects[n].resting() for n in names)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the magazine's tab, fingers closing across it, pulls the magazine out 20 cm and
# holds it there. The left hand takes the strip of staples off its tray, carries it over the open channel, sets it down
# on the channel's floor, lets go and rises. The right lets the magazine slide back in slowly, carrying the strip into
# the housing, lets go of the tab and goes back to where it started. The left presses its closed fingertips on the top
# sheet near its back edge and slides both sheets in under the head against the paper guide, presses the arm's pad down
# (the staple goes in), presses on the sheets again and slides them back out onto the paper rest, and goes back to where
# it started.

ACROSS = grip_rotation(math.pi / 2)  # right: straight down, fingers closing along x
ACROSS_L = grip_rotation(-math.pi / 2)  # left: the same
ALONG_L = grip_rotation(0.0)  # left: straight down, fingers closing along y (narrow along x)
TAB_OPEN = -0.016
STRIP_OPEN = -0.016
TAB_IN = -0.004
STRIP_IN = -0.006
TIP = 0.0105  # the closed fingertips reach this far below the TCP
PRESS_IN = 0.003  # pressing on the sheets: the fingertips this far into them
PULL = STROKE - 0.004
LEAD_S, LEAD_STEPS = 3.0, 15
SLIDE_IN = 0.058  # the sheets slid in this far (they stop at the paper guide after 5.3 cm)
SLIDE_OUT = 0.068


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


def _joint_path(arm: str, q0, q1, label: str, finger, secs: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(secs / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _tab_tcp(out: float) -> np.ndarray:
    return np.array([SX, SHUT_FRONT - CH_L - TAB[1] / 2 - out, TOP + MAG_Z + TAB[2] - TAB_IN])


def _right(dropped_at: float) -> tuple[list[Waypoint], float]:
    tab = _tab_tcp(0.0)
    out = _lead_in("right", tab + [0.0, 0.0, 0.05], ACROSS, "over the magazine's tab", TAB_OPEN)
    out += [_w(1.0, tab, TAB_OPEN, "down round it", ACROSS),
            _w(0.6, tab, FINGER_CLOSED, "take it", ACROSS),
            _w(2.5, _tab_tcp(PULL), FINGER_CLOSED, "pull the magazine out", ACROSS)]
    pulled = sum(w.duration for w in out)
    out.append(_w(max(dropped_at + 0.6 - pulled, 0.5), _tab_tcp(PULL), FINGER_CLOSED,
                  "hold it out while the staples go in", ACROSS))
    out += [_w(3.0, _tab_tcp(0.004), FINGER_CLOSED, "let it slide back in, carrying the staples into the housing",
               ACROSS),
            _w(0.6, _tab_tcp(0.0), TAB_OPEN, "let go", ACROSS),
            _w(0.8, _tab_tcp(0.0) + [0.0, 0.0, 0.05], TAB_OPEN, "up", ACROSS)]
    out += _go_home("right", _tab_tcp(0.0) + [0.0, 0.0, 0.05], ACROSS, TAB_OPEN)
    return out, pulled


def _press_point(x: float) -> np.ndarray:
    """The left TCP pressing its closed fingertips into the top sheet at x (on the sheet's middle line along y)."""
    top = TOP + BASE_H + 2 * SHEET[2]
    return np.array([x, TOP_SHEET_AT[1], top + TIP - PRESS_IN])


def _pad_tcp(pressed_deg: float) -> np.ndarray:
    """The left TCP on the arm's press pad with the arm pressed ``pressed_deg`` down from rest."""
    p = PIVOT + _rx(LEVER_TILT - pressed_deg) @ np.array([0.0, PAD_AT, LEVER_T + 0.006])
    return p + np.array([0.0, 0.0, TIP])


def _left(pulled_at: float, back_at: float, total: float) -> tuple[list[Waypoint], float]:
    x, y = TRAY_AT
    pick = np.array([float(x), float(y), TOP + TRAY_T + STRIP[2] - STRIP_IN])
    out = _lead_in("left", pick + [0.0, 0.0, 0.05], ACROSS_L, "over the strip of staples", STRIP_OPEN)
    out += [_w(1.0, pick, STRIP_OPEN, "down round it", ACROSS_L),
            _w(0.6, pick, FINGER_CLOSED, "take it", ACROSS_L),
            _w(1.0, pick + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift it", ACROSS_L)]
    floor = TOP + MAG_Z + CH_FLOOR
    drop = np.array([SX, -0.005, floor + STRIP[2] - STRIP_IN + 0.0005])
    t = sum(w.duration for w in out)
    out.append(_w(max(pulled_at + 0.3 - t, 1.5), drop + [0.0, 0.0, 0.05], FINGER_CLOSED, "over the open channel",
                  ACROSS_L))
    out += [_w(1.2, drop, FINGER_CLOSED, "lower it into the channel, onto its floor", ACROSS_L),
            _w(0.5, drop, STRIP_OPEN, "let go", ACROSS_L)]
    dropped = sum(w.duration for w in out)
    out.append(_w(0.8, drop + [0.0, 0.0, 0.06], STRIP_OPEN, "up", ACROSS_L))
    # wait clear of the magazine until it is back in, then the sheets
    x0 = TOP_SHEET_AT[0] - SHEET[0] / 2 + 0.020
    p_in = _press_point(x0)
    out.append(_w(2.0, p_in + [0.0, 0.0, 0.06], FINGER_CLOSED, "over the top sheet's back edge", ALONG_L))
    t = sum(w.duration for w in out)
    out.append(_w(max(back_at - t, 0.5), p_in + [0.0, 0.0, 0.06], FINGER_CLOSED, "wait for the magazine", ALONG_L))
    out += [_w(1.0, p_in, FINGER_CLOSED, "press on the sheets", ALONG_L),
            _w(2.5, p_in + [SLIDE_IN, 0.0, 0.0], FINGER_CLOSED, "slide them in under the head, to the paper guide",
               ALONG_L),
            _w(0.8, p_in + [SLIDE_IN, 0.0, 0.05], FINGER_CLOSED, "up", ALONG_L)]
    pad0 = _pad_tcp(0.0)
    out += [_w(1.5, pad0 + [0.0, 0.0, 0.04], FINGER_CLOSED, "over the arm's press pad", ALONG_L),
            _w(0.8, pad0, FINGER_CLOSED, "onto the pad", ALONG_L),
            _w(1.2, _pad_tcp(LEVER_TILT + 1.5), FINGER_CLOSED, "press the arm down: the staple goes in", ALONG_L),
            _w(0.5, _pad_tcp(LEVER_TILT + 1.5), FINGER_CLOSED, "hold it down", ALONG_L),
            _w(1.0, pad0 + [0.0, 0.0, 0.04], FINGER_CLOSED, "let it spring back up", ALONG_L)]
    p_out = _press_point(x0 + SLIDE_IN - 0.005 + 0.002)
    out += [_w(1.5, p_out + [0.0, 0.0, 0.05], FINGER_CLOSED, "over the stapled sheets", ALONG_L),
            _w(1.0, p_out, FINGER_CLOSED, "press on them", ALONG_L),
            _w(2.5, p_out - [SLIDE_OUT, 0.0, 0.0], FINGER_CLOSED, "slide them back out onto the paper rest", ALONG_L),
            _w(0.8, p_out - [SLIDE_OUT, 0.0, -0.06], FINGER_CLOSED, "up", ALONG_L)]
    out += _go_home("left", p_out - [SLIDE_OUT, 0.0, -0.06], ALONG_L, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(Waypoint(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                        "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, dropped


def demo():
    _, pulled = _right(0.0)
    _, dropped = _left(pulled, 0.0, 0.0)
    right, _ = _right(dropped)
    back = sum(w.duration for w in right[:-LEAD_STEPS - 2])  # the magazine back in and the tab let go
    left, _ = _left(pulled, back, 0.0)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    right.append(Waypoint(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, TAB_OPEN,
                          "wait at the start", right[-1].yaw))
    left, _ = _left(pulled, back, total)
    return {"left": left, "right": right}
