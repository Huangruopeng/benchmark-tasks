"""On the table stands the air intake of a machine (a fixed housing, like the one on a robot controller or a server),
its dust filter standing upright in a slot under a hinged cover. The cover is sprung shut; pressing down the pedal on
its tail, which sticks out beyond the cover's hinge, swings the cover up. Beside it stand a waste bin and, in a holder,
a new filter (white); the old one in the slot is grey with dust.

Task: swap the filter. One hand presses the cover's pedal down and holds it, keeping the cover open; the other lifts the
dirty filter out by its tab, drops it in the bin, takes the new filter from its holder and lowers it into the slot. Then
the pedal is let go, the cover springs shut over the new filter, and both arms go back to where they started. Let go of
the pedal while a filter is half in or out, and the cover shuts on it.

Run it:  inspect-robots run --embodiment yam_arena -E task=swap_the_dust_filter_under_a_spring_shut_cover.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, hinge, on_reset, on_step, rotation)  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the housing (fixed): a box with an opening in its top over the filter slot ----------------------------------------
HX = (0.18, 0.42)  # along x
HY = (-0.21, 0.025)  # along y; the cover's hinge runs along its left top edge
H_TOP = 0.10  # its top, above the table
OPEN_X, OPEN_Y = (0.200, 0.400), (-0.190, -0.070)  # the opening in its top (wide enough for a hand)
SLOT_Y = -0.130  # the filter slot's middle line (far from the hinge: the open cover leans over toward it) (the filter stands across y = SLOT_Y, along x)
FLOOR = 0.015  # the slot's floor, above the table
RAIL_GAP = 0.024  # each end rail: a channel this wide (the filter is 1.2 cm thick)
WALL = 0.006

# -- the filters: upright panels along x, 1.2 cm thick, with a tab on top in the middle -----------------------------------
FILTER = (0.110, 0.012, 0.035)
TAB = (0.012, 0.030, 0.035)  # a T across the filter's top: thin along the filter, so fingers close on it along x
FILTER_MASS = 0.06
SEATED_Z = TOP + FLOOR  # a seated filter's underside
NEW_AT = np.array([0.37, -0.26])  # the new filter's holder (its middle)
BIN_AT, BIN_IN = np.array([0.20, -0.27]), (0.13, 0.10, 0.07)  # the bin's middle; inside: along x, y, its height

# -- the cover: a plate over the housing's top, hinged along x at its left edge; its tail (a pedal) sticks out past
# the hinge to the left. Pressing the pedal down swings the cover up; a spring shuts it --------------------------------
HINGE_Y = HY[1]
COVER_W = HINGE_Y - HY[0] + 0.005  # from the hinge to its free (right) edge
TAIL = 0.090  # the pedal's middle, past the hinge, on a tail ...
TAIL_UP = 15.0  # ... rising this steeply from the hinge (pressed down, it swings the cover up)
PAD_T = 0.008
COVER_T = 0.006
SHUT_K = 0.003  # N m per degree
OPEN_MAX = 88.0
SHUT = 5.0  # within this of shut

GREY, HOUSE_C, DARK, DUST, WHITE, BIN_C, RUBBER, HOLD_C = (0.55, 0.56, 0.58), (0.80, 0.81, 0.82), \
    (0.18, 0.18, 0.20), (0.42, 0.40, 0.36), (0.96, 0.96, 0.94), (0.20, 0.35, 0.55), (0.08, 0.08, 0.08), \
    (0.35, 0.35, 0.38)

# -- scoring ----------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.010  # a seated filter: its underside within 1 cm of the slot's floor, in the slot, upright
UPRIGHT = 12.0
LET_GO = 0.04
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _rails(cx: float, cy: float, floor: float, top: float, color) -> list:
    """Two end rails (channels) holding a filter upright: at each end of a filter centred at (cx, cy), two plates
    either side of a RAIL_GAP channel, from ``floor`` to ``top`` (all in the body's frame)."""
    out = []
    for sx in (-1.0, 1.0):
        x = cx + sx * (FILTER[0] / 2 + 0.002)
        for sy in (-1.0, 1.0):
            out.append(BoxShape((x, cy + sy * (RAIL_GAP / 2 + WALL / 2), (floor + top) / 2), (0.016, WALL, top - floor),
                                color))
        out.append(BoxShape((x + sx * 0.008 + sx * WALL / 2, cy, (floor + top) / 2), (WALL, RAIL_GAP + 2 * WALL,
                                                                                       top - floor), color))
    return out


def _housing() -> Body:
    """Origin at the middle of its footprint on the table."""
    cx, cy = (HX[0] + HX[1]) / 2, (HY[0] + HY[1]) / 2
    o = np.array([cx, cy, TOP])

    def box(x0, x1, y0, y1, z0, z1, color=HOUSE_C, collide=True):
        return BoxShape(((x0 + x1) / 2 - cx, (y0 + y1) / 2 - cy, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color,
                        collide=collide)

    t = 0.008  # the top's thickness
    shapes = [box(HX[0], HX[1], HY[0], HY[1], 0.0, FLOOR),  # the body below the slot
              # the walls round the slot's space, up to the top
              box(HX[0], HX[1], HY[0], OPEN_Y[0], FLOOR, H_TOP), box(HX[0], HX[1], OPEN_Y[1], HY[1], FLOOR, H_TOP),
              box(HX[0], OPEN_X[0], OPEN_Y[0], OPEN_Y[1], FLOOR, H_TOP),
              box(OPEN_X[1], HX[1], OPEN_Y[0], OPEN_Y[1], FLOOR, H_TOP),
              box(OPEN_X[0] + 0.004, OPEN_X[1] - 0.004, OPEN_Y[0] + 0.002, OPEN_Y[1] - 0.002, FLOOR - 0.001, FLOOR,
                  DARK, collide=False),
              *_rails(float(np.mean(OPEN_X)) - cx, SLOT_Y - cy, FLOOR, FLOOR + FILTER[2], DARK)]  # as tall as a filter
    for k in range(5):  # the intake's grille on its front face
        z = 0.02 + 0.015 * k
        shapes.append(box(HX[0] - 0.0005, HX[0], HY[0] + 0.02, HY[1] - 0.02, z, z + 0.006, DARK, collide=False))
    return Body.build("housing", pos=tuple(o.tolist()), mass=3.0, shapes=shapes, kinematic=True, color=HOUSE_C,
                      friction=(0.5, 0.4))


def _cover() -> Body:
    """The cover and its pedal, one body; origin on the hinge line, at the middle of the housing's length."""
    cx = (HX[0] + HX[1]) / 2
    b = math.radians(TAIL_UP)
    cb, sb = math.cos(b), math.sin(b)
    q = (math.sin(b / 2), 0.0, 0.0, math.cos(b / 2))  # turned TAIL_UP about +x
    shapes = [BoxShape((0.0, -COVER_W / 2, COVER_T / 2), (HX[1] - HX[0] + 0.006, COVER_W, COVER_T), GREY),
              BoxShape((0.0, (TAIL + 0.02) / 2 * cb - 0.01 * cb, COVER_T / 2 + (TAIL + 0.02) / 2 * sb - 0.01 * sb),
                       (0.05, TAIL + 0.02, COVER_T), GREY, quat_xyzw=q),  # the tail ...
              BoxShape((0.0, TAIL * cb - (COVER_T / 2 + PAD_T / 2) * sb, COVER_T / 2 + TAIL * sb
                        + (COVER_T / 2 + PAD_T / 2) * cb), (0.05, 0.035, PAD_T), RUBBER, quat_xyzw=q),  # ... its pedal
              BoxShape((0.0, -COVER_W + 0.008, COVER_T + 0.0003), (0.10, 0.006, 0.0006), DARK, collide=False)]
    return Body.build("cover", pos=(cx, HINGE_Y, TOP + H_TOP), mass=0.12, shapes=shapes, friction=(0.8, 0.7),
                      color=GREY, com=(0.0, -0.05, COVER_T / 2))


def _filter(name: str, at, color) -> Body:
    """A filter: a pleated panel in a frame, its tab on top; origin at the middle of its underside."""
    shapes = [BoxShape((0.0, 0.0, FILTER[2] / 2), FILTER, color),
              BoxShape((0.0, 0.0, FILTER[2] + TAB[2] / 2), TAB, DARK)]
    for k in range(9):  # pleats, shown
        x = -FILTER[0] / 2 + 0.011 * (k + 1)
        for s in (-1.0, 1.0):
            shapes.append(BoxShape((x, s * (FILTER[1] / 2 + 0.0002), FILTER[2] / 2), (0.002, 0.0004, FILTER[2] - 0.006),
                                   DARK if color == WHITE else (0.30, 0.28, 0.25), collide=False))
    return Body.build(name, pos=tuple(at), mass=FILTER_MASS, shapes=shapes, friction=(1.0, 0.9), color=color,
                      com=(0.0, 0.0, FILTER[2] / 2))


def _holder() -> Body:
    """The new filter's holder: a block with end rails; origin on the table under the filter's middle."""
    base = 0.010
    return Body.build("holder", pos=(float(NEW_AT[0]), float(NEW_AT[1]), TOP), mass=0.5, kinematic=True, color=HOLD_C,
                      friction=(0.5, 0.4), shapes=[BoxShape((0.0, 0.0, base / 2), (0.15, 0.05, base), HOLD_C),
                                                   *_rails(0.0, 0.0, base, base + 0.03, HOLD_C)])


def _bin() -> Body:
    x, y = float(BIN_AT[0]), float(BIN_AT[1])
    lx, ly, h = BIN_IN
    w = 0.004
    shapes = [BoxShape((0.0, 0.0, w / 2), (lx + 2 * w, ly + 2 * w, w), BIN_C)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * (lx / 2 + w / 2), 0.0, h / 2), (w, ly + 2 * w, h), BIN_C))
        shapes.append(BoxShape((0.0, s * (ly / 2 + w / 2), h / 2), (lx, w, h), BIN_C))
    return Body.build("bin", pos=(x, y, TOP), mass=0.4, kinematic=True, color=BIN_C, friction=(0.5, 0.4), shapes=shapes)


_SLOT = np.array([float(np.mean(OPEN_X)), SLOT_Y])
OBJECTS = [_housing(), _cover(), _holder(), _bin(), _filter("old_filter", (*_SLOT, SEATED_Z), DUST),
           _filter("new_filter", (*NEW_AT, TOP + 0.010), WHITE)]
hinge("cover_hinge", "housing", "cover", anchor=(float(np.mean(HX)), HINGE_Y, TOP + H_TOP), axis="X",
      limits_deg=(-OPEN_MAX, 0.5), spring=SHUT_K, damping=0.0005, target_deg=0.0)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FILTER_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _opened(rot) -> float:
    """How far the cover is swung up (deg): its free edge (toward -y) rising turns it the negative way about +x."""
    return -math.degrees(math.atan2(float(rot[2, 1]), float(rot[1, 1])))


def _seated(pos, rot) -> bool:
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))
    return (abs(float(pos[0]) - _SLOT[0]) <= 0.012 and abs(float(pos[1]) - _SLOT[1]) <= 0.010
            and float(pos[2]) - SEATED_Z <= SEAT_TOL and tilt <= UPRIGHT)


def _in_bin(pos) -> bool:
    return (abs(float(pos[0]) - BIN_AT[0]) <= BIN_IN[0] / 2 and abs(float(pos[1]) - BIN_AT[1]) <= BIN_IN[1] / 2
            and float(pos[2]) < TOP + BIN_IN[2])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    st = {}
    for n in ("cover", "old_filter", "new_filter"):
        a = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        st[n] = (a[:3] - origin, rotation(a[3:7]))
    opened = _opened(st["cover"][1])
    _EP["tcps"] = _tcps(base, origin)
    c_pos, c_rot = st["cover"]
    b = math.radians(TAIL_UP)
    pedal = c_pos + c_rot @ np.array([0.0, TAIL * math.cos(b), COVER_T / 2 + TAIL * math.sin(b)])
    tabs = [p + r @ np.array([0.0, 0.0, FILTER[2] + TAB[2] / 2]) for p, r in (st["old_filter"], st["new_filter"])]
    clear = all(float(np.linalg.norm(t - q)) >= LET_GO for t in _EP["tcps"] for q in (pedal, *tabs))
    _EP["s"] = {"opened": opened, "new_in": _seated(*st["new_filter"]), "old_in_bin": _in_bin(st["old_filter"][0]),
                "old_in": _seated(*st["old_filter"]), "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            o, n = st["old_filter"][0], st["new_filter"][0]
            print(f"[filter] t={_DEBUG['steps'] / 20:5.1f}s cover={opened:5.1f}deg old={np.round(o * 1000).astype(int).tolist()} "
                  f"new={np.round(n * 1000).astype(int).tolist()} new_in={_EP['s']['new_in']} "
                  f"old_in_bin={_EP['s']['old_in_bin']} clear={clear} L-pedal={np.round((tl - pedal) * 1000).astype(int).tolist()} "
                  f"R-oldtab={np.round((tr - tabs[0]) * 1000).astype(int).tolist()} "
                  f"R-newtab={np.round((tr - tabs[1]) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    s = _EP["s"]
    return {"new_in": bool(s.get("new_in")), "old_in_bin": bool(s.get("old_in_bin")),
            "shut": abs(s.get("opened", 90.0)) <= SHUT, "clear": bool(s.get("clear")), "home": _arms_home()}


def _points(s) -> float:
    if s["new_in"] and s["old_in_bin"] and s["shut"] and s["clear"] and s["home"]:
        return 1.0
    if s["new_in"] and s["old_in_bin"]:
        return 0.5
    return 0.25 if s["old_in_bin"] else 0.0


def score(objects) -> float:
    """1.0 with the new filter seated in the slot (its underside within 1 cm of the slot's floor, upright within 12
    degrees), the old one in the bin, the cover shut (within 5 degrees) and both hands let go (each TCP at least 4 cm
    from the pedal and from both filters' tabs) and both arms back where they started (each TCP within 5 cm of its
    start); 0.5 with the new filter seated and the old one in the bin; 0.25 with
    the old one in the bin; else 0.0."""
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
    """Full score with the cover and both filters still."""
    still = all(objects[n].resting() for n in ("cover", "old_filter", "new_filter"))
    return score(objects) >= 1.0 and still


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down on the cover's pedal, fingers closed, and presses it down along its swing: the cover swings
# up 75 degrees. The right hand comes straight down on the dirty filter's tab, fingers closing across it, lifts the
# filter out of the slot, carries it over the bin and lets it drop in; takes the new filter by its tab out of its
# holder, carries it over the slot and lowers it in, lets go and rises. The left lets the pedal up: the cover springs
# shut. Both arms go back to where they started.

GRIP = grip_rotation(-math.pi / 2)  # left: straight down, fingers closed, closing along x (narrow along y: the
# housing is close)
RGRIP = grip_rotation(math.pi / 2)  # right: straight down, fingers closing along x, across a tab (narrow along y,
# clear of the open cover)
TIP = 0.013  # closed fingertips below the TCP
TAB_OPEN = -0.016  # fingers opened ~3.2 cm round the 1.2 cm tab
IN_TAB = -0.004  # the right TCP this far below the tab's top (above it: a V-jaw pinches a plate ~9 mm deep at its tips)
PRESS_TO = 80.0  # the cover swings up ~70 degrees (the pedal slips a little); the slot is far enough from the hinge
PRESS = 0.030  # aimed this far into the pedal
LIFT_TO = TOP + H_TOP + 0.012  # a held filter's underside this high to clear the housing
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


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, GRIP), label, finger)


def _go_home(arm: str, pos, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, GRIP), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot=GRIP) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _lead_in_rot(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home_rot(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _pedal(deg: float, extra: float = 0.0) -> np.ndarray:
    """The left TCP with its closed fingertips on the middle of the pedal's top, the cover swung up ``deg``; ``extra``
    aims that far into the pedal (along its swing)."""
    b = math.radians(TAIL_UP - deg)  # the tail's slope now
    r, h = TAIL, COVER_T / 2 + PAD_T
    y = HINGE_Y + r * math.cos(b) - h * math.sin(b)
    z = TOP + H_TOP + COVER_T / 2 + r * math.sin(b) + h * math.cos(b)
    push = np.array([0.0, math.sin(b), -math.cos(b)])  # the way the pedal moves as it is pressed
    return np.array([float(np.mean(HX)), y, z + TIP]) + extra * push


def _tab_tcp(underside) -> np.ndarray:
    """The right TCP holding a filter by its tab, the filter's underside middle at ``underside``."""
    return np.asarray(underside, float) + [0.0, 0.0, FILTER[2] + TAB[2] - IN_TAB]


def _left(open_from: float, shut_at: float, total: float) -> tuple[list[Waypoint], float]:
    out = _lead_in("left", _pedal(0.0) + [0.0, 0.03, 0.01], "beside the cover's pedal", FINGER_CLOSED)
    out.append(_w(0.8, _pedal(0.0), FINGER_CLOSED, "fingertips on the pedal"))
    t = sum(w.duration for w in out)
    if open_from > t:
        out.append(_w(open_from - t, _pedal(0.0), FINGER_CLOSED, "wait"))
    for k in range(1, 9):
        out.append(_w(0.4, _pedal(PRESS_TO * k / 8, PRESS), FINGER_CLOSED, "press the pedal down: the cover swings up"))
    opened = sum(w.duration for w in out)
    out.append(_w(max(shut_at - opened, 0.5), _pedal(PRESS_TO, PRESS), FINGER_CLOSED,
                  "hold it down while the filter is changed"))
    for k in range(1, 5):
        out.append(_w(0.3, _pedal(PRESS_TO * (1 - k / 4)) + [0.0, 0.0, 0.004 * k], FINGER_CLOSED,
                      "let the pedal up: the cover springs shut"))
    up = _pedal(0.0) + [0.0, 0.03, 0.01]
    out.append(_w(0.6, up, FINGER_CLOSED, "up"))
    out += _go_home("left", up, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(Waypoint(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                        "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, opened


def _right(opened_at: float) -> tuple[list[Waypoint], float]:
    old = _tab_tcp((*_SLOT, SEATED_Z))
    aside = old + [0.0, -0.13, 0.05]  # clear of the cover's free edge as it swings up
    out = _lead_in_rot("right", aside, RGRIP, "beside the cover", TAB_OPEN)
    t = sum(w.duration for w in out)
    out += [_w(max(opened_at + 0.5 - t, 0.3), aside, TAB_OPEN, "wait for the cover to swing up", RGRIP),
            _w(1.0, old + [0.0, 0.0, 0.04], TAB_OPEN, "over the dirty filter's tab", RGRIP)]
    up = _tab_tcp((*_SLOT, LIFT_TO))
    over_bin = _tab_tcp((*BIN_AT, LIFT_TO))
    out += [_w(1.0, old, TAB_OPEN, "down round its tab", RGRIP),
            _w(0.6, old, FINGER_CLOSED, "take it", RGRIP),
            _w(1.8, up, FINGER_CLOSED, "lift it straight out of the slot", RGRIP),
            _w(1.8, over_bin, FINGER_CLOSED, "carry it over the bin", RGRIP),
            _w(0.6, over_bin, FINGER_OPEN, "let it drop in", RGRIP)]
    new = _tab_tcp((*NEW_AT, TOP + 0.010))
    new_up = _tab_tcp((*NEW_AT, LIFT_TO))
    out += [_w(1.5, new_up + [0.0, 0.0, 0.0], TAB_OPEN, "over the new filter", RGRIP),
            _w(1.0, new, TAB_OPEN, "down round its tab", RGRIP),
            _w(0.6, new, FINGER_CLOSED, "take it", RGRIP),
            _w(1.4, new_up, FINGER_CLOSED, "lift it out of its holder", RGRIP),
            _w(1.8, up, FINGER_CLOSED, "carry it over the slot", RGRIP),
            _w(1.8, old + [0.0, 0.0, 0.001], FINGER_CLOSED, "lower it into the slot, down to its floor", RGRIP),
            _w(0.6, old + [0.0, 0.0, 0.001], TAB_OPEN, "let go", RGRIP)]
    rise = old + [0.0, 0.0, 0.06]
    out.append(_w(0.8, rise, TAB_OPEN, "up, out of the way", RGRIP))
    done = sum(w.duration for w in out)
    out += _go_home_rot("right", rise, RGRIP, TAB_OPEN)
    return out, done


def demo():
    _, opened = _left(0.0, 0.0, 0.0)
    _, done = _right(opened)
    right, _ = _right(opened)
    total = max(sum(w.duration for w in right), done + 0.3 + 1.2 + 0.6 + LEAD_S) + 3.0
    left, _ = _left(0.0, done + 0.3, total)
    right.append(Waypoint(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, FINGER_OPEN,
                          "wait at the start", right[-1].yaw))
    return {"left": left, "right": right}
