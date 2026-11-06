"""On the station table lies a lacquered tray, smooth as glass, and on it a long wooden puzzle box. Four panels on the
box's top slide, each marked with white dots: three small tiles (one, two and three dots) and a lid (four dots) over a
hidden compartment at the box's right end, where a red block waits. Each panel is locked until the one before it (by
its dots) has been slid open; then it slides stiffly and stays where it is left.

Task: open the box and take the block out. The box is light and slides freely on the tray, so one hand holds it by its
left end while the other slides the panels open in order (the one-dot tile away from the arms, the two-dot tile toward
them, the three-dot tile away, then the lid off to the right), lifts the block out and sets it on the tray. Then both
hands let go and both arms go back to where they started.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_a_sliding_panel_puzzle_box.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the tray: smooth, so the box slides on it at the lightest push ----------------------------------------------------
TRAY_X, TRAY_Y, TRAY_T = (0.10, 0.56), (-0.40, 0.32), 0.004
TT = TOP + TRAY_T  # the tray's top

# -- the box: long along y, its left end held; a compartment at its right end under the lid ---------------------------
BX = 0.30  # its middle line
BOX_W, BOX_Y, BOX_H = 0.065, (-0.215, 0.135), 0.05  # across (x), its ends (y), its height
WALL, FLOOR = 0.005, 0.010
HOLD_Y = 0.11  # where the box is held: 15 cm from the first knob (two wrists closer than that collide)
SOLID_Y = -0.13  # solid from here to its left end; the compartment from here to its right end
BOX_MASS = 0.20
BOX_TOP = TT + BOX_H

# -- the panels: (name, dots, middle y, sliding axis, direction (+1/-1), travel m, open past m) -------------------------
TILE = (0.050, 0.028, 0.006)  # across (x), along (y), thick
LID_Y = (-0.215, -0.127)
POST = (0.012, 0.012, 0.025)  # the knob on each panel
PANELS = [("tile_1", 1, -0.045, "X", +1.0, 0.015, 0.010),
          ("tile_2", 2, -0.077, "X", -1.0, 0.015, 0.010),
          ("tile_3", 3, -0.109, "X", +1.0, 0.015, 0.010),
          ("lid", 4, (LID_Y[0] + LID_Y[1]) / 2, "Y", -1.0, 0.085, 0.070)]
LID_POST_Y = LID_Y[1] - 0.010
K_LOCK = 20000.0  # N/m: a locked panel barely gives
K_HOLD = 300.0  # N/m: an unlocked panel holds where it last stopped ...
SLIP = 0.002  # ... until pushed this far past it (then the task file moves its hold there)
DRAG = 4.0  # N s/m

# -- the block, in the compartment ---------------------------------------------------------------------------------
BLOCK = (0.025, 0.025, 0.036)
BLOCK_AT = np.array([BX, -0.170, TT + FLOOR + BLOCK[2] / 2])

WOOD, DARK_WOOD, DARK, WHITE, RED, LACQUER = (0.66, 0.45, 0.25), (0.42, 0.26, 0.13), (0.15, 0.12, 0.10), \
    (0.95, 0.95, 0.93), (0.80, 0.10, 0.08), (0.22, 0.24, 0.28)

# -- scoring ----------------------------------------------------------------------------------------------------------
OUT = 0.03  # the block out: on the tray in front of the box, its middle at least this far beyond the box's near side
LET_GO = 0.04  # each TCP at least this far from the block
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _tray() -> Body:
    return Body.build("tray", pos=((TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2, TOP), mass=2.0,
                      shapes=[BoxShape((0.0, 0.0, TRAY_T / 2), (TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], TRAY_T),
                                       LACQUER)],
                      kinematic=True, friction=(0.02, 0.02), slick=True, color=LACQUER)


def _box() -> Body:
    """The box; origin at its bottom middle, under the middle of its length."""
    yc = (BOX_Y[0] + BOX_Y[1]) / 2
    o = np.array([BX, yc, TT])

    def at(x, y, z):
        return tuple((np.array([x, y, z]) - o).tolist())

    cav = (BOX_Y[0] + WALL, SOLID_Y)  # the compartment, along y
    shapes = [BoxShape(at(BX, yc, TT + FLOOR / 2), (BOX_W, BOX_Y[1] - BOX_Y[0], FLOOR), WOOD),  # the floor
              BoxShape(at(BX, (SOLID_Y + BOX_Y[1]) / 2, TT + BOX_H / 2), (BOX_W, BOX_Y[1] - SOLID_Y, BOX_H), WOOD),
              BoxShape(at(BX, BOX_Y[0] + WALL / 2, TT + BOX_H / 2), (BOX_W, WALL, BOX_H), WOOD)]  # its right end
    for s in (-1.0, 1.0):  # the compartment's front and back walls
        shapes.append(BoxShape(at(BX + s * (BOX_W - WALL) / 2, (cav[0] + cav[1]) / 2, TT + BOX_H / 2),
                               (WALL, cav[1] - cav[0], BOX_H), WOOD))
    gap, ridge_h = 0.003, 0.005  # a low socket on the compartment's floor keeps the block in place
    for sx, sy in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        off = BLOCK[0] / 2 + gap + 0.002
        size = (0.004, BLOCK[1] + 2 * gap + 0.008, ridge_h) if sx else (BLOCK[0] + 2 * gap + 0.008, 0.004, ridge_h)
        shapes.append(BoxShape(at(BX + sx * off, BLOCK_AT[1] + sy * off, TT + FLOOR + ridge_h / 2), size, DARK_WOOD))
    for i in range(1, 6):  # grain lines on the left end, where it is held
        shapes.append(BoxShape(at(BX - BOX_W / 2 + i * BOX_W / 6, HOLD_Y, BOX_TOP + 0.0002), (0.0012, 0.045, 0.0004),
                               DARK_WOOD, collide=False))
    return Body.build("box", pos=tuple(o.tolist()), mass=BOX_MASS, shapes=shapes, friction=(1.0, 0.9), color=WOOD,
                      com=(0.0, 0.0, BOX_H / 2))


def _dots(n: int, y_off: float) -> list:
    """``n`` white dots in a row across the panel's top, on the arms' side of its knob."""
    return [CylShape(((i - (n - 1) / 2) * 0.008 - 0.019, y_off, TILE[2] + 0.0002), 0.0025, 0.0004, WHITE,
                     collide=False) for i in range(n)]


def _panel(name: str, dots: int, y: float, axis: str) -> Body:
    """A panel; origin at its bottom middle (on the box's top)."""
    if name == "lid":
        size = (BOX_W, LID_Y[1] - LID_Y[0], TILE[2])
        post_y = LID_POST_Y - y
        marks = [CylShape((xo, post_y - 0.02, TILE[2] + 0.0002), 0.0025, 0.0004, WHITE, collide=False)
                 for xo in (-0.012, -0.004, 0.004, 0.012)]
    else:
        size = TILE
        post_y = 0.0
        marks = _dots(dots, 0.0)
    shapes = [BoxShape((0.0, 0.0, size[2] / 2), size, DARK_WOOD),
              BoxShape((0.0, post_y, size[2] + POST[2] / 2), POST, DARK), *marks]
    return Body.build(name, pos=(BX, y, BOX_TOP), mass=0.02 if name != "lid" else 0.04, shapes=shapes,
                      friction=(1.0, 0.9), color=DARK_WOOD, contact_offset=0.001)


def _block() -> Body:
    return Body.build("block", pos=tuple(BLOCK_AT.tolist()), mass=0.03, friction=(1.0, 0.9), color=RED,
                      shapes=[BoxShape((0.0, 0.0, 0.0), BLOCK, RED)])


OBJECTS = [_tray(), _box(), *[_panel(n, d, y, a) for n, d, y, a, *_ in PANELS], _block()]
for _n, _d, _y, _a, _s, _travel, _open in PANELS:
    _lim = (0.0, _travel) if _s > 0 else (-_travel, 0.0)
    slider(f"{_n}_slide", "box", _n, anchor=(BX, _y, BOX_TOP), axis=_a, limits=_lim,
           stiffness=K_HOLD if _d == 1 else K_LOCK, damping=DRAG, target=0.0)  # only the first is free at the start


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"unlocked": [True, False, False, False], "hold": [0.0] * 4, "opened": [False] * 4, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PUZZLE_DEBUG")), "steps": 0}


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


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    b = as_numpy(base.scene["box"].data.root_link_pose_w)[0]
    b_pos, b_rot = b[:3] - origin, rotation(b[3:7])
    box0 = np.array([BX, (BOX_Y[0] + BOX_Y[1]) / 2, TT])
    moved = []
    for k, (name, _d, y, axis, sign, _travel, open_at) in enumerate(PANELS):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0][:3] - origin
        local = b_rot.T @ (p - b_pos) - (np.array([BX, y, BOX_TOP]) - box0)
        d = float(local["XYZ".index(axis)])
        moved.append(d)
        if _EP["unlocked"][k] and abs(d - _EP["hold"][k]) > SLIP:  # pushed past its hold: it slides, and stays
            _EP["hold"][k] = d
            set_slider_drive(f"{name}_slide", target=d)
        if not _EP["opened"][k] and sign * d >= open_at:
            _EP["opened"][k] = True
            if k + 1 < len(PANELS) and not _EP["unlocked"][k + 1]:  # open: the next panel is free to slide
                _EP["unlocked"][k + 1] = True
                set_slider_drive(f"{PANELS[k + 1][0]}_slide", stiffness=K_HOLD)
    blk = as_numpy(base.scene["block"].data.root_link_pose_w)[0][:3] - origin
    v = b_rot.T @ (blk - b_pos)  # the block in the box's frame
    outside = max(abs(float(v[0])) - BOX_W / 2, abs(float(v[1])) - (BOX_Y[1] - BOX_Y[0]) / 2)
    _EP["tcps"] = _tcps(base, origin)
    clear = all(float(np.linalg.norm(t - blk)) >= LET_GO for t in _EP["tcps"])
    on_tray = blk[2] < TT + max(BLOCK) / 2 + 0.006  # resting on the tray, whichever way up
    in_front = float(v[0]) <= -(BOX_W / 2 + OUT)  # toward the arms, beyond the box's near side
    _EP["s"] = {"opened": sum(1 for k in range(4) if all(_EP["opened"][:k + 1])), "out": in_front and on_tray,
                "clear": clear, "outside": outside, "moved": moved}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[puzzle] t={_DEBUG['steps'] / 20:5.1f}s moved(mm)={[round(m * 1000, 1) for m in moved]} "
                  f"opened={_EP['opened']} box={np.round(b_pos * 1000).astype(int).tolist()} "
                  f"block={np.round(blk * 1000).astype(int).tolist()} outside={outside * 1000:.0f}mm on_tray={on_tray} "
                  f"clear={clear} rightTCP={np.round(tr * 1000).astype(int).tolist()} "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    return {"opened": st.get("opened", 0), "out": bool(st.get("out")), "clear": bool(st.get("clear"))}


def _points(s) -> float:
    if s["opened"] == 4 and s["out"] and s["clear"]:
        return 1.0
    if s["opened"] == 4:
        return 0.5
    return 0.25 if s["opened"] >= 2 else 0.0


def score(objects) -> float:
    """1.0 with all four panels slid open in order (each at some point, after the one before it) and the block taken
    out (on the tray in front of the box, its middle at least 3 cm beyond the box's near side) and let go of, and both
    arms back where they started; 0.5 with all four panels open; 0.25 with the first two open; else 0.0."""
    pts = _points(_status())
    if pts >= 1.0 and not _arms_home():
        return 0.5  # open and the block out, but the arms not back
    return pts


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
    """Full score with the block and the box still: the trial ends there."""
    return score(objects) >= 1.0 and objects["block"].resting() and objects["box"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down over the box's left end, fingers open, and closes across it, holding the box. The
# right hand takes each panel's knob in turn (fingers closing along the way it slides) and slides it open: the one-dot
# tile away from the arms, the two-dot tile toward them, the three-dot tile away, and the lid 8.5 cm off to the right.
# Then it reaches into the compartment, takes the red block, lifts it out and sets it on the tray in front of the box.
# The left lets go of the box, and both arms go back to where they started.

def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


HOLD_GRIP = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x, across the box
HOLD_OPEN = -0.0375  # the left's fingers opened only about 1 cm wider than the box: closing, they barely shove it
X_GRIP = grip_rotation(math.pi / 2)  # straight down, fingers closing along x: for knobs that slide along x
Y_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y: for the lid's knob and the block
HOLD_IN = 0.012  # the left TCP this far below the box's top
KNOB_IN = 0.008  # the right TCP this far below a knob's top
OVERSHOOT = 0.015  # aim past each panel's stop (the arm gives as it pushes)
BLOCK_IN = 0.012
HALF_OPEN = -0.022  # the fingers opened only this far (about 4.4 cm apart): they fit in the compartment
PUT_AT = np.array([0.19, -0.17])  # where the block is set down, in front of the box
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
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
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


def _knob(k: int) -> np.ndarray:
    name, _d, y, *_ = PANELS[k]
    ky = LID_POST_Y if name == "lid" else y
    return np.array([BX, ky, BOX_TOP + TILE[2] + POST[2] - KNOB_IN])


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    hold = np.array([BX, HOLD_Y, BOX_TOP - HOLD_IN])
    out = _lead_in("left", hold + [0.0, 0.0, 0.05], HOLD_GRIP, "over the box's left end", HOLD_OPEN)
    out += [_w(1.0, hold, HOLD_OPEN, "down round it", HOLD_GRIP),
            _w(0.6, hold, FINGER_CLOSED, "hold the box", HOLD_GRIP)]
    held_at = sum(w.duration for w in out)
    up = hold + [0.0, 0.0, 0.06]
    out += [_w(max(release_at - held_at, 0.3), hold, FINGER_CLOSED, "hold it still while the panels are slid",
               HOLD_GRIP),
            _w(0.5, hold, FINGER_OPEN, "let go", HOLD_GRIP),
            _w(0.8, up, FINGER_OPEN, "rise", HOLD_GRIP)]
    out += _go_home("left", up, HOLD_GRIP, FINGER_OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    out = _lead_in("right", _knob(0) + [0.0, 0.0, 0.04], X_GRIP, "over the one-dot tile's knob", FINGER_OPEN)
    out.append(_w(max(start - LEAD_S, 0.3), _knob(0) + [0.0, 0.0, 0.04], FINGER_OPEN, "wait for the box to be held",
                  X_GRIP))
    for k, (name, dots, _y, axis, sign, travel, _open) in enumerate(PANELS):
        grip = Y_GRIP if axis == "Y" else X_GRIP
        knob = _knob(k)
        step = np.zeros(3)
        step["XYZ".index(axis)] = sign * (travel + OVERSHOOT)
        what = "the lid" if name == "lid" else f"the {dots}-dot tile"
        out += [_w(1.2, knob + [0.0, 0.0, 0.04], FINGER_OPEN, f"over {what}'s knob", grip),
                _w(0.7, knob, FINGER_OPEN, "down round it", grip),
                _w(0.5, knob, FINGER_CLOSED, "take it", grip),
                _w(2.5 if name == "lid" else 1.5, knob + step, FINGER_CLOSED, f"slide {what} open", grip),
                _w(0.4, knob + step, FINGER_OPEN, "let go", grip),
                _w(0.6, knob + step + [0.0, 0.0, 0.04], FINGER_OPEN, "rise", grip)]
    blk = BLOCK_AT + [0.0, 0.0, BLOCK[2] / 2 - BLOCK_IN]
    put = np.array([PUT_AT[0], PUT_AT[1], TT + BLOCK[2] - BLOCK_IN + 0.003])
    out += [_w(1.2, blk + [0.0, 0.0, 0.05], HALF_OPEN, "over the red block, fingers half open", Y_GRIP),
            _w(1.0, blk, HALF_OPEN, "down round it, into the compartment", Y_GRIP),
            _w(0.5, blk, FINGER_CLOSED, "take it", Y_GRIP)]
    taken_at = sum(w.duration for w in out)
    out += [_w(1.2, blk + [0.0, 0.0, 0.06], FINGER_CLOSED, "lift it out of the box", Y_GRIP),
            _w(1.5, put + [0.0, 0.0, 0.04], FINGER_CLOSED, "over the tray in front of the box", Y_GRIP),
            _w(0.8, put, FINGER_CLOSED, "set it down", Y_GRIP),
            _w(0.4, put, FINGER_OPEN, "let go", Y_GRIP),
            _w(0.8, put + [0.0, 0.0, 0.05], FINGER_OPEN, "rise", Y_GRIP)]
    out += _go_home("right", put + [0.0, 0.0, 0.05], Y_GRIP, FINGER_OPEN)
    return out, taken_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, taken_at = _right(held_at)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(taken_at + 1.5, total)
    return {"left": left, "right": right}
