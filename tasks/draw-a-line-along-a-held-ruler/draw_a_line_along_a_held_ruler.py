"""On the station table lies a sheet of paper with a thick ruler across it, left to right. A felt-tip marker stands in
a holder beside the paper.

Task: draw a straight line along the ruler's near edge. One hand presses the ruler down on the paper; the other takes
the marker and draws with its tip riding against the ruler's edge, at least 12 cm, then lifts it off the paper. The
ruler is light: pushed against by the pen and not held, it slides away and the line goes astray.

Run it:  inspect-robots run --embodiment yam_arena -E task=draw_a_line_along_a_held_ruler.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the paper (fixed) and the ruler (loose), along y ------------------------------------------------------------------
PAPER = (0.21, 0.30, 0.0004)  # x, y, thick
PAPER_AT = (0.30, -0.02)
PAPER_TOP = TOP + PAPER[2]
RULER = (0.03, 0.30, 0.006)  # wide (x), long (y), thick
RULER_AT = (0.30, -0.03)
RULER_MASS = 0.025
EDGE_X = RULER_AT[0] - RULER[0] / 2  # the ruler's near edge (toward the arms)

# -- the marker: a felt tip under a thicker barrel; it stands in a holder ----------------------------------------------
TIP_R, TIP_L = 0.0025, 0.012
BARREL_R, BARREL_L = 0.007, 0.078
PEN_MASS = 0.012
HOLDER_AT = (0.20, -0.12)
HOLDER_R, HOLDER_H = 0.009, 0.02

# -- the ink: dots laid down where the tip touches the paper ----------------------------------------------------------
N_DOTS = 40
DOT_STEP = 0.0035  # a new dot each time the tip has moved this far on the paper
DOT_R = 0.0022  # overlapping: a continuous line
TOUCH = 0.0015  # the tip within this of the paper: it draws
HIDDEN = (PAPER_AT[0] + 0.09, PAPER_AT[1] + 0.13)  # where the unused dots wait, under the paper

# -- scoring ----------------------------------------------------------------------------------------------------------
LINE_X = EDGE_X - TIP_R  # where the tip's middle runs, riding the ruler's edge where the ruler lies
ALONG = 0.003  # every dot within this of that line
RIDES = 0.0007  # the tip touching the ruler's edge: within this of it
RIDE_SHARE = 0.8  # for at least this share of the dots
LONG = 0.12
LIFTED = 0.005
WHITE, WOOD, INK, DARK, BLUE = (0.95, 0.95, 0.93), (0.85, 0.72, 0.45), (0.05, 0.10, 0.45), (0.20, 0.21, 0.23), \
    (0.12, 0.25, 0.65)
SUCCESS_HOLD_S = 1.0


def _paper() -> Body:
    return Body.build("paper", pos=(PAPER_AT[0], PAPER_AT[1], TOP), mass=0.01, kinematic=True, color=WHITE,
                      shapes=[BoxShape((0.0, 0.0, PAPER[2] / 2), PAPER, WHITE)], friction=(0.4, 0.35))


def _ruler() -> Body:
    shapes = [BoxShape((0.0, 0.0, RULER[2] / 2), RULER, WOOD)]
    for k in range(31):  # the centimetre marks (show only)
        length = 0.008 if k % 5 == 0 else 0.004
        shapes.append(BoxShape((-RULER[0] / 2 + length / 2, -RULER[1] / 2 + k * 0.01, RULER[2] + 0.0001),
                               (length, 0.0006, 0.0002), DARK, collide=False))
    return Body.build("ruler", pos=(RULER_AT[0], RULER_AT[1], PAPER_TOP + 0.0002), mass=RULER_MASS, shapes=shapes,
                      friction=(0.25, 0.2), color=WOOD)


def _holder() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.002), HOLDER_R + 0.004, 0.004, DARK)]
    n = 10
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        r = HOLDER_R + 0.001
        shapes.append(plate((r * u[0], r * u[1], 0.004), (r * u[0], r * u[1], HOLDER_H), 2 * r * math.tan(math.pi / n) + 0.001,
                            0.002, u, DARK))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.1, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


def _pen() -> Body:
    """Origin at the tip's point; the pen stands upright."""
    shapes = [CylShape((0.0, 0.0, TIP_L / 2), TIP_R, TIP_L, INK),
              CylShape((0.0, 0.0, TIP_L + BARREL_L / 2), BARREL_R, BARREL_L, BLUE)]
    return Body.build("pen", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045), mass=PEN_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=BLUE, com=(0.0, 0.0, TIP_L + BARREL_L / 2))


def _dots() -> list[Body]:
    return [Body.build(f"ink_{k:02d}", pos=(HIDDEN[0], HIDDEN[1], TOP + 0.0001), mass=0.001, kinematic=True, color=INK,
                       shapes=[CylShape((0.0, 0.0, 0.0), DOT_R, 0.0002, INK, collide=False)]) for k in range(N_DOTS)]


PAPER_B, RULER_B, HOLDER, PEN = _paper(), _ruler(), _holder(), _pen()
DOTS = _dots()
DOT_NAMES = [d.name for d in DOTS]
OBJECTS = [PAPER_B, RULER_B, HOLDER, PEN, *DOTS]


# -- followed after every control step: the ink -------------------------------------------------------------------------

def _fresh() -> dict:
    return {"dots": [], "rides": [], "tip": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PEN_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _place(asset, origin, x: float, y: float) -> None:
    import torch

    pose = torch.tensor([[x + origin[0], y + origin[1], PAPER_TOP + 0.0002 + origin[2], 0.0, 0.0, 0.0, 1.0]],
                        dtype=torch.float32, device=asset.device)
    asset.write_root_pose_to_sim(pose)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["pen"].data.root_link_pose_w)[0]
    r = as_numpy(base.scene["ruler"].data.root_link_pose_w)[0]
    tip, r_pos, r_rot = p[:3] - origin, r[:3] - origin, rotation(r[3:7])
    _EP["tip"] = tip
    on_paper = tip[2] - PAPER_TOP < TOUCH
    if on_paper and len(_EP["dots"]) < N_DOTS:
        last = _EP["dots"][-1] if _EP["dots"] else None
        if last is None or math.hypot(tip[0] - last[0], tip[1] - last[1]) >= DOT_STEP:
            _place(base.scene[DOT_NAMES[len(_EP["dots"])]], origin, float(tip[0]), float(tip[1]))
            v = r_rot.T @ (tip - r_pos)  # the tip against the ruler's near edge (the ruler as it lies now)
            _EP["rides"].append(abs(-RULER[0] / 2 - v[0]) - TIP_R <= RIDES)
            _EP["dots"].append((float(tip[0]), float(tip[1])))
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            print(f"[pen] t={_DEBUG['steps'] / 20:5.1f}s tip={np.round(tip * 1000).astype(int).tolist()} "
                  f"ruler={np.round(r_pos * 1000).astype(int).tolist()} dots={len(_EP['dots'])} length={st['length'] * 1000:.0f}mm "
                  f"worst_off={st['worst'] * 1000:.1f}mm rides={st['ride_share']:.2f} lifted={st['lifted']} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    d = _EP["dots"]
    ys = [y for _, y in d]
    length = (max(ys) - min(ys)) if d else 0.0
    worst = max((abs(x - LINE_X) for x, _ in d), default=0.0)
    share = (sum(_EP["rides"]) / len(_EP["rides"])) if _EP["rides"] else 0.0
    tip = _EP["tip"]
    lifted = tip is not None and tip[2] - PAPER_TOP >= LIFTED
    return {"length": length, "worst": worst, "ride_share": share, "lifted": lifted}


def _points(st) -> float:
    straight = st["worst"] <= ALONG and st["ride_share"] >= RIDE_SHARE
    if st["length"] >= LONG and straight and st["lifted"]:
        return 1.0
    if st["length"] >= LONG or (straight and st["length"] >= LONG / 2):
        return 0.5
    return 0.25 if st["length"] >= 0.03 else 0.0


def score(objects) -> float:
    """1.0 for a line at least 12 cm long along the ruler's near edge where the ruler lies (every bit within 3 mm of
    it), drawn with the tip riding against the edge (for at least 80% of it), and the pen lifted off the paper; 0.5
    for a line at least 12 cm long, or one half as long that is along the edge; 0.25 for a line at least 3 cm long;
    else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the pen and the ruler still."""
    return score(objects) >= 1.0 and objects["pen"].resting() and objects["ruler"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the ruler's left end, fingers closed, and presses it onto the paper. The right
# hand comes straight down on the marker, fingers closing across its barrel, lifts it out of its holder and carries it
# to the right end of the ruler; it sets the tip on the paper against the ruler's near edge and draws along the edge
# to the left, 13 cm, pressing the tip lightly against the edge; then it lifts the marker off the paper.

GRIP_L = grip_rotation(0.0)  # straight down, fingers closed: it presses the ruler
GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y, across the barrel
PRESS = 0.012  # the left aims its fingertips this far into the ruler (the arm gives: it presses it down)
PRESS_Y = RULER_AT[1] + RULER[1] / 2 - 0.02  # where it presses, near the ruler's left end
PEN_IN_HAND = 0.02  # the right TCP this far below the barrel's top
TIP_DOWN = 0.002  # aim the tip this far into the paper (the arm gives: it presses lightly)
TIP_IN = 0.002  # ... and this far into the ruler's edge, so it rides against it
STROKE = (RULER_AT[1] - RULER[1] / 2 + 0.01, RULER_AT[1] - RULER[1] / 2 + 0.14)  # from y, to y
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
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


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


PEN_TCP = TIP_L + BARREL_L - PEN_IN_HAND  # the right TCP above the tip's point


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    press = np.array([RULER_AT[0], PRESS_Y, PAPER_TOP + RULER[2] + 0.013 - PRESS])
    out = _lead_in("left", press + [0.0, 0.0, 0.04 + PRESS], GRIP_L, "over the ruler's left end", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, PRESS], FINGER_CLOSED, "down onto it", GRIP_L),
            _w(0.8, press, FINGER_CLOSED, "press it onto the paper", GRIP_L)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), press, FINGER_CLOSED, "hold it while the line is drawn", GRIP_L),
            _w(1.0, press + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", GRIP_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), press + [0.0, 0.0, 0.05], FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", GRIP_L))
    return out, held_at


def _right(start: float) -> list[Waypoint]:
    pick = np.array([HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045 + PEN_TCP])
    x = LINE_X + TIP_IN
    a = np.array([x, STROKE[0], PAPER_TOP + PEN_TCP - TIP_DOWN])
    b = np.array([x, STROKE[1], PAPER_TOP + PEN_TCP - TIP_DOWN])
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], GRIP_R, "over the marker", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round its barrel", GRIP_R),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP_R),
            _w(1.0, pick + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it out of its holder", GRIP_R)]
    t = sum(w.duration for w in out)
    near = a  # (the arm comes down about 2.5 mm short of where it aims: the tip lands by the edge)
    out += [_w(max(start - t, 0.3) + 1.5, near + [0.0, 0.0, 0.03], FINGER_CLOSED, "over the ruler's right end", GRIP_R),
            _w(1.0, near, FINGER_CLOSED, "set the tip on the paper by the ruler's near edge", GRIP_R),
            _w(0.6, a, FINGER_CLOSED, "against the edge", GRIP_R),
            _w(4.0, b, FINGER_CLOSED, "draw along the edge, 13 cm", GRIP_R),
            _w(1.0, b + [-0.01, 0.0, 0.03], FINGER_CLOSED, "lift it off the paper", GRIP_R)]
    return out


def demo():
    probe, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "hold still", GRIP_R))
    left, _ = _left(t_r, total)
    return {"left": left, "right": right}
