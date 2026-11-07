"""A low partition crosses the station table from left to right, with a gateway in it closed by a small spring-hinged
gate. The gate opens toward the arms and swings shut on its own when let go (a closing spring in its hinge); a round
knob on its top edge, near its free end, turns freely in place so it can be held while the gate swings. Beyond the
partition a conveyor belt, flush with the table, runs to the right, ending on a steel run-off plate. A cardboard parcel
waits on the arms' side of the partition.

Task: get the parcel through the gateway onto the running belt without the gate touching it. One hand pulls the gate
open by its knob and holds it wide open; the other slides the parcel along the table through the gateway onto the
belt, which carries it away to the right onto the run-off plate, then draws back out of the gateway. The first hand
lets the gate swing back shut, gently, and both arms go back to where they started. The parcel stays on the table:
lifted over the partition, it does not count.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=hold_a_self_closing_gate_open_and_push_a_parcel_onto_a_conveyor.py \\
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
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the partition, the gate ----------------------------------------------------------------------------------------
WX, WALL_T, WALL_H = 0.36, 0.012, 0.11  # the partition's middle plane (x), thickness, height
HINGE_Y, LATCH_Y = 0.18, -0.012  # the gateway's hinge post and latch post (y)
WALL_Y = (-0.24, 0.26)  # the partition's ends
GATE_W, GATE_H, GATE_T = HINGE_Y - LATCH_Y - 0.008, 0.10, 0.010  # the gate: from the hinge to its free end
GATE_Z0 = TOP + 0.005  # its bottom edge, clear of the table
GATE_MASS = 0.25
SPRING, DRAG = 0.004, 0.0002  # N m per degree toward shut; N m s per degree
OPEN_MAX = 100.0  # its stop, wide open (degrees, toward the arms)
HINGE_AT = np.array([WX, HINGE_Y, GATE_Z0])
KNOB_R, KNOB_H, KNOB_ALONG = 0.007, 0.030, 0.135  # a round knob on the gate's top edge, this far from the hinge
KNOB_AT = np.array([WX, HINGE_Y - KNOB_ALONG, GATE_Z0 + GATE_H + KNOB_H / 2])

# -- the parcel -----------------------------------------------------------------------------------------------------
PARCEL = (0.08, 0.08, 0.06)
PARCEL_AT = np.array([0.125, 0.05, TOP + PARCEL[2] / 2])  # clear of the gate's swing
PARCEL_MASS = 0.15

# -- the conveyor beyond the partition: a belt flush with the table, running to the right (-y), and a run-off plate ------
BELT_UP = 0.002  # the belt's top stands this far above the table, with a ramp up to it from the partition
RAMP = (WX + WALL_T / 2, WX + WALL_T / 2 + 0.020)  # the ramp, from the partition's far face up to the belt
BELT_X = (RAMP[1], 0.500)  # across the belt
BELT_Y = (-0.180, 0.220)  # the belt runs from its start (+y) to its end (-y)
PLATE_Y = (-0.300, BELT_Y[0])  # the steel run-off plate past the belt's end, with a stop at its end
BELT_V = 0.10  # m/s
MARK_STEP = 0.030  # the belt's cross ribs, this far apart, shown running
DT = 0.05  # one control step at 20 Hz

WOOD, WHITE, STEEL, DARK, CARD, TAPE = (0.62, 0.50, 0.36), (0.88, 0.88, 0.86), (0.66, 0.68, 0.70), \
    (0.12, 0.12, 0.13), (0.72, 0.56, 0.36), (0.85, 0.78, 0.55)

# -- scoring ----------------------------------------------------------------------------------------------------------
TOUCH = 0.003  # the gate within this of the parcel: touching it
LIFTED = 0.02  # the parcel's bottom this far above the table: lifted (it must slide through, not over)
SHUT = 5.0  # the gate within this many degrees of shut
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _partition() -> Body:
    o = np.array([WX, 0.0, TOP])

    def at(x, y, z):
        return (x - o[0], y - o[1], z - o[2])

    shapes = [BoxShape(at(WX, (HINGE_Y + 0.012 + WALL_Y[1]) / 2, TOP + WALL_H / 2),
                       (WALL_T, WALL_Y[1] - HINGE_Y - 0.012, WALL_H), WHITE),
              BoxShape(at(WX, (WALL_Y[0] + LATCH_Y - 0.006) / 2, TOP + WALL_H / 2),
                       (WALL_T, LATCH_Y - 0.006 - WALL_Y[0], WALL_H), WHITE),
              CylShape(at(WX, HINGE_Y, TOP + (WALL_H + 0.01) / 2), 0.011, WALL_H + 0.01, STEEL),  # the hinge post
              BoxShape(at(WX, LATCH_Y, TOP + (WALL_H + 0.01) / 2), (0.016, 0.012, WALL_H + 0.01), STEEL)]  # latch post
    for y in (WALL_Y[0] + 0.01, WALL_Y[1] - 0.01):  # feet, so it stands
        shapes.append(BoxShape(at(WX - WALL_T / 2 - 0.02, y, TOP + 0.004), (0.04, 0.02, 0.008), WHITE))
    return Body.build("partition", pos=tuple(o.tolist()), mass=5.0, shapes=shapes, kinematic=True, color=WHITE)


def _gate() -> Body:
    """The gate, origin at the foot of its hinge line; shut, it spans the gateway toward -y."""
    shapes = [BoxShape((0.0, -GATE_W / 2 - 0.004, GATE_H / 2), (GATE_T, GATE_W, GATE_H), WOOD),
              BoxShape((0.0, -GATE_W / 2 - 0.004, GATE_H - 0.008), (GATE_T + 0.004, GATE_W, 0.016), (0.48, 0.38, 0.26)),
              BoxShape((0.0, -0.004, GATE_H / 2), (GATE_T + 0.006, 0.012, GATE_H), STEEL)]  # the hinge leaf
    for z in (0.025, 0.075):  # the hinge's knuckles
        shapes.append(CylShape((0.0, 0.0, z), 0.013, 0.02, STEEL, collide=False))
    return Body.build("gate", pos=tuple(HINGE_AT.tolist()), mass=GATE_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=WOOD, com=(0.0, -GATE_W / 2, GATE_H / 2))


def _knob() -> Body:
    return Body.build("knob", pos=tuple(KNOB_AT.tolist()), mass=0.02, color=DARK, friction=(1.0, 0.9),
                      shapes=[CylShape((0.0, 0.0, 0.0), KNOB_R, KNOB_H, DARK)])


def _parcel() -> Body:
    shapes = [BoxShape((0.0, 0.0, 0.0), PARCEL, CARD),
              BoxShape((0.0, 0.0, PARCEL[2] / 2 + 0.0004), (PARCEL[0] + 0.001, 0.02, 0.0008), TAPE, collide=False),
              BoxShape((0.022, -0.018, PARCEL[2] / 2 + 0.0006), (0.025, 0.03, 0.0008), WHITE, collide=False)]  # label
    return Body.build("parcel", pos=tuple(PARCEL_AT.tolist()), mass=PARCEL_MASS, shapes=shapes, friction=(0.5, 0.45),
                      color=CARD)


RUBBER, ALU, BELT_C = (0.10, 0.10, 0.11), (0.70, 0.72, 0.75), (0.16, 0.16, 0.17)


def _conveyor() -> Body:
    """The belt and its frame (fixed), origin on the table at the belt's middle; the belt's top stands 2 mm above the
    table (the frame is set into it), with a shallow ramp up to it from the partition, and is slick, so the belt alone
    moves what lies on it."""
    x0, x1 = BELT_X
    xm, w = (x0 + x1) / 2, x1 - x0
    ym, ln = (BELT_Y[0] + BELT_Y[1]) / 2, BELT_Y[1] - BELT_Y[0]
    o = np.array([xm, ym, TOP])

    def at(x, y, z):
        return (x - o[0], y - o[1], z - o[2])

    a = math.atan2(BELT_UP + 0.0003, RAMP[1] - RAMP[0])  # the ramp's slope
    rx, rz = (RAMP[0] + RAMP[1]) / 2 + 0.004 * math.sin(a), TOP + (BELT_UP - 0.0003) / 2 - 0.004 * math.cos(a)
    shapes = [BoxShape(at(xm, ym, TOP + BELT_UP - 0.005), (w, ln, 0.010), BELT_C),
              BoxShape(at(rx, ym, rz), ((RAMP[1] - RAMP[0]) / math.cos(a), ln, 0.008), ALU,
                       quat_xyzw=(0.0, -math.sin(a / 2), 0.0, math.cos(a / 2))),
              # the far side rail (the near side is open, for parcels pushed on from the gateway)
              BoxShape(at(x1 + 0.006, (PLATE_Y[0] + BELT_Y[1]) / 2, TOP + 0.015), (0.012, BELT_Y[1] - PLATE_Y[0], 0.030),
                       ALU),
              # the drums at both ends, shown as steel covers flush with the belt
              *[BoxShape(at(xm, y, TOP + BELT_UP + 0.0006), (w, 0.024, 0.0012), ALU, collide=False)
                for y in (BELT_Y[1] - 0.012, BELT_Y[0] + 0.012)],
              # a motor box on the frame at the belt's start
              BoxShape(at(x1 + 0.03, BELT_Y[1] - 0.03, TOP + 0.025), (0.04, 0.06, 0.05), (0.20, 0.35, 0.60)),
              BoxShape(at(x1 + 0.03, BELT_Y[1] - 0.03, TOP + 0.0505), (0.02, 0.02, 0.001), (0.15, 0.75, 0.25),
                       collide=False)]  # its green "running" light
    return Body.build("conveyor", pos=tuple(o.tolist()), mass=10.0, shapes=shapes, kinematic=True, color=BELT_C,
                      friction=(0.02, 0.02), slick=True)


def _plate() -> Body:
    x0, x1 = BELT_X
    xm, w = (x0 + x1) / 2, x1 - x0
    ym, ln = (PLATE_Y[0] + PLATE_Y[1]) / 2, PLATE_Y[1] - PLATE_Y[0]
    shapes = [BoxShape((0.0, 0.0, BELT_UP - 0.005), (w, ln, 0.010), STEEL),
              BoxShape((0.0, -ln / 2 - 0.006, 0.015), (w + 0.024, 0.012, 0.030), ALU)]  # the stop at its end
    return Body.build("runoff", pos=(xm, ym, TOP), mass=5.0, shapes=shapes, kinematic=True, color=STEEL,
                      friction=(0.5, 0.45))


def _marks() -> Body:
    """The belt's cross ribs (shown only), moved along with the belt every control step."""
    x0, x1 = BELT_X
    n = int((BELT_Y[1] - BELT_Y[0] - 0.048) / MARK_STEP)
    shapes = [BoxShape(((x0 + x1) / 2 - (x0 + x1) / 2, -k * MARK_STEP, 0.0), (x1 - x0 - 0.004, 0.005, 0.0004),
                       RUBBER, collide=False) for k in range(n)]
    return Body.build("belt_ribs", pos=((x0 + x1) / 2, BELT_Y[1] - 0.024, TOP + BELT_UP + 0.0002), mass=0.1, shapes=shapes,
                      kinematic=True, color=RUBBER)


MARKS = _marks()
OBJECTS = [_partition(), _gate(), _knob(), _conveyor(), _plate(), MARKS, _parcel()]
# the gate swings toward the arms (negative turn about z) and a spring pulls it shut; the knob turns freely on its top
hinge("gate_hinge", "partition", "gate", anchor=tuple(HINGE_AT.tolist()), axis="Z", limits_deg=(-OPEN_MAX, 0.0),
      spring=SPRING, damping=DRAG, target_deg=0.0)
hinge("knob_spin", "gate", "knob", anchor=tuple(KNOB_AT.tolist()), axis="Z")


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"touched": False, "lifted": False, "tcps": None, "s": {}, "ribs": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GATE_DEBUG")), "steps": 0}


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


def _seg_dist(p, q, a, b) -> float:
    """The shortest distance between segments pq and ab (2D)."""
    def pt_seg(x, s0, s1):
        d = s1 - s0
        t = float(np.clip((x - s0) @ d / max(float(d @ d), 1e-12), 0.0, 1.0))
        return float(np.linalg.norm(x - (s0 + t * d)))

    def cross(u, v):
        return float(u[0] * v[1] - u[1] * v[0])

    r, s = q - p, b - a
    den = cross(r, s)
    if abs(den) > 1e-12:
        t, u = cross(a - p, s) / den, cross(a - p, r) / den
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


def _gap(gate_rot, box_pos, box_rot) -> float:
    """How far the gate's face is from the parcel (seen from above; both reach from the table to above the
    parcel's top): the distance from the gate's middle line to the parcel's outline, less half the gate's thickness;
    negative or 0 when they overlap."""
    g0 = HINGE_AT[:2]
    g1 = g0 + (gate_rot @ np.array([0.0, -(GATE_W + 0.008), 0.0]))[:2]
    hx, hy = PARCEL[0] / 2, PARCEL[1] / 2
    corners = [box_pos[:2] + (box_rot @ np.array([sx * hx, sy * hy, 0.0]))[:2]
               for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    local = [box_rot.T @ np.array([*(c - box_pos[:2]), 0.0]) for c in (g0, g1)]
    if any(abs(float(v[0])) <= hx and abs(float(v[1])) <= hy for v in local):
        return -1.0
    d = min(_seg_dist(g0, g1, corners[k], corners[(k + 1) % 4]) for k in range(4))
    return d - GATE_T / 2


def _run_belt(base, origin, b_pos, corners) -> bool:
    """The belt carries a parcel lying on it (entirely past the partition, over the belt, on the table) to the right at
    BELT_V; once it is off the belt's end it stops on the run-off plate. Its ribs move along with it, shown."""
    import torch

    _EP["ribs"] = (_EP["ribs"] + BELT_V * DT) % MARK_STEP
    ribs = base.scene["belt_ribs"]
    p = np.array(MARKS.pos) + origin - np.array([0.0, _EP["ribs"], 0.0])
    ribs.write_root_pose_to_sim(torch.tensor([[*p.tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                                             device=ribs.device))
    on = (float(corners[:, 0].min()) > WX + WALL_T / 2 + 0.015 and BELT_X[0] <= float(b_pos[0]) <= BELT_X[1]
          and float(corners[:, 1].max()) > BELT_Y[0] and float(b_pos[1]) < BELT_Y[1]
          and float(corners[:, 2].min()) < TOP + BELT_UP + 0.01)
    if on:
        parcel = base.scene["parcel"]
        parcel.write_root_velocity_to_sim(torch.tensor([[0.0, -BELT_V, 0.0, 0.0, 0.0, 0.0]], dtype=torch.float32,
                                                       device=parcel.device))
    return on


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    g = as_numpy(base.scene["gate"].data.root_link_pose_w)[0]
    g_rot = rotation(g[3:7])
    turn = math.degrees(math.atan2(g_rot[1, 0], g_rot[0, 0]))  # negative: open toward the arms
    b = as_numpy(base.scene["parcel"].data.root_link_pose_w)[0]
    b_pos, b_rot = b[:3] - origin, rotation(b[3:7])
    gap = _gap(g_rot, b_pos, b_rot)
    if gap < TOUCH:
        _EP["touched"] = True
    corners = np.array([b_pos + b_rot @ (np.array([sx, sy, sz]) * np.array(PARCEL) / 2)
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    if float(corners[:, 2].min()) > TOP + LIFTED:
        _EP["lifted"] = True
    through = float(corners[:, 0].min()) > WX + WALL_T / 2 + 0.003
    half = float(b_pos[0]) > WX
    on_belt = _run_belt(base, origin, b_pos, corners)
    delivered = (through and float(corners[:, 1].max()) <= BELT_Y[0] + 0.005 and float(corners[:, 1].min()) >= PLATE_Y[0]
                 - 0.005 and BELT_X[0] <= float(b_pos[0]) <= BELT_X[1] and float(b_rot[2, 2]) > math.cos(math.radians(15)))
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"turn": turn, "through": through, "half": half, "delivered": delivered, "on_belt": on_belt}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            k = as_numpy(base.scene["knob"].data.root_link_pose_w)[0][:3] - origin
            print(f"[gate] t={_DEBUG['steps'] / 20:5.1f}s turn={turn:6.1f} gap={gap * 1000:6.1f}mm "
                  f"touched={_EP['touched']} lifted={_EP['lifted']} parcel={np.round(b_pos * 1000).astype(int).tolist()} "
                  f"through={through} belt={on_belt} delivered={delivered} knob={np.round(k * 1000).astype(int).tolist()} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"home={_arms_home()} score={_points(_status())}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[arm])[0], float)
                         for arm in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def _status() -> dict:
    st = _EP["s"]
    return {"touched": _EP["touched"], "lifted": _EP["lifted"], "through": bool(st.get("through")),
            "half": bool(st.get("half")), "delivered": bool(st.get("delivered")),
            "shut": abs(st.get("turn", 0.0)) <= SHUT, "home": _arms_home()}


def _points(s) -> float:
    if s["touched"] or s["lifted"]:
        return 0.0
    if s["delivered"] and s["shut"] and s["home"]:
        return 1.0
    if s["through"]:
        return 0.5
    return 0.25 if s["half"] else 0.0


def score(objects) -> float:
    """0.0 if the gate ever touched the parcel (came within 3 mm of it) or the parcel was ever lifted off the table
    (its bottom more than 2 cm up). Otherwise, at the end: 1.0 with the parcel carried off the belt's end onto the
    run-off plate (all of it past the belt, upright within 15 degrees), the gate shut (within 5 degrees) and both arms
    back where they started; 0.5 with the parcel entirely beyond the partition; 0.25 with its middle beyond the
    partition's middle; else 0.0. The belt carries a parcel that lies on it, entirely beyond the partition, to the
    right at 10 cm/s."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the parcel and the gate still."""
    return score(objects) >= 1.0 and objects["parcel"].resting() and objects["gate"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes in low over the gate's knob, fingers open, and closes on it; it pulls the knob along the gate's
# swing until the gate stands wide open (95 degrees) and holds it there. The right comes straight down behind the
# parcel, fingers closed, and slides it along the table through the gateway onto the belt, which carries it away to
# the right onto the run-off plate; the right draws back out of the gateway and goes back to where it started. The left lets the gate swing back shut along its
# swing, lets go of the knob, and goes back to where it started too.

KNOB_GRIP = grip_rotation(0.0)  # straight down, fingers closing along y: round the knob
PUSH_GRIP = grip_rotation(0.0)  # straight down, fingers closed: a flat side facing +-x
KNOB_IN = 0.010  # the left TCP this far below the knob's top
OPEN_TO = 95.0
SWING_STEPS, SWING_S = 10, 0.5
CLEAR = 0.022  # the right TCP this far behind the parcel's near face when pushing
PUSH_Z = TOP + 0.025
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


def _knob_at(deg: float) -> np.ndarray:
    """The left TCP on the knob with the gate open ``deg`` degrees."""
    a = math.radians(-deg)
    along = np.array([math.sin(a), -math.cos(a), 0.0])
    return np.array([WX, HINGE_Y, KNOB_AT[2] + KNOB_H / 2 - KNOB_IN]) + along * KNOB_ALONG


def _left(hold_until: float) -> tuple[list[Waypoint], float]:
    k0 = _knob_at(0.0)
    out = _lead_in("left", k0 + [-0.035, 0.0, 0.0], KNOB_GRIP, "level with the gate's knob, in front of it",
                   FINGER_OPEN)
    out += [_w(1.0, k0, FINGER_OPEN, "fingers round the knob", KNOB_GRIP),
            _w(0.6, k0, FINGER_CLOSED, "take it", KNOB_GRIP)]
    for k in range(1, SWING_STEPS + 1):
        out.append(_w(SWING_S, _knob_at(OPEN_TO * k / SWING_STEPS), FINGER_CLOSED,
                      "pull the gate open along its swing", KNOB_GRIP))
    opened = sum(w.duration for w in out)
    out.append(_w(max(hold_until - opened, 0.5), _knob_at(OPEN_TO), FINGER_CLOSED,
                  "hold it wide open while the parcel goes through", KNOB_GRIP))
    for k in range(1, SWING_STEPS + 1):
        out.append(_w(SWING_S, _knob_at(OPEN_TO * (1 - k / SWING_STEPS)), FINGER_CLOSED,
                      "let it swing back shut, gently", KNOB_GRIP))
    out += [_w(0.5, k0, FINGER_OPEN, "let go of the knob", KNOB_GRIP),
            _w(0.8, k0 + [-0.04, 0.0, 0.0], FINGER_OPEN, "draw back", KNOB_GRIP)]
    out += _go_home("left", k0 + [-0.04, 0.0, 0.0], KNOB_GRIP, FINGER_OPEN)
    return out, opened


def _right(start: float) -> tuple[list[Waypoint], float]:
    near = PARCEL_AT[0] - PARCEL[0] / 2
    p0 = np.array([near - CLEAR - 0.01, PARCEL_AT[1] - 0.01, PUSH_Z])  # a little right of its middle: it tends to veer left
    out = _lead_in("right", p0 + [0.0, 0.0, 0.05], PUSH_GRIP, "over the near side of the parcel", FINGER_CLOSED)
    out.append(_w(max(start - LEAD_S, 0.3), p0 + [0.0, 0.0, 0.05], FINGER_CLOSED, "wait for the gate to be held open",
                  PUSH_GRIP))
    out += [_w(1.0, p0, FINGER_CLOSED, "down behind the parcel", PUSH_GRIP)]
    end_x = WX + WALL_T / 2 + 0.035 - CLEAR  # the parcel's near face 3.5 cm past the partition
    steps = 12
    for k in range(1, steps + 1):
        out.append(_w(0.55, [p0[0] + (end_x - p0[0]) * k / steps, p0[1], p0[2]], FINGER_CLOSED,
                      "slide it along the table through the gateway, onto the belt", PUSH_GRIP))
    back = np.array([0.24, PARCEL_AT[1], PUSH_Z + 0.04])
    out += [_w(0.4, [end_x - 0.01, p0[1], p0[2]], FINGER_CLOSED, "ease off", PUSH_GRIP),
            _w(1.5, back, FINGER_CLOSED, "draw back out of the gateway", PUSH_GRIP)]
    done = sum(w.duration for w in out)
    out += _go_home("right", back, PUSH_GRIP, FINGER_CLOSED)
    return out, done


def demo():
    _, opened = _left(0.0)
    right, done = _right(opened + 0.3)
    left, _ = _left(done + 0.3)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    for arm in (left, right):
        arm.append(_w(max(total - sum(w.duration for w in arm), 0.5), arm[-1].pos, arm[-1].finger,
                      "hold still: the trial ends once the full score has held 1 s", arm[-1].yaw))
    return {"left": left, "right": right}
