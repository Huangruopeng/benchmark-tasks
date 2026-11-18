"""Drill a 5 mm hole through a small aluminium plate at its punch mark on a bench drill press (yam_arena task file).

Scene (drill_press.py): an 8-inch bench drill press; on its table a waxed MDF spoil board; on that a loose 130 x 150 x
10 mm aluminium plate whose punch mark (a 90-degree centre-punch dimple 2.0 mm across where two scribed lines cross,
inside a scribed 8.0 mm check circle) starts 20 mm in front of the 5 mm bit; a screw hold-down clamp (a star-knob M8
screw whose bonded rubber tip turns with it) over the plate's front-left part. The motor has its own push-button switch
(green start, red stop); a three-spoke capstan handle feeds the quill down against its return spring.

SCORING (score() below; everything it reads is latched in EP by _follow() after every control step):
  clamped      the clamp's rubber tip on the plate (within 0.5 mm of its top) pressing at least 85 N (a knob tightened
               to about 0.5 N m); the force is the rubber's compression, 1.04 N per degree of knob turn past contact
  at the mark  a hole drilled through (the turning bit's tip 11.5 mm below the plate's top, into the spoil board, cut
               while the plate was clamped) whose centre is within 1.5 mm of the punch mark's: the 5 mm hole then
               takes in the whole 2.0 mm dimple (2.5 mm - 1.0 mm). Only the first hole counts.
  unsafe       while the chuck turned faster than 1 rad/s, any point of either hand or wrist within 5 cm of the bit or
               the chuck (latched)
  spoilt       the turning bit met (0.5 mm into) a plate that was not clamped: it is caught and flung (latched, 0.0)
  left safe    the quill within 3 mm of its top stop; the motor off and the chuck slower than 1 rad/s
  1.0   drilled through at the mark, never unsafe, and at the end the plate clamped and the drill left safe
  0.5   never unsafe, and either drilled through at the mark but the drill left running, the quill down or the clamp
        loosened; or the plate clamped with its mark within 1.0 mm of the bit's axis (the bit's point, lowered with
        the motor off, drops into the dimple) but no hole drilled through at the mark
  0.25  the plate clamped, but off its mark or with a hole off the mark; or clamped or drilled while unsafe
  0.0   otherwise, or spoilt
success() adds stillness: the trial ends once the full score has held 1 s with the plate, quill, handle, screw and
chuck still.

THE PUNCH DIMPLE (_seat(), before every physics step): the bit has no collider. With the motor off, its 118-degree
point stops on the plate's top (a stiff stop on the quill, 50 kN/m) and, within 1.0 mm of the mark, in the dimple: its
flank rests on the dimple's rim, so it sits (1.0 mm - offset) x tan 31 degrees deeper, and the rim pushes the plate to
bring the dimple under the point, with at most 0.10 times the point's push while the plate moves toward the point and
1.49 times while something pushes it away (the 31-degree flank with friction 0.47, aluminium on steel, dry). On the
flat the point drags on the plate with that friction. So lowering the stopped bit into the punch mark with the clamp loose centres the plate on it,
as a machinist does; the turning bit does not stop: it drills (cutting thrust as damping on the feed).

Run: inspect-robots run --embodiment yam_arena -E task=clamp_a_small_part_before_drilling_it.py --scorer task_score
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import allow_matrix_grips, as_numpy, on_step  # noqa: E402
from drill_demo import demo, steer  # noqa: E402,F401  (the scripted demo yam_scripted plays, and its steering)
from drill_press import (BIT_R, BTN_SWITCH, DIMPLE_R, EP, MARK_IN_PLATE, OBJECTS, PLATE_T, PLATE_X, PLATE_Y,  # noqa: E402
                         POINT_SLOPE, PRE_STEP, SCENE, SCREW0, SPX, SPY, TIP0, TIP_ON, TURNING, button_in, clearance, feed_state,
                         fling, hook_physics, motor, plate_force, pose, tcps, teleport)

allow_matrix_grips()
__all__ = ["OBJECTS", "SCENE", "demo", "score", "success"]

# -- the scorer's thresholds -------------------------------------------------------------------------------------------
CLAMPED_N = 85.0  # N: the clamp's tip pressing at least this (a real knob tightened to 0.5 N m)
HOLE_ON_MARK = BIT_R - DIMPLE_R  # 1.5 mm: the hole's centre within this of the mark's, so the hole takes in the dimple
IN_DIMPLE = DIMPLE_R  # 1.0 mm: before drilling, the mark this close to the bit's axis: the point drops into the dimple
THROUGH = PLATE_T + 0.0015  # the hole this deep: through the 10 mm plate and 1.5 mm into the spoil board
MEETS = 0.0005  # the turning bit's tip this far into the plate: it has met it
QUILL_UP = 0.003  # the quill within this of its top stop: up
SAFE = 0.05  # every point of each hand and wrist at least this far from the bit and the chuck while they turn


def _status() -> dict:
    """The checks, from the latches and the last control step's state (EP)."""
    st = EP["st"]
    clamped = bool(st.get("clamped"))
    return {"through": EP["hole"] >= THROUGH and EP["hole_ok"], "unsafe": EP["unsafe"], "spoilt": EP["caught"],
            "clamped": clamped,
            "in_place": clamped and st.get("mark_off", 1.0) <= IN_DIMPLE and bool(st.get("flat")),
            "quill_up": st.get("down", 1.0) < QUILL_UP, "off": not EP["motor"] and EP["omega"] <= TURNING}


def _points(s: dict) -> float:
    if s["spoilt"]:
        return 0.0
    if not s["unsafe"]:
        if s["through"] and s["clamped"] and s["quill_up"] and s["off"]:
            return 1.0
        if s["through"] or s["in_place"]:
            return 0.5
    return 0.25 if s["clamped"] or s["through"] else 0.0


def score(objects) -> float:
    """1.0: a hole drilled through within 1.5 mm of the punch mark (so it takes in the 2.0 mm dimple) while the plate
    was clamped at 85 N or more, no part of either hand within 5 cm of the turning bit or chuck, and at the end the
    plate still clamped, the quill up and the motor off with the chuck stopped. 0.5: that hole but the drill left
    running, its quill down or the clamp loosened; or the plate clamped with its mark within 1.0 mm of the bit's axis
    but not drilled through at the mark. 0.25: the plate clamped off its mark (or drilled off it), or clamped or
    drilled with a hand near the turning bit. 0.0: otherwise, or the turning bit caught a plate that was not clamped."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the plate, the quill, the feed handle, the clamp's screw and the chuck still (held 1 s)."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("plate", "quill", "capstan", "screw", "chuck"))


# -- the punch dimple: the stopped bit's point on the plate, before every physics step ---------------------------------
SEAT_K, SEAT_D = 5.0e4, 150.0  # N/m, N s/m: the point's stop on the plate (on the quill)
POINT_MU = 0.47  # the steel point on the aluminium, dry (sliding)
IN_RATIO = math.tan(POINT_SLOPE - math.atan(POINT_MU))  # 0.10: the rim's centring push per newton of point force
OUT_RATIO = math.tan(POINT_SLOPE + math.atan(POINT_MU))  # 1.49: what it holds against a push away from the point
SEAT_KC, SEAT_DC = 2.0e4, 60.0  # N/m, N s/m: the rim's centring push, and the point's grip on the flat (stick-slip:
# a stiff spring to where the point touched down, which slips at POINT_MU times the point's force)


def _seat(base) -> None:
    """With the motor off, stop the bit's point on the plate (the force goes to the quill through EP["seat_n"], read by
    the gear in drill_press) and push the plate: down, plus the dimple's centring or the point's friction on the flat."""
    origin = as_numpy(base.scene.env_origins[0])
    k_pos, _ = pose(base, origin, "chuck")  # the bit's tip
    n, force = 0.0, np.zeros(3)
    if EP["omega"] <= TURNING and not EP["caught"]:
        b_pos, b_rot = pose(base, origin, "plate")
        tip = b_rot.T @ (k_pos - b_pos)
        if abs(tip[0]) < (PLATE_X[1] - PLATE_X[0]) / 2 and abs(tip[1]) < (PLATE_Y[1] - PLATE_Y[0]) / 2:
            e = tip[:2] - MARK_IN_PLATE
            rho = float(np.linalg.norm(e))
            if EP["hole_xy"] is not None and float(np.linalg.norm(tip[:2] - EP["hole_xy"])) < 0.0005:
                seat = EP["hole"]  # the point back in its own hole
            else:
                seat = max(DIMPLE_R - rho, 0.0) * math.tan(POINT_SLOPE)
            pen = PLATE_T / 2 - float(tip[2]) - seat
            if pen > 0.0:
                data = base.scene["plate"].data
                w = as_numpy(data.root_ang_vel_w)[0]
                v = as_numpy(data.root_lin_vel_w)[0] + np.cross(w, k_pos - b_pos)  # the plate under the point
                closing = float(v[2] - as_numpy(base.scene["chuck"].data.root_lin_vel_w)[0][2])
                n = max(SEAT_K * pen + SEAT_D * closing, 0.0)
                up = b_rot[:, 2]
                lat_v = v - up * float(v @ up)
                if rho < DIMPLE_R:
                    e_w = b_rot @ np.array([e[0], e[1], 0.0])  # from the mark toward the point
                    f = SEAT_KC * e_w - SEAT_DC * lat_v
                    inward = rho > 1e-6 and float(lat_v @ e_w) / rho > 0.0005
                    cap = n * (IN_RATIO if inward else OUT_RATIO)
                    EP["anchor"], EP["located"] = None, True
                else:  # on the flat: friction, sticking where the point touched down until it slips
                    if EP.get("anchor") is None:
                        EP["anchor"] = np.array(tip[:2])
                    d = tip[:2] - EP["anchor"]
                    cap = POINT_MU * n
                    if SEAT_KC * float(np.linalg.norm(d)) > cap:  # slipping: the stuck point follows
                        EP["anchor"] = tip[:2] - d * cap / (SEAT_KC * float(np.linalg.norm(d)))
                        d = tip[:2] - EP["anchor"]
                    f = SEAT_KC * (b_rot @ np.array([d[0], d[1], 0.0])) - SEAT_DC * lat_v
                mag = float(np.linalg.norm(f))
                force = (f * min(1.0, cap / mag) if mag > 0.0 else f) - n * up
    if n == 0.0:
        EP["anchor"] = None
    EP["seat_n"] = n
    EP["seat_max"] = max(EP["seat_max"], n)
    plate_force(base, origin, force, k_pos)


PRE_STEP.append(_seat)


# -- after every control step: the switch, the clamp, the plate, the hands' clearance and the drilling ----------------

def _state(b_pos, b_rot, s_pos, down) -> dict:
    """The clamp, the plate and the quill as they are."""
    pad = b_rot.T @ (s_pos - b_pos)  # the clamp tip's underside, in the plate's frame
    on_plate = (abs(pad[0]) < (PLATE_X[1] - PLATE_X[0]) / 2 and abs(pad[1]) < (PLATE_Y[1] - PLATE_Y[0]) / 2
                and abs(pad[2] - PLATE_T / 2) < TIP_ON)
    mark = b_pos + b_rot @ np.array([MARK_IN_PLATE[0], MARK_IN_PLATE[1], PLATE_T / 2])
    return {"slide": float(SCREW0[2] - s_pos[2]), "force": EP["press"], "on_plate": on_plate,
            "clamped": on_plate and EP["press"] >= CLAMPED_N, "down": down,
            "mark_off": float(np.linalg.norm(mark[:2] - np.array([SPX, SPY]))), "mark_err": np.array([SPX, SPY]) - mark[:2],
            "flat": float(b_rot[2, 2]) > math.cos(math.radians(10.0))}


_DEBUG = {"on": bool(os.environ.get("DRILL_DEBUG")), "steps": 0}


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    hook_physics(base)
    origin = as_numpy(base.scene.env_origins[0])
    b_pos, b_rot = pose(base, origin, "plate")
    s_pos, _ = pose(base, origin, "screw")
    k_pos, _ = pose(base, origin, "chuck")  # the bit's tip
    EP["tcps"] = tcps(base, origin)
    b_start, b_stop = button_in(base, origin, "btn_start"), button_in(base, origin, "btn_stop")
    if b_stop >= BTN_SWITCH and EP["motor"]:
        motor(False)
    elif b_start >= BTN_SWITCH and b_stop < BTN_SWITCH and not EP["motor"]:
        motor(True)
    omega = abs(float(as_numpy(base.scene["chuck"].data.root_ang_vel_w)[0][2]))
    EP["omega"] = omega
    turning = omega > TURNING
    down = float(TIP0 - k_pos[2])
    st = _state(b_pos, b_rot, s_pos, down)
    EP["st"] = st
    clear = clearance(base, origin, k_pos)
    EP["clear"] = clear
    if turning:
        EP["clear_min"] = [min(a, b) for a, b in zip(EP["clear_min"], clear)]
        if min(clear) < SAFE:
            EP["unsafe"] = True
    tip = b_rot.T @ (k_pos - b_pos)  # the bit's tip in the plate's frame
    over = abs(tip[0]) < (PLATE_X[1] - PLATE_X[0]) / 2 and abs(tip[1]) < (PLATE_Y[1] - PLATE_Y[0]) / 2
    depth = PLATE_T / 2 - float(tip[2])  # below the plate's top
    EP["cutting"] = (down - depth, bool(st["clamped"]) and over) if over else None
    if turning and over and not EP["caught"] and depth > (MEETS if EP["hole_xy"] is None else EP["hole"] + MEETS):
        if not st["clamped"]:
            EP["caught"] = True  # the turning bit catches a loose plate and flings it
            fling(base)
        elif EP["hole_xy"] is None:  # the first hole: where it is, on the plate
            EP["hole_xy"] = np.array(tip[:2])
            EP["hole_ok"] = float(np.linalg.norm(tip[:2] - MARK_IN_PLATE)) <= HOLE_ON_MARK
            teleport(base, origin, "hole", b_pos + b_rot @ np.array([tip[0], tip[1], PLATE_T / 2 + 0.0003]))
    if turning and over and st["clamped"] and EP["hole_xy"] is not None:
        EP["hole"] = max(EP["hole"], depth)
    EP["k"] = EP.get("k", 0) + 1
    steer(EP["k"] * 0.05, st["mark_err"], EP["seat_n"], st["mark_off"] < DIMPLE_R and EP["seat_n"] > 0.0)
    if _DEBUG["on"]:
        _debug(base, origin, b_pos, b_rot, st, omega, (b_start, b_stop), clear)


def _debug(base, origin, b_pos, b_rot, st, omega, buttons, clear) -> None:
    """DRILL_DEBUG=1: what the score is built from, twice a second."""
    _DEBUG["steps"] += 1
    if _DEBUG["steps"] % 10:
        return
    s_q, s_c = feed_state(base, origin)
    tl, tr = EP["tcps"]
    hole_off = (float(np.linalg.norm(EP["hole_xy"] - MARK_IN_PLATE)) * 1000) if EP["hole_xy"] is not None else -1.0
    print(f"[drill] t={_DEBUG['steps'] / 20:5.1f}s turns={EP['turns']:5.2f} force={st['force']:5.1f}N "
          f"max={EP['press_max']:5.1f}N clamped={st['clamped']} plate={np.round(b_pos * 1000).astype(int).tolist()} "
          f"yaw={math.degrees(math.atan2(b_rot[1, 0], b_rot[0, 0])):5.1f} mark_off={st['mark_off'] * 1000:5.2f}mm "
          f"seat={EP['seat_n']:5.1f}N seat_max={EP['seat_max']:5.1f}N located={EP['located']} "
          f"quill={s_q * 1000:5.1f} handle={s_c * 1000:5.1f}mm knob={EP.get('knob_n', 0.0):5.1f}N "
          f"btn=({buttons[0] * 1000:.1f},{buttons[1] * 1000:.1f})mm motor={EP['motor']} omega={omega:5.1f} "
          f"hole={EP['hole'] * 1000:5.1f}mm hole_off={hole_off:5.2f}mm hole_ok={EP['hole_ok']} caught={EP['caught']} "
          f"unsafe={EP['unsafe']} clear=({clear[0] * 1000:.0f},{clear[1] * 1000:.0f})mm "
          f"clear_min=({EP['clear_min'][0] * 1000:.0f},{EP['clear_min'][1] * 1000:.0f}) "
          f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
          f"steer=({_steer_dbg()}) score={_points(_status())}", file=sys.stderr, flush=True)


def _steer_dbg() -> str:
    import drill_demo

    r, lf = drill_demo.LIVE.get("right"), drill_demo.LIVE.get("left")
    return (f"d={r['d'] if r else 0:.2f}deg seated={r['seated'] if r else None} go={r.get('go') if r else None} "
            f"off={np.round((lf['off'] if lf else np.zeros(2)) * 1000, 2).tolist()}mm done={lf['done'] if lf else None}")
