"""On the station table: a ChArUco calibration board leans on a wooden book stand in front of the two arms, on the
centre line between them, its printed face toward them. Each arm carries a camera on its wrist (a RealSense-class
camera on the gripper's side, looking out along the fingers).

Task (instruction): hand-eye calibrate both wrist cameras with the ChArUco board on the bench (measure where each
camera sits on its gripper), then take both arms back to where they started. The instruction does not say how. In an eye-in-hand calibration the board stays put
and each arm moves its own camera round it, stopping at many poses with the whole board in view; at each stop the
calibration tool records the robot's pose (from its joints) and the board's pose in the image, and from these it solves
for the camera's mount (AX = XB). MoveIt Calibration's tutorial: the target must be "stationary in the robot's base
frame", "once five such samples have been collected, the calibration can be calculated", it "will typically plateau
after about 12 or 15 samples", "include some rotation between each pair of poses", "don't always rotate around the
same axis" and "at least two rotation axes are needed to uniquely solve for the calibration". OpenCV's
calibrateHandEye: "A minimum of 2 motions with non parallel rotation axes are necessary". easy_handeye: "the end
effector must be rotated as much as possible (up to 90°) about each axis, in both directions".

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- board: an A4 plate (297 x 210 x 3 mm aluminium composite, 0.24 kg), loose, landscape, leaning back 45 degrees from
  upright on the stand, its bottom edge on the stand's base against a lip, its bottom edge's middle at x 0.38, y 0.
  Printed on its front, in the middle: a ChArUco board of 7 x 5 squares of 30 mm (210 x 150 mm, 6 x 4 = 24 inner
  corners) with 17 ArUco markers of 22 mm (OpenCV dictionary DICT_5X5_250, ids 0-16) in its white squares, as OpenCV's
  CharucoBoard draws it. Its back is grey.
- stand: fixed, a beech book stand: a base 17.8 cm deep and 30 cm wide, 12 mm thick; a lip 10 mm tall in front of the
  board's bottom edge; a sloped rest under the board's back; a block under the rest's upper end.
- cameras: on each wrist, the camera the station's arms carry (pose on the gripper and lens from the station's own
  calibration): 7 cm off the gripper's axis, 7.3 cm along it, looking along the fingers and turned 25 degrees in
  toward them; recorded for the calibration at 1280 x 720 pixels (focal length 781 pixels).

What each camera sees of the board is computed from the camera's pose (its gripper's pose in the simulation and the
camera's mount) and its intrinsics: the board's print is drawn into a 1280 x 720 frame (an ideal pinhole, no lens
distortion), and OpenCV's ChArUco detector (cv2.aruco.CharucoDetector) is run on it. A sample of a camera is taken
automatically (as a "take sample" press) when: the whole printed area is at least 8 pixels inside the frame; no part of
either arm (links, gripper, fingers) or of the stand is in front of the print as seen from the camera (a line-of-sight
test to points 1 cm apart over it); the board's image has moved less than 2 pixels over the last 0.5 s (the camera held
still); the camera is turned 15 degrees or more from every earlier sample of that camera; and the detector finds all
24 ChArUco corners. The sample is the robot's pose (the gripper in the arm's base frame, from its joint angles) and the
board's pose in the camera (OpenCV's solvePnP on the 24 corners).

Per camera: 0.05 per sample, up to 12 (0.6); +0.1 once its samples include two rotations between samples of 30
degrees or more about axes 45 degrees or more apart (two non-parallel rotation axes); +0.1 once it has 12 samples or
more, the rotations condition holds, and OpenCV's calibrateHandEye (Park's method), run on all its samples, gives the
camera's mount within 5 mm and 1 degree of the true one. Score: the mean of the two cameras' scores (at most 0.8).
Full score (1.0, success): both cameras calibrated, the board moved less than 1 mm from where it settled, both arms
back at their start (each fingertip centre within 5 cm) and still, held for 1 s. The score is 0 for the rest of the run
once the board has moved more than 2 mm from where it settled, or an arm has touched it (any part of an arm within 2 mm
of the plate). Doing nothing scores 0.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps what happened over the run (the samples taken, a knocked board), so the score is read at the end.
SCORE_OVER = "end"

# -- the board -------------------------------------------------------------------------------------------------------
PLATE_W, PLATE_H, PLATE_T, BOARD_KG = 0.297, 0.210, 0.003, 0.24  # A4, 3 mm aluminium composite (~3.8 kg/m^2)
NX, NY, SQ, MK = 7, 5, 0.030, 0.022  # ChArUco: 7 x 5 squares of 30 mm, 22 mm markers
PRINT_W, PRINT_H = NX * SQ, NY * SQ  # 210 x 150 mm, centred on the plate
LEAN = math.radians(45.0)  # back from upright
BOTTOM_X = 0.38  # the board's bottom edge (middle of its thickness), on the centre line

WHITE, BLACK, BACK = (0.95, 0.95, 0.94), (0.03, 0.03, 0.035), (0.55, 0.56, 0.58)
BEECH, BEECH_DARK = (0.80, 0.62, 0.42), (0.62, 0.45, 0.28)


def _quat(m) -> tuple:
    """(x, y, z, w) of a rotation matrix (Shepperd's method, safe for any rotation)."""
    m = np.asarray(m, float)
    tr = float(np.trace(m))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (float(x / n), float(y / n), float(z / n), float(w / n))


# the board's frame: x along its width to the robot's right (world -y), y up its face, z out of its printed face
R_B = np.column_stack([[0.0, -1.0, 0.0], [math.sin(LEAN), 0.0, math.cos(LEAN)], [-math.cos(LEAN), 0.0, math.sin(LEAN)]])
BASE_T = 0.012  # the stand's base
_B = np.array([BOTTOM_X, 0.0, TOP + BASE_T + PLATE_T / 2 * R_B[2, 2]])  # the bottom edge's middle, settled
C_B = _B + PLATE_H / 2 * R_B[:, 1]  # the board's middle (and the print's), settled


def _aruco():
    import cv2

    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    return cv2, d, cv2.aruco.CharucoBoard((NX, NY), SQ, MK, d)


def _print_shapes() -> list:
    """The ChArUco print as thin visual boxes on the plate's face, laid out from OpenCV's own CharucoBoard: black
    squares, and in each white square a marker (a black square with its white bits on top)."""
    cv2, d, board = _aruco()
    z1, z2 = PLATE_T / 2 + 0.0002, PLATE_T / 2 + 0.0005

    def uv(x, y):  # OpenCV's board frame (from the print's top left, x right, y down) -> the body frame
        return x - PRINT_W / 2, PRINT_H / 2 - y

    out = []
    for i in range(NX):
        for j in range(NY):
            if (i + j) % 2 == 0:
                u, v = uv((i + 0.5) * SQ, (j + 0.5) * SQ)
                out.append(BoxShape((u, v, z1), (SQ, SQ, 0.0004), BLACK, collide=False))
    cell = MK / 7
    for mid, corners in zip(np.array(board.getIds()).ravel(), np.array(board.getObjPoints())):
        x0, y0 = float(corners[:, 0].min()), float(corners[:, 1].min())
        u, v = uv(x0 + MK / 2, y0 + MK / 2)
        out.append(BoxShape((u, v, z1), (MK, MK, 0.0004), BLACK, collide=False))
        bits = cv2.aruco.generateImageMarker(d, int(mid), 7, borderBits=1) > 127
        for r in range(7):
            c = 0
            while c < 7:
                if bits[r, c]:
                    c1 = c
                    while c1 + 1 < 7 and bits[r, c1 + 1]:
                        c1 += 1
                    u, v = uv(x0 + (c + c1 + 1) / 2 * cell, y0 + (r + 0.5) * cell)
                    out.append(BoxShape((u, v, z2), ((c1 - c + 1) * cell, cell, 0.0004), WHITE, collide=False))
                    c = c1 + 1
                else:
                    c += 1
    return out


def _board_shapes() -> list:
    return ([BoxShape((0.0, 0.0, 0.0), (PLATE_W, PLATE_H, PLATE_T), WHITE),
             BoxShape((0.0, 0.0, -PLATE_T / 2 - 0.0002), (PLATE_W - 0.002, PLATE_H - 0.002, 0.0004), BACK,
                      collide=False)]
            + _print_shapes())


# -- the stand (world-aligned frame at the middle of its base's underside) ---------------------------------------------
LIP_H, LIP_T, LIP_GAP = 0.010, 0.008, 0.001
REST_T, REST_W, REST_V = 0.012, 0.26, (-0.085, 0.060)  # the sloped rest: thickness, width, extent up the board
_front = BOTTOM_X - PLATE_T / 2 * math.cos(LEAN) - LIP_GAP  # the lip's back face
_rest_top = C_B + R_B @ np.array([0.0, REST_V[1], -PLATE_T / 2 - REST_T])  # the rest's upper back edge
BASE_X = (_front - LIP_T - 0.012, float(_rest_top[0]) + 0.03)
STAND_O = np.array([(BASE_X[0] + BASE_X[1]) / 2, 0.0, TOP])


def _stand_shapes() -> list:
    o = STAND_O
    out = [BoxShape((0.0, 0.0, BASE_T / 2), (BASE_X[1] - BASE_X[0], 0.30, BASE_T), BEECH),
           BoxShape((_front - LIP_T / 2 - o[0], 0.0, BASE_T + LIP_H / 2), (LIP_T, 0.28, LIP_H), BEECH_DARK)]
    vmid = (REST_V[0] + REST_V[1]) / 2
    centre = C_B + R_B @ np.array([0.0, vmid, -PLATE_T / 2 - REST_T / 2]) - o
    out.append(BoxShape(tuple(float(v) for v in centre), (REST_W, REST_V[1] - REST_V[0], REST_T), BEECH,
                        quat_xyzw=_quat(R_B)))
    bx0, bx1 = float(_rest_top[0]) - 0.012, float(_rest_top[0]) + 0.020
    h = float(_rest_top[2] - TOP) - BASE_T + 0.004
    out.append(BoxShape(((bx0 + bx1) / 2 - o[0], 0.0, BASE_T + h / 2), (bx1 - bx0, REST_W, h), BEECH_DARK))
    return out


BOARD = Body.build("charuco_board", pos=tuple(float(v) for v in C_B + 0.001 * R_B[:, 2] + [0.0, 0.0, 0.001]),
                   mass=BOARD_KG, color=WHITE, start_xyzw=_quat(R_B), friction=(0.5, 0.4), solver_iterations=32,
                   shapes=_board_shapes())
STAND = Body.build("book_stand", pos=tuple(float(v) for v in STAND_O), mass=1.0, color=BEECH, kinematic=True,
                   friction=(0.6, 0.5), shapes=_stand_shapes())
OBJECTS = [STAND, BOARD]

# -- geometry of what the cameras see ---------------------------------------------------------------------------------
IMG_W, IMG_H = 1280, 720
EDGE_PX = 8.0  # the whole print this far inside the frame
STILL_STEPS, STILL_PX = 10, 2.0  # 0.5 s at 20 Hz: the board's image moved under 2 pixels
NEW_DEG = 15.0  # a sample's camera turned this far from every earlier one
NEED = 12
SAMPLE_CREDIT, DIV_CREDIT, CAL_CREDIT = 0.05, 0.1, 0.1
DIV_DEG, DIV_AXES_DEG = 30.0, 45.0
CAL_MM, CAL_DEG = 5.0, 1.0
MOVED_OK, MOVED_FAIL, TOUCH = 0.001, 0.002, 0.002
HOME_WITHIN, HOME_STILL = 0.05, 0.002
REF_TICK = 10  # the board's settled pose is taken 0.5 s into the run

_PU = np.linspace(-PRINT_W / 2, PRINT_W / 2, 22)
_PV = np.linspace(-PRINT_H / 2, PRINT_H / 2, 16)
PRINTED = np.array([[u, v, PLATE_T / 2] for u in _PU for v in _PV])  # over the print, every 1 cm
OUTLINE = np.array([[sx * PRINT_W / 2, sy * PRINT_H / 2, PLATE_T / 2] for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))])
_IC = np.array([[(i + 1) * SQ - PRINT_W / 2, PRINT_H / 2 - (j + 1) * SQ, PLATE_T / 2] for j in range(NY - 1)
                for i in range(NX - 1)])  # the 24 inner corners, in OpenCV's order
PLATE_CORNERS = np.array([[sx * PLATE_W / 2, sy * PLATE_H / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])

_CACHE: dict = {}


def cameras() -> dict:
    """Per side: the camera's mount in the gripper frame (4 x 4, OpenCV axes: x right, y down, z forward) and its
    intrinsics at 1280 x 720 (the station's 640 x 360 lens, doubled)."""
    if "cams" not in _CACHE:
        from yam_arena import station

        out = {}
        for side in ("left", "right"):
            cam = station.wrist_camera(side)
            t = np.eye(4)
            t[:3, :3] = rotation(cam.rot_xyzw) @ np.diag([1.0, -1.0, -1.0])  # OpenGL (y up, -z forward) -> OpenCV
            t[:3, 3] = cam.pos
            ln = cam.lens
            k = np.array([[2 * ln.fx, 0.0, 2 * ln.cx + 0.5], [0.0, 2 * ln.fy, 2 * ln.cy + 0.5], [0.0, 0.0, 1.0]])
            out[side] = (t, k)
        _CACHE["cams"] = out
    return _CACHE["cams"]


def project(t_wc: np.ndarray, k: np.ndarray, pts_w: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pc = (np.asarray(pts_w, float) - t_wc[:3, 3]) @ t_wc[:3, :3]
    z = pc[:, 2]
    safe = np.where(np.abs(z) < 1e-9, 1e-9, z)
    return np.stack([k[0, 0] * pc[:, 0] / safe + k[0, 2], k[1, 1] * pc[:, 1] / safe + k[1, 2]], 1), z


PPM = 8000  # the print drawn at 8 pixels per mm


def _pattern() -> np.ndarray:
    if "pattern" not in _CACHE:
        cv2, _, board = _aruco()
        pat = np.full((int(round(PLATE_H * PPM)), int(round(PLATE_W * PPM))), 242, np.uint8)
        p = board.generateImage((int(round(PRINT_W * PPM)), int(round(PRINT_H * PPM))), marginSize=0, borderBits=1)
        ox, oy = int(round((PLATE_W - PRINT_W) / 2 * PPM)), int(round((PLATE_H - PRINT_H) / 2 * PPM))
        pat[oy:oy + p.shape[0], ox:ox + p.shape[1]] = np.where(p > 127, 242, 14).astype(np.uint8)
        _CACHE["pattern"] = pat
    return _CACHE["pattern"]


def camera_frame(t_wc: np.ndarray, k: np.ndarray, pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """The board (middle ``pos``, rotation ``rot``) as the camera at ``t_wc`` sees it: 1280 x 720, 8-bit grey, drawn at
    twice the resolution and averaged down, over a mid-grey background (the arms are the line-of-sight test's)."""
    import cv2

    origin = pos + rot @ np.array([-PLATE_W / 2, PLATE_H / 2, PLATE_T / 2])  # the plate's top left, on its face
    a_w = np.column_stack([rot[:, 0], -rot[:, 1], origin])  # world point = a_w @ [x, y, 1] (x right, y down, m)
    rcw = t_wc[:3, :3].T
    a_c = rcw @ a_w
    a_c[:, 2] -= rcw @ t_wc[:3, 3]
    s = np.array([[1 / PPM, 0.0, 0.5 / PPM], [0.0, 1 / PPM, 0.5 / PPM], [0.0, 0.0, 1.0]])
    k2 = np.array([[2 * k[0, 0], 0.0, 2 * k[0, 2] + 0.5], [0.0, 2 * k[1, 1], 2 * k[1, 2] + 0.5], [0.0, 0.0, 1.0]])
    big = cv2.warpPerspective(_pattern(), k2 @ a_c @ s, (2 * IMG_W, 2 * IMG_H), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=110)
    return cv2.resize(big, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)


def detect(img: np.ndarray, k: np.ndarray):
    """OpenCV's ChArUco detector on the frame: (rotation, translation) of the board in the camera from solvePnP on its
    corners, or None unless all 24 are found."""
    cv2, _, board = _aruco()
    if "det" not in _CACHE:
        _CACHE["det"] = cv2.aruco.CharucoDetector(board)
    cc, cid, _, _ = _CACHE["det"].detectBoard(img)
    if cid is None or len(cid) < (NX - 1) * (NY - 1):
        return None
    obj, ipt = board.matchImagePoints(cc, cid)
    ok, rvec, tvec = cv2.solvePnP(obj, ipt, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return cv2.Rodrigues(rvec)[0], tvec.ravel()


def calibrate(samples: list, side: str) -> tuple[float, float]:
    """OpenCV's calibrateHandEye (Park) on the samples: its error in position (mm) and turn (deg) from the true
    mount."""
    import cv2

    rg = [s["a"][:3, :3] for s in samples]
    tg = [s["a"][:3, 3] for s in samples]
    rt = [s["r_tc"] for s in samples]
    tt = [s["t_tc"] for s in samples]
    r_x, t_x = cv2.calibrateHandEye(rg, tg, rt, tt, method=cv2.CALIB_HAND_EYE_PARK)
    true = cameras()[side][0]
    return float(np.linalg.norm(t_x.ravel() - true[:3, 3]) * 1000), _angle(r_x, true[:3, :3])


def _angle(ra, rb) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(np.asarray(ra).T @ rb)) - 1.0) / 2.0))))


def diverse(samples: list) -> bool:
    """Two rotations between samples of DIV_DEG or more whose axes (in the gripper frame) are DIV_AXES_DEG or more
    apart."""
    import cv2

    axes = []
    for a, b in itertools.combinations(samples, 2):
        rv = cv2.Rodrigues(a["a"][:3, :3].T @ b["a"][:3, :3])[0].ravel()
        if math.degrees(float(np.linalg.norm(rv))) >= DIV_DEG:
            axes.append(rv / np.linalg.norm(rv))
    for u, w in itertools.combinations(axes, 2):
        if math.degrees(math.acos(min(1.0, abs(float(u @ w))))) >= DIV_AXES_DEG:
            return True
    return False


# -- the arms as boxes (each in its link's frame), from the YAM's collision meshes -------------------------------------
LINK_BOXES = {
    "link2": ((-0.031, -0.0668, -0.0309), (0.031, -0.0001, 0.295)),
    "link3": ((-0.0912, -0.0799, -0.2735), (0.0315, 0.01, 0.0315)),
    "link4": ((-0.0982, -0.013, -0.1023), (0.0175, 0.073, 0.0175)),
    "link5": ((0.0, -0.0285, -0.0356), (0.069, 0.0285, 0.0335)),
    "gripper": ((-0.051, -0.0325, -0.0105), (0.051, 0.0325, 0.0635)),
}
# Each finger in the gripper frame (z toward the tips; the TCP at z 0.134), its inner face at x = q (the finger joint,
# <= 0) for the left finger and x = -q for the right: (z from, z to, thickness along x, half width along y).
FINGER_PARTS = ((0.131, 0.1468, 0.0064, 0.0137), (0.109, 0.131, 0.0100, 0.0201), (0.0635, 0.109, 0.0238, 0.034))


def _fk(q_arm, q_fingers=(0.0, 0.0)) -> dict:
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
    from yam_arena.urdf import forward_kinematics

    q = {n: float(v) for n, v in zip(ARM_JOINTS, q_arm)}
    q.update({n: float(v) for n, v in zip(FINGER_JOINTS, q_fingers)})
    return forward_kinematics(YAM_URDF, q)


def arm_boxes(fk: dict, q_fingers, base) -> list:
    """(centre, rotation, half size) of each box of one arm in the world."""
    out = []
    b = np.asarray(base, float)
    for link, (lo, hi) in LINK_BOXES.items():
        t = fk[link]
        lo, hi = np.array(lo), np.array(hi)
        out.append((t[:3, :3] @ ((lo + hi) / 2) + t[:3, 3] + b, t[:3, :3], (hi - lo) / 2))
    g = fk["gripper"]
    for k, sign in ((0, -1.0), (1, 1.0)):
        qf = float(q_fingers[k])
        for z0, z1, th, hw in FINGER_PARTS:
            inner = qf if sign < 0 else -qf
            x0, x1 = (inner - th, inner) if sign < 0 else (inner, inner + th)
            c = np.array([(x0 + x1) / 2, 0.0, (z0 + z1) / 2])
            out.append((g[:3, :3] @ c + g[:3, 3] + b, g[:3, :3], np.array([(x1 - x0) / 2, hw, (z1 - z0) / 2])))
    return out


def blocked(eye: np.ndarray, points_w: np.ndarray, boxes: list) -> np.ndarray:
    """Which points are hidden from ``eye`` by any box (slab test on the segment eye -> point)."""
    hit = np.zeros(len(points_w), bool)
    for c, r, h in boxes:
        a = r.T @ (eye - c)
        b = (np.asarray(points_w, float) - c) @ r
        d = b - a[None]
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-h[None] - a[None]) / d
            t2 = (h[None] - a[None]) / d
        lo = np.where(np.abs(d) < 1e-12, np.where(np.abs(a)[None] <= h[None], -np.inf, np.inf), np.minimum(t1, t2))
        hi = np.where(np.abs(d) < 1e-12, np.where(np.abs(a)[None] <= h[None], np.inf, -np.inf), np.maximum(t1, t2))
        tlo, thi = lo.max(1), hi.min(1)
        hit |= (tlo <= thi) & (thi >= 0.0) & (tlo <= 1.0 - 1e-6)
    return hit


_BOX_PTS = np.array(list(itertools.product((-1.0, 0.0, 1.0), repeat=3)))  # corners, edge and face middles, centre


def plate_gap(boxes: list, pos, rot) -> float:
    """The smallest distance from points over the arm boxes to the board's plate."""
    h = np.array([PLATE_W / 2, PLATE_H / 2, PLATE_T / 2])
    best = 1.0
    for c, r, hb in boxes:
        pts = c + (_BOX_PTS * hb) @ r.T
        p = (pts - pos) @ rot
        best = min(best, float(np.linalg.norm(np.maximum(np.abs(p) - h, 0.0), axis=1).min()))
    return best


def _world_boxes(spec: Body) -> list:
    r0 = rotation(spec.start_xyzw) if spec.start_xyzw else np.eye(3)
    out = []
    for s in spec.shapes:
        if isinstance(s, BoxShape) and s.collide:
            out.append((np.asarray(spec.pos) + r0 @ np.asarray(s.centre), r0 @ rotation(s.quat()), np.array(s.size) / 2))
    return out


STAND_BOXES = _world_boxes(STAND)

# -- tracking ---------------------------------------------------------------------------------------------------------
_DEBUG = os.environ.get("HEC_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("HEC_DEBUG_EVERY", "20"))
_EP: dict = {}
_IDS: dict = {}
SIDES = ("left", "right")


def _fresh_cam() -> dict:
    return dict(hist=[], samples=[], tried=None, why="-", diverse=False, cal=None, calibrated=False)


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, failed="", ref=None, moved=0.0, gap=1.0, cams={s: _fresh_cam() for s in SIDES}, tcp={},
               tcp_hist=[], step=0, pos=None)


_reset()


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("HEC_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _arms(base, origin) -> dict:
    from yam_arena import station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    out = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        if (side, "arm") not in _IDS:
            _IDS[(side, "arm")] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[(side, "fingers")] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[(side, "gripper")] = next(j for j, n in enumerate(robot.body_names)
                                          if n == "gripper" or n.endswith("/gripper"))
        q = as_numpy(robot.data.joint_pos)[0]
        qa, qf = q[_IDS[(side, "arm")]], q[_IDS[(side, "fingers")]]
        g = as_numpy(robot.data.body_link_pose_w)[0][_IDS[(side, "gripper")]]
        t_wg = np.eye(4)
        t_wg[:3, :3] = rotation(g[3:7])
        t_wg[:3, 3] = g[:3] - origin
        out[side] = dict(qa=qa, qf=qf, t_wg=t_wg, fk=_fk(qa, qf), base=np.asarray(station.BASES[side], float))
    return out


def _track_inner(env) -> None:
    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    bp = as_numpy(base.scene["charuco_board"].data.root_link_pose_w)[0]
    pos, rot = bp[:3] - origin, rotation(bp[3:7])
    arms = _arms(base, origin)
    tcps = {s: a["t_wg"][:3, :3] @ np.array(TCP_OFFSET) + a["t_wg"][:3, 3] for s, a in arms.items()}
    _EP["tcp_hist"].append(tcps)
    del _EP["tcp_hist"][:-STILL_STEPS]
    corners = pos + PLATE_CORNERS @ rot.T
    if _EP["ref"] is None and _EP["tick"] >= REF_TICK:
        _EP["ref"] = corners.copy()
    boxes = {s: arm_boxes(a["fk"], a["qf"], a["base"]) for s, a in arms.items()}
    gap = min(plate_gap(b, pos, rot) for b in boxes.values())
    moved = float(np.linalg.norm(corners - _EP["ref"], axis=1).max()) if _EP["ref"] is not None else 0.0
    if not _EP["failed"]:
        if _EP["ref"] is not None and moved > MOVED_FAIL:
            _EP["failed"] = f"the board moved {moved * 1000:.1f} mm at step {_EP['tick']}"
        elif gap < TOUCH:
            _EP["failed"] = f"an arm touched the board at step {_EP['tick']}"
    _EP.update(pos=pos, rot=rot, moved=moved, gap=gap, tcp=tcps)
    if _EP["failed"] or _EP["ref"] is None:
        return
    every = [b for bs in boxes.values() for b in bs] + STAND_BOXES
    for side in SIDES:
        _camera_step(side, arms[side], pos, rot, every)


def _camera_step(side: str, arm: dict, pos, rot, boxes) -> None:
    cam = _EP["cams"][side]
    t_gc, k = cameras()[side]
    t_wc = arm["t_wg"] @ t_gc
    px, depth = project(t_wc, k, pos + _IC @ rot.T)
    opx, odepth = project(t_wc, k, pos + OUTLINE @ rot.T)
    cam["hist"].append(px)
    del cam["hist"][:-STILL_STEPS]
    inside = bool((odepth > 0.05).all() and (opx[:, 0] >= EDGE_PX).all() and (opx[:, 0] <= IMG_W - 1 - EDGE_PX).all()
                  and (opx[:, 1] >= EDGE_PX).all() and (opx[:, 1] <= IMG_H - 1 - EDGE_PX).all())
    if not inside:
        cam["why"] = "board not all in the frame"
        return
    motion = max(float(np.abs(h - px).max()) for h in cam["hist"]) if len(cam["hist"]) == STILL_STEPS else 99.0
    if motion >= STILL_PX:
        cam["why"] = f"moving ({motion:.1f} px in 0.5 s)"
        return
    r_wc = t_wc[:3, :3]
    if any(_angle(s["r_wc"], r_wc) < NEW_DEG for s in cam["samples"]):
        cam["why"] = "too like an earlier sample"
        return
    if cam["tried"] is not None and float(np.abs(cam["tried"] - px).max()) < 5.0:
        cam["why"] = "detector found no board here"
        return
    hidden = int(blocked(t_wc[:3, 3], pos + PRINTED @ rot.T, boxes).sum())
    if hidden:
        cam["why"] = f"{hidden} points of the print hidden"
        return
    found = detect(camera_frame(t_wc, k, pos, rot), k)
    if found is None:
        cam["tried"] = px.copy()
        cam["why"] = "detector found no board here"
        if _DEBUG:
            print(f"DBG {side} step {_EP['tick']}: the ChArUco detector did not find all 24 corners", flush=True)
        return
    cam["tried"] = None
    cam["samples"].append(dict(r_wc=r_wc.copy(), a=arm["fk"]["gripper"].copy(), r_tc=found[0], t_tc=found[1]))
    cam["why"] = "sample taken"
    n = len(cam["samples"])
    if not cam["diverse"] and n >= 3:
        cam["diverse"] = diverse(cam["samples"])
    if n >= NEED and cam["diverse"]:
        cam["cal"] = calibrate(cam["samples"], side)
        cam["calibrated"] = cam["cal"][0] <= CAL_MM and cam["cal"][1] <= CAL_DEG
    if _DEBUG:
        cal = "-" if cam["cal"] is None else f"{cam['cal'][0]:.2f} mm {cam['cal'][1]:.3f} deg"
        print(f"DBG {side} sample {n} at step {_EP['tick']}: depth {float(depth.mean()):.3f} m, diverse "
              f"{cam['diverse']}, calibration {cal}", flush=True)


def camera_score(side: str) -> float:
    cam = _EP["cams"][side]
    s = SAMPLE_CREDIT * min(len(cam["samples"]), NEED)
    if cam["diverse"]:
        s += DIV_CREDIT
    if cam["calibrated"]:
        s += CAL_CREDIT
    return s


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    if "home" not in _CACHE:
        _CACHE["home"] = {s: np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[s])[0], float) for s in SIDES}
    hist = _EP["tcp_hist"]
    if len(hist) < STILL_STEPS:
        return False
    home = all(float(np.linalg.norm(hist[-1][s] - _CACHE["home"][s])) < HOME_WITHIN for s in SIDES)
    still = all(float(np.linalg.norm(h[s] - hist[-1][s])) < HOME_STILL for h in hist for s in SIDES)
    return home and still


def _done() -> bool:
    return bool(not _EP["failed"] and _EP["ref"] is not None and all(_EP["cams"][s]["calibrated"] for s in SIDES)
                and _EP["moved"] < MOVED_OK and _arms_home())


def _debug() -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY or _EP["pos"] is None:
        return
    p = _EP["pos"]
    parts = []
    for s in SIDES:
        c = _EP["cams"][s]
        cal = "-" if c["cal"] is None else f"{c['cal'][0]:.2f}mm/{c['cal'][1]:.3f}deg"
        t = _EP["tcp"].get(s)
        tt = "-" if t is None else f"({t[0]:.3f},{t[1]:.3f},{t[2] - TOP:.3f})"
        parts.append(f"{s}: n={len(c['samples'])} div={c['diverse']} cal={cal} why={c['why']} tcp={tt}")
    print(f"DBG step={_EP['step']} board=({p[0]:.4f},{p[1]:.4f},{p[2] - TOP:.4f}) moved={_EP['moved'] * 1000:.2f}mm "
          f"gap={_EP['gap']:.3f} failed={_EP['failed'] or '-'} home={_arms_home()} | " + " | ".join(parts), flush=True)


def score(objects) -> float:
    """0 once the board has moved more than 2 mm or an arm has touched it. Otherwise the mean of the two cameras'
    scores (each 0.05 per sample up to 12, +0.1 for two rotation axes, +0.1 for a calibration within 5 mm and 1 deg),
    and 1.0 with both calibrated, the board unmoved (< 1 mm) and both arms home and still."""
    if _DEBUG:
        _debug()
    if _EP["failed"]:
        return 0.0
    if _done():
        return 1.0
    return round(sum(camera_score(s) for s in SIDES) / 2, 4)


def success(objects) -> bool:
    """Full score with the board at rest (held for 1 s by SUCCESS_HOLD_S)."""
    return score(objects) >= 1.0 and objects["charuco_board"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left arm closes its fingers and moves its camera through 13 fixed poses round the board, from 25 to 40 cm away,
# looking at it from straight in front and from 15 to 45 degrees off to its left, right, top and bottom, with the
# camera rolled 0 to 180 degrees, stopping about 1.4 s at each; then it goes home. The right arm then does the same on
# its side. HEC_ONE=1: only the left arm sweeps (the right camera stays uncalibrated).

ONE = os.environ.get("HEC_ONE") == "1"
V0 = 250.0  # the board's middle this many pixels from the top of the frame (above the fingers, which fill the bottom)
# (distance m, tilt from the board's normal deg, direction of the tilt in the board's plane deg, roll about the view)
VIEWS = {
    "left": [(0.40, 30, 180, 0), (0.40, 45, 180, -45), (0.40, 30, 150, -45), (0.40, 15, 150, 0), (0.30, 0, 0, 0),
             (0.35, 30, 120, 0), (0.30, 45, 120, -90), (0.40, 45, 150, -45), (0.40, 30, 210, 45), (0.40, 45, 180, 0),
             (0.35, 45, 180, 45), (0.40, 15, 150, 90), (0.40, 30, 270, 180)],
    "right": [(0.40, 45, 0, 0), (0.40, 30, 0, 0), (0.40, 45, 0, 45), (0.40, 30, 30, 45), (0.40, 15, 30, 0),
              (0.30, 0, 0, 0), (0.35, 30, 60, 0), (0.30, 45, 60, 90), (0.40, 45, 30, 45), (0.40, 30, 330, -45),
              (0.35, 45, 0, -45), (0.40, 15, 30, -90), (0.40, 45, 270, 180)],
}
MOVE_S, HOLD_S, SUBSTEPS = 1.4, 1.8, 8
LEAD_S, LEAD_STEPS = 3.0, 15


def _look_at(p, target, roll: float) -> np.ndarray:
    """A camera (OpenCV axes) at p looking at target, rolled about its view."""
    f = np.asarray(target, float) - p
    f /= np.linalg.norm(f)
    up0 = np.array([0.0, 0.0, 1.0]) if abs(f[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
    x = np.cross(f, up0)
    x /= np.linalg.norm(x)
    y = np.cross(f, x)  # image down
    c, s = math.cos(roll), math.sin(roll)
    t = np.eye(4)
    t[:3, :3] = np.column_stack([c * x + s * y, -s * x + c * y, f])
    t[:3, 3] = p
    return t


def view_camera(side: str, d: float, tilt: float, az: float, roll: float) -> np.ndarray:
    """The camera pose (world, OpenCV axes) d from the board's middle, tilted ``tilt`` from its normal toward ``az``
    (0: the board's x, the robot's right; 90: up its face), rolled ``roll`` (deg), with the board's middle V0 pixels
    from the top of the frame."""
    _, k = cameras()[side]
    t, a = math.radians(tilt), math.radians(az)
    n, e1, e2 = R_B[:, 2], R_B[:, 0], R_B[:, 1]
    p = C_B + d * (math.cos(t) * n + math.sin(t) * (math.cos(a) * e1 + math.sin(a) * e2))
    t_wc = _look_at(p, C_B, math.radians(roll))
    up = math.atan((V0 - k[1, 2]) / k[1, 1])  # aim this far below the middle so the board sits above it
    return _look_at(p, C_B + d * math.tan(-up) * t_wc[:3, 1], math.radians(roll))


def view_tcp(side: str, view) -> tuple[np.ndarray, np.ndarray]:
    t_gc, _ = cameras()[side]
    t_wg = view_camera(side, *view) @ np.linalg.inv(t_gc)
    return t_wg[:3, :3] @ np.array(TCP_OFFSET) + t_wg[:3, 3], t_wg[:3, :3]


SEEDS = [[0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.4, 1.2, -0.3, 0.0, 0.0], [0.3, 1.0, 0.8, 0.0, 0.0, 0.5],
         [-0.3, 1.0, 0.8, 0.0, 0.0, -0.5], [0.0, 0.8, 0.6, 0.5, 0.0, 1.0], [0.0, 0.8, 0.6, 0.5, 0.0, -1.0],
         [0.0, 1.6, 1.6, -1.2, 0.5, 0.0], [0.0, 1.6, 1.6, -1.2, -0.5, 0.0], [0.8, 1.2, 1.0, -0.8, 0.8, 0.0],
         [-0.8, 1.2, 1.0, -0.8, -0.8, 0.0]]


def good_ik(side: str, pos, rot, near=None, min_room: float = math.radians(8.0)):
    """Joint angles for the TCP pose with at least ``min_room`` to every joint limit, the nearest to ``near``."""
    from yam_arena import ik, station

    sols = []
    seeds = ([list(near)] if near is not None else []) + SEEDS
    for sd in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[side], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if room >= min_room:
                sols.append(q)
                if near is not None and len(sols) >= 3:
                    break
    if not sols:
        return None
    if near is None:
        return max(sols, key=lambda q: float(np.minimum(q - ik.LOWER, ik.UPPER - q).min()))
    return min(sols, key=lambda q: float(np.abs(q - np.asarray(near)).max()))


def _joint_path(side: str, q0, q1, seconds: float, steps: int, finger: float, label: str) -> list:
    from yam_arena import ik, station

    out = []
    for i in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * i / steps, station.BASES[side])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def arm_sweep(side: str) -> tuple[list, list]:
    """The waypoints of one arm's sweep from READY back to READY, and the joint angles at each view."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q = np.array(READY, float)
    out, qs = [], []
    for i, view in enumerate(VIEWS[side]):
        pos, rot = view_tcp(side, view)
        q1 = good_ik(side, pos, rot, near=q)
        if q1 is None:
            raise ValueError(f"no IK for the {side} view {view}")
        if i == 0:
            out += _joint_path(side, q, q1, LEAD_S, LEAD_STEPS, FINGER_CLOSED, "to the first view")
        else:
            out += _joint_path(side, q, q1, MOVE_S, SUBSTEPS, FINGER_CLOSED, f"to view {i + 1}")
        p, r = ik.tcp_pose(q1, station.BASES[side])
        out.append(Waypoint(HOLD_S, tuple(float(v) for v in p), FINGER_CLOSED, f"hold view {i + 1}", r))
        q = q1
        qs.append(q1)
    out += _joint_path(side, q, np.array(READY, float), LEAD_S, LEAD_STEPS, FINGER_CLOSED, "home")
    return out, qs


def _wait(side: str, seconds: float, label: str) -> Waypoint:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    p, r = ik.tcp_pose(np.array(READY, float), station.BASES[side])
    return Waypoint(seconds, tuple(float(v) for v in p), FINGER_CLOSED, label, r)


def demo():
    left, _ = arm_sweep("left")
    t_left = sum(w.duration for w in left)
    if ONE:
        return {"left": left + [_wait("left", 4.0, "stay home")],
                "right": [_wait("right", t_left + 4.0, "stay home")]}
    right, _ = arm_sweep("right")
    return {"left": left + [_wait("left", sum(w.duration for w in right) + 4.0, "stay home")],
            "right": [_wait("right", t_left, "wait at home")] + right + [_wait("right", 4.0, "stay home")]}
