"""Write the low-cost real-trial fixture's printable parts as STL files, from the task file's own geometry.

The simulated head is built from boxes; this script writes those same boxes, in millimetres, as the parts to print, so
the printed mouth, ridge, tongue and denture match the simulation exactly:

  mouth_lower.stl   the lower half of the mouth: chin (with the pocket the silicone lip sits in), the mouth's sides, the
                    floor, the lower ridge, the tongue, the cheeks and the throat, up to the mouth's top edge
  mouth_upper.stl   the upper half: upper lip, upper denture, nose, the face above and the cheeks' upper part
  lower_denture.stl the resident's lower full denture: base, channel, flanges and teeth (print it in pink resin, or PLA
                    at 100% infill: about 25 g)
  lip_mould.stl     an open-top mould to cast the soft lower lip (12 x 64 x 31 mm) in Ecoflex 00-30 silicone
  riser.stl         a hollow block that sets the mouth's centre 20 cm above the table

Frame of the mouth parts: origin at the mouth's centre on the lips' front face; x back (into the mouth), y to her
left, z up. The denture's frame: origin at the arch's centre at the channel's ceiling; x back, z up (teeth up).

Run:  PYTHONPATH=<path to yam-isaac-lab-arena> python3 make_fixture.py
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import seat_a_residents_lower_denture as T  # noqa: E402
from bodykit import BoxShape, rotation  # noqa: E402

MM = 1000.0
FACES = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))  # outward, by corner index


def _corners(centre, size, quat=(0.0, 0.0, 0.0, 1.0)) -> np.ndarray:
    r = rotation(quat)
    h = np.asarray(size, float) / 2
    out = []
    for i in range(8):
        s = np.array([1.0 if i & 4 else -1.0, 1.0 if i & 2 else -1.0, 1.0 if i & 1 else -1.0])
        out.append(np.asarray(centre, float) + r @ (s * h))
    return np.array(out)


def _box(lo, hi) -> np.ndarray:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return _corners((lo + hi) / 2, hi - lo)


def _write(name: str, boxes: list) -> None:
    lines = [f"solid {name}"]
    for c in boxes:
        c = c * MM
        for f in FACES:
            for tri in ((f[0], f[1], f[2]), (f[0], f[2], f[3])):
                a, b, d = c[tri[0]], c[tri[1]], c[tri[2]]
                n = np.cross(b - a, d - a)
                n = n / (np.linalg.norm(n) or 1.0)
                lines.append(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
                lines.append("    outer loop")
                for v in (a, b, d):
                    lines.append(f"      vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}")
                lines.append("    endloop")
                lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    (HERE / f"{name}.stl").write_text("\n".join(lines) + "\n")
    lo = np.min([c.min(axis=0) for c in boxes], axis=0) * MM
    hi = np.max([c.max(axis=0) for c in boxes], axis=0) * MM
    print(f"{name}.stl: {len(boxes)} boxes, {np.round(hi - lo, 1)} mm")


def _volume_cm3(boxes: list, n: int = 400_000, seed: int = 0) -> float:
    """The union's volume, by sampling points in the bounding box (boxes overlap where they meet)."""
    lo = np.min([c.min(axis=0) for c in boxes], axis=0)
    hi = np.max([c.max(axis=0) for c in boxes], axis=0)
    pts = np.random.default_rng(seed).uniform(lo, hi, size=(n, 3))
    inside = np.zeros(n, bool)
    for c in boxes:
        centre = c.mean(axis=0)
        axes = [c[4] - c[0], c[2] - c[0], c[1] - c[0]]
        q = pts - centre
        ok = np.ones(n, bool)
        for a in axes:
            length = np.linalg.norm(a)
            ok &= np.abs(q @ (a / length)) <= length / 2
        inside |= ok
    return float(inside.mean() * np.prod(hi - lo) * 1e6)


def main() -> None:
    head, denture = T.OBJECTS[0], T.OBJECTS[2]
    origin = np.array([T.XL, 0.0, T.ZM])  # the mouth's centre on the lips' front face
    hp = np.asarray(head.pos, float)
    split = T.OPEN_Z  # the mouth's top edge: the two halves meet here
    lip_lo = T.LIP_LO_Z0 - T.ZM
    chin_lo = np.array([T.XL - 0.010, -0.045, T.ZM - 0.085]) - origin
    chin_hi = np.array([T.XL + T.LIP_T, 0.045, T.ZM - 0.047]) - origin
    pocket_d, pocket_w = 0.010, T.OPEN_Y - 0.001  # the lip's base sits 10 mm down in the chin

    lower, upper = [], []
    for s in head.shapes:
        if not isinstance(s, BoxShape) or not s.collide or tuple(s.color) == tuple(T.WOOD):
            continue  # render-only parts, the bedside table and the cup are not printed
        c = hp + np.asarray(s.centre, float) - origin
        if np.allclose(c - np.asarray(s.size) / 2, chin_lo) and np.allclose(c + np.asarray(s.size) / 2, chin_hi):
            continue  # the chin: written below, with the lip's pocket
        if s.quat() != (0.0, 0.0, 0.0, 1.0):
            lower.append(_corners(c, s.size, s.quat()))  # the ridge's segments, all below the split
            continue
        lo, hi = c - np.asarray(s.size) / 2, c + np.asarray(s.size) / 2
        if hi[2] <= split + 1e-9:
            lower.append(_box(lo, hi))
        elif lo[2] >= split - 1e-9:
            upper.append(_box(lo, hi))
        else:  # crosses the split: cut in two
            lower.append(_box(lo, (hi[0], hi[1], split)))
            upper.append(_box((lo[0], lo[1], split), hi))
    x0, x1, y1, zb, zt = chin_lo[0], chin_hi[0], chin_hi[1], chin_lo[2], chin_hi[2]
    lower += [
        _box((x0, -y1, zb), (x1, y1, lip_lo - pocket_d)),  # under the pocket
        _box((x0, -y1, lip_lo - pocket_d), (0.0, y1, zt)),  # in front of it
        _box((0.0, pocket_w, lip_lo - pocket_d), (x1, y1, zt)),  # beside it
        _box((0.0, -y1, lip_lo - pocket_d), (x1, -pocket_w, zt)),
    ]
    _write("mouth_lower", lower)
    _write("mouth_upper", upper)

    teeth = [_corners(s.centre, s.size, s.quat()) for s in denture.shapes]
    _write("lower_denture", teeth)
    v = _volume_cm3(teeth)
    print(f"lower_denture: {v:.1f} cm3, {v * 1.24:.0f} g in PLA at 100% infill, {v * 1.15:.0f} g in resin")

    # the lip's mould: a 12.4 x 64.4 x 31 mm cavity (21 mm of free lip above the chin, 10 mm in the pocket), 3 mm walls
    cx, cy, cz, w = T.LIP_T + 0.0004, 2 * pocket_w + 0.0004, 0.031, 0.003
    _write("lip_mould", [
        _box((-w, -w, -w), (cx + w, cy + w, 0.0)),
        _box((-w, -w, 0.0), (0.0, cy + w, cz)), _box((cx, -w, 0.0), (cx + w, cy + w, cz)),
        _box((0.0, -w, 0.0), (cx, 0.0, cz)), _box((0.0, cy, 0.0), (cx, cy + w, cz)),
    ])

    # the riser: open at the bottom, 3 mm walls, its top under the head's lower part (whose bottom is 11.5 cm up)
    h, w = T.ZM - 0.085 - T.TOP, 0.003
    xa, xb, ya = 0.000, 0.110, 0.050
    _write("riser", [
        _box((xa, -ya, h - w), (xb, ya, h)),
        _box((xa, -ya, 0.0), (xa + w, ya, h - w)), _box((xb - w, -ya, 0.0), (xb, ya, h - w)),
        _box((xa + w, -ya, 0.0), (xb - w, -ya + w, h - w)), _box((xa + w, ya - w, 0.0), (xb - w, ya, h - w)),
    ])


if __name__ == "__main__":
    main()
