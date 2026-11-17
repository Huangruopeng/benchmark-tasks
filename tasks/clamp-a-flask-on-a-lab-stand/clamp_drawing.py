"""Writes clamp_drawing.svg and clamp_drawing.png: the build drawing of the remote-release spring flask clamp and the
stand layout.

Every number comes from the task file, or from BUILD below for the parts the simulation hides inside the clamp head
and the arm tube, so the drawing, the README and the simulation agree. Run:  python3 clamp_drawing.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import clamp_a_flask_on_a_lab_stand as T  # noqa: E402

MM = 1000.0
# Parts hidden inside the clamp head and the arm tube (mm). The simulation uses only what they add up to: T.JAW_F0,
# T.JAW_K, T.WEDGE, T.STUB, T.JAW_OPEN and T.LEVER_STOP_DEG.
BUILD = {
    "rod_d": 5.0,  # 2 ground steel guide rods along x, the head's full length
    "rod_y": (23.0, 30.0),  # behind the jaws' middle (y)
    "carrier": (20.0, 12.0),  # x, y of each jaw's carrier block (14 mm tall), sliding on both guide rods
    "carrier_y0": 21.0,  # its front face behind the jaws' middle
    "carrier_in": 3.0,  # its inner face from the middle line, jaws shut
    "wall_in": 66.0,  # the head's end walls: inner face from the middle line
    "roller_d": 8.0,  # a 693ZZ bearing (3 x 8 x 4 mm) on a pin at each carrier's inner rear corner
    "roller_x": 2.0,  # its centre this far outside the carrier's inner face ...
    "roller_y": 33.0,  # ... and this far behind the jaws' middle
    "wedge_len": 14.0,  # the wedge, tip to back end (y)
    "spring": {"wire": 0.7, "mean_d": 7.5, "active": 22.6, "free": 55.0, "g": 79.3e3},  # music wire, on the rear rod
    "push_rod": (4.0, 72.0),  # diameter, length (steel), wedge to stub
    "tube": (12.7, 1.65),  # the arm: aluminium tube OD, wall
    "slot": (6.0, 20.0),  # slot in the tube's top for the lever's stub
    "pin": 4.0,
}


def spring_numbers() -> dict:
    sp = BUILD["spring"]
    rate = sp["g"] * sp["wire"] ** 4 / (8 * sp["mean_d"] ** 3 * sp["active"])  # N/mm
    shut = BUILD["wall_in"] - (BUILD["carrier_in"] + BUILD["carrier"][0])
    return {"rate": rate, "shut_len": shut, "open_len": shut - T.JAW_OPEN * MM, "solid": (sp["active"] + 2) * sp["wire"],
            "preload_mm": sp["free"] - shut, "preload_n": T.JAW_F0, "full_n": T.JAW_F0 + T.JAW_K * T.JAW_OPEN}


def wedge_tip(opening: float) -> float:
    """Where the wedge's tip is (mm behind the jaws' middle) with each jaw opened ``opening`` mm: its flank
    (x = 1 + WEDGE * (y - tip)) touches the roller where the flank's normal passes through the roller's centre."""
    b = BUILD
    r, w = b["roller_d"] / 2, T.WEDGE
    n = math.hypot(1.0, w)
    cx, cy = b["carrier_in"] + opening + b["roller_x"], b["roller_y"]
    px, py = cx - r / n, cy + r * w / n  # contact point on the roller
    return py - (px - 1.0) / w


def _head(ax, opening: float, title: str) -> None:
    """Plan view of the clamp head (x across, y up = toward the stand), each jaw opened ``opening`` mm."""
    b = BUILD
    hw, y0, y1 = T.HEAD[0] * MM / 2, T.YOKE_Y * MM, (T.YOKE_Y + T.HEAD[1]) * MM
    ax.add_patch(Rectangle((-hw, y0), 2 * hw, y1 - y0, fc="#e8ecf0", ec="k"))
    ax.add_patch(Rectangle((b["wall_in"], y0), hw - b["wall_in"], y1 - y0, fc="#c9d1da", ec="k"))
    ax.add_patch(Rectangle((-hw, y0), hw - b["wall_in"], y1 - y0, fc="#c9d1da", ec="k"))
    for ry in b["rod_y"]:
        ax.plot([-hw + 2, hw - 2], [ry, ry], color="#666", lw=2.2)
    wedge_travel = opening / T.WEDGE
    for side in (1, -1):
        cin = b["carrier_in"] + opening
        cx0 = cin if side > 0 else -cin - b["carrier"][0]
        ax.add_patch(Rectangle((cx0, b["carrier_y0"]), b["carrier"][0], b["carrier"][1], fc="#b7c6d6", ec="k"))
        face = T.GAP0 * MM / 2 + opening
        px = face if side > 0 else -face - T.PAD[0] * MM
        ax.add_patch(Rectangle((px, T.PAD_Y[0] * MM), T.PAD[0] * MM, (T.PAD_Y[1] - T.PAD_Y[0]) * MM, fc="#d9b38c",
                               ec="k"))
        ax_ = face + T.PAD[0] * MM if side > 0 else -face - T.PAD[0] * MM - 8
        ax.add_patch(Rectangle((ax_, T.PAD_Y[0] * MM), 8, b["carrier_y0"] - T.PAD_Y[0] * MM, fc="#9aa3ad", ec="k"))
        s0, s1 = cin + b["carrier"][0], b["wall_in"]
        n = 16
        xs = [side * (s0 + (s1 - s0) * k / n) for k in range(n + 1)]
        ys = [b["rod_y"][1] + (2.5 if k % 2 else -2.5) for k in range(n + 1)]
        ax.plot(xs, ys, color="#b03030", lw=1.2)
        ax.add_patch(Circle((side * (cin + b["roller_x"]), b["roller_y"]), b["roller_d"] / 2, fc="#eee", ec="k",
                            zorder=5))
    # the wedge: flanks of slope WEDGE (x per y) touch the rollers' inner rear sides; pushed toward the jaws (-y)
    tip, back = wedge_tip(opening), wedge_tip(opening) + b["wedge_len"]
    hb = 1 + T.WEDGE * b["wedge_len"]
    ax.add_patch(Polygon([(-1, tip), (1, tip), (hb, back), (-hb, back)], closed=True, fc="#aaa", ec="k", zorder=4))
    ax.plot([0, 0], [back, y1 + 25], color="#444", lw=3)
    ax.add_patch(Circle((0, 0), T.NECK_R * MM, fc="none", ec="#2a8", ls="--"))
    ax.text(0, -2, "neck\n25", ha="center", va="center", fontsize=7)
    ax.set_title(title, fontsize=9)
    ax.set_xlim(-hw - 4, hw + 4)
    ax.set_ylim(T.PAD_Y[0] * MM - 10, y1 + 28)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25)


def _lever(ax) -> None:
    """Side view from the right: y across (toward the stand to the right), z up; the tube's axis at z = 0."""
    b = BUILD
    py, pz = T.PIVOT[1] * MM, T.STUB * MM
    ax.add_patch(Rectangle(((T.YOKE_Y + T.HEAD[1]) * MM, -b["tube"][0] / 2), (T.ARM_END_Y - T.YOKE_Y - T.HEAD[1]) * MM,
                           b["tube"][0], fc="#cfd3d8", ec="k"))
    ax.add_patch(Rectangle((T.YOKE_Y * MM, -T.HEAD[2] * MM / 2), T.HEAD[1] * MM, T.HEAD[2] * MM, fc="#e8ecf0", ec="k"))
    ax.add_patch(Rectangle((py - b["slot"][1] / 2, b["tube"][0] / 2 - 1.2), b["slot"][1], 1.2, fc="w", ec="k"))
    ax.plot([py - 75, py - 3], [0, 0], color="#444", lw=2.5)  # the push rod, inside the tube
    ax.add_patch(Rectangle((py - 7, b["tube"][0] / 2), 14, pz - b["tube"][0] / 2 + 5, fc="#dfe5ea", ec="k"))
    for deg, col, ls in ((0.0, "#b02020", "-"), (T.LEVER_STOP_DEG, "#d07070", "--")):
        a = math.radians(deg)
        ex, ez = T.LEVER_LEN * MM * math.cos(a), -T.LEVER_LEN * MM * math.sin(a)
        ax.plot([py, py + ex], [pz, pz + ez], color=col, lw=4, ls=ls)
        sx, sz = -T.STUB * MM * math.sin(a), -T.STUB * MM * math.cos(a)
        ax.plot([py, py + sx], [pz, pz + sz], color=col, lw=3, ls=ls)
        cx, cz = T.PAD_ARM * MM * math.cos(a), -T.PAD_ARM * MM * math.sin(a)
        nx, nz = math.sin(a), math.cos(a)
        ax.add_patch(Polygon([(py + cx - 12.5 * math.cos(a) + 3 * nx, pz + cz + 12.5 * math.sin(a) + 3 * nz),
                              (py + cx + 12.5 * math.cos(a) + 3 * nx, pz + cz - 12.5 * math.sin(a) + 3 * nz),
                              (py + cx + 12.5 * math.cos(a) + 9 * nx, pz + cz - 12.5 * math.sin(a) + 9 * nz),
                              (py + cx - 12.5 * math.cos(a) + 9 * nx, pz + cz + 12.5 * math.sin(a) + 9 * nz)],
                             closed=True, fc="#222" if deg == 0 else "#888", ec="k"))
    ax.add_patch(Circle((py, pz), 3, fc="w", ec="k", zorder=5))
    a = math.radians(T.LEVER_STOP_DEG)
    ax.annotate("", (py + T.PAD_ARM * MM * math.cos(a), pz - T.PAD_ARM * MM * math.sin(a) + 9),
                (py + T.PAD_ARM * MM, pz + 9), arrowprops={"arrowstyle": "->", "color": "#06c"})
    ax.set_title(f"Lever, side view: at rest (solid) and on its {T.LEVER_STOP_DEG:.0f} deg stop (dashed)", fontsize=9)
    ax.set_xlim(0, (T.ARM_END_Y + 0.005) * MM)
    ax.set_ylim(-20, 50)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25)
    ax.set_xlabel("mm from the jaws' middle, toward the stand", fontsize=7)


def _layout(ax) -> None:
    """Plan view in the task's frame: x forward from the arm bases' line, y left."""
    def P(x, y):  # draw with y to the left
        return -y * MM, x * MM

    bx, by = T.BASE_C
    ax.add_patch(Rectangle(P(bx - T.BASE[0] / 2, by + T.BASE[1] / 2), T.BASE[1] * MM, T.BASE[0] * MM, fc="#555",
                           ec="k"))
    ax.add_patch(Circle(P(*T.ROD_XY), T.ROD_R * MM * 1.5, fc="#ddd", ec="k", zorder=4))
    ax.plot(*zip(P(T.CROSS_X0, T.ROD_XY[1]), P(T.CROSS_X0 + T.CROSS_LEN, T.ROD_XY[1])), color="#999", lw=4)
    ax.plot(*zip(P(T.CX, T.YOKE_Y + T.HEAD[1]), P(T.CX, T.ARM_END_Y)), color="#9ab", lw=4)
    hx, hy = P(T.CX - T.HEAD[0] / 2, T.YOKE_Y + T.HEAD[1])
    ax.add_patch(Rectangle((hx, hy), T.HEAD[1] * MM, T.HEAD[0] * MM, fc="#cfd8e3", ec="k"))
    for name, xy in (("boss head", (T.ROD_XY[0], T.ROD_XY[1])), ("boss head", (T.CX, T.ROD_XY[1]))):
        c = P(*xy)
        ax.add_patch(Rectangle((c[0] - 15, c[1] - 15), 30, 30, fc="#bbb", ec="k", zorder=3))
    pad = P(T.CX + T.LEVER_DX, T.PIVOT[1] + T.PAD_ARM)
    ax.add_patch(Rectangle((pad[0] - 12.5, pad[1] - 12.5), 25, 25, fc="#222", ec="k", zorder=4))
    ax.add_patch(Circle(P(T.CX, 0.0), T.BULB_R * MM, fc="none", ec="#2a8", lw=1.5))
    ax.add_patch(Circle(P(*T.RING[0]), T.RING[2] * MM, fc="#d9b38c", ec="k"))
    ax.add_patch(Circle(P(*T.RING[0]), T.BULB_R * MM, fc="none", ec="#2a8"))
    for side in ("left", "right"):
        bxy = P(0.0, 0.305 if side == "left" else -0.305)
        ax.add_patch(Rectangle((bxy[0] - 40, bxy[1] - 40), 80, 80, fc="#eee", ec="#888"))
        ax.text(bxy[0], bxy[1], f"{side}\narm", ha="center", va="center", fontsize=7)
    ax.text(*P(T.ROD_XY[0] + 0.02, T.ROD_XY[1] + 0.09), f"rod {2 * T.ROD_R * MM:.1f} x {T.ROD_H * MM:.0f}", fontsize=7)
    ax.text(*P(T.BASE_C[0] - 0.13, T.BASE_C[1] + 0.07), f"base {T.BASE[1] * MM:.0f} x {T.BASE[0] * MM:.0f} x "
            f"{T.BASE[2] * MM:.0f}", fontsize=7)
    ax.text(*P(T.ROD_XY[0] + 0.035, T.ROD_XY[1] - 0.04), "crossbar 12.7 x 305", fontsize=7)
    ax.text(*P(T.CX + 0.045, T.CX * 0 + 0.05), "flask hangs\nhere", fontsize=7, ha="center")
    ax.text(*P(T.CX - 0.05, T.PIVOT[1] + 0.1), "lever pad", fontsize=7)
    ax.text(*P(T.RING[0][0] - 0.06, T.RING[0][1] + 0.03), "cork ring\n(flask at start)", fontsize=7)
    ax.set_title("Layout, plan view (x forward from the arm bases, y left; mm)", fontsize=9)
    ax.set_xlim(-400, 400)
    ax.set_ylim(-60, 560)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25)
    ax.set_xticks(range(-400, 401, 100), [str(-v) for v in range(-400, 401, 100)])


def build_notes() -> list[str]:
    """The build specification in words, every number from the code (also pasted into the README)."""
    sp = spring_numbers()
    b = BUILD
    a = math.radians(T.LEVER_STOP_DEG)
    return [
        f"Clamp head: aluminium {T.HEAD[0] * MM:.0f} (x) x {T.HEAD[1] * MM:.0f} (y) x {T.HEAD[2] * MM:.0f} (z), its front "
        f"face {T.YOKE_Y * MM:.0f} behind the jaws' middle line (the neck's stop), end walls' inner faces "
        f"{b['wall_in']:.0f} either side of the middle; the arm tube enters its back face.",
        f"Guide rods: 2 x {b['rod_d']:.0f} mm ground steel, {T.HEAD[0] * MM:.0f} long, {b['rod_y'][0]:.0f} and "
        f"{b['rod_y'][1]:.0f} behind the middle line, 3 below the head's mid-plane. Jaw carriers: "
        f"{b['carrier'][0]:.0f} (x) x {b['carrier'][1]:.0f} (y) x 14 (z), front face {b['carrier_y0']:.0f} behind the "
        f"middle line, inner faces {b['carrier_in']:.0f} from it with the jaws shut.",
        f"Jaws: 8 mm aluminium arms from the carriers forward to {abs(T.PAD_Y[0]) * MM:.0f} in front of the middle line, "
        f"cork pads {T.PAD[1] * MM:.0f} long x {T.PAD[2] * MM:.0f} tall x {T.PAD[0] * MM:.0f} thick; shut on their stops "
        f"{T.GAP0 * MM:.0f} apart, fully open {(T.GAP0 + 2 * T.JAW_OPEN) * MM:.1f} apart ({T.JAW_OPEN * MM:.2f} each).",
        f"Springs, one per jaw on the rear rod between carrier and end wall: music wire {b['spring']['wire']} mm, mean "
        f"coil {b['spring']['mean_d']} mm (bore {b['spring']['mean_d'] - b['spring']['wire']:.1f}), {b['spring']['active']} "
        f"active coils, ends closed and ground, free length {b['spring']['free']:.0f}, solid {sp['solid']:.1f}; rate "
        f"G d^4 / (8 D^3 n) = {sp['rate']:.2f} N/mm (G = 79.3 GPa); {sp['shut_len']:.0f} long with the jaws shut "
        f"({sp['preload_mm']:.0f} compressed, {sp['preload_n']:.1f} N), {sp['open_len']:.2f} long fully open "
        f"({sp['full_n']:.2f} N).",
        f"Rollers: a 693ZZ bearing (3 x 8 x 4 mm) on a pin at each carrier's inner rear corner, centre "
        f"{b['roller_x']:.0f} outside the carrier's inner face and {b['roller_y']:.0f} behind the middle line, 2 to 6 mm "
        f"above the head's mid-plane.",
        f"Wedge: 4 mm steel plate in the rollers' layer, {b['wedge_len']:.0f} long, flanks {math.degrees(math.atan(T.WEDGE)):.1f}"
        f" deg from the push rod (each jaw opens {T.WEDGE:.1f} mm per mm the rod moves); tip {wedge_tip(0):.1f} behind "
        f"the middle line at rest, {wedge_tip(T.JAW_OPEN * MM):.1f} on the stop; it clears the carriers by "
        f"{b['carrier_in'] + T.JAW_OPEN * MM - (1 + T.WEDGE * (b['carrier_y0'] + b['carrier'][1] - wedge_tip(T.JAW_OPEN * MM))):.1f}"
        f" fully open. Push rod: {b['push_rod'][0]:.0f} x {b['push_rod'][1]:.0f} steel inside the arm tube "
        f"({b['tube'][0]} OD x {b['tube'][1]} wall aluminium, {(T.ARM_END_Y - T.YOKE_Y - T.HEAD[1]) * MM:.0f} long behind "
        "the head).",
        f"Lever: 10 x 6 aluminium bell crank on a {b['pin']:.0f} mm steel pin in a split collar clamped on the tube, "
        f"{T.PIVOT[1] * MM:.0f} behind the middle line and {T.STUB * MM:.0f} above the tube's axis. Its stub reaches "
        f"{T.STUB * MM:.0f} down through a {b['slot'][0]:.0f} x {b['slot'][1]:.0f} slot in the tube's top to the push rod; "
        f"its arm runs {T.LEVER_LEN * MM:.1f} toward the stand, cranked {T.LEVER_DX * MM:.1f} to the side of the tube, "
        f"with a {T.PAD_LEVER[0] * MM:.0f} x {T.PAD_LEVER[1] * MM:.0f} x {T.PAD_LEVER[2] * MM:.0f} rubber pad centred "
        f"{T.PAD_ARM * MM:.0f} from the pivot.",
        f"Stops: at rest 0 deg (the crank on the collar); pressed {T.LEVER_STOP_DEG:.0f} deg (an M4 stop screw in the "
        f"collar). On the stop the pad is {T.PAD_ARM * MM * math.sin(a):.1f} lower and the push rod "
        f"{T.STUB * MM * math.sin(a):.1f} nearer the jaws. Pad force {T.pad_force(0):.1f} N to start it, "
        f"{T.pad_force(T.LEVER_STOP_DEG):.1f} N on the stop (linkage efficiency {T.ETA} assumed).",
        f"Mounting: the arm tube in a boss head on a 1/2 in (12.7) x 12 in (305) support rod used as a crossbar, "
        f"{(T.JAW_Z - T.CROSS_Z) * MM:.0f} below the tube; the crossbar in a second boss head on the stand's "
        f"{2 * T.ROD_R * MM:.1f} x {T.ROD_H * MM:.0f} rod. The jaws' middle is {(T.JAW_Z - T.TOP) * MM:.0f} above the table.",
    ]


def main() -> None:
    sp = spring_numbers()
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Remote-release spring flask clamp: build drawing (mm)", fontsize=14, fontweight="bold")
    _head(fig.add_axes([0.03, 0.62, 0.30, 0.30]), 0.0, "Clamp head, plan view, jaws shut on their stops")
    _head(fig.add_axes([0.35, 0.62, 0.30, 0.30]), T.JAW_OPEN * MM,
          f"Lever on its stop: wedge {T.JAW_OPEN * MM / T.WEDGE:.1f} in, jaws {(T.GAP0 + 2 * T.JAW_OPEN) * MM:.1f} apart")
    _lever(fig.add_axes([0.03, 0.30, 0.62, 0.30]))
    _layout(fig.add_axes([0.67, 0.30, 0.31, 0.62]))
    import textwrap

    wrapped = [textwrap.fill(n, 200, subsequent_indent="    ") for n in build_notes()]
    fig.text(0.03, 0.27, "\n".join(wrapped), fontsize=8.2, va="top", linespacing=1.5)
    fig.savefig(HERE / "clamp_drawing.svg")
    fig.savefig(HERE / "clamp_drawing.png", dpi=90)
    print({k: round(v, 3) for k, v in sp.items()})


if __name__ == "__main__":
    main()
