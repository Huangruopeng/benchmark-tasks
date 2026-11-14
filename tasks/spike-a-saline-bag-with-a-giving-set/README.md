# Spike a 500 ml saline bag with an infusion giving set, keeping the spike sterile

A full 500 ml bag of 0.9 % saline lies flat on a smooth stainless-steel tray, its spike port still closed by its blue
tamper-evident protector. Beside it lies a gravity infusion giving set, its spike under a white protective cap. The job
is the one nurses do before every gravity infusion: spike the bag with the set, keeping the spike sterile, and leave
the bag on the tray. That means tearing the protector off the bag's spike port, pulling the cap off the spike, and
pushing the bare spike straight into the port, through the port's diaphragm and on until the set's flange meets the
port's mouth, without the bare spike touching anything else on the way. The task is one task file,
[`spike_a_saline_bag_with_a_giving_set.py`](spike_a_saline_bag_with_a_giving_set.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder. (The file keeps the folder's name; priming is not part of the task, see below.)

The instruction gives the goal and the constraint only ("Spike the saline bag with the giving set, keeping the spike
sterile, and leave the bag on the tray."). Which covers come off, in which order, how hard to push and what to hold are
left to the policy. The demo holds the bag down on the tray with one hand while the other works; nurses also often
steady the port itself, which a rigid bag does not need.

What it tests: three steps that each need a set force, on hand-sized parts at their real size (a 570 g bag; an 11 mm
protector; a 5.6 mm spike under a 10 mm cap): a 10 N pull that tears the protector's seal, a 4 N pull on the spike's
cap, and a 20 N push through the port's diaphragm, lined up to within 3.5 mm of the port's axis; and aseptic
non-touch technique: once its cap is off, the spike may touch nothing but the port's opened end.

Priming is out of scope. Priming fills the drip chamber from the bag, which needs liquid; no liquid is simulated, so
the chamber stays empty and no fill level is scored. The task stops where spiking stops: the spike seated in the port.

Why two hands: the full bag slides on the tray at about 2.5 N (soft PVC on smooth steel, friction 0.45), and the giving
set weighs 27 g. The 10 N pull on the protector and the 20 N push through the diaphragm shove the unheld bag along the
tray, and the 4 N pull on the spike's cap carries the unheld set along with it. In a one-hand trial (the right arm
doing exactly its part of the demo, the left never moving) the pull on the protector dragged the whole bag about 18 cm
along the tray without loading its seal, the spike's cap stayed on, and the capped spike only shoved the bag about:
0.0.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray is kinematic, fixed where it is put; the bag,
its port's sterile end, the protector, the giving set, its spike and the spike's cap are loose rigid bodies made of
shapes from `bodykit.py`.

- **The bag.** A full 500 ml PVC bag (13.0 x 22.5 cm with its welded seams), modelled as one rigid body: a pillow
  11.5 x 18.5 cm and 3.0 cm thick with rounded long sides (602 ml inside), holding 540 ml of saline (500 ml nominal;
  makers overfill, for example 530-565 ml) and about 30 ml of air. Its mass is 570 g: 543 g of saline (1.0046 g/ml) and
  27 g of film, ports and protector. A real full bag is firm but gives a little under the hand; here it does not, and
  its port tubes do not bend.
- **The ports.** Two 8 mm tubes at the bottom seam, 36 mm apart: the spike port (36 mm long) and the medication port
  (20 mm) with its rubber plug. The spike port's outer 26 mm, the part under the protector, is its own body welded to
  the bag, so that the spike can enter it (the giving set and the spike do not collide with it; the joint below is the
  bore) while still colliding with the rest of the bag.
- **The protector.** A blue sleeve 11 mm across and 25 mm long over the port's sterile end, closed 2 mm past the mouth,
  with a flat pull tab. It sits on a slide joint along the port. Its tamper seal holds it like a stiff spring
  (2,500 N/m) until the pull reaches 10 N (4 mm; an estimate of a firm finger pull), then tears for good; after that
  the sleeve slides off against about 1 N, and once clear of the mouth it is free.
- **The spike's cap.** A white cap 10 mm across and 30 mm long, seated on the flange, on a slide joint along the spike:
  it lets go at 4 N (2,000 N/m, 2 mm; an estimate for a push fit), slides off against about 0.5 N and is free once
  past the point.
- **The spike in the port.** The spike (5.6 mm across, 24 mm from the flange to its point) is its own body welded to
  the set. A slide joint between the bag and the set (a PhysX D6 joint) is switched on when the bare point enters the
  port's mouth within 3.5 mm of its axis and 12 degrees of square; its slide is the point's depth, with 0.8 mm of play
  and 3 degrees of tilt. The bore grips the spike: after every control step the joint's spring hold (4,000 N/m) moves
  to wherever the spike has been pushed or pulled more than 0.75 mm past it, so it slides at about 3 N and stays where
  it is left. 10 mm in, the diaphragm holds the point until the push reaches 20 N (5 mm of give), then gives for good;
  20 N is the load at which the standard spike-insertion bench test (ISO/TS 23128) starts.
  At 24 mm the flange meets the mouth (the joint's stop). Twisting the spike, which nurses do to ease it in, neither
  helps nor hinders here.
- **Sterility.** PhysX contact views report the contact forces on the spike from the bag (its film, seams, ports'
  bases and medication port), the tray, the protector, the spike's own cap and every link of both arms, read after
  every physics step (240 Hz). Once the cap has started to come off, a contact force over 0.01 N from the bag, the tray
  or the protector counts as a touch, and so does the spike's point or base reaching the table outside the tray; once
  the cap is fully off, so does one from the cap or either gripper (until then the cap lies between them and the spike,
  and a hard squeeze on the 10 mm cap can press a finger through its 2.2 mm wall in PhysX, which no real squeeze does).
  The port's opened end (the 26 mm its protector covered: its mouth and bore) is the one thing the spike may touch.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step. This copy of `bodykit.py`
also lets a shape be collision only (`color=None`): not drawn, as the clear drip chamber's wall is.

## Scoring, and how a real trial measures it

| Score | Simulation | Real trial |
|---|---|---|
| 0.0, latched | the bare spike touched anything but the port's opened end | the video reviewed frame by frame for any touch of the bare spike, as an assessor marks aseptic non-touch technique |
| 1.0 | at the end: the spike in up to its flange (flange within 1.5 mm of the mouth), the bag lying on the tray, neither gripper touching the bag or the set | flange-to-mouth gap with a caliper or feeler gauge; bag on the tray and hands off it from the last frames |
| 0.75 | at the end: the spike in up to its flange | as above |
| 0.5 | at the end: the spike through the diaphragm (more than 11 mm in) | gap under 13 mm; a squeeze of the bag drips saline into the chamber |
| 0.25 | the protector torn off | the torn tamper seal, by inspection |

The trial ends at the run limit, or once the full score has held for 1 s with the bag, the giving set, the protector,
the cap and both arms still.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the bag, the set, the protector, the cap and both
arms still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see
`task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spike_a_saline_bag_with_a_giving_set.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Spike the saline bag with the giving set, keeping the spike sterile, and leave the bag on the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ivbag-dream
```

The scripted demo, which reaches full score: the left hand comes down on the bag and presses it flat on the tray, in
line with the spike port. The right takes the protector by its sleeve, pulls it straight off the port (its seal
tears) and drops it at the front of the tray. It takes the giving set by its drip chamber and holds it up level, the
spike toward the left. The left lets go of the bag, comes level with the spike from behind its cap, pointing back
along it, takes the cap's end in a gentle grip, pulls it straight off and drops it on the tray, and presses the bag
down again. The
right lines the bare spike up with the port and pushes it straight in, through the diaphragm and on until the flange
meets the port's mouth (the bag, held by the left, gives about 15 mm), eases off, lets go and rises; the left lets go
of the bag, and both arms go back to where they started and rest there:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spike_a_saline_bag_with_a_giving_set.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Spike the saline bag with the giving set, keeping the spike sterile, and leave the bag on the tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ivbag-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/ivbag-dream/*.json; do inspect-robots video "$log"; done
```

Set `IVBAG_DEBUG=1` to print what the score is built from twice a second: the protector's and the cap's slides,
whether each has let go, the spike's depth in the port and how far its point is off the port's axis, whether it is
caught and through the diaphragm, any touches of the bare spike, whether either gripper touches the bag or the set,
and where the hands are.

## Runtime per trial

On one RTX 5090: about 16 minutes for a Dream-1 trial of 2,400 steps (939 s on the clock, with Dream answering in 0.58 s on average), and about 5 minutes for the scripted demo (764 steps, 38.2 s of simulated time, 294 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
