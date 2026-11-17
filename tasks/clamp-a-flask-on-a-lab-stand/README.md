# Clamp a round-bottom flask on a lab stand

A lab stand (a 6 x 9 in cast-iron base and a 24 in steel rod) stands at the back left of the table. Through two boss
heads and a 1/2 in crossbar it holds a spring flask clamp whose arm reaches toward the middle of the table and ends in
two cork-lined jaws. Springs keep the jaws shut; a red lever on the arm opens them while it is pressed down. An empty
100 mL round-bottom flask stands in a cork ring to the right.

The instruction states only the goal: clamp the flask by its neck so it hangs clear of the table, without dropping or
tipping it, and take both arms back. In the scripted demo, one hand presses the lever down to its stop and keeps it
there. The other lifts the flask out of its cork ring by its neck, slides the neck in between the open jaws against the
clamp head, and lowers it to the jaws' height. The lever is let up, so the jaws close on the neck. The flask is let go,
hanging in the clamp, and both arms go back to where they started.

The task is one task file, [`clamp_a_flask_on_a_lab_stand.py`](clamp_a_flask_on_a_lab_stand.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder. [`clamp_drawing.py`](clamp_drawing.py) writes the clamp's build drawing,
[`clamp_drawing.png`](clamp_drawing.png) and [`clamp_drawing.svg`](clamp_drawing.svg), from the task file's numbers.

**What it tests:**
- holding a spring mechanism open while placing a fragile part into it with the other hand;
- carrying glass without dropping or tipping it;
- the right order: the jaws must close before the flask is let go;
- finishing with both hands free.

**Why two hands:** the jaws are open only while the lever is held down, and the neck has to be between them as they
close. With one hand, the jaws are shut whenever that hand holds the flask, so the neck cannot get in. A neck pushed at
the shut jaws can't force them apart: their corners meet a centred 25 mm neck at asin(1/12.5) = 4.6 degrees, so even a
26 N push gives each jaw only 13 x tan 4.6 = 1.05 N sideways, below its 3.0 N spring preload. The one-hand check scored
0: the right did its part as in the demo while the left never moved. The neck stopped at the shut jaws' front edge
(their gap stayed 2.0 mm), the flask tilted 15 degrees in the hand and fell when it was let go.

## The real hardware

Clamping a round-bottom flask by its neck on a stand is a common wet-lab step. A widely used organic-lab manual says the
clamp "should securely grasp the neck of a round bottomed flask below the glass protrusion" (Nichols, Organic Chemistry
Lab Techniques, 1.2C Clamping, LibreTexts, CC BY-NC-ND 4.0), and the flask is let go only once the clamp holds it. Most
stand clamps are tightened with a thumbscrew; spring-closing lab clamps exist for burettes. No ready-made flask clamp
has spring-closed jaws with a release lever on its arm, so this one is specified completely for building below.
Everything else is bought. The lever sits on the arm 12 cm behind the jaws so that the hand on it stays clear of the
hand bringing the flask (two YAM wrists closer than about 15 cm collide); it doesn't make the job easier: the jaws are
open only while it is held.

**Bought (ordinary lab items):**
- Lab stand: "Support Stand with a 6" by 9" Cast-Iron Base and a 24" by 1/2" Rod" (MPN 4-SSC16; Geyer Instructional
  SKU 251554, US$25.33 in Nov 2026:
  <https://www.geyerinstructional.com/support-stand-with-a-6-by-9-cast-iron-base-and-a-24-by-1-2-rod>). The listing
  gives a 6 x 9 in enamelled cast-iron base and a 24 in x 1/2 in zinc-plated steel rod, overall 6.25 x 9.125 x 24.5 in.
  The base's 1.3 cm thickness is derived from that (24.5 in overall minus the 24 in rod), not measured.
- Two boss heads for 1/2 in rods, e.g. Bel-Art H18350-0000 ("open-sided stainless steel bosshead accepts rods up to
  13 mm (1/2 in) diameter. Rods are held at right angles to one another and secured with flat wing-head clamping
  screws", Bel-Art clamps catalogue, p. 61:
  <https://www.novatech-usa.com/pdf/Bel-Art%20Clamps%20and%20Holders%20Catalog.pdf>). Any 1/2 in boss head works.
- A 1/2 in x 12 in steel support rod as the crossbar.
- An empty 100 mL borosilicate round-bottom flask with a 19/22 joint, and an 80 mm cork ring.

**Build drawing of the clamp** (all mm; the same numbers are drawn in `clamp_drawing.png`). The jaws' middle line runs
forward through the flask's neck; "behind" means toward the stand along the arm.
- Clamp head: aluminium 150 (x) x 30 (y) x 16 (z), its front face 20 behind the jaws' middle line (the neck's stop), end walls' inner faces 66 either side of the middle; the arm tube enters its back face.
- Guide rods: 2 x 5 mm ground steel, 150 long, 23 and 30 behind the middle line, 3 below the head's mid-plane. Jaw carriers: 20 (x) x 12 (y) x 14 (z), front face 21 behind the middle line, inner faces 3 from it with the jaws shut.
- Jaws: 8 mm aluminium arms from the carriers forward to 22 in front of the middle line, cork pads 40 long x 16 tall x 6 thick; shut on their stops 2 apart, fully open 39.5 apart (18.75 each).
- Springs, one per jaw on the rear rod between carrier and end wall: music wire 0.7 mm, mean coil 7.5 mm (bore 6.8), 22.6 active coils, ends closed and ground, free length 55, solid 17.2; rate G d^4 / (8 D^3 n) = 0.25 N/mm (G = 79.3 GPa); 43 long with the jaws shut (12 compressed, 3.0 N), 24.25 long fully open (7.69 N).
- Rollers: a 693ZZ bearing (3 x 8 x 4 mm) on a pin at each carrier's inner rear corner, centre 2 outside the carrier's inner face and 33 behind the middle line, 2 to 6 mm above the head's mid-plane.
- Wedge: 4 mm steel plate in the rollers' layer, 14 long, flanks 56.3 deg from the push rod (each jaw opens 1.5 mm per mm the rod moves); tip 35.1 behind the middle line at rest, 22.6 on the stop; it clears the carriers by 5.2 fully open. Push rod: 4 x 72 steel inside the arm tube (12.7 OD x 1.65 wall aluminium, 200 long behind the head).
- Lever: 10 x 6 aluminium bell crank on a 4 mm steel pin in a split collar clamped on the tube, 120 behind the middle line and 25 above the tube's axis. Its stub reaches 25 down through a 6 x 20 slot in the tube's top to the push rod; its arm runs 62.5 toward the stand, cranked 22.5 to the side of the tube, with a 25 x 25 x 6 rubber pad centred 50 from the pivot.
- Stops: at rest 0 deg (the crank on the collar); pressed 30 deg (an M4 stop screw in the collar). On the stop the pad is 25.0 lower and the push rod 12.5 nearer the jaws. Pad force 5.0 N to start it, 12.8 N on the stop (linkage efficiency 0.9 assumed).
- Mounting: the arm tube in a boss head on a 1/2 in (12.7) x 12 in (305) support rod used as a crossbar, 25 below the tube; the crossbar in a second boss head on the stand's 12.7 x 610 rod. The jaws' middle is 95 above the table.

The head, carriers, jaw arms and lever are simple aluminium parts (or PETG prints at full infill); the rest is stock
hardware: two 5 mm ground rods, two springs to the specification above (any spring maker's stock or made to order),
two 693ZZ bearings, a 4 mm pin, a 4 mm rod, an M4 screw, 6 mm cork sheet and a rubber pad. The springs' rate is
computed from the wire, coil and turns; nothing here was measured on a built clamp.

**Forces, travel and work, checked** (the task file's `lever_torque` and `pad_force` compute them):
- Each jaw opens 25 (stub) x 1.5 (wedge) x sin(lever angle) = 37.5 sin(angle) mm; on the 30 degree stop that is
  18.75 mm, so the jaws go from 2 to 2 + 2 x 18.75 = 39.5 mm apart.
- Each jaw spring: 3.0 N shut; 3.0 + 0.25 x 18.75 = 7.69 N fully open; 3.0 + 0.25 x 11.5 = 5.88 N on the 25 mm neck
  (each jaw 11.5 mm from its stop).
- Work to open both jaws fully: 2 x (3.0 x 18.75 + 0.5 x 0.25 x 18.75^2) = 2 x (56.25 + 43.95) = 200.4 mJ = 0.200 J.
- At the pad, each jaw moves 37.5 / 50 = 0.75 mm per mm of pad travel, so the pad needs 2 x 0.75 / 0.9 = 1.67 times the
  jaw spring force: 1.67 x 3.0 = 5.0 N to start and 1.67 x 7.69 = 12.8 N on the stop, over 50 x sin 30 = 25 mm of
  travel. Both forces grow linearly with the pad's travel, so the work done at the pad is (5.0 + 12.8) / 2 x 25 mm =
  0.223 J = 0.200 J / 0.9. Lever work and spring work agree.
- The earlier version's 2 N lever over 1 cm (0.02 J) could not open its two 2,000 N/m jaws; those numbers are gone.
- Holding the flask: 5.88 N per jaw on the neck. Cork on glass at an assumed friction coefficient of 0.5 holds up to
  2 x 0.5 x 5.88 = 5.9 N, 6.7 times the empty flask's 0.88 N weight.
- The robot presses at most 12.8 N on the pad, well within a YAM finger's ~40 N.
- The stand can't tip when the lever is pressed: the pad (centre 30.25 cm forward, 17.0 cm left, 16.6 cm on its stop) lies over the base
  (27.6 to 50.5 cm forward, 15.4 to 30.6 cm left), so the press pushes the base down. The hanging flask (0.88 N, 15.4 cm
  outside the base's edge, 0.14 N m) is far less than the stand's own weight resists. In a real trial the base is held
  by a C-clamp or a 2 kg weight so that it can't shift on the table.

## Real-trial safety and rig protection

- The flask is empty borosilicate glass. Work over a 5 mm rubber bench mat, so a flask dropped from the carrying height
  (about 15 cm) is unlikely to break. If one does break, stop the trial, sweep up the glass with a brush and dustpan,
  and wipe the gripper pads and the mat before the next trial.
- The grippers touch only the dry glass neck and the lever's rubber pad: nothing can contaminate them. Deburr the
  clamp's aluminium edges. The jaws close with at most 7.7 N each, which can't harm a finger pad.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stand (base, rod, boss heads, crossbar, the
clamp's arm tube and head, the lever's bracket) is one kinematic body made of shapes from `bodykit.py`. The two jaws,
the lever and the flask are rigid bodies.

- **The jaws:** each jaw is a slider across the clamp's arm. Its spring drive (250 N/m) has its target 12 mm past the
  shut stop, so a jaw clear of the wedge is pushed shut with 3.0 N + 250 N/m x its opening, as the real spring does.
- **The linkage:** the push rod and the wedge are not bodies. After every control step the task file computes where
  the wedge holds each jaw (37.5 mm x sin of the lever's angle) and moves the jaw's drive target there while the jaw
  rests on the wedge. The wedge only pushes: a jaw held wider by the neck feels only its spring.
- **The lever:** a hinge with a spring of 0.0102 N m per degree aimed 24.6 degrees above its rest stop, so it presses
  up with 0.25 N m at rest and 0.555 N m on its 30 degree stop: the jaw springs' torque through the linkage at both
  ends (between them the simulated spring is within 8% of it).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s; `success()` means full score (which
includes both arms back where they started) with the flask still. Otherwise it gives `score()` at the end of the
episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clamp_a_flask_on_a_lab_stand.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Clamp the round-bottom flask by its neck in the lab stand's clamp, so that it hangs there clear of the table, without dropping or tipping it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flask-dream
```

The scripted demo reaches full score:
1. The left hand comes down on the lever's pad with its fingers closed and presses it down to its stop: the jaws open.
2. The right hand comes straight down over the flask, fingers closing across the neck just under the lip, and lifts it
   out of the cork ring.
3. The right carries the flask to the front of the clamp and slides the neck in between the open jaws, against the
   clamp head. It lowers the flask 1 cm.
4. The left lets the lever up: the jaws close on the neck.
5. The right lets go, rises and draws back toward the arms, its open fingers passing either side of the neck.
6. Both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clamp_a_flask_on_a_lab_stand.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Clamp the round-bottom flask by its neck in the lab stand's clamp, so that it hangs there clear of the table, without dropping or tipping it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flask-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/flask-dream/*.json; do inspect-robots video "$log"; done
```

Set `FLASK_DEBUG=1` to print what the score is built from twice a second:
- the lever's angle, how far the wedge holds the jaws open, and the jaws' gap;
- where the neck is against the jaws, and the flask's tilt and height above the table;
- whether the neck has been between the open jaws, whether the flask is clamped, and whether it was dropped;
- where each TCP is;
- whether both arms are home.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 900 steps (346 s on the clock, with Dream answering in 0.70 s on average), and about 2 minutes for the scripted demo (378 steps, 18.9 s of simulated time, 130 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Third-party assets

None. Every shape is a box, cylinder or sphere built by the task file; the arms are yam_arena's own.

## Changes in this revision

This is a revision of TK-2RTKP9. Its review said:

> **Not accepted** (TK-2RTKP9).
>
> Clarity and reproducibility (not met): The custom remote-lever clamp is the task's main mechanism but has neither build drawings nor a named off-the-shelf equivalent.
>
> Feasibility (not met): Opening both specified 2,000 N/m jaws requires roughly 0.6 J, far more than the approximately 0.02 J supplied by a 2 N lever moving 1 cm.
>
> What to change: The clamp's specified force and travel are not physically consistent: opening both jaws requires substantially more work than the 2 N lever can supply over 1 cm. Use a realizable laboratory clamp, provide a commercial model or build drawings for its mechanism, and document its actual lever travel and forces. Update the simulation, setup and demonstration to match the real hardware while retaining the flask-placement task.

There was no earlier review of this task.

What changed in response:
- **Build drawings (Clarity and reproducibility):** no ready-made flask clamp has spring-closed jaws with a release lever
  on its arm, so the clamp is now fully specified for building: a dimensioned drawing (`clamp_drawing.png` and `.svg`,
  generated from the task file's numbers by `clamp_drawing.py`) and the written specification above, covering the
  head, guide rods, carriers, jaws, springs (wire, coil, turns, free length, rate), rollers, wedge, push rod, lever,
  stops and mounting. The stand (MPN 4-SSC16, US$25.33) and the boss heads (Bel-Art H18350-0000) are named products.
- **Consistent force and travel (Feasibility):** the mechanism is a 25 mm bell-crank stub pushing a wedge (1.5 mm of
  jaw per mm of rod) against two 0.25 N/mm springs with 3.0 N preload. Opening both jaws fully (18.75 mm each) takes
  0.200 J; the pad supplies it with 5.0 N rising to 12.8 N over 25 mm, 0.223 J, which is 0.200 J at the stated 0.9
  linkage efficiency. The arithmetic is above and in `lever_torque` and `pad_force`.
- **Actual lever travel and forces documented:** 30 degrees to a stop screw, 25 mm at the pad, 5.0 to 12.8 N; jaws 2 to
  39.5 mm apart; 5.88 N per jaw on the 25 mm neck.
- **Simulation, setup and demonstration updated to this hardware:** each jaw's drive is now the real spring (3.0 N
  preload, 250 N/m) and follows the wedge only while it rests on it; the lever's hinge spring carries the jaw springs'
  load (0.25 N m at rest, 0.555 N m on its stop); the 16 cm rod became the real stand's 61 cm rod, placed at the back
  left with a crossbar so the arms clear it; the flask's bulb is the real 6.4 cm. The demo presses the lever down its
  30 degree arc to the stop, and re-scores 1.0 (see the scores). The flask-placement task is unchanged.
- **Also:** the setup and scoring state how a real trial checks each condition (video for drops, tips and the open-jaw
  moment; a side photo with a protractor card and a 5 mm gauge block at the end), and the context places the task in
  laboratory research automation.
