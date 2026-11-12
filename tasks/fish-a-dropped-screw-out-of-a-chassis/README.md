# Fish a screw that slid under a circuit board out of an open chassis and into the magnetic parts dish

An open steel equipment chassis lies on the table, its right end propped on a wooden block, so it slopes gently down to
the left; a screw dropped inside has slid under the circuit board covering the left half of its floor, where nothing can
reach it. One hand lifts the chassis's left end by its handle, tipping it the other way, so the screw slides out to the
open right end, and holds it there. The other takes a magnetic pick-up tool out of its holder, picks the screw up with
the magnet at its foot and touches it into a magnetic parts dish. Then the tool goes back in its holder, the chassis is
set back down on its block and both arms go back to where they started. The task is one task file,
[`fish_a_dropped_screw_out_of_a_chassis.py`](fish_a_dropped_screw_out_of_a_chassis.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding a load in a set pose with one hand while the other works a tool into it; carrying a small part
clear of obstacles and handing it over to a dish that takes it; putting everything back.

Why two hands: the screw only comes out from under the board while the chassis is held tipped: set down, the chassis
slopes back to the left and the screw slides back under the board. With one hand (the right doing its part as in the
demo, the left never moving) the demo scored 0: the chassis stayed sloping to the left, the screw stayed under the
circuit board, 24 cm from the magnet, and the tool came back empty.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The chassis: one rigid body (floor, walls, the circuit board on its standoffs, four rubber feet and a carrying
  handle), lying on the table and on a fixed block, so that it slopes 6 degrees down to the left. Lifted by its handle,
  it turns about its right feet on the block.
- The screw: a 2.5 g rigid body (an M4 x 14 mm pan-head screw) with friction 0.06 on the chassis's floor, as on the
  PTFE sheet that lines a real trial's floor, so it slides down any slope steeper than about 3.4 degrees: the chassis
  at rest slopes 6 degrees (the screw stays under the board) and tipped by its handle about 9 degrees the other way.
- The magnet: a fixed joint (a `Glue` from `bodykit.py`) between the tool and the screw, authored switched off. After
  every control step the task file checks how far the screw is from the magnet's face; within 2 cm it sets the joint's
  frames to hold the screw where it is and switches it on. Touched into the parts dish (the screw's middle inside the
  dish, less than 1.5 cm over its rim), the joint switches off and the screw drops into the dish.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the screw, the tool and the chassis still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fish_a_dropped_screw_out_of_a_chassis.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "A screw has slid under the circuit board in this open chassis. Get it out and into the magnetic parts dish, then leave the pick-up tool back in its holder and the chassis back on its block, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/screw-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the chassis's handle, fingers closing
across its bar, and lifts the left end along its arc until the chassis slopes about 9 degrees down to the right; the
screw slides out to the right end. The right hand takes the pick-up tool out of its holder by its handle, brings the
magnet down on the screw, lifts it straight up clear of the walls, carries it over the parts dish and lowers it in; the
dish takes the screw. The right puts the tool back in its holder, the left lowers the chassis back onto its block and
lets go, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fish_a_dropped_screw_out_of_a_chassis.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "A screw has slid under the circuit board in this open chassis. Get it out and into the magnetic parts dish, then leave the pick-up tool back in its holder and the chassis back on its block, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/screw-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/screw-dream/*.json; do inspect-robots video "$log"; done
```

Set `SCREW_DEBUG=1` to print what the score is built from twice a second: how far the chassis is tipped, where the screw
and the magnet's face are and how far apart, whether the magnet holds the screw, whether it is in the dish, whether the
tool is on its holder and the chassis back down, whether both hands are clear, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,000 steps (292 s on the clock, with Dream answering in 0.65 s on average), and about 2 minutes for the scripted demo (486 steps, 24.3 s of simulated time, 144 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
