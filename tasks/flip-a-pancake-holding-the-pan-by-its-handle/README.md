# Flip a pancake: hold the light pan still by its handle while the other hand turns the pancake over with a turner

A light frying pan stands on a smooth glass hob, its long handle out to the left, with a pancake lying golden side up a
little right of the pan's middle; a steel turner lies to the right, its blade on the worktop and its handle on a small
rest. One hand holds the pan still by its handle while the other takes the turner, slides its blade under the pancake's
right edge and lifts that edge up and over, so the pancake turns over onto its browned side in the pan. Then the turner
goes back on its rest, the pan is let go, and both arms go back to where they started. The task is one task file,
[`flip_a_pancake_holding_the_pan_by_its_handle.py`](flip_a_pancake_holding_the_pan_by_its_handle.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: using a tool in a container that must be held still; getting a thin blade under a flat object and
turning it over with it; putting the tool back where it belongs.

Why two hands: the pan weighs 600 g and slides on the glass hob with friction under 0.1, so the blade pressed onto its
floor and pushed in under the pancake drags the pan along with it unless the other hand holds the handle. With one hand
(the right doing its part as in the demo, the left never moving) the demo scored 0: as the blade slid in, the pan
slid 3.3 cm away across the hob with the pancake in it, the blade never got under the pancake's edge, and the
lift went past it, leaving it golden side up.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The pancake: a rigid disc 8 cm across resting on a hidden base 4.8 cm across and 4 mm tall, so its edge stands 4 mm
  off the pan's floor for 1.6 cm in and a 2 mm blade slides under it. Its golden side faces up at the start and its browned side
  down; it counts as flipped once its axis points within 37 degrees of straight down.
- The turner: a beech handle, a steel neck dropping 4.5 cm at 70 degrees, and a thin steel blade, as one rigid body.
- The pan: a loose 600 g aluminium pan on the hob, sliding with friction 0.08-0.1.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the pancake, the pan and the turner still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E
cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flip_a_pancake_holding_the_pan_by_its_handle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Flip the pancake: hold the frying pan still by its handle with one hand, and with the other take the turner, slide its blade under the pancake's right edge and lift that edge up and over, so the pancake turns over onto its other side in the pan. Then put the turner back on its rest, let go of the pan, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/pancake-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the pan's handle near its end,
fingers closing across it, and holds the pan. The right takes the turner by its handle, lifts it off its rest, turns it
to point at the pan and lays the blade flat on the pan's floor right of the pancake; it slides the blade in under the
pancake's right edge, then lifts that edge up and over to the left, until the pancake stands on edge and topples onto
its other side. The right draws the turner back out, lays it on its rest and goes back to where it started; the left
lets go of the pan and goes back to where it started too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flip_a_pancake_holding_the_pan_by_its_handle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Flip the pancake: hold the frying pan still by its handle with one hand, and with the other take the turner, slide its blade under the pancake's right edge and lift that edge up and over, so the pancake turns over onto its other side in the pan. Then put the turner back on its rest, let go of the pan, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/pancake-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pancake-dream/*.json; do inspect-robots video "$log"; done
```

Set `CAKE_DEBUG=1` to print what the score is built from twice a second: which way the pancake faces, where it and the
pan are, whether it is in the pan and the pan on the hob, where the turner's blade tip is, how far the turner is from
its rest, whether both arms are back, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 900 steps (176 s on the clock, with Dream answering in 0.95 s on average), and about 2 minutes for the scripted demo (471 steps, 23.6 s of simulated time, 121 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
