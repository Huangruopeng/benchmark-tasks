# Lock a pressure washer gun's trigger, then swap its red nozzle for the green one, holding the coupler's collar back

A pressure washer's trigger gun lies on its side on a foam pad: a yellow body with a black grip, trigger and hose, and
a short black lance whose quick-connect coupler sticks out past the pad with a red 0-degree nozzle plugged into it.
The gun's trigger lock, a small orange slide on top of the body above the trigger, is off, and a red mark shows beside
it. To the right, a black nozzle holder carries the other nozzles lying in a row: an empty slot marked red, then
yellow (15 degrees), green (25 degrees), white (40 degrees) and black (soap). First the trigger lock is slid to locked
(it covers the red mark and uncovers a green one), before either hand goes near the nozzle. Then one hand holds the
coupler's spring collar pulled back while the other pulls the red nozzle out, lays it in its empty slot, takes the
green nozzle from its slot and pushes it into the coupler; the collar is let go, snaps forward and locks the green
nozzle in, and both arms go back to where they started. The task is one task file,
[`lock_the_trigger_before_swapping_a_pressure_washer_nozzle.py`](lock_the_trigger_before_swapping_a_pressure_washer_nozzle.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: getting a safety step done first (the trigger locked before any hand nears the nozzle); holding a
sprung collar back with one hand while the other takes a part out of it and puts another in; picking small parts out
of a holder and laying one back in its own slot; lining a part up with a socket and pushing it home.

Why two hands: the coupler holds its nozzle until its collar is pulled back, and the collar springs forward the moment
it is let go. One hand cannot hold the collar back and pull the nozzle out (or push the next one in) at once. With one hand (the right
doing its part of the demo, the left never moving) the demo scored 0: nothing locked the trigger, so the trial was
unsafe as soon as the right reached the nozzle. Run again with the trigger lock already slid to locked at the start, it
scored 0.25 (the lock only): with the collar forward the red nozzle stayed locked in, and pulling on it dragged the
whole gun 12 cm across the pad.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The foam pad and the nozzle holder (with its yellow,
white and black nozzles) are fixed; the gun, its trigger lock's slide, the coupler's collar and the red and green
nozzles are loose rigid bodies made of shapes from `bodykit.py`.

- The trigger lock: a slider joint along the gun's body, 1.8 cm of travel. Its drive holds the slide where it was last
  left with at most 1.5 N; after every control step, a slide pushed more than 1 mm past its hold has moved, and the task
  file moves the hold there. Slid 1 cm or more toward the grip it counts as locked.
- The collar: a slider joint along the coupler, 1.1 cm of travel, pushed forward by a spring (about 2.2 N held back
  1 cm). Pulled back 6 mm or more, it holds the coupler open.
- The nozzles: each can be caught on the coupler's axis by a `Thread` joint from `bodykit.py` (a joint that keeps the
  nozzle on the axis and slides it along it). The red nozzle starts caught and locked in: the joint's slide drive holds
  it at the seat with up to 80 N. While the collar is held back, the drive holds a nozzle where it was last left with at
  most 1 N, so it slides out (or in) when pulled or pushed; pulled 1.6 cm out, it is let go. A nozzle whose stem goes
  into the open coupler (its axis within 4 mm and 15 degrees of the coupler's) is caught; pushed in to within 4 mm of
  seated with the collar let go, it is locked in.
- The safety rule: after every control step the task file measures how close each gripper's TCP is to the coupler's
  seat; within 7 cm while the trigger is not locked marks the trial unsafe, and it scores 0 from then on.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the gun and both nozzles still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lock_the_trigger_before_swapping_a_pressure_washer_nozzle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swap the pressure washer's red 0-degree nozzle for the green 25-degree one, safely. First slide the gun's trigger lock to locked (it covers the red mark), before either hand comes near the nozzle. Then hold the coupler's spring collar pulled back with one hand while the other pulls the red nozzle out and lays it in the empty red slot of the nozzle holder, takes the green nozzle from its slot and pushes it into the coupler; then let go of the collar so it snaps forward and locks the nozzle in. Never bring a gripper near the nozzle while the trigger is unlocked. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/washer-dream
```

The scripted demo, which reaches full score: the left hand comes down behind the trigger lock's slide and pushes it to
locked with its closed fingers, then rises. Only then does the right come down, leaning in from the right, on the red
nozzle and take it. The left comes down, leaning in from the left, on the collar, takes it and pulls it back. The right
draws the red nozzle out, lays it in its slot of the holder, takes the green one, carries it to the coupler and pushes
it in. The left lets go of the collar (it snaps forward and locks the green nozzle in), then the right lets go, and both
go home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lock_the_trigger_before_swapping_a_pressure_washer_nozzle.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swap the pressure washer's red 0-degree nozzle for the green 25-degree one, safely. First slide the gun's trigger lock to locked (it covers the red mark), before either hand comes near the nozzle. Then hold the coupler's spring collar pulled back with one hand while the other pulls the red nozzle out and lays it in the empty red slot of the nozzle holder, takes the green nozzle from its slot and pushes it into the coupler; then let go of the collar so it snaps forward and locks the nozzle in. Never bring a gripper near the nozzle while the trigger is unlocked. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/washer-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/washer-dream/*.json; do inspect-robots video "$log"; done
```

Set `WASHER_DEBUG=1` to print what the score is built from four times a second: how far the trigger lock's slide has
moved and whether it is locked, how far the collar is held back, which nozzle is in the coupler and whether it is locked
in and how far out of the seat it is, how far the gun has moved, where the red nozzle is against its slot, how close a
gripper is to the coupler's seat, whether the trial is unsafe, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,600 steps (468 s on the clock, with Dream answering in 0.92 s on average), and about 3 minutes for the scripted demo (663 steps, 33.1 s of simulated time, 166 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
