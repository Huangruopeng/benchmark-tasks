# Fold a camp stool shut with a hand outside each top bar, never a gripper in the pinch point, then lay it down

A small folding camp stool stands open on the table: two beech-wood frames crossed in an X on a steel pivot bolt, a
green canvas seat slung between their round top bars. Folding it brings the two top bars together, which is where
fingers get crushed; yellow and black stripes on the bars' inner faces mark that pinch point. Both hands fold it, each
pushing its own top bar from the outside, until the stool is shut, with neither gripper ever between the bars. Then
one hand tips the folded stool over so it lies flat, and both arms go back to where they started. The task is one task
file, [`fold_a_camp_stool_without_pinching_a_gripper.py`](fold_a_camp_stool_without_pinching_a_gripper.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: folding a mechanism with both hands while keeping them out of the place it closes; choosing where to
push (outside the bars), not just pushing until it is shut.

Why two hands: the stool weighs 320 g and stands on plain wooden feet. Folding it means lifting its pivot as the feet
slide together, about 2 to 3 N at each top bar, while a push at one bar slides the whole stool or tips it over at about
1 N. Squeezed from both sides, the pushes cancel and the stool folds. With one hand (the right pushing its bar as in the
demo, the left never moving) the demo scored 0.0: the stool slid 2 cm and tipped 12 degrees on its far feet without folding at all, and settled back open.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The two frames are loose rigid bodies made of shapes
from `bodykit.py`, joined by a hinge along the pivot bolt.

- The pivot: a revolute joint (a `Hinge` from `bodykit.py`) between the two frames, from open (each leg 36 degrees
  from vertical) to shut (the top bars touching, about 57 degrees of fold). A spring holds the stool where it last
  stopped; after every control step, a pivot turned more than 2 degrees past its hold has given, and the task file
  moves the hold there, so the stool stays as folded as it is left.
- The canvas seat is drawn only: two thin panels the task file hangs from the two top bars after every control step,
  sagging between them as the bars close.
- The pinch point: after every control step, once the stool is folded 34 degrees or more (the bars under about 3.5 cm
  apart), a TCP within 2.5 cm of the middle between the two top bars, within 3 cm of their height and between their
  ends counts as caught, and the trial scores 0.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with both frames of the stool still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x
704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fold_a_camp_stool_without_pinching_a_gripper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fold the camp stool shut and lay it down: push its two top bars toward each other from the outside, one hand on each bar, until the stool is shut, never putting a gripper between the bars where they close (the pinch point, marked with yellow and black stripes); then tip the folded stool over so it lies flat on the table, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/stool-dream
```

The scripted demo, which reaches full score: each hand comes in level from its own side, fingers closed and pointing
across the stool, and sets its fingertips against the outside of its top bar. Together they push the bars toward each
other, rising with them as the pivot lifts, until the bars touch and the stool is shut. Both draw back; the right arm
goes back to where it started while the left pushes the folded stool's top to the right, tipping it over so it lies
flat on the table, and then goes back too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fold_a_camp_stool_without_pinching_a_gripper.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fold the camp stool shut and lay it down: push its two top bars toward each other from the outside, one hand on each bar, until the stool is shut, never putting a gripper between the bars where they close (the pinch point, marked with yellow and black stripes); then tip the folded stool over so it lies flat on the table, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/stool-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/stool-dream/*.json; do inspect-robots video "$log"; done
```

Set `STOOL_DEBUG=1` to print what the score is built from twice a second: how far the stool is folded, the pivot's
hold, how far its up axis leans, where its pivot and top bars are, whether each TCP is in the pinch point, and where
each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 900 steps (280 s on the clock, with Dream answering in 0.73 s on average), and about 3 minutes for the scripted demo (400 steps, 20.0 s of simulated time, 169 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
