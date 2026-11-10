# Slice a cucumber on a mandoline with its hand guard, holding the mandoline steady, then slide out the tray of slices

A mandoline slicer stands on the table, its stainless-steel deck rising toward the left between two black rails, with
a slanted blade across its lower part, a folding wire leg under its upper end and a loop handle beyond it. A red hand
guard sits on the deck above the blade with a chunk of cucumber standing in it, and a white tray stands under the
mandoline's lower half, under the blade, its pull tab sticking out past the lower end. One hand holds the mandoline by
its handle while the other slides the guard down the deck across the blade and back up, four times: each pass cuts a
slice, which drops into the tray. Then the guard is left at the top of the deck, the tray of slices is slid out from
under the mandoline by its tab, and both arms go back to where they started. A gripper that comes near the blade
itself scores 0. The task is one task file,
[`slice_a_cucumber_on_a_mandoline_with_its_hand_guard.py`](slice_a_cucumber_on_a_mandoline_with_its_hand_guard.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: holding a light tool steady against a pushing load while the other hand works it; a repeated stroke
along a sloping track to a set depth and back; keeping both grippers away from a blade; a last step that shows the
result (sliding a tray out from under the mandoline).

Why two hands: the guard slides in its rails only when pushed with more than about 2.5 N (the rails and the blade's
cut), while the mandoline stands on smooth feet that let it slide at about 1 N. Pushed with nobody holding it, the
whole mandoline slides away with the guard and the cucumber never crosses the blade. With one hand (the right doing
its part of the demo, the left never moving) the demo scored 0: the mandoline slid down the table with the guard on
each push and back up with it on each pull, ending 11 cm from where it started; the guard's middle never got below
11 cm up the deck (the cucumber crosses the blade below 7.8 cm), so no slice was cut.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mandoline, its guard, the tray and the slices are
loose rigid bodies made of shapes from `bodykit.py`; a hidden shelf under the table is fixed.

- The guard: a prismatic joint along the deck (authored by the task file with the guard, its axis along the slope)
  joins it to the mandoline, from 4.5 to 24.5 cm up the deck. Its drive holds it where it was last left with at most
  2.5 N (the drive's force cap); after every control step, a guard pushed more than 1 mm past its hold has slid, and
  the task file moves the hold there. So it slides when pushed harder than 2.5 N, as against friction, and stays where
  it is left.
- The slices: each time the guard's middle goes from more than 15 cm up the deck to less than 7.8 cm up it (the
  cucumber pushed across the blade at 9.5 cm), the task file takes the next of six ready-made slices from the shelf
  under the table and drops it from just under the blade, a little to one side or the other of the last, so they land
  in the tray in a loose pile.
- The blade: after every control step the task file measures how close each gripper's TCP comes to the blade's edge;
  closer than 3 cm, at any time, marks the trial unsafe, and it scores 0 from then on.
- The tray stands clear of the mandoline and never touches it (the two do not collide), so it never stops the
  mandoline sliding.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the mandoline, the guard and the tray still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25,
0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so
every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slice_a_cucumber_on_a_mandoline_with_its_hand_guard.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Slice the cucumber on the mandoline using its hand guard: hold the mandoline steady by its handle with one hand while the other slides the red hand guard, which holds the cucumber, down the deck across the blade and back up, four times. Never bring a gripper near the blade itself. Then leave the guard at the top of the deck, slide the tray of slices out from under the mandoline by its pull tab, and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/mandoline-dream
```

The scripted demo, which reaches full score: the left hand comes down on the near side bar of the mandoline's handle
and holds it. The right comes down on the guard's knob, takes it and slides the guard down the deck past the blade and
back up four times (a slice drops into the tray each time), then leaves it at the top of the deck, lets go and rises;
the left lets go and goes home. The right takes the tray's pull tab, slides the tray 15 cm out from under the
mandoline toward the right, so the slices show, lets go and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slice_a_cucumber_on_a_mandoline_with_its_hand_guard.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Slice the cucumber on the mandoline using its hand guard: hold the mandoline steady by its handle with one hand while the other slides the red hand guard, which holds the cucumber, down the deck across the blade and back up, four times. Never bring a gripper near the blade itself. Then leave the guard at the top of the deck, slide the tray of slices out from under the mandoline by its pull tab, and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/mandoline-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mandoline-dream/*.json; do inspect-robots video "$log"; done
```

Set `MANDOLINE_DEBUG=1` to print what the score is built from four times a second: how far up the deck the guard is
and its hold, how many slices have been cut and how many lie in the tray, how far the mandoline and the tray have
moved, how close a gripper has come to the blade, whether the trial is unsafe, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,400 steps (374 s on the clock, with Dream answering in 0.68 s on average), and about 3 minutes for the scripted demo (624 steps, 31.2 s of simulated time, 173 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
