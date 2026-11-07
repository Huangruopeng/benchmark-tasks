# Cut a copper riser with a tube cutter while the other hand holds it still by its valve, then bin the offcut

A short riser of 22 mm copper pipe stands in a plumber's bench stand whose collar lets it turn freely. A tee low on
the riser carries a branch to the left, ending in a brass ball valve; at the top the riser ends in a brass adapter. A
tube cutter is already clamped on the riser at a pencil mark, its feed knob pointing to the right. One hand holds the
riser still by its valve; the other swings the cutter back and forth round the pipe until the top piece is cut free,
then lifts the cut-off piece off and drops it in the scrap bucket, and both arms go back to where they started. The
task is one task file,
[`cut_a_copper_riser_with_a_tube_cutter_holding_it_by_its_valve.py`](cut_a_copper_riser_with_a_tube_cutter_holding_it_by_its_valve.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: holding a part against a torque from the other hand's work; a repeated swinging stroke round an axis,
the wrist turning with it; noticing when the cut is through, then clearing the offcut.

Why two hands: the cutter's wheel and rollers drag on the pipe about 25 times harder than the collar holds the riser,
so a free riser simply turns with the cutter and nothing is cut. With one hand (the right swinging the cutter as in the
demo, the left never moving) the demo scored 0.0: the riser swung back and forth in its collar, up to 30 degrees each way, with the cutter, which turned only 40 degrees on the pipe in all.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stand and the bucket are kinematic, fixed where
they are put; the riser (with its tee, branch and valve), the top piece above the mark and the cutter are loose rigid
bodies made of shapes from `bodykit.py`.

- The riser turns in the stand's collar on a hinge about its axis (a `Hinge` from `bodykit.py`) with almost no
  resistance (0.011 N m s/rad).
- The cutter rides round the pipe at the mark on a second hinge, whose damping (0.3 N m s/rad) stands for the drag
  of its wheel and rollers: turning the cutter on the pipe takes a torque in proportion to how fast it turns.
- The cut: after every control step the task file adds up how far the cutter has turned relative to the riser,
  whichever way. The top piece is held on the riser by a stiff slide along the axis (a `Slider`); at 720 degrees the
  task file makes the slide slack, so the top piece rests loose on the cut, and once it has been lifted 2 cm it frees
  it altogether. No metal is removed.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the cut-off piece still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cut_a_copper_riser_with_a_tube_cutter_holding_it_by_its_valve.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the copper riser at the pencil mark with the tube cutter, swinging it back and forth around the pipe until the top piece comes free (the riser turns in its stand unless you hold it still by its brass valve); then drop the cut-off piece in the blue scrap bucket and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/cutter-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the valve, fingers closing along the
branch, and holds the riser still. The right hand comes down with its fingers part open on either side of the knob's
grip block and swings the cutter back and forth round the pipe, the wrist turning with it, ten strokes, until the top
piece is cut free. It rises off the knob; the left lets go of the valve and goes back to where it started. The right
takes the cut-off piece by its brass hex, lifts it off, carries it over the bucket and drops it in, and goes back to
where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cut_a_copper_riser_with_a_tube_cutter_holding_it_by_its_valve.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the copper riser at the pencil mark with the tube cutter, swinging it back and forth around the pipe until the top piece comes free (the riser turns in its stand unless you hold it still by its brass valve); then drop the cut-off piece in the blue scrap bucket and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/cutter-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/cutter-dream/*.json; do inspect-robots video "$log"; done
```

Set `CUTTER_DEBUG=1` to print what the score is built from twice a second: how far the cutter has turned on the pipe
in all and where it stands now, how far the riser has turned in its collar, whether the pipe is cut through, how far
the top piece has been lifted and whether it is off, where it is and whether it is in the bucket, and where each TCP
is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,400 steps (182 s on the clock, with Dream answering in 0.69 s on average), and about 4 minutes for the scripted demo (815 steps, 40.8 s of simulated time, 236 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
