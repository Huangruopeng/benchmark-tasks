# Hold a stick umbrella by its handle with one hand while the other pushes the runner up until it latches open

A closed stick umbrella lies on a stand. One hand holds it by its handle; the other grips the runner, the sleeve on the
shaft, and pushes it up toward the tip against the ribs' springs until it latches; the canopy opens. Then the runner is
let go: latched, it stays. The task is one task file,
[`open_an_umbrella_until_its_runner_latches.py`](open_an_umbrella_until_its_runner_latches.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

Why two hands: the umbrella lies loose on its stand, and the ribs' springs push back on the runner harder and harder
(up to 6 N). Pushed on alone, the runner shoves the whole umbrella along the stand: with the left hand kept clear, the
scripted demo's push slides the umbrella 13 cm and gets the runner only a third of the way up before the umbrella goes
off the stand (0.0). One hand holds the handle while the other pushes.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The umbrella (shaft, tip, notch and handle), the
runner, the canopy's eight panels and the stand are rigid bodies made of shapes from `bodykit.py`; the stand is
kinematic, fixed where it is put.

- The runner slides on the shaft on a slide (a `Slider` from `bodykit.py`) with a spring back toward closed (1.2 N to
  start it, 6 N at the top: the ribs' springs). After every control step the task file reads its travel; at 95% it
  sets the slide to hold it at the top (latched).
- The canopy's panels follow the runner: after every control step the task file sets each panel's pose from the
  umbrella's pose and the runner's travel (5 degrees from the shaft closed, 60 open), as the stretchers would. They
  do not collide.
- The stand is a cradle under the shaft by the tip and a flat pad under the handle; nothing stops the umbrella sliding
  along it. The runner does not collide with the shaft.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the umbrella still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object
exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for
sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_an_umbrella_until_its_runner_latches.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the umbrella by its handle with one hand, and with the other push the runner up the shaft toward the tip until it latches and the umbrella is open; then let go of the runner." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/umbrella-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the umbrella's handle and holds it on
the stand. The right hand comes straight down on the runner, grips it and pushes it up the shaft toward the tip until
it latches; the canopy opens. Then the right hand lets go and rises:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_an_umbrella_until_its_runner_latches.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the umbrella by its handle with one hand, and with the other push the runner up the shaft toward the tip until it latches and the umbrella is open; then let go of the runner." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/umbrella-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/umbrella-dream/*.json; do inspect-robots video "$log"; done
```

Set `UMB_DEBUG=1` to print what the score is built from (where the umbrella is, how far the runner has travelled, the
ribs' angle, whether it has latched and the runner is let go, and where each gripper's TCP is) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (148 s on the clock, with Dream answering in 0.80 s on average), and under 1 minute for the scripted demo (332 steps, 16.6 s of simulated time, 51 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
