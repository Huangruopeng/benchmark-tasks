# Juice half a lemon: hold a light hand juicer still by its handle while the other hand twists the lemon on its cone

A light white plastic citrus juicer stands on a smooth worktop, a bowl with a cone in the middle and a long flat handle
out to the left, with half a lemon sitting cut side down on the cone; a blue saucer waits to the right. One hand holds
the juicer still by the end of its handle while the other twists the lemon to and fro on the cone until it is squeezed
dry and its juice is in the bowl. Then the lemon is lifted off and put on the saucer, the handle is let go, and both
arms go back to where they started. The task is one task file,
[`juice_half_a_lemon_on_a_hand_juicer.py`](juice_half_a_lemon_on_a_hand_juicer.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding a light object still against the other hand's twisting; repeated to-and-fro twisting; finishing
the job (the squeezed lemon put away, both hands free).

Why two hands: the lemon grips the cone, and the light juicer slides and turns freely on the smooth worktop, so twisted
with nothing holding the juicer, the juicer just turns with the lemon and no juice comes. With one hand (the right
twisting the lemon as in the demo, the left never moving) the demo scored 0: the juicer turned to and fro with the
lemon, up to 36 degrees, not a drop of juice came, and lifting the lemon off carried the whole juicer with it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The lemon on the cone: a hinge about the cone's axis (a `Hinge` from `bodykit.py`) whose spring holds the lemon where
  it last stopped; after every control step, a lemon turned more than 2 degrees past its hold has given (0.1 N m), and
  the task file moves the hold there, so the lemon turns on the cone only against that grip and stays where it is left.
- The juice: 10 small beads parked out of sight in a closed box above the scene. For every 25 degrees the lemon has
  turned against the juicer, the task file moves the next bead into the bowl beside the cone's foot. After the tenth,
  the lemon is squeezed dry: the task file switches the hinge off, and the lemon sits loose on the cone.
- The juicer's base is smooth (friction 0.1) and the juicer weighs 150 g, so it turns on the worktop at well under the
  lemon's grip.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the juicer, the lemon and the juice still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=juice_half_a_lemon_on_a_hand_juicer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Juice the lemon half: hold the juicer still by the end of its handle with one hand, and with the other twist the lemon half to and fro on the juicer's cone until it is squeezed dry and its juice is in the bowl. Then lift the squeezed lemon off and put it on the blue saucer, let go of the handle, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/lemon-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the end of the juicer's handle,
fingers closing across it, and holds it. The right hand comes down round the lemon half, fingers closing across it, and
twists it to and fro on the cone, 45 degrees each way, until it is squeezed dry; then it lifts it off and puts it on the
saucer. The left lets go of the handle, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=juice_half_a_lemon_on_a_hand_juicer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Juice the lemon half: hold the juicer still by the end of its handle with one hand, and with the other twist the lemon half to and fro on the juicer's cone until it is squeezed dry and its juice is in the bowl. Then lift the squeezed lemon off and put it on the blue saucer, let go of the handle, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/lemon-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/lemon-dream/*.json; do inspect-robots video "$log"; done
```

Set `LEMON_DEBUG=1` to print what the score is built from twice a second: how far the lemon is turned on the cone and
how much it has turned in all, how many drops have been let into the bowl and how many are in it, whether the lemon is
loose, how far the juicer itself has turned, where the juicer and the lemon are, whether the lemon is on the saucer,
whether both hands are clear, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 900 steps (309 s on the clock, with Dream answering in 0.65 s on average), and about 2 minutes for the scripted demo (402 steps, 20.1 s of simulated time, 99 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
