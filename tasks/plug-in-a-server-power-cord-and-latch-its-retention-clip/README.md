# Hold a server's power plug fully home by its cord with one hand while the other swings its retention latch shut

A server's power supply stands on the table, its back face toward the arms. A thick, stiff power cord's plug sits in
its power inlet but 1.5 cm short of fully in: the stiff cord pushes it back out whenever it is let go, and the power
light above the inlet is red. Beside the inlet, a steel retention latch stands swung open: a short arm hinged on a
bracket, with a hook at its end and an orange knob on top. One hand pushes the plug fully home by its cord and keeps it
there (the light turns green); the other swings the latch a quarter turn round by its knob, until its hook lies across
behind the plug and clicks shut. Then both hands let go, the latch holding the plug in, and both arms go back to where
they started. The task is one task file,
[`plug_in_a_server_power_cord_and_latch_its_retention_clip.py`](plug_in_a_server_power_cord_and_latch_its_retention_clip.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: holding a part home against a spring with one hand while the other closes a latch over it; reading the
power light as feedback; finishing with the part secured and both hands free.

Why two hands: the stiff cord pushes the plug back out as soon as it is let go, and the latch's hook only passes
behind a plug that is fully in; the hand holding the plug home by its cord is 15 cm from the latch. With one hand (the
right pushing the plug home and tugging the cord as in the demo, the left never moving) the demo scored 0.25: the plug
went fully home (the light turned green), but sprang back out 1.5 cm when the hand let go, with the latch still open.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The power supply is kinematic, fixed where it is put;
the plug with its cord and the latch are loose rigid bodies made of shapes from `bodykit.py`.

- The plug slides in the inlet along a prismatic joint (a `Slider` from `bodykit.py`) between 1.9 cm out and fully in,
  pushed back out by a spring (250 N/m, 3.75 N when fully in) standing in for the stiff cord.
- The latch turns on a hinge about the vertical (a `Hinge` from `bodykit.py`) with a spring whose hold the task file
  moves to where the latch is left (whenever it is pushed more than 2 degrees past it), so it stays where it is left.
  Its hook passes behind the plug only when the plug is within a few millimetres of fully in; a plug sticking out is in
  its way. Swung at least 86 degrees round with the plug home, it clicks: the task file stiffens its spring toward 90
  degrees, so it holds the plug in. Pulled back under 70 degrees, it is open again.
- The power light: after every control step the task file moves a green light over the red one while the plug is home,
  and away (under the table) while it is not.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the plug and the latch still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=plug_in_a_server_power_cord_and_latch_its_retention_clip.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Power up the server's power supply and lock its cord in: with one hand push the black plug fully into the power inlet by its cord and keep it there (the light above the inlet turns green); with the other swing the retention latch round by its orange knob until its hook closes behind the plug and clicks shut; then let go of both, the latch holding the plug in, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/plug-dream
```

The scripted demo, which reaches full score: the right hand comes straight down on the cord 18 cm behind the plug,
fingers closing across it, pushes the plug fully home (the power light turns green) and keeps it there. The left comes
straight down behind the latch's orange knob, fingers closed, and pushes the knob round its hinge, a quarter turn,
until the hook lies across behind the plug and clicks shut. The left rises; the right gives the cord a 1 cm tug (the
latch holds the plug in), lets go, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=plug_in_a_server_power_cord_and_latch_its_retention_clip.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Power up the server's power supply and lock its cord in: with one hand push the black plug fully into the power inlet by its cord and keep it there (the light above the inlet turns green); with the other swing the retention latch round by its orange knob until its hook closes behind the plug and clicks shut; then let go of both, the latch holding the plug in, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/plug-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plug-dream/*.json; do inspect-robots video "$log"; done
```

Set `PLUG_DEBUG=1` to print what the score is built from twice a second: how far the plug is from fully in and whether
it counts as home, how far the latch has swung and whether it has clicked shut, whether both hands are clear, and where
each TCP, the latch's knob and the cord's held point are.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 900 steps (175 s on the clock, with Dream answering in 1.49 s on average), and about 2 minutes for the scripted demo (468 steps, 23.4 s of simulated time, 108 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
