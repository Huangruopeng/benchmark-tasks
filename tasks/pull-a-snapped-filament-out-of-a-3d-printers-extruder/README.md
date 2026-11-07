# Clear a 3D printer's extruder: hold its tension lever down with one hand while the other pulls a snapped filament out

The corner of a 3D printer stands on the table: a black aluminium frame beam with the stepper motor and the black
extruder on it, the extruder's red tension lever lying on top, and a spool of orange filament on its holder behind. A
13 cm piece of orange filament has snapped off inside the extruder: it sticks out of the extruder's inlet to the left,
its broken tip held between the drive gear and the idler bearing that the lever's spring presses onto it. One hand
presses the lever down by its pad and keeps it down, which lifts the idler off the filament; the other takes the piece
where it sticks out and pulls it straight out of the inlet, then drops it in the grey scrap tray in front. The lever is
let go and both arms go back to where they started. The task is one task file,
[`pull_a_snapped_filament_out_of_a_3d_printers_extruder.py`](pull_a_snapped_filament_out_of_a_3d_printers_extruder.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: holding a sprung lever down steadily with one hand while the other does the work; pulling a thin part
straight out along its length; finishing the job (the piece in the tray, the lever let go, both hands back).

Why two hands: the drive gear holds the piece until the lever is pressed down, and the lever springs back up as soon
as it is let go; the pad is 16 cm from where the piece sticks out. With one hand (each arm doing its part of the demo alone, the other never moving) the demo scored 0 both ways: the left alone pulled the piece 6.7 mm before its fingers slid off it, and it sprang back into the drive gear; the right alone pressed the lever down, and the piece stayed where it was.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The printer (frame beam, motor, extruder housing,
PTFE tube, spool) and the scrap tray are kinematic, fixed where they are put; the lever and the piece of filament are
loose rigid bodies made of shapes from `bodykit.py`.

- The lever: a hinge about the left-right axis on top of the housing (a `Hinge` from `bodykit.py`), sprung down onto
  its stop; pressing its pad down turns it up to 20 degrees. Pressed past 8 degrees, the idler is off the filament.
- The piece in the extruder: a joint (a `Thread` from `bodykit.py`, switched on from the start) keeps it on the
  filament path, free to slide along it, pulled by a stiff spring (4,000 N/m) toward a target the task file sets after
  every control step. While the lever is up and the piece's tip is still between the drive gear and the idler, the
  target stays where the piece is held: pulling it moves it a few millimetres at most, and it springs back. While the
  lever is pressed down, or once the tip is past the gear, the target follows the piece, so it slides out freely.
  Once its tip is out of the inlet the task file switches the joint off and the piece is loose. The piece passes
  through the extruder (they never collide).

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the piece still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pull_a_snapped_filament_out_of_a_3d_printers_extruder.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "A piece of orange filament has snapped off inside the 3D printer's extruder. Press the extruder's red tension lever down by its pad with one hand and keep it down, and with the other hand pull the broken piece straight out of the extruder's inlet; then drop the piece in the grey scrap tray, let the lever go, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/filament-dream
```

The scripted demo, which reaches full score: the right hand comes down on the lever's pad with its fingers closed and
presses it down, and holds it. The left comes down over the piece where it sticks out of the inlet, takes it across,
pulls it straight out along its length, lifts it, carries it over the scrap tray, lays it in and lets go. The right lets
the lever back up, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pull_a_snapped_filament_out_of_a_3d_printers_extruder.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "A piece of orange filament has snapped off inside the 3D printer's extruder. Press the extruder's red tension lever down by its pad with one hand and keep it down, and with the other hand pull the broken piece straight out of the extruder's inlet; then drop the piece in the grey scrap tray, let the lever go, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/filament-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/filament-dream/*.json; do inspect-robots video "$log"; done
```

Set `FILAMENT_DEBUG=1` to print what the score is built from twice a second: the lever's angle and whether the idler
is off the filament, how far the piece has come out (and the most so far), whether it is still in the extruder's
path, where it is and whether it lies in the tray, where each TCP is and the fingers' openings.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 900 steps (138 s on the clock, with Dream answering in 0.79 s on average), and about 1 minute for the scripted demo (362 steps, 18.1 s of simulated time, 77 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
