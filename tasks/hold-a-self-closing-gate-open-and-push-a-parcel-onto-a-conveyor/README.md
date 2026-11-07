# Hold a self-closing gate open by its knob while the other hand pushes a parcel through onto a running conveyor

A low partition crosses the table with a gateway in it, closed by a small spring-hinged gate that swings shut on its
own when let go. Beyond the partition a conveyor belt runs to the right, ending on a steel run-off plate. A cardboard
parcel waits on the arms' side. One hand pulls the gate open by the knob on its top edge and holds it wide open; the
other slides the parcel along the table through the gateway onto the belt, which carries it away to the run-off plate,
and draws back out. The first hand then lets the gate swing back shut, gently, without the gate ever touching the
parcel, and both arms go back to where they started. The task is one task file, [`hold_a_self_closing_gate_open_and_push_a_parcel_onto_a_conveyor.py`](hold_a_self_closing_gate_open_and_push_a_parcel_onto_a_conveyor.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a self-closing door open with one hand while feeding a load through it with the other; keeping
the door clear of the load; finishing with the door shut and both hands free.

Why two hands: the gate's spring swings it shut as soon as it is let go (0.36 N m at 90 degrees), and pushing the
parcel into the shut gate only presses it against its stop: the gate opens toward the arms, against the parcel. With
one hand (the right sliding the parcel as in the demo, the left never moving) the demo scored 0.0: the parcel ran into the shut gate and stopped against it, short of the partition and the belt.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The partition, the conveyor and the run-off plate are
kinematic, fixed where they are put; the gate, its knob and the parcel are loose rigid bodies made of shapes from
`bodykit.py`.

- The gate: a revolute joint (a `Hinge` from `bodykit.py`) on the hinge post, from shut to 100 degrees open toward
  the arms, with a spring drive pulling it shut (0.004 N m per degree) and a little damping.
- The knob: its own body on a free hinge on the gate's top edge, so it turns in a hand that holds it while the gate
  swings.
- The belt: a slick fixed surface 2 mm above the table, with a shallow ramp up to it from the partition. After every
  control step, a parcel lying on it (entirely beyond the partition, its middle over the belt) is set moving to the
  right at 10 cm/s; once all of it is past the belt's end it slides onto the run-off plate and stops there. The belt's
  cross ribs are a separate body, shown only, moved along at belt speed every control step, so the belt is seen running.
- Touching: after every control step the task file measures, seen from above, how far the gate's panel is from the
  parcel's outline (both reach from the table to above the parcel's top); within 3 mm counts as touching, for good.
  It also checks that the parcel's bottom stays within 2 cm of the table.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the parcel and the gate still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps
every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hold_a_self_closing_gate_open_and_push_a_parcel_onto_a_conveyor.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Send the cardboard parcel through the gateway in the low partition onto the running conveyor belt beyond it, without the gate touching it: with one hand pull the gate open by its knob and hold it wide open; with the other slide the parcel along the table through the gateway onto the belt, which carries it away to the right onto the steel run-off plate at its end, then draw that hand back out; then let the gate swing back shut gently, and take both arms back to where they started. Keep the parcel on the table: do not lift it over the partition." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/gate-dream
```

The scripted demo, which reaches full score: the left hand comes in low over the gate's knob, fingers open, closes on
it and pulls it along the gate's swing until the gate stands open 95 degrees, and holds it there. The right comes
straight down behind the parcel, fingers closed, slides it along the table through the gateway onto the belt until
its near face is 3.5 cm past the partition (the belt takes it away to the right, onto the run-off plate), draws back
out of the gateway and goes back to where it started. The left lets the gate swing back along its swing until it is
shut, lets go of the knob and goes back to where it started too:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hold_a_self_closing_gate_open_and_push_a_parcel_onto_a_conveyor.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Send the cardboard parcel through the gateway in the low partition onto the running conveyor belt beyond it, without the gate touching it: with one hand pull the gate open by its knob and hold it wide open; with the other slide the parcel along the table through the gateway onto the belt, which carries it away to the right onto the steel run-off plate at its end, then draw that hand back out; then let the gate swing back shut gently, and take both arms back to where they started. Keep the parcel on the table: do not lift it over the partition." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/gate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/gate-dream/*.json; do inspect-robots video "$log"; done
```

Set `GATE_DEBUG=1` to print what the score is built from twice a second: how far the gate is open, how far it is from
the parcel, whether it has touched the parcel or the parcel has been lifted, where the parcel and the knob are,
whether the parcel is through, on the running belt or delivered to the run-off plate, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,000 steps (178 s on the clock, with Dream answering in 0.79 s on average), and about 1 minute for the scripted demo (589 steps, 29.4 s of simulated time, 88 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
