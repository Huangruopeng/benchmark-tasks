# Zero a robot arm's elbow: hold its forearm level at the zero marks while the other hand pushes in the zeroing pin

A small orange six-axis robot arm stands at the left of the table, reaching across it. Its elbow's brake has been
released for mastering, so its forearm hangs on a free joint, 30 degrees below level on its stop. Yellow zero marks on
top of the elbow line up only when the forearm is level, and a zeroing hole is drilled through the front of the elbow
housing into the forearm's hub. One hand takes the zeroing pin out of its holder and brings it in front of the hole;
the other comes down on the forearm near its wrist and lifts it until the marks line up, and holds it there. The first
hand pushes the pin into the hole until it is home: the elbow's light turns green and the pin now holds the forearm
level. Both hands let go and go back to where they started. The task is one task file, [`zero_a_robot_arms_elbow_with_its_mastering_pin.py`](zero_a_robot_arms_elbow_with_its_mastering_pin.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding a link up against its own weight at a precise angle while the other hand lines a pin up with a
hole and pushes it home; reading alignment marks; finishing the job by letting go once the pin holds.

Why two hands: the released forearm drops back onto its stop whenever it is let go, and the pin goes past the front
cheek into the hub only while the forearm is held with its marks lined up. With one hand nothing pins the elbow. With
the right hand only (doing its part of the demo, the left never moving) the demo scored 0: the forearm was lifted level
and held, but with no pin it dropped back onto its stop when let go. With the left hand only it scored 0.25: the pin
went into the hole but stopped against the hub's face, the forearm hanging 30 degrees below level.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The robot (base, shoulder, upper arm and elbow
housing), the pin's holder and the teach pendant are kinematic, fixed where they are put; the forearm and the pin are
loose rigid bodies made of shapes from `bodykit.py`.

- The released elbow: the forearm turns about the elbow's axis on a `Hinge` from `bodykit.py`, nearly free (light
  gearbox drag), with a stop 30 degrees below level, so it hangs on its stop unless it is held.
- The zeroing hole: when the pin's tip comes to the hole's mouth on its axis (within 6 mm, tilted less than 12 degrees)
  a `Thread` from `bodykit.py` (a joint keeping it on the hole's axis) is switched on and the pin slides along the hole.
  After every control step the task file lets it slide as far as the hub's face (1.1 cm); further in only while the
  forearm is within 4 degrees of level, the angle at which the hub's hole lines up.
- Pinned: once the pin is 1.6 cm deep with the marks lined up, a fixed joint (a `Glue` from `bodykit.py`) holds the
  forearm level, as the pin through the hub does, and the elbow's green light is shown.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the forearm and the pin still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=zero_a_robot_arms_elbow_with_its_mastering_pin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Zero the robot arm's elbow: take the zeroing pin out of its holder; with one hand lift the robot's hanging forearm until the two yellow zero marks on top of the elbow line up and hold it there, while the other hand pushes the pin into the zeroing hole on the front of the elbow until it is home and the elbow's green light comes on; then let go of the forearm and the pin, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zero-dream
```

The scripted demo, which reaches full score: the left hand takes the pin out of its holder by its handle, straight
down, fingers closing across it, and brings it in front of the zeroing hole. The right hand comes straight down on the
hanging forearm near its wrist, fingers closing across it, and lifts it along its arc until it is level (the forearm
pivots between the fingers), and holds it there. The left pushes the pin into the hole until it is home and the light
comes on; both hands let go and go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=zero_a_robot_arms_elbow_with_its_mastering_pin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Zero the robot arm's elbow: take the zeroing pin out of its holder; with one hand lift the robot's hanging forearm until the two yellow zero marks on top of the elbow line up and hold it there, while the other hand pushes the pin into the zeroing hole on the front of the elbow until it is home and the elbow's green light comes on; then let go of the forearm and the pin, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/zero-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/zero-dream/*.json; do inspect-robots video "$log"; done
```

Set `ZERO_DEBUG=1` to print what the score is built from four times a second: the forearm's angle below level, whether
the pin is in the hole and how deep, whether the elbow is pinned, and where each TCP is against the pin's handle and
the forearm.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (263 s on the clock, with Dream answering in 0.76 s on average), and about 1 minute for the scripted demo (332 steps, 16.6 s of simulated time, 66 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
