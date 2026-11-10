# Make another coffee of the same kind: bin the stuck used capsule, then load a new capsule of the same colour from memory

A capsule coffee machine stands on the table, a white cup under its spout and a used capsule stuck in its holder. To its
right stand a wooden tray with three new capsules of three kinds (red, gold, blue) and a green recycling tin for used
capsules. The left hand takes the machine's water tank from above and holds the machine down; the right pulls the used
(blue) capsule up out of the holder, carries it over the tin and drops it through the hole in the lid, where it slides
out of sight. The left lets go and goes home. The right then takes the blue capsule from the tray, lowers it into the
holder and lets go, presses the brew button (it lights, coffee runs from the spout and the cup fills) and goes home.
The task is one task file,
[`make_another_coffee_with_a_capsule_of_the_same_kind.py`](make_another_coffee_with_a_capsule_of_the_same_kind.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: memory of an object's identity. The used capsule's colour is in view only while it is in the holder and
in the hand. Once it has gone into the tin, nothing in view shows which kind it was, and the instruction does not say:
the three new capsules differ only in colour. Taking a new capsule before the used one is in the tin scores 0 (so the
two cannot be compared side by side), and so does taking a capsule of another kind (so a policy cannot try them in
turn). A policy that acts only on the current camera frame has one chance in three. It also tests holding a light
appliance down while pulling a stuck part out of it, dropping a part through a hole, seating a part in a holder and
pressing a button.

Why two hands: the used capsule is stuck in the holder (a joint that breaks under a 6 N pull), and the machine weighs
only 2.9 N, so pulling the capsule lifts the whole machine unless the other hand holds it down. With one hand (the right
doing its part exactly as in the demo, the left never moving) the demo scored 0: the pull lifted the machine 5 cm and
dragged it across the table into the tray of new capsules, knocking the red and gold ones off their places.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray and the tin are kinematic, fixed where they
are put; the machine (with its tank, drip tray and cup) and the four capsules are loose rigid bodies made of shapes from
`bodykit.py`.

- The stuck capsule: a revolute joint between the machine and the used capsule, with its turn limited to half a degree
  either way and a break force of 6 N (PhysX breaks it for good once exceeded; it is re-armed at every reset).
- The tin: a slick ramp inside it slides a capsule dropped through the hole in the back part of its lid forward, under
  the solid front part of the lid, where neither the overhead camera nor a wrist camera looking down through the hole
  can see it.
- Brewing: after every control step the task file checks whether the blue capsule is seated in the holder (its middle
  within 1.8 cm of the holder's middle, its underside within 8 mm of the pocket's floor, tilted at most 15 degrees) and
  whether a gripper's fingertip centre is on the brew button (within 2.2 cm sideways and 1.2 cm above its top). Then the
  button's lit cap, a coffee stream from the spout for 3 s and the coffee rising in the cup (drawn shapes) are shown.
- The rules, each latched for the rest of the trial: a red or gold capsule moved more than 1.5 cm from its place on the
  tray; the blue capsule moved that far before the used capsule is in the tin.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the machine and the new capsule still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5,
0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every
trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every
frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=make_another_coffee_with_a_capsule_of_the_same_kind.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Make another coffee of the same kind. First take the used capsule out of the coffee machine's holder and drop it into the green recycling tin through the hole in its lid: the used capsule sticks, so hold the machine down by its water tank with your other hand while you pull it out. Only once it is in the tin, take the new capsule of the same kind, the same colour, as the used one from the wooden tray, put it in the holder, and press the brew button. Do not take any other capsule. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/coffee-dream
```

The scripted demo, which reaches full score: the left hand comes down over the water tank, closes on it (opened only 3
mm wider than the tank each side, so it barely shoves the machine) and holds the machine down. The right comes down
round the used capsule's rim, closes and pulls it 3.5 cm up (the stuck joint gives), carries it over the tin's hole and
lets it drop in. The left lets go and goes home. The right takes the blue capsule from the tray deep in its jaw (a
capsule held only at the fingertips flies out when carried), carries it over the holder and lets go with its underside
already 1 cm down in the holder's opening, aimed against two measured offsets: a capsule this wide hangs about 2 cm off
the TCP along the closing axis, and the left's grip leaves the machine about 6 mm to the right. It presses the brew
button with its closed fingers and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=make_another_coffee_with_a_capsule_of_the_same_kind.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Make another coffee of the same kind. First take the used capsule out of the coffee machine's holder and drop it into the green recycling tin through the hole in its lid: the used capsule sticks, so hold the machine down by its water tank with your other hand while you pull it out. Only once it is in the tin, take the new capsule of the same kind, the same colour, as the used one from the wooden tray, put it in the holder, and press the brew button. Do not take any other capsule. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/coffee-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/coffee-dream/*.json; do inspect-robots video "$log"; done
```

Set `COFFEE_DEBUG=1` to print what the score is built from four times a second: where the machine is and how far it is
tilted, where the used capsule is and whether it is in the tin, where the blue capsule is and whether it is seated, the
rule flags (wrong kind taken, taken too early), when the coffee was brewed, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 1,200 steps (332 s on the clock, with Dream answering in 0.92 s on average), and about 2 minutes for the scripted demo (625 steps, 31.2 s of simulated time, 148 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed (the used capsule is always the blue one), and the scripted
demo follows fixed waypoints.
