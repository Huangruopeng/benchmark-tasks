# Set a spring rat trap safely: hold its hammer back with one hand, latch the lock bar with the other, no snapping

A large wooden snap trap for rats is screwed to a pine plank. Its steel hammer lies sprung forward, and its lock bar
stands up at the near end. Two hands set it:
- One hand pulls the hammer up and over, against its spring, to the trap's near end, and holds it there.
- The other swings the lock bar forward over the hammer and presses its tip into the brass catch. The lock bar now holds
  the hammer back.
- The first hand lets go, and the hammer settles against the lock bar. It must never snap forward, and no gripper may
  be in its path while nothing holds it back. A gripper over the trigger pedal of the set trap springs it.
- Both arms go back to where they started.

The task is one task file,
[`set_a_rat_trap_without_a_gripper_in_its_strike_path.py`](set_a_rat_trap_without_a_gripper_in_its_strike_path.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests:
- working a strong spring mechanism with one hand while the other sets a latch;
- moving a part along an arc about its axle;
- keeping out of a hazard's path;
- not touching the trigger of a set mechanism;
- finishing with both hands free.

Why two hands: the hammer's spring pushes it forward as soon as it is let go, and only a latched lock bar holds it back.
In the one-hand check, the right hand did its part as in the demo and the left never moved; it scored 0. The right
pulled the hammer back and held it, but with nobody to latch the lock bar, the hammer snapped forward the moment the
right let go. With the left hand alone, the lock bar comes down on a sprung hammer and the catch cannot take it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The plank and the trap's board are one kinematic body,
fixed where it is put. The hammer and the lock bar are loose rigid bodies made of shapes from `bodykit.py`.

- **The hammer.** A hinge about the axle (a `Hinge` from `bodykit.py`), from sprung forward (0 degrees) over the top to
  the near end (178 degrees). Its spring is the hinge's drive: about 0.3 N m with the hammer pulled right back.
- **The lock bar.** A free hinge on its staple post. Gravity keeps it against its back stop while it stands up.
- **The catch.** After every control step, the task file checks whether the hammer is pulled back at least 165 degrees
  and the lock bar's tip is within 8 mm of the catch. If so, it holds the lock bar where it is with a stiff hinge drive.
  The hammer, pressing up on the lock bar, then stays back.
- **The trigger.** A gripper over the trigger pedal of the set trap makes the task file let the lock bar go: the hammer
  snaps forward.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. This gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, which includes both hands let go
and both arms back where they started, with the hammer and the lock bar still. Otherwise it gives `score()` at the end
of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_rat_trap_without_a_gripper_in_its_strike_path.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Set the rat trap without ever letting its hammer snap. With one hand, pull the steel hammer up and over to the near end of the trap and hold it there. With the other, swing the lock bar forward over the hammer and press its tip down into the brass catch. Keep every gripper out of the hammer's path until the lock bar is latched, unless that gripper is holding the hammer back. Then let the hammer settle against the lock bar, never touch the yellow trigger pedal of the set trap, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/trap-dream
```

The scripted demo reaches full score:
1. The right hand, leaning out to the right, comes down round the hammer's cross bar near its right end and pinches it
   lightly.
2. It carries the cross bar up and over the top to the trap's near end, the bar turning in its fingers, and holds it
   down there.
3. The left, leaning out to the left, pinches the lock bar near its tip and swings it forward over the hammer, pressing
   its tip down into the catch.
4. The left lets go and withdraws to the left, out of the trap.
5. The right lets go, the hammer settling against the lock bar, and rises.
6. Both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_rat_trap_without_a_gripper_in_its_strike_path.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Set the rat trap without ever letting its hammer snap. With one hand, pull the steel hammer up and over to the near end of the trap and hold it there. With the other, swing the lock bar forward over the hammer and press its tip down into the brass catch. Keep every gripper out of the hammer's path until the lock bar is latched, unless that gripper is holding the hammer back. Then let the hammer settle against the lock bar, never touch the yellow trigger pedal of the set trap, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/trap-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/trap-dream/*.json; do inspect-robots video "$log"; done
```

Set `TRAP_DEBUG=1` to print what the score is built from twice a second:
- how far the hammer is pulled back and how fast it swings;
- the lock bar's angle and its tip's offset from the catch;
- whether it is latched, the hammer held, snapped, or a gripper was in its path unheld;
- whether both hands are clear;
- where each TCP is from the cross bar and the lock bar's tip.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 900 steps (117 s on the clock, with Dream answering in 0.79 s on average), and about 2 minutes for the scripted demo (630 steps, 31.5 s of simulated time, 139 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
