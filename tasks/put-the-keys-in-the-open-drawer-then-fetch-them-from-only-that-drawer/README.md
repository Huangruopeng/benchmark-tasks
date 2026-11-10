# Put the keys in the drawer standing open, do another job, then fetch them back opening only that drawer

A low walnut-stained chest of four identical drawers stands on a polished marble top, with a brass carry handle on top
near each end. One drawer stands open a little. A bunch of keys lies on the table to the right, a navy glasses case to
the left, a leather valet tray beside the chest on the left and a white key dish beside it on the right. The robot puts
the keys in the drawer that stands open and pushes it shut, holding the chest by a carry handle with one hand while the
other works the drawer. It then puts the glasses case in the tray and takes both arms home. Then it fetches the keys:
it opens only the drawer it put them in, takes the keys out into the key dish, pushes the drawer shut and takes both
arms home. The task is one task file,
[`put_the_keys_in_the_open_drawer_then_fetch_them_from_only_that_drawer.py`](put_the_keys_in_the_open_drawer_then_fetch_them_from_only_that_drawer.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: memory. Once the drawer is shut the four drawers look the same, and the glasses case and the trip home
come in between, so when the robot comes back for the keys nothing in view says where they are. A policy that acts on
the current frame alone can only guess, one chance in four, and a wrong guess scores 0: once the keys are put away,
opening any other drawer (past 8 mm, too narrow a crack to see the keys through) is latched as a failure. Which drawer stands open at the start depends on the
seed, so a fixed habit (always the left-hand drawer, say) does not pass either. Along the way: holding a light piece
of furniture still while working its drawer, and putting small things into and out of a drawer.

Why two hands: the drawers run stiffly (about 1.8 N to move one), while the chest stands on furniture glides on the
polished marble and slides at about 0.6 N. Pulled or pushed with nothing holding the chest, the whole chest comes along
and the drawer does not move in it, so one hand holds the chest by a carry handle while the other works the drawer.
With one hand (the left working the drawer exactly as in the demo, the right never moving) the demo scored 0: pulling
the open drawer slid the whole chest 9 cm toward the arms while the drawer came out only 1.7 cm more, so the keys were
never put away.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The marble top, the tray and the key dish are
kinematic, fixed where they are put; the chest, its four drawers, the keys and the glasses case are loose rigid bodies
made of shapes from `bodykit.py`. Each drawer runs on a prismatic joint in the chest, from shut to 11.5 cm out; jointed
bodies do not collide, and the drawer boxes clear the chest's openings by 1 to 4 mm. A drawer pulls out at most 12.5 cm.

- The runners: after every control step the task file moves each runner's drive target to where the drawer is, so the
  drive holds the drawer there with up to 1.8 N, like felt runners. Within 12 mm of shut the target is shut: the
  magnetic catch pulls the drawer the rest of the way.
- The marble's contacts take the lower of the two frictions: the chest's glides slide at 0.045.
- The open drawer: as the embodiment resets, the task file catches the trial's seed and sets the runner of drawer
  number (seed mod 4) (from the left, from 0) to hold it 4 cm out; it settles about 2.8 cm out while the arms and
  objects settle. With no seed it is drawer 2.
- Put away: the keys' fob inside the drawer that stood open, with all four drawers shut. The glasses case counts only
  if it is put into the tray while the keys are put away (it starts outside the tray), and the arms only if both are
  home while the keys are still put away and the case is in the tray.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the chest, its drawers, the keys and the glasses case still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task
file puts it. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=put_the_keys_in_the_open_drawer_then_fetch_them_from_only_that_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the keys in the drawer of the chest that is standing open and push it shut, holding the chest by a carry handle with one hand while the other works the drawer. Then put the glasses case in the leather tray and take both arms back to where they started. Then take the keys back out, opening only the drawer you put them in (once the keys are put away, never open any other drawer), put them in the key dish, push that drawer shut and take both arms back to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/keys-dream
```

The scripted demo, which reaches full score: the right hand holds the chest by its right carry handle, the one nearest
the open drawer (drawer 2 with the default seed), leaning against the pull, while the left pulls the drawer out to its
stop by the knob. The left waits out of the way while the right picks up the keys by the fob and lowers them into the
drawer; then the right holds the chest again and the left pushes the drawer shut with its closed fingers against the
front, beside the knob (the magnetic catch pulls it the last few millimetres). The left takes the glasses case to the
tray, carried high over the chest's end, and both arms go home and wait. Then the same the other way: the right holds,
the left pulls the same drawer out, the right takes the keys out by the fob and carries them round the front of the
chest to the key dish, holds again, the left pushes the drawer shut, and both arms go home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=put_the_keys_in_the_open_drawer_then_fetch_them_from_only_that_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the keys in the drawer of the chest that is standing open and push it shut, holding the chest by a carry handle with one hand while the other works the drawer. Then put the glasses case in the leather tray and take both arms back to where they started. Then take the keys back out, opening only the drawer you put them in (once the keys are put away, never open any other drawer), put them in the key dish, push that drawer shut and take both arms back to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/keys-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/keys-dream/*.json; do inspect-robots video "$log"; done
```

Set `KEYS_DEBUG=1` to print what the score is built from four times a second: how far each drawer is out, where the
chest, the keys and the glasses case are, which drawer the keys are in, whether they are put away, whether another
drawer was opened, whether the case is in the tray, whether both arms are home, whether the keys are in the dish, the
finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,600 steps (194 s on the clock, with Dream answering in 0.64 s on average), and about 3 minutes for the scripted demo (1,423 steps, 71.2 s of simulated time, 171 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

`inspect-robots run` passes every trial a seed (with `--seed 0`, its default, and one ad-hoc scene, the same seed every
time: 1415158438, which opens drawer 2, the third from the left). The task file uses it for one thing: which drawer
stands open at the start, seed mod 4 counted from the left starting at 0. The scripted demo is a fixed list of
waypoints for that default seed; other seeds open other drawers, and the demo would then work the wrong drawer.
`-E object_jitter=0` keeps everything else where the task file puts it.
