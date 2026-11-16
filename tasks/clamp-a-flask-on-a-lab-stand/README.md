# Clamp a round-bottom flask on a lab stand

A lab stand (a cast-iron base plate and a 16 cm steel rod) carries a spring flask clamp. Its arm reaches toward the middle of
the table and ends in two cork-lined jaws, which a spring keeps shut. A red lever on top of the arm opens them while it
is pressed down. A 100 mL round-bottom flask stands in a cork ring to the right.

The instruction states only the goal: clamp the flask by its neck so it hangs clear of the stand's base, without
dropping or tipping it, and take both arms back. In the scripted demo, one hand presses the lever down and keeps it
down. The other lifts the flask out of its cork ring by its neck, slides
the neck in between the open jaws against the clamp's back, and lowers it to the jaws' height. The lever is let up, so
the jaws close on the neck. The flask is let go, hanging in the clamp above the stand's base, and both arms go back to
where they started.

The task is one task file, [`clamp_a_flask_on_a_lab_stand.py`](clamp_a_flask_on_a_lab_stand.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

**What it tests:**
- holding a spring mechanism open while placing a fragile part into it with the other hand;
- carrying glass without dropping or tipping it;
- the right order: the jaws must close before the flask is let go;
- finishing with both hands free.

**Why two hands:** the jaws are open only while the lever is held down, and the neck has to be between them as they
close. With one hand, the jaws are shut whenever that hand holds the flask, so the neck cannot get in. The one-hand
check scored 0: the right did its part as in the demo while the left never moved. The neck met the shut
jaws, and the flask was knocked out of the hand and fell.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The stand (base plate, rod, boss head, arm, the
clamp's back) is one kinematic body made of shapes from `bodykit.py`. The two jaws, the lever and the flask are rigid
bodies.

- **The jaws:** each jaw is a slider across the clamp's arm with a spring drive (2,000 N/m, at most 15 N) holding it
  shut.
- **The lever:** a hinge with a spring holding it level. It turns down at most 15 degrees.
- **The linkage:** after every control step, the task file moves both jaws' drive targets with the lever's angle. The
  jaws are fully open (5 cm apart) at 12 degrees down.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s; `success()` means full score (which
includes both arms back where they started) with the flask still. Otherwise it gives `score()` at the end of the
episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
  layout.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clamp_a_flask_on_a_lab_stand.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Clamp the round-bottom flask by its neck in the lab stand's clamp, so that it hangs there clear of the stand's base, without dropping or tipping it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flask-dream
```

The scripted demo reaches full score:
1. The left hand comes down on the lever's pad with its fingers closed and presses it down: the jaws open.
2. The right hand comes straight down over the flask, fingers closing across the neck just under the lip, and lifts it
   out of the cork ring.
3. The right carries the flask to the front of the clamp and slides the neck in between the open jaws, against the
   clamp's back. It lowers the flask 1 cm.
4. The left lets the lever up: the jaws close on the neck.
5. The right lets go, rises and draws back toward the arms, its open fingers passing either side of the neck.
6. Both arms go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clamp_a_flask_on_a_lab_stand.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Clamp the round-bottom flask by its neck in the lab stand's clamp, so that it hangs there clear of the stand's base, without dropping or tipping it, and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flask-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/flask-dream/*.json; do inspect-robots video "$log"; done
```

Set `FLASK_DEBUG=1` to print what the score is built from twice a second:
- the lever's angle and how far the jaws are open;
- where the neck is against the jaws, and the flask's tilt and height above the base plate;
- whether the neck has been between the open jaws, whether the flask is clamped, and whether it was dropped;
- where each TCP is;
- whether both arms are home.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 900 steps (263 s on the clock, with Dream answering in 0.72 s on average), and about 2 minutes for the scripted demo (486 steps, 24.3 s of simulated time, 138 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
