# Hold a clockwork toy with one hand while the other winds its key, then let it run over the finish line

A run-down clockwork tin locomotive stands on the table with its winding key on top; a finish line is painted across
the table ahead of it. One hand holds the toy still; the other winds the key, taking it and turning it half a turn at a
time, at least a turn and a half. Then both hands let go and the toy runs forward on what was wound, over the finish
line. The task is one task file,
[`wind_a_clockwork_toy_and_let_it_run.py`](wind_a_clockwork_toy_and_let_it_run.py), for the `yam_arena` embodiment
(scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: a repeated grasp, turn and regrasp on a small key while the other hand keeps the toy still, then letting
go of both at the right time.

Why two hands: the key is stiff (0.6 N m) and the toy is light and rides on wheels that skid sideways easily, so a key
turned on its own turns the whole toy. With the left hand kept clear, the scripted demo's winding spins the toy half
round on the table and winds the key 5 degrees (0.0). One hand holds the toy still while the other turns the key.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The toy's body, its two axles of wheels and its key are
rigid bodies made of shapes from `bodykit.py`, joined by hinges (`Hinge` from `bodykit.py`): each axle turns in the
body, the key turns on its shaft. The wheels are slick against the table (they skid sideways easily).

- The mainspring and its ratchet: the key's hinge has a stiff spring, whose target the task file keeps 6 degrees behind
  the most the key has been wound (and further behind as more is wound). So turning the key on takes 0.6 N m (0.05 N m
  more for each turn), turning it back does nothing, and let go it springs back only those 6 degrees. It is fully
  wound at three turns.
- The run: once the toy is let go (both TCPs at least 3 cm from it and from the space 3 cm above it) and stands on its
  wheels, the task file sets it going at 10 cm/s along its heading every control step (and pushes it with a force it
  learns, to cancel the table's friction), and uses up the winding as it goes, 12 cm per turn; the key turns back as
  it runs down. Its wheels are braked except while it runs.
- It counts as over the line only if the run carried it there from behind the line.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the toy run down and still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wind_a_clockwork_toy_and_let_it_run.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the clockwork toy still with one hand, and with the other wind the key on its top, half a turn at a time, at least a turn and a half; then let go of the toy so that it runs over the finish line." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toy-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the toy's back end, fingers closed,
and presses it onto the table. The right hand comes straight down on the key's wing, fingers closing on its faces,
turns it half a turn, lets go, turns back in the air and takes it again: four times, two turns. Then both hands rise
and move aside, and the toy runs about 24 cm, over the line:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=wind_a_clockwork_toy_and_let_it_run.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hold the clockwork toy still with one hand, and with the other wind the key on its top, half a turn at a time, at least a turn and a half; then let go of the toy so that it runs over the finish line." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/toy-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/toy-dream/*.json; do inspect-robots video "$log"; done
```

Set `TOY_DEBUG=1` to print what the score is built from (where the toy is and which way it heads, how far the key has
turned against it, how much is wound and the most ever wound, whether it is running and has crossed the line, and where
each gripper's TCP is from the toy and from the key's wing) twice a second.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 1,200 steps (146 s on the clock, with Dream answering in 0.74 s on average), and about 2 minutes for the scripted demo (606 steps, 30.3 s of simulated time, 92 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
