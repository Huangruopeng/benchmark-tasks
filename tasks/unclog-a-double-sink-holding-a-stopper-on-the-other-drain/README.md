# Unclog a double sink: one hand holds a stopper down on the other drain while the other pumps the plunger

A white ceramic double sink is set in a grey worktop, both basins standing full of water: the waste pipe they share is
blocked. A red rubber plunger stands on the right basin's drain; a black rubber stopper lies on the worktop in front of
the left basin. One hand puts the stopper on the left basin's drain and holds it down; the other pumps the plunger over
the right drain, four pushes, until the clog gives and the water drains away. Then the stopper comes off (the left
basin drains too) and goes back on the worktop, the plunger is stood on the worktop in front of the right basin, and
both arms go back to where they started. The task is one task file, [`unclog_a_double_sink_holding_a_stopper_on_the_other_drain.py`](unclog_a_double_sink_holding_a_stopper_on_the_other_drain.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: knowing why the second drain has to be shut (the instruction says so, the scene shows two basins on
one pipe), holding a part down in one place while the other hand works a tool in another, pumping a sprung tool,
then putting both things back.

Why two hands: the two drains are 17 cm apart, and a push only works while the other drain is held shut; a stopper
lying there unheld is blown off by the push. With one hand (the right pumping the plunger as in the demo, the left
never moving) the demo scored 0.25: every push threw water up out of the open left drain, and the water stayed.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The worktop with the sink in it is kinematic, fixed
where it is put; the plunger (a cup and a handle), the stopper and four water beads are loose rigid bodies made of
shapes from `bodykit.py`.

- The plunger's handle slides into its cup (a `Slider` from `bodykit.py`) as the rubber squashes, at most 1.2 cm,
  sprung back out. A push: the handle pressed at least 8 mm in while the cup stands on the right drain (within 1.2 cm
  of it, upright), counted again only once the handle is back out to within 3 mm.
- The left drain is shut while the stopper lies on it (within 1 cm, flat) and a gripper is on its knob (within 2 cm).
  A push with it shut works the clog; after four the clog is through. A push with the left drain open throws four
  8 mm water beads up out of it; one with the stopper lying there unheld also throws the stopper off.
- The water is a see-through slab in each basin (it does not collide). Once the clog is through, the task file lowers
  the right basin's slab to the floor over 3 s, and the left basin's once its drain is open too.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the plunger and the stopper still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=unclog_a_double_sink_holding_a_stopper_on_the_other_drain.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Unclog the double sink. Both basins drain into one pipe, so a push of the plunger just blows water up out of the other basin's drain unless that drain is shut: with one hand, put the black stopper on the left basin's drain and hold it down there; with the other, pump the red plunger over the right basin's drain at least four times until the water drains away. Then lift the stopper off the drain and put it back on the worktop in front of the left basin, stand the plunger on the worktop in front of the right basin, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/sink-dream
```

The scripted demo, which reaches full score: the left hand takes the stopper by its knob off the worktop, sets it on
the left drain and holds it pressed down. The right hand takes the plunger by its knob and pushes it down five times
(four are needed); after the fourth the water drains away. The right lifts the plunger out of the basin and stands it
on the worktop in front; the left lifts the stopper off the drain (the left basin drains too) and puts it back on the
worktop; both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=unclog_a_double_sink_holding_a_stopper_on_the_other_drain.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Unclog the double sink. Both basins drain into one pipe, so a push of the plunger just blows water up out of the other basin's drain unless that drain is shut: with one hand, put the black stopper on the left basin's drain and hold it down there; with the other, pump the red plunger over the right basin's drain at least four times until the water drains away. Then lift the stopper off the drain and put it back on the worktop in front of the left basin, stand the plunger on the worktop in front of the right basin, and take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/sink-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/sink-dream/*.json; do inspect-robots video "$log"; done
```

Set `SINK_DEBUG=1` to print what the score is built from four times a second: how far the plunger's handle is
pressed in, whether the plunger stands on the right drain, whether the stopper lies on the left drain and is held,
how many pushes there have been and how many worked, whether the clog is through, how far each basin has drained,
where the plunger and the stopper are, whether each is back on the worktop, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,000 steps (175 s on the clock, with Dream answering in 0.74 s on average), and about 2 minutes for the scripted demo (572 steps, 28.6 s of simulated time, 91 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
