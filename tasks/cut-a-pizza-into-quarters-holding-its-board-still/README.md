# Cut a pizza into quarters with a pizza wheel, holding its board still by the handle with the other hand

A pepperoni pizza lies on a round beech board on a polished grey quartz worktop, the board's short handle to its left. A
pizza wheel stands in its wooden stand to the right. One hand takes the board's handle and holds the board still. The
other takes the pizza wheel out of its stand, sets the wheel on the pizza's back edge and runs it forward through the
middle, pressing down, then turns it a quarter turn and runs it from the right edge to the left edge through the middle,
so the pizza is cut into quarters. It puts the pizza wheel back in its stand, the first hand lets go, and both arms go
back to where they started. The task is one task file,
[`cut_a_pizza_into_quarters_holding_its_board_still.py`](cut_a_pizza_into_quarters_holding_its_board_still.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the
same folder.

What it tests: holding a light object still against a pull in changing directions; using a hand tool along a straight
line while pressing down; turning the tool for a second, crossing cut; putting the tool back where it belongs.

Why two hands: the worktop is polished (friction 0.1), so the 0.55 kg board slides at about 0.5 N, while the wheel
pressed onto the pizza drags it with a few newtons. Cutting counts only while the board keeps still. With one hand (the
right doing its part exactly as in the demo, the left never moving) the demo scored 0: the wheel pushed the board 14 cm forward across the worktop, to its front edge, and only 2 of the 16 cut
segments counted.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The worktop and the stand are kinematic, fixed where
they are put; the board (with the pizza on it) and the pizza wheel are loose rigid bodies made of shapes from
`bodykit.py`. The worktop's contacts take the lower of the two frictions (a polished surface).

- Cutting: after every control step the task file looks at the wheel's lowest edge in the board's frame. Each cut line
  (front-to-back and right-to-left through the pizza's middle) is 8 segments 2 cm long. While the wheel presses on the
  pizza (its edge within 5 mm of the pizza's top) and the board moved less than 1.5 mm and turned less than 0.75 degrees
  in that step, every segment the edge passes (within 1.2 cm of its middle along the line, 1.4 cm across) is cut, and a
  dark cut line is shown there.
- The pizza is a rigid disc on the board, and the wheel is part of the pizza wheel's body: it does not turn, but slides
  on the pizza with friction, which drags the board as a rolling, cutting wheel does.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the board and the pizza wheel still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts
from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cut_a_pizza_into_quarters_holding_its_board_still.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the pizza into quarters: hold the pizza board still by its handle with one hand. With the other, take the pizza wheel from its stand and run it across the pizza twice through the middle, pressing down so it cuts: once along the line from the back edge to the front edge and once along the line from the right edge to the left edge. Then put the pizza wheel back in its stand and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/pizza-dream
```

The scripted demo, which reaches full score: the left hand takes the board's handle from above, its pads square to the
handle's sides and its wrist leaning out to the left, and holds it. The right waits at home until the board is held,
then takes the pizza wheel out of its stand by the middle of its handle, sets the wheel on the pizza's back edge and runs
it forward through the middle, aiming 3 mm into the pizza so it presses down. It lifts the wheel, turns it a quarter turn,
sets it on the right edge and runs it to the left edge through the middle. The cuts are aimed along the board's lines
where the board ends up once held: the left's pinch nudges it about 1 cm forward and 3.4 degrees round on the polished
worktop (measured, and the same in every run). The right puts the pizza wheel back in its stand, the left lets go and
both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cut_a_pizza_into_quarters_holding_its_board_still.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Cut the pizza into quarters: hold the pizza board still by its handle with one hand. With the other, take the pizza wheel from its stand and run it across the pizza twice through the middle, pressing down so it cuts: once along the line from the back edge to the front edge and once along the line from the right edge to the left edge. Then put the pizza wheel back in its stand and take both arms back to where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/pizza-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/pizza-dream/*.json; do inspect-robots video "$log"; done
```

Set `PIZZA_DEBUG=1` to print what the score is built from four times a second: where the board is and how far it has
turned, where the pizza wheel and its edge are on the board, whether it is pressing, whether the board kept still in the
last step, which segments of each cut line are cut, whether the pizza wheel is in its stand, the finger openings and
where each TCP is.

## Runtime per trial

On one RTX 5090: about 2 minutes for a Dream-1 trial of 900 steps (110 s on the clock, with Dream answering in 0.86 s on average), and about 2 minutes for the scripted demo (671 steps, 33.5 s of simulated time, 97 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
