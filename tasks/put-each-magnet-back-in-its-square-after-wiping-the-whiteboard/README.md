# Take the magnets off a planning whiteboard, wipe it holding it still, then put each magnet back in its own square

A planning whiteboard lies on a smooth laminate desk, its far edge against the desk's low back rail. It has a white surface in an aluminium frame, ruled into three
rows of three squares under a green, an amber and a grey column tab, and a pen tray along its near edge. Three round
push-pin magnets, red, blue and yellow, stand in three of the squares; which three depends on the seed. Old blue and
black marker notes cover part of the board, and a felt eraser lies in the pen tray. The robot does three things in turn:

1. It takes the three magnets off the board and lays them in the pen tray.
2. It wipes the old marker off with the eraser, one hand pressing the board's frame down while the other rubs, and puts
   the eraser back in the tray.
3. It puts each magnet back in the square it came from, and both arms go home.

The task is one task file,
[`put_each_magnet_back_in_its_square_after_wiping_the_whiteboard.py`](put_each_magnet_back_in_its_square_after_wiping_the_whiteboard.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: memory. The magnets mark where things stand on the plan. Once they lie in the tray and the board is
wiped, nothing on the board shows which square each came from, and the wiping takes some 30 s in between. A policy that
acts on the current frame alone can only guess, one chance in nine per magnet. The three squares depend on the seed, so
a fixed habit does not pass either. Along the way it also tests:

- picking small knobbed parts;
- holding a board still while rubbing it;
- carrying and laying an eraser.

Why two hands: the board stands on rubber feet that hold it against a light touch, but a firm rub slides it across the
desk. The felt grips the board's surface, so rubbing it with nothing holding the board drags the whole board along under
the eraser, and no marker comes off. One hand presses the board's frame down while the other wipes.
To check this, the demo was run with only the hold taken out: the left hovers over the frame instead of pressing it, and
everything else is as in the demo. The board slid up to 6 cm back and forth under the eraser, only 5 of the 10 marks came
off, and the demo scored 0. A run with the left never moving also scores 0, trivially, because the left also moves the
red magnet.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The desk and its back rail are kinematic, fixed where
they are put. The
board (frame, grid lines, tabs and pen tray in one body), the three magnets and the eraser are loose rigid bodies made of
shapes from `bodykit.py`.

- The board's feet: two prismatic joints, one along the desk and one across it, through a hidden body under the table.
  Each has a drive that holds the board with up to 1.2 N. When the board is pushed more than 1.2 mm past the drive's
  target, the task file moves the target along, so the board slips like a rubber-footed board pushed hard and then
  sticks again. The board cannot turn.
- The marker notes are ten thin strokes drawn on the board. After every control step the task file finds the eraser's
  felt in the board's frame. When the felt lies flat on the surface and moves, the board has moved less than 1.5 mm (and
  turned less than 0.75 degrees) since the last step, and all three magnets lie in the pen tray, the felt's travel is
  added to every stroke under it. A stroke is hidden once 6 cm has been added, and the board counts as wiped once every
  stroke is hidden.
- The magnets' squares: as the embodiment resets, the task file catches the trial's seed and spawns each magnet in the
  square it gives (a seeded shuffle of the nine squares; the first three go to red, blue and yellow).
- A magnet is in a square when it stands upright on the surface with its middle at least 4 mm inside the square's lines.
  The eraser is back when its felt lies in the pen tray.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives `score()`
each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s: full score, which includes both arms back
where they started, with the board, the magnets and the eraser still. Otherwise it gives `score()` at the end of the
episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it. The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=put_each_magnet_back_in_its_square_after_wiping_the_whiteboard.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Take the three magnets off the whiteboard and lay them in its pen tray. Then wipe all the old marker off the board with the eraser, holding the board still with your other hand, and put the eraser back in the pen tray. Then put each magnet back in the same square of the grid it was in at the start, and take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/whiteboard-dream
```

The scripted demo reaches full score:

1. The left takes the red magnet from the far left square to the pen tray. The right takes the blue one (near middle)
   and the yellow one (near right).
2. The left presses the board's left frame down with its closed fingers, near its front corner. The right takes the
   eraser from the tray and rubs it along each line of old marker and back, twice. The left leans 1 cm against each rub,
   in time with it, because the arm gives a little under the load. The right lays the eraser back in the tray, and the
   left lets go.
3. Both arms go home and wait 1.5 s with the board wiped.
4. The right puts the yellow and the blue magnets back, and the left the red one.
5. Both arms go home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=put_each_magnet_back_in_its_square_after_wiping_the_whiteboard.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Take the three magnets off the whiteboard and lay them in its pen tray. Then wipe all the old marker off the board with the eraser, holding the board still with your other hand, and put the eraser back in the pen tray. Then put each magnet back in the same square of the grid it was in at the start, and take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/whiteboard-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/whiteboard-dream/*.json; do inspect-robots video "$log"; done
```

Set `WB_DEBUG=1` to print what the score is built from four times a second:

- where the board has moved and whether it stayed still;
- the felt's position and whether it touches the board;
- how many strokes are rubbed off, and the travel on each;
- where each magnet stands (in the tray, or which square);
- which magnets are in their own squares;
- whether the eraser is back in the tray and whether both arms are home;
- the finger openings and where each TCP is.

## Runtime per trial

On one RTX 5090: about 11 minutes for a Dream-1 trial of 2,400 steps (633 s on the clock, with Dream answering in 0.55 s on average), and about 10 minutes for the scripted demo (2,100 steps, 105.0 s of simulated time, 618 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders. These times were measured with
other simulations sharing the GPU; alone it runs faster.

## Seeds

`inspect-robots run` passes every trial a seed. With `--seed 0` (its default) and one ad-hoc scene, that is the same seed
every time: 1415158438. With that seed, red stands in the far left square, blue in the near middle one and yellow in the
near right one. The task file uses the seed for one thing: which three squares the magnets stand in. The scripted demo is
a fixed list of waypoints for that default seed; with other seeds the magnets stand elsewhere and the demo would miss
them. `-E object_jitter=0` keeps everything else where the task file puts it.
