# Hold a light puzzle box still with one hand while the other slides its four panels open in order and takes out the block

A long wooden puzzle box lies on a lacquered tray so smooth that the box slides at the lightest push. Four panels on its
top slide, marked with one to four white dots: three small tiles and a lid over a compartment at the box's right end,
where a red block waits. Each panel is locked until the one before it has been slid open. One hand holds the box by
its left end; the other slides the panels open in order (the one-dot tile away from the arms, the two-dot tile toward
them, the three-dot tile away, the lid off to the right), lifts the block out and sets it on the tray in front of the box. Then both arms
go back to where they started. The task is one task file,
[`open_a_sliding_panel_puzzle_box.py`](open_a_sliding_panel_puzzle_box.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: holding a light object still while the other hand pushes and pulls on it; following an order shown on
the object (the dots); reaching into a small compartment.

Why two hands: the box (200 g) slides on the tray at about 0.04 N, while each panel needs about 0.6 N to move. Not held,
the box slides away from every push. With one hand (the right working the panels as in the demo, the left never moving)
the demo scored 0.0: the box slid back and forth across the tray and no panel moved.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The tray is kinematic, fixed where it is put, and
slick (contacts with it take its friction, 0.02); the box, its four panels and the block are loose rigid bodies made of
shapes from `bodykit.py`.

- The panels: each slides on the box along a prismatic joint (a `Slider` from `bodykit.py`) between stops (1.5 cm for
  the tiles, 8.5 cm for the lid). A locked panel's drive is stiff (20,000 N/m toward where it started). After every
  control step the task file checks how far each panel has slid on the box; once a panel has slid 1 cm (the lid 7 cm),
  the next one's drive turns soft (300 N/m), and the task file moves its target to the panel whenever the panel has
  been pushed more than 2 mm past it, so a slid panel stays where it is left. The one-dot tile is soft from the start.
- The block stands in a low socket (5 mm ridges) on the compartment's floor.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the block and the box still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_a_sliding_panel_puzzle_box.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the puzzle box and take out the red block: hold the box still by its left end with one hand; with the other slide its panels open in the order of the white dots on them (the one-dot tile away from you, the two-dot tile toward you, the three-dot tile away, then the four-dot lid off to the right); lift the red block out and set it on the tray in front of the box, let go of the box, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/puzzle-dream
```

The scripted demo, which reaches full score: the left hand comes straight down over the box's left end, fingers open
just wider than the box, and closes across it, holding the box. The right hand takes each panel's knob in turn
(fingers closing along the way it slides) and slides it open: the one-dot tile away from the arms, the two-dot tile
toward them, the three-dot tile away, and the lid off to the right. Then, fingers half open to fit the compartment, it
takes the red block, lifts it out and sets it on the tray in front of the box. The left lets go, and both arms go back
to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_a_sliding_panel_puzzle_box.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Open the puzzle box and take out the red block: hold the box still by its left end with one hand; with the other slide its panels open in the order of the white dots on them (the one-dot tile away from you, the two-dot tile toward you, the three-dot tile away, then the four-dot lid off to the right); lift the red block out and set it on the tray in front of the box, let go of the box, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/puzzle-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/puzzle-dream/*.json; do inspect-robots video "$log"; done
```

Set `PUZZLE_DEBUG=1` to print what the score is built from twice a second: how far each panel has slid on the box,
which are open, where the box and the block are, how far the block is outside the box's outline and whether it is on
the tray, whether both hands are clear of it, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (156 s on the clock, with Dream answering in 0.82 s on average), and about 4 minutes for the scripted demo (723 steps, 36.1 s of simulated time, 231 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
