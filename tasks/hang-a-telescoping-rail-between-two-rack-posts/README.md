# Hang a telescoping rack rail: hold its front pegs in one post while the other hand pulls its rear out to latch

Two square steel rack posts stand 38 cm apart on the table, each with a pair of square holes at the rail's height; in front of them lies a telescoping rack rail, pushed short: an outer channel with a front bracket and an inner channel, sliding in it, with a rear bracket. One hand puts the front bracket's pegs into the left post's holes and holds the rail there while the other pulls the inner channel out until the rear bracket's pegs go into the right post's holes and latch. Then both hands let go, the rail hanging between the posts, and both arms go back to where they started. The task is one task file, [`hang_a_telescoping_rail_between_two_rack_posts.py`](hang_a_telescoping_rail_between_two_rack_posts.py), for the `yam_arena` embodiment (scene, scoring and the
scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: two hands carrying and placing one long part together; one hand holding against the drag of the other's pull; pegs into holes at both ends.

Why two hands: the front pegs only rest in their holes until the rail is latched, and the inner channel takes about
1.5 N to pull out, which drags the outer one along unless it is held. With one hand (the right doing its part as in
the demo, the left never moving) the rail hung tilted from the hand, its front end 3.5 cm low, the front pegs never got
into the left post, and pulling the inner channel dragged the whole rail 6 cm away from it (0.0).

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The posts are kinematic, fixed where they are put; the two channels are loose rigid bodies made of shapes from `bodykit.py`, joined by a slider.

- The posts' holes and the brackets' pegs are real geometry (6 mm pegs, 14 mm holes): pegs that are not latched only rest in their holes.
- The telescope: a slider (a `Slider` from `bodykit.py`) between the channels. A spring holds the inner channel where it last stopped; after every control step, a channel pulled more than 1 mm past its hold has given, and the task file moves the hold there, so it takes about 1.5 N to pull it out, and that drags the outer channel with it unless it is held.
- The latch: after every control step the task file checks both brackets' pegs. Once both pairs are at least 5 mm into their holes at once, a fixed joint (a `Glue` from `bodykit.py`, to a hidden fixed body under the table, its frames set to where the rail is) holds the rail there and the slide is locked.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
both channels still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hang_a_telescoping_rail_between_two_rack_posts.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hang the rack rail between the two rack posts: hold its outer channel with one hand and put its front pegs into the square holes of the left post, keep it there while the other hand pulls the inner channel out until its rear pegs go into the right post's holes and latch. Then let go of the rail and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/rail-dream
```

The scripted demo, which reaches full score: both hands come straight down on the rail where it lies, fingers closing across it: the left on the outer channel, the right on the end of the inner one. Together they lift it to the holes' height, carry it in between the posts and push it 2 cm toward the left post, so the front pegs go into its holes. The left keeps it there while the right pulls the inner channel out about 6.5 cm until the rear pegs go into the right post's holes and latch. Both let go, rise and go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hang_a_telescoping_rail_between_two_rack_posts.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Hang the rack rail between the two rack posts: hold its outer channel with one hand and put its front pegs into the square holes of the left post, keep it there while the other hand pulls the inner channel out until its rear pegs go into the right post's holes and latch. Then let go of the rail and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/rail-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/rail-dream/*.json; do inspect-robots video "$log"; done
```

Set `RAIL_DEBUG=1` to print what the score is built from twice a second: how far the inner channel has been pulled out, how deep each bracket's pegs are past their post's face and whether they are inside their holes, whether the rail is latched and level, whether both hands are clear, and where each TCP is from its channel.

## Runtime per trial

On one RTX 5090: about 4 minutes for a Dream-1 trial of 1,200 steps (241 s on the clock, with Dream answering in 0.72 s on average), and under 1 minute for the scripted demo (354 steps, 17.7 s of simulated time, 56 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
