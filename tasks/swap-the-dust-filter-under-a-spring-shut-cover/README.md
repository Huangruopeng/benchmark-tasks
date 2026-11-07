# Swap a dust filter under a sprung cover: hold the cover open by its pedal while the other hand swaps the filter

A machine's air intake stands on the table: a fixed housing whose dust filter stands upright in a slot under a hinged
cover. The cover is sprung shut; pressing down the pedal on its tail, which sticks out beyond the cover's hinge, swings
the cover up. One hand presses the pedal down and holds it, keeping the cover open; the other lifts the dirty filter out
by its tab, drops it in a bin, takes a new filter from its holder and lowers it into the slot. Then the pedal is let up,
the cover springs shut over the new filter, and both arms go back to where they started. The task is one task file,
[`swap_the_dust_filter_under_a_spring_shut_cover.py`](swap_the_dust_filter_under_a_spring_shut_cover.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: holding a spring-loaded cover open with one hand for the whole job while the other hand works under it
(lift out, discard, fetch, insert), in the right order.

Why two hands: the cover springs shut over the slot whenever the pedal is let go, and the pedal is 20 cm from the slot.
With one hand (the right doing its part as in the demo, the left never moving) the demo scored 0.0: the cover stayed shut, the dirty filter stayed under it in its slot, and the new filter, brought over, had nowhere to go.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The housing, the holder and the bin are fixed; the
cover and both filters are loose rigid bodies made of shapes from `bodykit.py`.

- The cover: one rigid plate with its pedal tail, on a hinge (a `Hinge` from `bodykit.py`) along the housing's left top
  edge, up to 85 degrees open, with a spring that shuts it. Pressing the pedal down turns it up, like a pedal bin's lid.
- The filters: rigid panels with a tab on top; the slot and the holder hold them upright by rails at their ends.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, both arms back where they started, with the cover and both filters still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see
`task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from
the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176
before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swap_the_dust_filter_under_a_spring_shut_cover.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swap the dust filter: with one hand press the pedal on the cover's tail down and keep it down, so the cover stays open; with the other lift the grey dirty filter out of its slot by its tab and drop it in the bin, then take the new white filter from its holder and lower it into the slot. Let the pedal up so the cover shuts, and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/filter-dream
```

The scripted demo, which reaches full score: the left hand comes down on the cover's pedal, fingers closed, and presses
it down along its swing; the cover swings up. The right hand comes straight down on the dirty filter's tab, fingers
closing across it, lifts the filter out of the slot, carries it over the bin and lets it drop in; takes the new filter
by its tab out of its holder, carries it over the slot, lowers it in and lets go. The left lets the pedal up: the cover
springs shut. Both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=swap_the_dust_filter_under_a_spring_shut_cover.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Swap the dust filter: with one hand press the pedal on the cover's tail down and keep it down, so the cover stays open; with the other lift the grey dirty filter out of its slot by its tab and drop it in the bin, then take the new white filter from its holder and lower it into the slot. Let the pedal up so the cover shuts, and take both arms back to where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/filter-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/filter-dream/*.json; do inspect-robots video "$log"; done
```

Set `FILTER_DEBUG=1` to print what the score is built from twice a second: how far the cover is open, where both
filters are, whether the new one is seated and the old one in the bin, whether both hands are clear, and where each
TCP is from the pedal and the tabs.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,400 steps (403 s on the clock, with Dream answering in 0.74 s on average), and about 2 minutes for the scripted demo (584 steps, 29.2 s of simulated time, 112 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
