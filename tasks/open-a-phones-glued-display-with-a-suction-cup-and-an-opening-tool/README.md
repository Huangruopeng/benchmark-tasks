# Open a phone's glued-on display like a book without tearing its cables

A smartphone lies face up in a repair cradle. Its glass display is glued to the aluminium frame by an adhesive strip
along its right edge and the right halves of its top and bottom edges; along its left edge, short flex cables join it
to the phone. On the cradle lie a suction cup and an opening tool (a thin steel blade on a blue handle), and left of
the phone stands a foam block to lean the display against. The job is the first step of most phone repairs: free the
display from its adhesive, open it like a book over its cable edge until it leans on the block (no further, or the
cables tear), put the tools down clear of the phone and take both arms home. The task is one task file,
[`open_a_phones_glued_display_with_a_suction_cup_and_an_opening_tool.py`](open_a_phones_glued_display_with_a_suction_cup_and_an_opening_tool.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

The instruction gives the goal and its limits only (free it, open it over the cable edge onto the block, no further,
tools down, arms home); how to get the display open is left to the policy. The way repair guides do it: the suction cup
lifts the display's free edge to open a gap, a thin blade slips into the gap while it is held open, the blade runs along
the glued edges, and the freed display is swung over.

Why two hands: the adhesive is springy. Pulled up by a suction cup near the right edge, the display's right edge lifts
about 3 mm; the moment the pull stops it snaps back flush. The blade can't enter a closed seam (its square end meets the
display's edge face), so one hand must hold the gap open with the cup while the other slides the blade in. With one hand (the right doing
exactly its part of the demo, the left never moving) the demo scored 0: with nothing lifting the display, the blade's
end stopped against the display's right edge at every attempt and never got into the seam, so no adhesive came free.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cradle (with the phone's frame in its recess) and
the foam block are kinematic, fixed where they are put; the display, the opening tool and the suction cup are loose
rigid bodies made of shapes from `bodykit.py`.

- **Cables:** the display is joined to the phone along its left edge by a hinge with a stop at 115 degrees, standing
  in for its flex cables. Opened past 108 degrees, or pulled 5 mm off that edge, the cables count as torn, for the
  rest of the trial.
- **Adhesive:** seven glued segments: the right edge in three, and the right halves of the top and bottom edges in two
  each. Each acts as a spring (257 N/m) at its distance from the cable edge, against the display's turn about that edge.
  Together they make a drive spring on the hinge of 7.0 N m per radian, so a 3 mm gap at the right edge takes about
  6 N at the cup. A segment comes free for good when a point of the blade is in the seam under it: inside the
  display's outline (at least 2 mm, at most 16 mm in from that edge, within that segment's stretch), no more than
  2.5 mm below the display's underside. The blade can be there only once a gap has let it in. After every control step
  the task file sets the hinge's spring to what the segments still glued give; once all are free, only the cables'
  slight damping is left.
- **Suction cup:** its seal is a joint from the display's face to the cup's rim, authored switched off. Held flat on the
  glass (within 1 mm, tilted under 8 degrees, a hand on its handle) for 0.3 s, it seals: the cup then stays on the
  glass, square to it (it may spin about its own axis), and holds where it was put with springs capped at 10 N. It pops
  off when the pull passes 11 N, read from how far the pull has opened the display against the adhesive (the
  adhesive's torque over the cup's lever, which is the force in the seal), or when pushed 6 mm along the glass.
- **Seam:** the cradle's top is flush with the seam, the frame's top under the display, so the blade slides flat along
  the cradle into the seam. The blade is 1.5 mm thick and the gap about 3 mm; in a real phone they are about 0.4 mm and
  1 mm.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the display and both tools still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75
or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it. The cameras
render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_a_phones_glued_display_with_a_suction_cup_and_an_opening_tool.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "This phone's display is glued to its frame along its right edge and the right halves of its top and bottom edges, and joined to the phone by short cables along its left edge. Using the tools on the cradle, free the display from all of its adhesive and open it like a book over its left edge until it leans on the foam block, never opening it further than that. Then put both tools down on the cradle, clear of the phone and with nothing left stuck to the display, and take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/phone-dream
```

The scripted demo, which reaches full score:
1. The left hand takes the suction cup by its handle, sets it flat on the display near the top of its right edge and
   presses it on until it seals.
2. It pulls straight up: the right edge lifts about 3 mm.
3. The right hand slides the opening tool flat along the cradle, blade first, into the gap low on the right edge.
4. The left pulls harder until the cup pops off, puts the cup back on the cradle and goes home. The display now rests
   on the blade.
5. The right runs the blade down to the bottom end of the right edge, up the whole edge, and pushes it in under the top
   edge and back. It runs it down again and pushes it in under the bottom edge and back.
6. It lifts the freed display on the blade, swings it up over its cable edge and pushes it just past upright, so it
   leans on the foam block.
7. It lays the tool back where it was and goes home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_a_phones_glued_display_with_a_suction_cup_and_an_opening_tool.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "This phone's display is glued to its frame along its right edge and the right halves of its top and bottom edges, and joined to the phone by short cables along its left edge. Using the tools on the cradle, free the display from all of its adhesive and open it like a book over its left edge until it leans on the foam block, never opening it further than that. Then put both tools down on the cradle, clear of the phone and with nothing left stuck to the display, and take both arms back to where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/phone-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/phone-dream/*.json; do inspect-robots video "$log"; done
```

Set `PHONE_DEBUG=1` to print four times a second what the score is built from:
- how far the display is open;
- which adhesive segments are free, and whether the blade has been in the seam;
- whether the cup is sealed, the pull it carries, and how often it has sealed and popped;
- where the blade's tip, the tool and the cup are;
- whether the cables are torn;
- whether the tools are down;
- whether both arms are home, and where each TCP is.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 2,400 steps (416 s on the clock, with Dream answering in 0.60 s on average), and about 4 minutes for the scripted demo (867 steps, 43.4 s of simulated time, 254 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

The layout is the same in every trial; the task file uses no randomness. `-E object_jitter=0` keeps every object where
the task file puts it.
