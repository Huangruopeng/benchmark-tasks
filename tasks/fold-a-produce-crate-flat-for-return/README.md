# Fold a produce crate flat for return

An empty green folding produce crate stands up on the returns bench between the two arms. It is a reusable plastic
container of a standard size: 400 x 300 x 174 mm, 0.88 kg. Its 400 mm length points away from the robot, so one
short end wall faces the robot and the other faces away. The job is the whole real one: fold the crate flat for the
return trip, then bring both arms back. The robot is told only:

> Fold the empty crate flat for return, and finish with both arms back where they started.

The task is one task file, [`fold_a_produce_crate_flat_for_return.py`](fold_a_produce_crate_flat_for_return.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

What it tests: freeing a latched wall by working two catches at once, in the order the crate's hardware sets (end
walls first, then the long walls on top), without shoving the crate along the bench, and leaving the crate flat with
nothing holding it.

Why two hands: each end wall is latched to the long walls by a sprung catch at each of its top corners. The two thumb
buttons are 200 mm apart, too far for one gripper, and while either catch is latched the wall stays up. A button
pushed from outside alone shoves the 0.88 kg crate along the bench (it slides at about 3 N); squeezing the button and
the top rail together holds it in without moving the crate. With one hand (the right doing exactly its part of the
demo, the left never moving) the demo scored 0. The right squeezed the near end wall's right-hand button fully in
(6.2 mm), but the other catch held the wall: tipping it dragged the whole crate 17 mm along the bench while the wall
turned 0.4 degrees. The far end wall stayed latched too, and the push on the right long wall, still held up by the
latched end walls, shoved the crate 60 mm sideways. Nothing folded.

Reusable folding crates carry fresh produce from growers to stores. Emptied at the store or distribution centre, they
are folded flat and stacked for the return truck, by hand: by packers and packagers (SOC 53-7064, about 0.6 million
US workers, BLS) and by stockers and order fillers (53-7065, about 2.8 million), whose O*NET tasks include "Pack
and unpack items to be stocked on shelves in stockrooms, warehouses, or storage yards."

## The real crate: what is sourced and what is inferred

This is a **corner-catch folding crate**: each end wall has its own catch at each top corner, worked by a thumb
button. It is not an IFCO crate.

- **Corner catches, one per corner.** Rehrig Pacific's patent
  [US7017766](https://patents.google.com/patent/US7017766) ("Collapsible container with side wall latching
  capability") puts a latch at each end of each side wall, each with its own release tab in a recess on the outside of
  the wall. The two latches of an end wall sit at its two corners.
- **Makers' fold instructions.**
  - Bag-A-Nut's collapsible harvest crate
    ([listing](https://baganut.com/product/collapsible-storage-bin-39-liter-foldable-crate-stackable-plastic-utility-basket/)):
    "Using your thumbs, press buttons on the short ends and fold the flaps down. Fold the long sides in."
  - NPC's 48 L collapsible container, NCC705
    ([listing](https://eezee.sg/product/npc-48l-plastic-collapsible-container-ncc705-111d12f74c)): "First, tap the
    lower part of the short sides inward and then press both ends of the short sides inward with fingers."
  - The order, short end walls first and then the long walls down onto them, is the same on Bag-A-Nut's
    [lift-lock crate](https://baganut.com/product/8-collapsible-storage-crate/): "Unlock short side walls by lifting
    the handles (lift locks) and fold them down. Fold long sides down onto the folded short sides."
- **Not IFCO.** IFCO's Lift Lock crates free each end wall with one lift bar, not two catches. IFCO's patents
  [US7011225](https://patents.google.com/patent/US7011225) and [US8424710](https://patents.google.com/patent/US8424710)
  describe one bar along the short wall carrying a lug at each end. Lifting it at the handhold frees both corners, and
  the wall can be folded in with the same hand. IFCO's Lift Lock product text says: "With just a slight upward
  lift on each locking bar ... users can unfold or fold the IFCO RPCs". The same patents set the order modelled
  here: end walls first, then the long walls down on top.
- **Size and mass.** 400 x 300 mm folding produce crates are a standard footprint: Euro Pool System's green tray 154
  is 400 x 300 x 153 mm at 690 g
  ([datasheet](https://www.europoolsystem.com/app/uploads/sites/2/2024/02/0.1.021RF-Product-datasheet-green-trays.EN_.pdf)),
  and Logimarkt's [FSC4318](https://logimarkt.com/foldable-agri-crates/) is 400 x 300 x 180 mm at 1.13 kg. The
  174 mm height and 0.88 kg are within that range.
- **Inferred, not measured:**
  - the buttons' size (16 x 16 mm) and travel (6 mm proud, freeing the corner at 3.5 mm);
  - their spring (3 N at rest, 9 N pressed flush);
  - their place, 100 mm either side of the wall's middle on the top rail's outer face;
  - how hard a latched wall must be pushed before its catch counts as forced (about 30 N at the rail).

## How it is modelled

The crate is nine loose rigid bodies made of shapes from `bodykit.py`, at their real sizes:

| Body | Mass | Shape |
|---|---|---|
| base | 0.40 kg | 6 mm lattice floor; side rims 68 mm tall, end rims 48 mm tall |
| 2 end walls | 0.09 kg each | 270 x 126 mm, with a 120 x 36 mm handhold under a 16 mm round top rail |
| 2 long walls | 0.13 kg each | 396 x 106 mm, with a 14 mm round top rail and vents |
| 4 thumb buttons | 0.01 kg each | 16 x 16 mm |

All walls and rims are 10 mm thick.

- **Hinges.** Each wall hangs on a hinge (a PhysX revolute joint) at the inner top edge of its rim, with stops at
  upright and at lying flat. A wall's weight sits just outboard of its hinge, so a free wall stands on its upright stop
  until it is tipped in about 5 degrees. Then it folds down onto its stop.
- **Folded stack.** The end walls lie lowest. The long walls hinge 20 mm higher and lie 5-10 mm above them. Folded
  flat, the crate is 80 mm tall.
- **Catches.** Each button rides on a sprung slide (a prismatic joint) in its end wall's top rail.
  - A latched end wall is held upright by a stiff hinge spring (1.2 N m per degree).
  - The spring lets go when both of that wall's buttons are in 3.5 mm or more at the same time. The wall is then free
    on its hinge, against light hinge friction.
  - Swung back up to within 1 degree of upright with both buttons out, it latches again.
- **Long walls.** Each is held up by the same kind of spring while either end wall is latched (the end walls' catches
  hold them), and is free once neither is.
- **Order.** It is set by the hardware, as on the real crate. The long walls cannot fold while an end wall is latched,
  and a long wall lying flat blocks the end walls under it from coming down.
- **Friction.** Polypropylene friction is 0.35 against the bench (the lower of the two is used). PhysX runs at
  240 Hz, the arms at 20 Hz.
- **Simplified:** the walls are solid plates with the real outer size (real crate walls are ribbed lattices). The
  catches are a hinge spring switched by the two buttons' positions, not a moulded bolt.

## Scoring

Followed over the whole trial, a trial scores 0 for good if:

- the crate falls off the bench (its base more than 10 cm below the bench top); or
- a catch is forced: a latched end wall, or a long wall held up by a latched end wall, is pushed in more than
  3 degrees (about 30 N at its top rail).

Judged at the end, on live states, a wall lies flat when it is within 5 degrees of its folded stop.

- **1.0** needs all of these:
  - all four walls flat, with the long walls over the flat end walls;
  - the folded crate under 90 mm tall at its highest point;
  - its base on the bench and within 3 degrees of level;
  - both arms back where they started (each gripper's fingertip centre within 5 cm of its start).
- **Otherwise:** 0.2 for each end wall lying flat, plus 0.2 for each long wall lying flat over both flat end walls,
  at most 0.8.

The trial ends at the run limit, or once the full score has held for 1 s with the crate and its walls still.

In a real trial:

- measure the folded height with a ruler (under 90 mm);
- check from the video that the end walls went down first and the long walls lie on top;
- check that no catch was forced: each end wall still clicks into both catches when stood back up;
- check the arms against marks at their start.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the crate and its walls still) has held for 1 s,
otherwise `score()` at the end of the episode. `-E object_jitter=0` keeps the crate exactly where the task file puts
it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos. Dream-1's client
scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same
input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=fold_a_produce_crate_flat_for_return.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fold the empty crate flat for return, and finish with both arms back where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/crate-dream
```

The scripted demo reaches full score from fixed waypoints and reads no simulator state.

1. Both hands go to the near end wall's top corners. Each pinches the top rail with the thumb button under its outer
   finger, the grips leaning 15 degrees forward. The squeeze presses both buttons in without pushing the crate along.
2. Holding them in, both hands tip the wall about 20 degrees in and let go, and it drops flat.
3. The same at the far end wall.
4. With both end walls down, each hand pushes its long wall in over its stop from outside, and the wall drops flat on
   top.
5. Both arms go back to where they started, showing the folded crate.

Measured over three demo runs, all 1.0:

- the squeezes pressed the buttons 6.3-6.8 mm in (3.5 mm frees a corner);
- each end wall was tipped about 20 degrees before it was let go, then it dropped flat;
- the crate moved 3 mm along the bench in all;
- folded, it stood 80 mm tall;
- the full score held from 25.5 s, and the trial ended at 26.2 s (525 steps).

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=fold_a_produce_crate_flat_for_return.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Fold the empty crate flat for return, and finish with both arms back where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/crate-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/crate-dream/*.json; do inspect-robots video "$log"; done
```

Set `CRATE_DEBUG=1` to print what the score is built from twice a second:

- each wall's fold angle;
- how far each button is pressed in;
- which end walls are latched and which long walls are held;
- whether a catch was forced;
- where the crate's base is and its tilt;
- the crate's height;
- where each TCP is, the finger openings, and whether the arms are home.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,600 steps (537 s on the clock, with Dream answering in 0.76 s on average), and about 2 minutes for the scripted demo (525 steps, 26.2 s of simulated time, 95 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
