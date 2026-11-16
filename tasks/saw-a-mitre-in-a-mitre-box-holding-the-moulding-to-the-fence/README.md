# Saw a 45-degree mitre through a picture-frame moulding in a mitre box

On the table stands a beech mitre box on a long base board: a channel between a front wall and a back fence, with
saw slots cut through both walls at 90 degrees and at 45 degrees either way. In front of it lies a length of
black-lacquered pine picture-frame moulding with a gold lip, a white pencil line across it at 45 degrees near its right
end. To the right a 10-inch gents saw (a fine-toothed steel blade with a brass back and a turned rosewood handle) stands
in its wooden rest. The job: saw the moulding right through at its pencil line in the 45-degree slot, the moulding
against the back fence, so the cut is a true 45-degree mitre (within half a degree, so that it closes a frame corner
without a gap), then leave the saw back in its rest, the cut moulding on the table in front of the box and both arms
back where they started. The instruction states only this goal and these constraints. One way, as in the scripted demo: one hand lays
the moulding in the box at its mark against the fence and holds it there, while the other saws back and forth in the
slot until the offcut comes free and stands the saw back in its rest; the first hand then lifts the cut moulding, its
fresh 45-degree end showing bare wood, out onto the table. The task is one task file,
[`saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py`](saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests: placing a long, light workpiece precisely in a fixture (against a fence, a mark on a slot), holding it
there, square to the fence, with one hand for a long time while the other hand works a tool through a guide, a
repeated stroke that only counts when the workpiece stays put and that must run true to half a degree, and putting the
tool and the work back where they belong.

Why two hands: the moulding weighs 94 g and the box's floor and fence are smooth, while the saw's teeth grip the
bottom of the cut: every stroke drags a moulding nobody holds along the box with the saw, off its mark, and a stroke
only counts while the moulding stays put. One-hand check (the moulding already laid at the mark against the fence, the
right hand doing its part of the demo, the left never moving): 0.25, only for the moulding having been at the mark.
The first stroke dragged it 6.5 mm and the next ones 2.6 cm along the box, off the mark, and no stroke counted. The left
hand alone (laying and holding the moulding) saws nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mitre box and the saw rest are kinematic, fixed
where they are put; the moulding (two pieces, see below), the saw and the plate at the bottom of the cut are loose
rigid bodies made of shapes from `bodykit.py`.

- The box: the walls' collision is built from thin slices with openings where the three slots cross them. Each slot
  is 2.5 mm wide (square to it) for the simulated saw's 2 mm blade, so the slots hold the saw's direction as a real
  box's do: through both walls (12.7 cm from outer face to outer face along a 45-degree slot) the blade can turn at
  most atan(0.5 mm / 127 mm) = 0.23 degree. The top 8 mm of each slot is chamfered out to 1 cm, a lead-in that lets a
  saw held a few millimetres off find the slot. The walls stand 4 cm over the floor, below the saw's brass back when
  its teeth reach the floor. What is drawn is a solid wall with a dark kerf down each slot. The box's floor and fence
  are smooth (their contacts take the lower of the two frictions, 0.18): a moulding nobody holds slides in the box.
- The cut: the moulding is two bodies joined by a fixed joint (a `Glue` from `bodykit.py`), the length left of the
  pencil line and the 5 cm offcut right of it, each built of slices across its width so that both ends at the line
  run at 45 degrees. The saw passes through the moulding (they do not collide); its teeth ride instead on a plate
  that is not drawn, held inside the moulding at the pencil line by a sprung slide (a `Slider` from `bodykit.py`).
  Its friction with the saw's teeth (1.2) drags the moulding along with every stroke.
- A stroke: after every control step the task file finds where the saw lies against the 45-degree slot (along it,
  across it, how it is turned and leaning) and how high its teeth are over the plate. The saw is in the slot when it
  is turned within 8 degrees of the slot, leans less than 12 degrees, lies within 9 mm of the slot's line, and its
  teeth over the plate are within 7 mm above and 8 mm below the plate's top. A stroke counts when the saw, in the slot,
  is pushed 3 cm along it while the moulding lies at the mark and moves no more than 4 mm. Each stroke lowers the
  plate 2.875 mm (a deeper cut); after the eighth the simulation counts the moulding as cut through, and the joint
  between the two pieces switches off. This stroke count stands in for sawing, which the simulation does not model; a
  real backsaw's depth per stroke varies with the wood, the teeth and the pressure, so real trials measure the kerf
  instead (see below).
  Sawdust is drawn on the box's floor after two and six strokes, and the saw cut across the moulding's top while
  sawing.
- The cut's angle: after every control step the task file also reads the blade's direction in the moulding's own
  frame, from the two bodies' poses: its angle to the moulding's back edge is the angle the saw is cutting into the
  wood. Averaged over each counted stroke, that is the stroke's angle. The cut is true when every counted stroke's
  angle is within 0.5 degree of 45. The slot fixes the blade's direction in the box to within 0.23 degree, so what
  moves the cut off 45 degrees is mostly the moulding turning off the fence, as in a real box.
- At the mark: the moulding's pencil line within 1 cm of the 45-degree slot along the box, its back face within 6 mm
  of the fence, turned less than 4 degrees and lying on the box's floor.

## Scoring a real trial

The tiers are the same; the cut is measured on the wood instead of read from the simulation:
- **Where the cut is.** After the trial the cut counts only where it crosses the moulding within 1 cm of the pencil
  line. Whether the moulding was laid at its mark, and the end state, are judged from the video.
- **True to 45 degrees (needed for 0.75 and 1.0).** Hold a 45-degree mitre square to the kept piece's back edge (the
  edge that lay against the fence), its blade along the cut face across the top, and try a 0.33 mm (0.013 in) feeler
  gauge between them: it must not enter anywhere. The cut face is 26 mm / sin 45 degrees = 36.8 mm long across the top,
  so the gap at its open end is 36.8 mm x tan(error): 0.32 mm at 0.5 degree, and 0.33 mm is 0.51 degree. A bevel
  protractor or a digital angle gauge reading 0.1 degree, at 44.5 to 45.5 degrees, does the same job.
- **Why half a degree.** Dry-fitted against a true 45-degree mitre (both pieces' outer edges on a framing square), a
  cut e degrees off leaves the corner joint touching at one end and open at the other by 36.8 mm x tan e: 0.32 mm at
  0.5 degree, 0.64 mm if two mating cuts are both 0.5 degree off the same way, and 2.6 mm at 4 degrees.
- **0.5: kerf depth.** A kerf at least 11 mm deep, half the moulding's 22 mm height, measured with a caliper's depth
  rod from the moulding's top face to the deepest point of the kerf; or a cut through that is not true. In simulation,
  four counted strokes (11.5 mm), or eight with a stroke more than 0.5 degree off.
- **0.75: cut through and true.** The offcut has come free along the kerf, sawn over the moulding's full height and
  width, and the cut passes the mitre-square test. An offcut snapped or torn off with uncut fibres is not a
  through-cut; the kerf's sawn depth scores instead. In simulation, the eighth counted stroke (23 mm), every counted
  stroke within 0.5 degree of 45.
- **1.0** adds the end state, judged as in simulation: the saw standing back in its rest, the cut moulding on the table
  in front of the box and both arms back where they started.

The real rig: a beech (or hardwood) mitre box with the slots as specified, sawn to the saw's kerf, screwed or clamped
down; a 34 cm length of 26 x 22 mm pine picture-frame moulding (about 94 g), black-lacquered, marked with a 45-degree
pencil line 5 cm from its right end; and a 10-inch gents saw with a brass back and 20 teeth per inch, such as the
Thomas Flinn Pax 10-inch gent's saw (255 mm cutting length, 0.56 mm plate, about 0.7 mm kerf), standing in a wooden
rest. A slot sawn by that saw leaves it about 0.15 mm of play, under 0.1 degree over the 12.7 cm through both walls;
the simulation's 0.5 mm of play is looser. Use a fresh moulding for each trial.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes both arms back where they started, with
the saw and both pieces of the moulding still) has held for 1 s, otherwise `score()` at the end of the episode (0,
0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it,
so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales
every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Saw the moulding right through at its pencil line in the mitre box's 45-degree slot, with the moulding against the box's back fence, so the cut is a true 45-degree mitre (within half a degree). Leave the saw standing back in its rest, the cut moulding lying on the table in front of the box and both arms back where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/mitre-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the moulding 13.7 cm from its left end, fingers closing across it, its grip turned 4 degrees against the moulding's measured turn in the fingers, lifts it over the box's front wall and lays it in the channel 7 mm clear of the fence with the pencil line at the 45-degree slot, then slides it back against the fence and holds it there, pressing it down. The right hand comes down on the saw's handle, fingers closing across it, lifts the saw out of its rest, turns it 45 degrees over the slot (its grip turned 2.9 degrees against the saw's measured turn in the fingers) and lowers it through the slots' chamfered mouths, teeth on the pencil line. It saws ten strokes, 6.5 cm forward and back along the slot, each aimed a little deeper; the offcut comes free after the eighth. It eases the saw up out of the cut, toe raised 4 degrees, lifts it clear, turns it back and sets it in its rest toe first (toe 5 degrees down into the front V-block, then the rest of it), and lets go. The left hand lifts the cut moulding out of the box over the front wall and lays it on the table in front of the box, its fresh 45-degree end showing bare wood, and lets go; both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Saw the moulding right through at its pencil line in the mitre box's 45-degree slot, with the moulding against the box's back fence, so the cut is a true 45-degree mitre (within half a degree). Leave the saw standing back in its rest, the cut moulding lying on the table in front of the box and both arms back where they started." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/mitre-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/mitre-dream/*.json; do inspect-robots video "$log"; done
```

Set `MITRE_DEBUG=1` to print what the score is built from four times a second: where the moulding is against its
mark and how it is turned, whether it is at the mark, where the saw lies against the slot (along, across, how high
its teeth are over the bottom of the cut) and how it is turned, leaning and tipped, whether it is in the slot, how far
the moulding has moved during the stroke, how many strokes have counted, the blade's angle and lean in the moulding's
frame now and each counted stroke's angle off 45 degrees, whether the cut is through, how deep the cut is, where the
saw is against its rest, each hand's finger opening and where each TCP is.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,400 steps (446 s on the clock, with Dream answering in 0.69 s on average), and about 2 minutes for the scripted demo (754 steps, 37.7 s of simulated time, 121 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-FSGBBT. Its review said:

> Scoring: should require much tighter tolerances as to not reveal a gap when assembled into a picture frame, that would be about half a degree.

The review before that (TK-X5B2Y5) asked for real-world scoring by measured kerf depth for partial credit and an
actual through-cut for cut credit, keeping the stroke count only as the simulation's approximation; that was done in
the previous revision and is kept.

What changed in response:

- **Half-degree tolerance:** the cut must now be true to 45 degrees within 0.5 degree (it was plus or minus 4) for the
  0.75 tier and for full score. In simulation every counted stroke must be within it. The instruction now says so:
  "so the cut is a true 45-degree mitre (within half a degree)".
- **The gap it allows:** a cut e degrees off, dry-fitted against a true 45-degree mitre, leaves the joint open at one
  end by (w / sin 45 degrees) x tan e, the cut face being w / sin 45 degrees long across a moulding w wide. For this
  26 mm moulding that is 36.8 mm x tan 0.5 degree = 0.32 mm (0.64 mm if two mating cuts are both 0.5 degree off the
  same way); the old 4 degrees allowed 2.6 mm.
- **Real-trial measurement:** a 45-degree mitre square held to the kept piece's back edge, its blade along the cut face
  across the top; a 0.33 mm (0.013 in) feeler gauge must not enter between them anywhere (0.33 mm over 36.8 mm is 0.51
  degree). A bevel protractor or digital angle gauge reading 0.1 degree, at 44.5 to 45.5, does the same.
- **What the simulation measures:** each stroke's angle is the blade's direction in the moulding's own frame, read from
  the two bodies' poses after every control step and averaged over the push: the angle the saw cuts into the wood.
  The slots were 9.9 mm wide square to the cut (1.4 cm along the walls), so a saw could turn 3.6 degrees in them. They
  are now 2.5 mm wide for the simulated 2 mm blade, with a chamfered lead-in at the top: as in a real box they hold the
  saw's direction, here to 0.23 degree (0.5 mm over the 12.7 cm through both walls), so what moves the cut off 45
  degrees is the moulding turning off the fence. The walls are now 4 cm tall (were 5), below the saw's brass back at
  full depth.
- **Demo meets it:** its eight counted strokes ran at 45.05 to 45.16 degrees to the moulding's back edge (worst 0.16
  degree off, a 0.10 mm gap). Before this revision the demo's moulding lay 3.2 degrees off the fence, which the old
  plus or minus 4 accepted; the left hand's grip is now turned 4 degrees against the moulding's turn in its fingers, so
  it lies square (0.0 to 0.3 degree off while it is sawn).
- **Intermediate tiers:** 0.5 now also covers a cut through that is not true (a mitre that would show a gap is worth no
  more than a half-depth kerf); 0.75 is a true cut through; 0.25 (laid at the mark) is unchanged.
- **Saw:** a 10-inch gents saw, 25 cm blade with 20 teeth per inch (was 24 cm, about 15), as real gents saws are.
- **Re-run:** demo 1.0, one-hand check 0.25 (unchanged reason: the strokes drag a moulding nobody holds 2.6 cm
  along the box and none counts), Dream-1 0.0.
