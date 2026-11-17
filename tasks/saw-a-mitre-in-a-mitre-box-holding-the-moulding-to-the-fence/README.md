# Saw a 45-degree mitre through a picture-frame moulding in a mitre box

On the table stands a beech mitre box: a channel between a front wall and a back fence, with saw slots cut through
both walls at 90 degrees and at 45 degrees either way. In front of it lies a length of black picture-frame moulding
with a gold lip, a white pencil line across it at 45 degrees near its right end. To the right a 10-inch gents saw (a
fine-toothed steel blade with a brass back and a turned walnut handle) stands in its wooden rest. The job: saw the
moulding right through at its pencil line in the 45-degree slot, the moulding against the back fence, so the cut is a
true 45-degree mitre (within half a degree, so that it closes a frame corner without a gap), then leave the saw back
in its rest, the cut moulding on the table in front of the box and both arms back where they started. The instruction
states only this goal and these constraints. One way, as in the scripted demo: one hand lays the moulding in the box
at its mark against the fence and holds it there, while the other saws in the slot, cutting on the push, until the
offcut comes free and stands the saw back in its rest; the first hand then lifts the cut moulding, its fresh 45-degree
end showing bare wood, out onto the table. The task is one task file,
[`saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py`](saw_a_mitre_in_a_mitre_box_holding_the_moulding_to_the_fence.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder. The box, the saw and the moulding are real, purchasable products at their listed sizes
(see "The real rig").

Why it matters: sawing a piece of wood by hand is one of the most familiar manual jobs there is, and the mitre box is
how it is done at home when the cut must be angled: making a picture frame, or fitting a skirting board or trim round
a corner. A few seconds of video of one hand holding a moulding to the box's fence while the other saws are enough
for anyone to see what is being done and why it is hard to do well. The tools are everyday ones (the box and saw used
here cost $56 together). The catch everyone who has done it knows: the box sets the saw's angle, but the cut is only as
true as the hand holding the moulding to the fence, and a frame corner shows a gap if a mitre is half a degree off.

What it tests: placing a long, light workpiece precisely in a fixture (against a fence, a mark on a slot) and holding
it there, square to the fence to half a degree, for the whole cut, while the other hand drives a tool through a guide
against the wood's resistance. This is more than holding a part still while the other hand works on it: every push
of the saw drags on the moulding with several newtons along a slick box, so the hold has to resist a repeated load
without letting the workpiece slide or turn, and the sawing hand has to keep the teeth bearing on the cut with the
right feed (a stroke counts only under real contact and resistance) along a slot that leaves the blade 0.2 degree of
play. Then the tool and the work go back where they belong.

Why two hands: the moulding weighs about 98.5 g and the box's fence and packing strip are smooth, while the cut resists every stroke
of the saw with several newtons: each stroke drags a moulding nobody holds along the box, off its mark, and a stroke
only counts while the moulding stays put. One-hand check (the moulding already laid at its mark against the fence,
the right hand doing its part of the demo, the left never moving): 0.25, only for the moulding having been at its
mark. The first push dragged it 8 mm along the box within half a second and 26 mm by the push's end, off its mark,
and no stroke counted. The left hand alone (laying and holding the moulding) saws nothing.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mitre box (with the support block) and the saw
rest are kinematic, fixed where they are put, as is the packing strip; the moulding (two pieces, see below), the saw
and the plate at the bottom of the cut are loose rigid bodies made of shapes from `bodykit.py`.

- The box: a Pinie 250 mm solid-beech mitre box at its listed size, 245 x 103 x 60 mm outside: an 18 mm birch-ply base
  and two 19 mm beech walls standing 41 mm over it either side of a 65 mm channel. The walls' collision is built from
  thin slices with openings where the three slots cross them. Each slot is 2.5 mm wide (square to it) for the
  simulated saw's 2 mm blade, so the slots hold the saw's direction as a real box's do: through both walls (145.7 mm
  from outer face to outer face along a 45-degree slot, (65 + 2 x 19) mm x sqrt 2) the blade can turn at most
  atan(0.5 mm / 145.7 mm) = 0.20 degree. The top 8 mm of each slot is chamfered out to 1 cm, a lead-in that lets a saw
  held a few millimetres off find the slot (the real box's slots have none). What is drawn is a solid wall with a dark
  kerf down each slot. A 6 mm MDF packing strip lies along the channel's floor and on over a support block of the same
  18 mm ply butted to the box's left end, under the moulding's far end. The moulding lies on it, so the saw, whose
  teeth reach 40 mm below the walls' tops (its depth under the brass back), cuts through the moulding and up to 5 mm
  into the strip instead of stopping on the box's floor (in simulation the saw passes through the strip). The fence is smooth (its contacts take the lower of
  the two frictions, 0.18) and so is the strip's face (0.30): a moulding nobody holds slides in the box.
- The cut: the moulding is two bodies joined by a fixed joint (a `Glue` from `bodykit.py`), the length left of the
  pencil line and the 5 cm offcut right of it, each built of slices across its width so that both ends at the line
  run at 45 degrees. The saw passes through the moulding (they do not collide); its teeth ride instead on a plate
  that is not drawn, held inside the moulding at the pencil line by a sprung slide (a `Slider` from `bodykit.py`).
- Sawing, and the cut's resistance: the plate's friction on the teeth (1.5) is the cut's resistance. It acts on the
  saw against its motion and, through the plate, drags the moulding along with every stroke. After every control step
  the task file reads from PhysX's contact report the force with which the plate presses up on the teeth and the
  friction between them. A control step saws only if the saw is in the 45-degree slot (turned within 8 degrees of
  it, leaning less than 12 degrees, within 9 mm of its line), moves forward along it while the moulding lies at the
  mark, and its teeth press on the plate with at least 1.5 N (about the saw's own weight) at both the step's start
  and its end. A stroke is 3 cm of such sawing in one push, against a mean resistance of at least 3.5 N (at least
  0.105 J of work done against the cut), while the moulding moves no more than 4 mm. A saw moved above the cut, even a millimetre above it, touches nothing
  and saws nothing (see "The cut's resistance in numbers"). Each stroke lowers the plate 3 mm (a deeper cut); after
  the eighth (24 mm, the moulding's 23 mm and 1 mm into the strip) the simulation counts the moulding as cut through,
  and the joint between the two pieces switches off. The plate then drops away below the moulding, out of the saw's
  reach, so the box's floor carries the saw as it would at the end of a real cut, and once the whole blade is out of
  the box the plate goes back up inside the moulding. This stroke count stands in for the kerf's depth, which the
  simulation does not model: a real backsaw's depth per stroke varies with the wood, the teeth and the pressure, so
  real trials measure the kerf instead (see below). Sawdust is drawn on the strip after two and six strokes, and the
  saw cut across the moulding's top while sawing.
- The cut's angle: after every control step the task file also reads the blade's direction in the moulding's own
  frame, from the two bodies' poses: its angle to the moulding's back edge is the angle the saw is cutting into the
  wood. Averaged over each counted stroke's sawing steps, that is the stroke's angle. The cut is true when every
  counted stroke's angle is within 0.5 degree of 45. The slot fixes the blade's direction in the box to within 0.20
  degree, so what moves the cut off 45 degrees is mostly the moulding turning off the fence, as in a real box.
- At the mark: the moulding's pencil line within 1 cm of the 45-degree slot along the box, its back face within 6 mm
  of the fence, turned less than 4 degrees and lying on the strip.

## The cut's resistance in numbers

- **What a real backsaw meets.** Mellqvist, Buck and Johansson (2026, "Cutting forces in handheld wood sawing: a
  factorial study of tooth geometry and cutting direction", European Journal of Wood and Wood Products 84:132)
  measured handsaw teeth cutting Norway spruce, a softwood like this pine moulding: a mean main cutting force of
  27.3 N with a normal force of 9.4 N, a ratio of 2.9 (coarse 7-teeth-per-inch teeth taking 0.15 mm per tooth). A fine
  backsaw is meant to be fed by its own weight, the hand only guiding and pushing it, and the 160 g saw (design value)
  weighs 1.57 N:
  at that ratio its teeth meet about 2.9 x 1.57 N = 4.6 N of resistance. The simulation requires a mean of at least
  3.5 N, about three quarters of that, over every counted stroke, so each stroke does at least 3.5 N x 3 cm = 0.105 J
  of work against the cut (the 27.3 N and 9.4 N are published measurements; 4.6 N and 0.105 J are computed).
- **The simulated teeth need more feed.** The simulated friction is 1.5, so 3.5 N of resistance needs a mean of 2.3 N
  pressing the teeth in, more than a real saw's ~1.6 N. With the measured ratio itself as the friction (2.9) the
  softly held simulated saw stuck, dug in (2.5 to 3.2 N, 6.5 to 8.9 N of drag), dragged the moulding 4.4 mm and jumped
  out of the cut; this is disclosed in fidelity.
- **In the demo.** Its eight counted strokes pressed the teeth on the cut with a mean of 2.6 to 3.0 N and met a
  mean resistance of 4.0 to 4.5 N (at least 0.12 J of work each), while the held moulding moved at most 3.4 mm during
  a push. Two of its pushes passed 1.6 to 6.8 mm above the cut without touching it (the saw had tipped toe up in the
  fingers): the old rule, which accepted teeth up to 7 mm above the cut, would have counted both; the new rule counted
  neither.
- **On the moulding.** The resistance pulls the moulding along the slot, at 45 degrees to it: 4.5 N of it is 3.2 N
  along the box and 3.2 N into the fence. The fence takes the second. Against the first, the strip and the fence hold
  back at most 0.30 x (0.97 N of moulding + 3.0 N of saw pressing down) + 0.18 x 3.2 N against the fence = 1.8 N, so
  a moulding nobody holds is dragged along, as the one-hand check shows; the holding hand supplies the rest.
- **Sawing above the cut earns nothing.** The old rule counted a stroke with the teeth anywhere up to 7 mm above the
  bottom of the cut; now only contact and resistance count. In the demo itself two pushes, of 35 and 77 mm, passed
  1.6 to 6.8 mm above the cut without touching it (0 N of contact): the old rule would have counted both, the new one
  counted neither. An adversarial run replayed the demo with the saw aimed 15 to 31 mm above the uncut moulding on all
  17 pushes, the left hand holding the moulding at its mark. The saw, which pitches almost freely about the fingers on
  its round handle, drooped until its blade brushed the far edge of the cut with less than its own weight (at most
  1.7 N of contact, at most 1.5 N of resistance), its teeth 0.8 to 3.7 mm above the middle of the cut. The old rule
  would have counted all 17 pushes and cut the moulding through; the new rule counted none, and the run scored 0.25
  (for the moulding laid at its mark).

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
- **0.5: kerf depth.** A kerf at least 11.5 mm deep, half the moulding's 23 mm height, measured with a caliper's depth
  rod from the moulding's top face to the deepest point of the kerf; or a cut through that is not true. In simulation,
  four counted strokes (12 mm), or eight with a stroke more than 0.5 degree off.
- **0.75: cut through and true.** The offcut has come free along the kerf, sawn over the moulding's full height and
  width, and the cut passes the mitre-square test. An offcut snapped or torn off with uncut fibres is not a
  through-cut; the kerf's sawn depth scores instead. In simulation, the eighth counted stroke (24 mm), every counted
  stroke within 0.5 degree of 45.
- **1.0** adds the end state, judged as in simulation: the saw standing back in its rest, the cut moulding on the table
  in front of the box and both arms back where they started.

The simulation and a real trial apply the same rule: only sawing deepens the cut. A real kerf deepens only while the
teeth bear on the wood and are pushed against its resistance, which the real measurement (depth, through-cut) records;
the simulation counts a stroke only under measured contact and resistance.

## The real rig

Every item was checked on the seller's page on 2026-11-16; prices are as listed then.

- **Mitre box:** Pinie solid-beech portable mitre box, 250 mm, SKU/MPN 24-250653819. $14.99 at Taytools
  (https://taytools.com/pinie-solid-beech-portable-mitre-box-for-hand-cut-mitres?Subtype=550, which lists it as
  250 mm long, 65 mm inner width, 38 mm inner height, solid European beech) or GBP 9.00 at Workshop Heaven
  (https://workshopheaven.com/pinie-solid-beech-mitre-box/: "245mm x 103mm x 60mm" outside, "245 x 65 x 41mm" inner,
  "Cutting Height: 41mm", "19mm solid beech" sides on "18mm birch ply", "slots for 90° and two opposing 45° angles").
  The simulated box follows Workshop Heaven's figures; the slots' positions along the box and their 2.5 mm width are
  design values. Screw the box down to the bench through its base.
- **Support block and packing strip** (cut from stock sheet): a 185 x 103 mm offcut of 18 mm birch ply butted to the
  box's left end, under the moulding's far end, and a 430 x 64 mm strip of 6 mm MDF laid along the channel's floor and
  on over the block. The strip lets the saw finish the cut: the Pax's 40 to 41.3 mm under its back against the box's
  38 to 41 mm walls would otherwise stop the teeth at or just above the box's floor, under the moulding. On the strip
  the teeth reach through the moulding and up to 5 mm into the strip, and the strip, not the box, takes the kerfs;
  renew it once its kerf at the slot is about 5 mm deep.
- **Saw:** Thomas Flinn Pax 10-inch gents saw, model PG10 (made in Sheffield). $40.99 at Taytools
  (https://taytools.com/thomas-flinn-pax-10-inch-gent-dovetail-saw: "Blade depth is 1-5/8"", "Saw plate is 0.022"
  thick with just 0.003" set per side", 20 TPI, rolled solid brass back, walnut handle) or GBP 37.60 at Workshop Heaven
  (https://workshopheaven.com/pax-gentlemans-saw-10-20tpi/: "Cutting Length: 255mm", "Useable Blade Depth: 40mm",
  "0.5mm blade", "0.2mm of set on either side", "20 rip filed teeth per inch", "Handle Length: 133mm"). The simulated
  saw is 255 mm long, 40 mm deep under its back, with a 133 mm handle, and weighs 160 g (design value). Its kerf
  (plate plus set, computed) is 0.71 mm by Taytools' figures or 0.9 mm by Workshop Heaven's.
- **Moulding:** LION L2525 "26x23mm 'Fino' Antique Black Frame Moulding", finger-jointed pine or fir with a waxy
  paint finish, GBP 7.93 per metre ex VAT, sold in 2.3 to 3 m lengths
  (https://www.lionpic.co.uk/p/38375/26x23mm-Fino-Antique-Black-Frame-Moulding). Cut it into 34 cm lengths, a fresh one
  for each trial, marked with a 45-degree pencil line 5 cm from the right end. The simulation models it as a plain
  26 x 23 mm section, 98.5 g for 34 cm (computed at 0.485 g/cm3, pine).
- **Saw rest** (buildable from beech offcuts): a 310 x 60 x 4 mm base; on it two V-blocks under the blade, 30 mm long
  and 50 mm wide, 165 mm apart, their V (cheeks leaning out 25 degrees, 8 mm apart at the bottom) bottoming 10 mm above
  the table; and a V-cradle under the handle, 16 mm long, its V (cheeks at 35 degrees, 6 mm apart) bottoming 41 mm
  above the table. Exact dimensions are in the task file (`_rest`); any rest that holds the saw upright, teeth down,
  handle toward the arms, does.
- **Cost:** the box, saw, one length of moulding (6 to 8 trials), a blade guard and offcuts of ply and MDF come to
  about $100.

Slot play: a slot wider than the saw's 0.71 to 0.9 mm kerf lets the saw turn in it, by 0.20 degree with 0.5 mm of play
over the 145.7 mm through both walls (as simulated) and 0.39 degree with 1 mm (computed). Check the play with a feeler
gauge beside the plate before a series of trials. The real scoring measures the cut itself, so a looser slot makes the
task harder, not easier to score.

## Real-trial safety and rig protection

The only hazard is the saw's teeth: a light hand saw, no power tools, no flying parts (the 14.5 g offcut drops onto
the strip). The gripper pads are protected from the teeth as follows.

- **Blade guard during setup and handling.** Whoever sets up or resets the saw keeps a slip-on blade guard on its
  teeth: a 26 cm length cut from a Tools for Working Wood GT-SBG saw blade guard, "a flexible channel that snaps onto
  the blade", sold in five-foot lengths, $7.95 (https://toolsforworkingwood.com/store/item/GT-SBG). It comes off only
  once the saw stands in its rest with both arms at home, and goes back on before anyone lifts the saw out at reset.
- **The robot holds only the handle.** The saw stands handle toward the arms, so a gripper closes on the walnut handle,
  5.8 cm behind the blade's heel and level with the brass back, never near the teeth. In the scripted demo the
  sawing hand's fingertip centre stayed at least 6.7 cm from the toothed edge and the holding hand's at least
  10.5 cm (the fingertips reach 1.3 cm past it).
- **Stop rule.** The operator stops the robot if either gripper's fingers come within about 2 cm of the teeth, or the
  saw falls or jams; the trial is scored as it stands.
- **Pad sleeves (optional).** For extra protection each finger pad can be sleeved in cut-resistant knit cut from a Magid
  M-GARD 21G sleeve, TGS186 (Hyperon, ANSI/ISEA 105 cut level A6, from $25.95 a pair,
  https://www.magidglove.com/magidr-m-gard-21g-hi-vis-ultra-thin-sleeve-with-thumb-slot-ansi-cut-level-a6-tgs186pp);
  re-check the grips on the saw's handle and on the moulding with the sleeves on, since they change the pads' friction.
- **Cleaning.** Pine dust is dry: between trials brush or vacuum the box, the strip and the table, and wipe the pads
  with a dry cloth so dust does not lower their grip. Look the pads over for nicks after each session.

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

The scripted demo, which reaches full score: the left hand comes straight down on the moulding 14 cm from its left end, fingers closing across it, its grip turned 4 degrees against the moulding's measured turn in the fingers, lifts it over the box's front wall and lays it in the channel 7 mm clear of the fence with the pencil line at the 45-degree slot, then slides it back against the fence and holds it there, pressing it down. The right hand comes down on the saw's handle, fingers closing across it, lifts the saw out of its rest, turns it 45 degrees over the slot (its grip turned 2.9 degrees against the saw's measured turn in the fingers) and lowers it through the slots' chamfered mouths, teeth on the pencil line. It saws as a backsaw is used, cutting on the push: each push runs 7.7 cm along the slot with the saw aimed at the bottom of the cut, so that the arm, sagging under the saw, bears on it with about 2.5 to 3 N; the saw is eased 6 mm up off the cut for the return. It pushes twice at each depth (three times at the last), in case a push does not count, each depth 3 mm lower than the last (aimed 0.25 mm deeper still per depth, as the deeper strokes bear more lightly); the offcut comes free after the eighth counted stroke. Once the offcut is free it settles the teeth on the box's floor, the saw well forward along the slot, and presses the handle down, levering the saw 12 degrees toe up about the floor's front edge (the fingers on the round handle can barely hold the saw's pitch, so lifted level it would tip toe down onto the far slot's bottom), then lifts it briskly out of the slots and clear of the box, turns it back and sets it in its rest toe first (into the front V-block, then the rest of it), and lets go. The left hand lifts the cut moulding out of the box over the front wall and lays it on the table in front of the box, its fresh 45-degree end showing bare wood, and lets go; both arms go back to where they started:

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

Set `MITRE_DEBUG=1` to print what the score is built from four times a second (`MITRE_DEBUG_EVERY=1`: every control step): where the moulding is against its mark and how it is turned, whether it is at the mark, where the saw lies against the slot (along, across, how high its teeth are over the bottom of the cut) and how it is turned, leaning and tipped, whether it is in the slot, the contact force pressing the teeth on the cut and the friction resisting the saw, whether the step saws, the push's sawing travel and the work done against the cut, how far the moulding has moved during the push, how many strokes have counted with each one's angle off 45 degrees, mean force and mean resistance, whether the cut is through, how deep the cut is, where the saw is against its rest, how near each fingertip centre has come to the saw's teeth, each hand's finger opening and where each TCP is.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,400 steps (479 s on the clock, with Dream answering in 0.71 s on average), and about 7 minutes for the scripted demo (1,175 steps, 58.8 s of simulated time, 410 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Third-party assets

None. Every shape in the scene is a box, cylinder or sphere built in the task file; the robot arms and the
station come with the `yam_arena` embodiment.

## Changes in this revision

This is a revision of TK-HBMK3E. Its review said:

> Flags: Note for the team: protect the gripper pads from the saw teeth during setup and handling.
>
> Importance (not met): Carpenters regularly cut mitres, but the submission does not establish that over 500,000 workers routinely cut picture-frame moulding with a hand saw and mitre box; the recognizable hand-sawing task fits Societal relatability instead.
>
> Construct validity (not met): The documented stroke rule counts a saw up to 7 mm above the cut plate without contact, allowing simulated cutting without the resistance of sawing.
>
> What to change: The previous requests for a half-degree cut tolerance and real-world measurement of the kerf and through-cut are addressed. Two new points were not raised before: (1) Please change the category to Societal relatability and explain the familiar hand-sawing task there; the submission does not establish that this specific hand-saw and mitre-box method is regular work for more than 500,000 workers. (2) Change the simulated cutting rule so strokes deepen the kerf only when the teeth contact the workpiece or its stand-in under representative sawing resistance; the documented 7 mm above-plate allowance currently permits progress without contact.

The review before that (TK-FSGBBT) asked for a cut true to about half a degree, so that a frame corner shows no gap;
that was done in the previous revision and is kept. The first review (TK-X5B2Y5) asked for real-world scoring by
measured kerf depth and an actual through-cut, keeping the stroke count only as the simulation's approximation; that
is kept too.

What changed in response:

- **Category (Importance):** now Societal relatability. The context and a new "Why it matters" paragraph explain the
  familiar job instead of a workforce: hand-sawing a mitre in a mitre box to make a picture frame or fit a skirting
  board or trim round a corner at home, recognisable at a glance, with tools that cost about $56.
- **No progress without contact (Construct validity):** the 7 mm above-plate allowance is gone, and with it any test of
  the teeth's height. The task file now reads PhysX's contact report between the saw and the bottom of the cut after
  every control step. A step saws only if the teeth press on the cut with at least 1.5 N (about the saw's own
  weight) at both its start and its end.
- **Representative sawing resistance:** the cut's resistance is the teeth's friction on the bottom of the cut, which
  drags on the saw and, through the cut, on the moulding. The task file measures it, and a stroke counts only if its
  mean over the stroke's 3 cm of sawing is at least 3.5 N, i.e. at least 0.105 J of work done against the cut. The
  level comes from published data: Mellqvist, Buck and Johansson (2026) measured handsaw teeth cutting Norway spruce at
  a 27.3 N cutting force for 9.4 N pressing them in (a ratio of 2.9), so a backsaw fed by its own weight (1.57 N for
  this 160 g saw) meets about 2.9 x 1.57 N = 4.6 N; the rule asks for about three quarters of that. In the demo the eight counted strokes met 4.0 to 4.5 N with the teeth
  pressed in at 2.6 to 3.0 N.
- **Adversarial check, sawing 2 to 7 mm above the work:** a run replayed the demo with the saw aimed 15 to 31 mm above
  the uncut moulding on all 17 pushes. Its teeth stroked 0.8 to 3.7 mm above the middle of the cut, at most brushing
  the cut's far edge (at most 1.7 N of contact and 1.5 N of resistance). The old rule would have counted all 17 pushes
  and cut the moulding through; the new rule counted none (score 0.25, for the moulding laid at its mark). In the demo
  itself two pushes passed 1.6 to 6.8 mm above the cut with no contact at all, and neither counted.
- **Gripper pads (flag):** a new section, "Real-trial safety and rig protection", covers it: a slip-on blade guard
  (Tools for Working Wood GT-SBG, $7.95) on the teeth whenever a person sets up or handles the saw; the robot holds only
  the handle, 5.8 cm behind the blade's heel (in the demo its fingertip centre stayed at least 6.7 cm from the teeth); a stop rule at 2 cm; optional cut-resistant pad sleeves
  (Magid TGS186, ANSI A6); and pad cleaning and inspection.
- **Real products, real sizes:** the props now match purchasable products, with links and quoted specifications in "The
  real rig": a Pinie 250 mm beech mitre box (24-250653819; the channel is now 65 mm wide, was 60; walls 19 mm thick,
  were 15, and 41 mm tall, were 40; base 18 mm, was 15), a Thomas Flinn Pax 10-inch gents saw (PG10; blade now 255 mm,
  was 250, and 40 mm under the back, was 45; walnut handle 133 mm) and a LION L2525 26 x 23 mm frame moulding (was
  26 x 22 mm, so half depth is now 11.5 mm and each counted stroke 3 mm). A 6 mm MDF packing strip under the
  moulding lets the saw cut through, and a ply support block carries the moulding's far end.
- **Re-run:** demo 1.0 (1,175 steps; every counted stroke within 0.13 degree of 45); one-hand check 0.25 (the first
  push dragged the unheld moulding 26 mm along the box and no stroke counted); adversarial check 0.25 with no stroke
  counted; Dream-1 0.0 (it moved its right hand to the saw and never picked up the moulding).
