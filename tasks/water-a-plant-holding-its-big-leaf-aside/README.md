# Water a house plant at its roots without wetting its leaves

A house plant in a terracotta pot spreads its broad leaves low over the soil: three in the crown over the back and
sides, and one big leaf in front on a springy stalk, lying over the front of the soil. A blue watering jug with water
in it stands on a stool beside the pot. One hand pushes the big leaf round on its stalk, aside, and holds it there;
the other lifts the jug, carries it over the front of the pot and pours the water onto the soil the leaf covered,
without wetting any leaf. The jug goes back on its stool, the leaf is let go back over the soil, and both arms go back
to where they started. The task is one task file,
[`water_a_plant_holding_its_big_leaf_aside.py`](water_a_plant_holding_its_big_leaf_aside.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a sprung part out of the way while the other hand works where it was; pouring a light stream
into a target without spilling it on what is around it; putting everything back.

Why two hands: the big leaf springs back over the soil as soon as nothing holds it, and the soil it covers is the only
soil not under a leaf, so water poured with the leaf in place lands on the leaf. With one hand (the left pouring as in
the demo, the right never moving) the demo scores 0: the jug touches the big leaf, which lies over the soil, and all
12 beads land on it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The pot, its soil, the stem and the crown's three leaves are one fixed body. Each leaf is a thin rigid blade (seven
  boxes giving its oval outline) 15 cm long. The crown's leaves point straight back, back-left (60 degrees round from
  straight back) and right (78 degrees round the other way).
- The big front leaf is a loose rigid blade joined to the stem by a hinge about the vertical (`Hinge` from
  `bodykit.py`), its spring (0.0016 N m per degree) holding it over the front of the soil; pushed round 85 degrees it
  pushes back with 0.14 N m.
- The water is 12 rigid beads in the jug, which is a cup built from a floor and 16 wall panels; its pouring lip and
  handle are drawn only.
- After every physics step (240 a second, not only after each 50 ms control step), the task file checks each bead and
  the jug against every leaf. A bead that touches any leaf's blade, or comes within 4 mm of it (a margin for the
  splash and spread a rigid bead lacks), has made liquid contact with that leaf for good, even if it runs off straight
  away. Any part of the jug as drawn (body, rim, floor, pouring lip and handle: 1,609 points at most 4 mm apart)
  within 4 mm of a leaf's blade is jug contact. Either scores 0. A real trial records the same two events, on
  water-sensitive cards on the leaves and on the side video (see "A real trial"). A bead counts as watering the soil
  only if it ends lying on the soil inside the pot's wall (its centre within 7.25 cm of the pot's axis and between
  3 mm below and 15 mm above the soil's surface) and never touched a leaf. The saucer is drawn only: a bead that misses
  the pot or bounces out of it lands on the table under the saucer and never counts.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the jug, the big leaf and every bead still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=water_a_plant_holding_its_big_leaf_aside.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Water the plant at its roots, onto the soil, without wetting its leaves. Then put the jug back on its stool, leave the plant's leaves as they were and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plant-dream
```

The scripted demo, which reaches full score: the right hand comes down beside the big leaf's left edge, fingers
closed, pushes the leaf round on its stalk 85 degrees, until it points to the right, clear of the front of the soil,
and holds it there. The left comes in from the side on the jug, fingers closing across it low down, lifts it, carries
it over the front of the pot and tips it out onto the soil the leaf covered. It sets the jug back on its stool, lets
go and goes back to where it started; the right lets the leaf go back round over the soil, rises and goes back to
where it started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=water_a_plant_holding_its_big_leaf_aside.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Water the plant at its roots, onto the soil, without wetting its leaves. Then put the jug back on its stool, leave the plant's leaves as they were and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plant-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/plant-dream/*.json; do inspect-robots video "$log"; done
```

Set `PLANT_DEBUG=1` to print what the score is built from twice a second: how far round the big leaf is, how many
beads lie on the soil and how many have touched a leaf, whether the jug has touched a leaf and how close it has come,
how many beads are still in the jug, where the jug is and how far it is tipped, whether the jug and the leaf are back,
whether both hands are clear, and where each TCP is.

## A real trial

The plant is an ordinary artificial house plant (an artificial peace lily, say) with moulded green plastic leaves,
which take up no water. Its leaves are 15 cm long and up to 13 cm wide, set as in the simulation: three in the crown
over the back and sides of the pot, and the big lower leaf on a sprung wire stalk lying over the front of the soil,
which springs back when let go. Each leaf is fixed to its wire stalk with a small clip, so it can be unclipped for the
weighings and photographs. The plant stands in real potting soil, just moist, in a 17 cm glazed terracotta pot on a
saucer, its drainage hole plugged with a rubber bung for the trial. A 150 ml watering jug holding 120 ml of plain tap
water at room temperature stands on a 6 cm stool beside it. Two cameras film the trial at 60 frames a second:
overhead, and from the side level with the leaves.

### The leaf-contact instrument: water-sensitive paper

Leaf contact is read with water-sensitive paper, the standard collector agriculture uses to check where a spray lands:
TeeJet part 20301-1N, 3 x 1 in (76 x 26 mm) cards, 50 to a pack, $54.44 a pack
([Westech Equipment](https://westechequipment.com/product/203011n-3-in.-x-1-in.-water-sensitive-paper)). Its maker's
data sheet (Spraying Systems Co. Data Sheet 20301,
[PDF](https://www.teejet.com/en/-/media/dam/agricultural/usa/technical-documentation/product-data-sheet/20301_tj.pdf))
documents what the read-out relies on:

- "Water-sensitive paper is a rigid paper with a specially coated, yellow surface which is stained dark blue by aqueous
  droplets."
- "The resulting stain is contrasty, permanent and ensures detection of droplets larger than 50 microns."
- It is used "under conditions of atmospheric humidity up to 80% relative humidity (RH)".
- For orchard sprayers: "Staple the cards to leaves in upper, center and lower parts of the tree."
- "Gloves should be used (polyethylene or rubber) for handling the papers to avoid staining and contaminating the
  water-sensitive papers. The collector support (natural or artificial) must be dry."
- Storage: "under dry conditions in airtight boxes or bags", and "Water and oil sensitive paper sold by Spraying
  Systems Co. is manufactured by Syngenta Crop Protection AG."

Syngenta's own page adds: "Once dry, the blue stains on WSP are irreversible"
([Syngenta Canada](https://www.syngenta.ca/agronomy/water-sensitive-paper)).

The liquid is therefore plain tap water, with nothing added, and the pot is weighed with the same water it receives.

### Fitting the cards

All four leaves are covered, in gloves, with the leaves unclipped and dry, in a room at no more than 80% RH (read on a
room hygrometer):

- **Upper face:** whole cards laid edge to edge across the blade, trimmed with scissors to the leaf's outline, each
  held by its water-repellent back on thin double-sided tape. A blade is about 141 cm² (computed from its outline),
  so about 10 trimmed cards cover it.
- **Edges and underside margin:** cards cut lengthwise into 13 x 76 mm strips, folded in half along their length over
  the leaf's edge (6.5 mm on the upper face, 6.5 mm on the underside), nicked every 2 cm so they follow the curve, and
  taped the same way, all the way round each leaf.
- **Where a hand holds the big leaf:** the hold zone, fixed in advance, is the big leaf's left edge from 6 to 14 cm out
  from the stem, up to 3 cm in from the edge, both faces; the demo pushes there, 10 cm out. A hand may hold a leaf
  anywhere. The hold zone and every other place a gripper touches are covered by cards like the rest, and nothing is
  excluded: stains there count like anywhere else. The robot's fingers wear dry nitrile finger covers (rubber, as the
  maker specifies for handling the cards), so a gripper's touch leaves no stain; a crease or scuff is not blue.
- Every card is numbered on its back, as the data sheet suggests; a card that comes loose during a trial is found by
  its number and read like the rest.

Cards are kept in their sealed foil bag with its silica gel until fitted, and fitted leaves are stored in a sealed
bag with silica gel between sessions.

### Procedure

1. Fit the cards (above). Photograph each leaf on both faces against a white card, with the camera on a fixed stand
   and two fixed lamps. These are the before photos.
2. Lift the pot out of its saucer, with its soil, stem, wire stalks, clips and bung but without its leaves, and weigh
   it, and the dry saucer, on a kitchen scale (1 g steps). Clip the leaves back on and lay the big leaf over the soil.
3. Run the trial.
4. Unclip each leaf in gloves, keeping it level, and lay it on a white card. Lift the pot out, let it drip into the
   saucer, and weigh the pot and the saucer.
5. Once the cards are dry, photograph each leaf on both faces exactly as in step 1. These are the after photos.

**Liquid delivered to the soil** is the pot's gain (1 g = 1 ml). The pot is weighed in the same state both times, its
four leaves unclipped. With the drain plugged, and 1.3 cm of headroom above the soil (about 235 ml), none of it leaves
the pot. The saucer's gain is spilled water and never counts as delivered.

**Leaf contact** has two independent reads, and either one is leaf contact:

1. **The cards.** Each after photo is compared, at full size, with the before photo of the same face. Any blue stain
   on any card, however small (a dot, a streak or a patch), that is not in the before photo means that leaf was
   wetted: on any of the four leaves, either face, edges and hold zone included.
2. **The side video.** The 60 fps side video is checked frame by frame for the jug, or the stream of water, touching
   any leaf.

The cards record contact wherever it happens, seen or not. A touch lasting an instant stains the card at that point; a
drop that lands and runs off leaves its track; a touch at a leaf's edge stains the folded edge strip; a touch behind
the jug or a gripper stains the card just the same; and the photos are taken afterwards, with nothing in the way. The
stain is permanent and dark blue on yellow, and droplets larger than 50 microns register (maker's data sheet).

Any leaf contact scores 0.0, since it breaks the instruction's one constraint ("without wetting its leaves"). The
simulation scores the same event: a bead within 4 mm of a leaf, or any part of the jug within 4 mm of a leaf.

| Score | Real trial | Simulation (10 ml per bead) |
|---|---|---|
| 0.0 | leaf contact: a new blue stain on a card, or the jug or stream touching a leaf in the side video | a bead or the jug ever within 4 mm of a leaf |
| 1.0 | no leaf contact; pot gain at least 90 g; and the end state (marks on the stool and the pot's rim, ruler and protractor; the robot's joint log for the arms) | no leaf contact; at least 9 of the 12 beads on the soil; and the end state |
| 0.5 | no leaf contact; pot gain at least 90 g | no leaf contact; at least 9 beads on the soil |
| 0.25 | no leaf contact; pot gain at least 40 g | no leaf contact; at least 4 beads on the soil |

Water in the saucer, on the table or left in the jug never counts. To reset, pull the bung and let the pot drain, then
plug it again; it is weighed afresh, leaves unclipped, before each trial. Empty and dry the saucer. Replace every
stained card and photograph the leaves afresh (step 1). Wipe the finger covers and the jug's outside dry, refill the
jug with 120 ml of tap water and lay the big leaf back over the soil.

Fitting all four leaves takes about 56 cards (computed: about 10 for each upper face and 4 cut into edge strips), so
two packs ($108.88). A clean trial stains none, and only stained cards are replaced.

**Rig protection.** The only liquid is plain tap water. The finger covers keep the grippers' pads dry and come off
after the session; the jug's outside is dried before each trial.

## Third-party assets

None. Every body is built from boxes, cylinders and spheres in the task file and `bodykit.py`.

## Runtime per trial

On one RTX 5090: about 9 minutes for a Dream-1 trial of 1,200 steps (563 s on the clock, with Dream answering in 1.04 s on average), and about 7 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 416 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-7FFS8K. Its review said:

> Scoring (not met): The proposed indicator film has not been tested to show that brief edge contact and runoff leave a
> detectable lasting mark.
>
> What to change: The matched pot and saucer weighings and the real-trial setup address the earlier points. The
> requested validation of leaf-contact detection remains open: please perform and report fast-pour tests using the
> proposed indicator-coated leaves and dilute vinegar, including brief contact at leaf edges, runoff, and contact
> obscured by the jug or gripper. Include areas where the gripper has touched the film, and show that the dried marks
> remain detectable in the before-and-after photos. If any contact is missed, adjust the method and repeat the tests.

Earlier reviews:

- TK-CHR9EE asked for validation of the video-and-photo contact method; the read-out now rests on a commercial
  instrument (below).
- TK-T4P2JA asked for tests around a 10 ml leaf allowance; the allowance was removed (any leaf contact scores 0.0), and
  that is kept.
- TK-EYKT6P asked for the pot to be weighed in the same leaf state before and after; the matched, leafless weighings are
  kept.
- TK-A2TMY6 asked to tell water on the soil from water in the saucer; the separately weighed pot and saucer and the
  plugged drain are kept.
- TK-J457LS asked for a reproducible real plant, a real water volume with a reset procedure, and real-world thresholds
  for every tier; all three are kept.

What changed in response:

- **Scoring (not met), the indicator film:** the home-made film and the dilute vinegar are gone. The leaf-contact
  read-out now uses a standardized commercial instrument, water-sensitive paper (TeeJet 20301-1N, 26 x 76 mm cards,
  made by Syngenta Crop Protection AG), whose staining is documented by its maker: "stained dark blue by aqueous
  droplets", the stain "contrasty, permanent", detecting "droplets larger than 50 microns". No home-made indicator
  needs validation. The jug holds plain tap water. Where: setup, scoring, reset_notes, fidelity, "A real trial".
- **Brief contact at leaf edges:** every leaf's edge is wrapped all round in 13 mm card strips folded over it (6.5 mm
  on each face), so an edge touch stains a card. Where: setup, scoring, "Fitting the cards".
- **Runoff:** the upper faces are covered edge to edge with cards and the underside margin by the folded strips; a drop
  that runs off stains its track. Where: scoring, "A real trial".
- **Contact obscured by the jug or a gripper:** the cards record contact wherever it happens, seen or not, and are
  photographed afterwards with nothing in the way. A second, independent read is kept: the 60 fps side video, checked
  frame by frame for the jug or the stream touching a leaf. Either read scores 0.0. Where: scoring.
- **Areas the gripper touches:** the hold zone (the big leaf's left edge, 6 to 14 cm from the stem, up to 3 cm in, both
  faces) is fixed in advance and carded like the rest; stains there count. The robot's fingers wear dry nitrile covers,
  rubber being what the maker specifies for handling the cards, so a gripper's touch leaves no stain. Where: setup,
  scoring.
- **Dried marks detectable in the before-and-after photos:** the maker documents the stain as permanent ("Once dry,
  the blue stains on WSP are irreversible", Syngenta). Each leaf is photographed on both faces before and after, camera
  on a fixed stand under two fixed lamps; any new blue stain is leaf contact. Where: scoring, "Procedure".
- **The simulation scores the same two events:** the leaf-wet rule already flagged any bead ever within 4 mm of any
  leaf, checked 240 times a second, and is kept. It now also flags any part of the jug as drawn (body, rim, floor,
  lip, handle) ever within 4 mm of a leaf, matching the side-video read. Either scores 0.0. Where: task file
  (`_jug_gap`, `_points`), scoring, fidelity, "How it is modelled".
- **Scene and demo, corrected by the new jug check:** at rest the jug stood 0.8 mm from the tip of the left crown leaf
  (simulation), and the old pour path pressed the jug against the pot's rim. The left crown leaf now points 60 degrees
  round from straight back (it was 78), 1.9 cm clear of the jug; the demo takes the jug 8 mm higher on its body and
  pours from a lip 2 cm higher (7.5 cm above the soil). Where: task file, setup.
- **Adversarial checks (simulation, scripted variants of the demo):**
  - the stream grazing the held leaf's edge (the jug's lip moved 3.5 cm toward it): 4 beads touch the leaf's edge,
    8 reach the soil, and the jug stays 8.7 mm clear; score 0.0 (the leaf-dry credit is lost; without that rule the 8
    beads would earn 0.25);
  - a clean pour, then the jug set back on its stool 2.1 cm from where it started, against the left crown leaf
    (inside the 3 cm limit for "back on its stool"): 10 beads on the soil, none on a leaf, every end-state check met;
    score 0.0;
  - one hand pouring, the big leaf left in place: the jug touches the big leaf and all 12 beads land on it; score 0.0.
- **Kept, as the review accepted them:** the matched pot and saucer weighings and the real-trial setup, with the same
  thresholds (no leaf contact; pot gain at least 90 g and the end state for 1.0, at least 90 g for 0.5, at least 40 g
  for 0.25).
- **Runs:** the scripted demo scores 1.0 (607 steps; in a debug run of the same demo 10 of the 12 beads end on the
  soil, none comes within 4 mm of a leaf, and the jug stays at least 17.8 mm from every leaf), and Dream-1 scores 0.0
  (1,200 steps).
