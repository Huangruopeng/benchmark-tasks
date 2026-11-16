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
the demo, the right never moving) the demo scored 0: all 12 beads landed on the big leaf, which lay over the
soil, and ran off it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The pot, its soil, the stem and the crown's three leaves are one fixed body. Each leaf is a thin rigid blade (seven
  boxes giving its oval outline) 15 cm long.
- The big front leaf is a loose rigid blade joined to the stem by a hinge about the vertical (`Hinge` from
  `bodykit.py`), its spring (0.0016 N m per degree) holding it over the front of the soil; pushed round 85 degrees it
  pushes back with 0.14 N m.
- The water is 12 rigid beads in the jug, which is a cup built from a floor and 16 wall panels.
- After every physics step (240 a second, not only after each 50 ms control step), the task file checks each bead:
  within 4 mm of any leaf's blade, it has touched that leaf for good, even if it runs off straight away, and the trial
  scores 0. A bead counts as watering the soil only if it ends lying on the soil inside the pot's wall (its centre
  within 7.25 cm of the pot's axis and between 3 mm below and 15 mm above the soil's surface) and never touched a
  leaf. The saucer is drawn only: a bead that misses the pot or bounces out of it lands on the table under the saucer
  and never counts.

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
beads lie on the soil and how many have touched a leaf, how many are still in the jug, where the jug is and how far
it is tipped, whether the jug and the leaf are back, whether both hands are clear, and where each TCP is.

## A real trial

The plant is an ordinary artificial house plant (an artificial peace lily, say) with moulded green plastic leaves.
They take up no water, so nothing about them needs measuring or calibrating. Its leaves are 15 cm long and up to 13 cm
wide, set as in the simulation: three in the crown over the back and sides of the pot, and the big lower leaf on a
sprung wire stalk lying over the front of the soil, which springs back when let go. Each leaf is fixed to its wire
stalk with a small clip, so it can be unclipped for the weighings. The plant stands in real potting soil, just moist,
in a 17 cm glazed terracotta pot on a saucer, its drainage hole plugged with a rubber bung for the trial. A 150 ml
watering jug holding 120 ml of room-temperature water, tinted dark blue with food colouring so that water on a green
leaf shows clearly, stands on a 6 cm stool beside it. Two cameras film the trial at 60 frames a second, synchronized:
one overhead and one from the side, level with the leaves.

**Water delivered to the soil** is weighed (1 g of water = 1 ml). The pot is weighed in the same state before and
after the trial: its four plastic leaves unclipped, its soil, stem, wire stalks, clips and bung left in.

1. Before the trial, unclip the leaves, lift the pot out of its saucer and weigh it, and the dry saucer, on a kitchen
   scale (1 g steps). Clip the leaves back on.
2. After the trial, photograph the leaves (below), unclip them, lift the pot out, let it drip into the saucer, and
   weigh the pot and the saucer.

The pot's gain is the water delivered to the soil. With the drain plugged, and 1.3 cm of headroom above the soil
(about 235 ml), none of it leaves the pot. The saucer's gain is spilled water and never counts as delivered.

**Leaf contact** is observed, not measured.

- The two synchronized videos are reviewed frame by frame. Any frame in which the stream or a droplet of the blue
  water touches a leaf, anywhere on it, edges included, counts as leaf contact.
- As a backup, every leaf is photographed close up after the trial, before it is unclipped. A blue droplet or wet mark
  on a leaf also counts as contact.

Nothing is weighed on the leaves, so the result does not depend on how much water a leaf holds, how fast it was
poured or whether it ran off: the first touch already counts. Any leaf contact scores 0.0, since it breaks the
instruction's one constraint ("without wetting its leaves").

| Score | Real trial | Simulation (10 ml per bead) |
|---|---|---|
| 0.0 | any leaf contact, in any video frame or on the photographs | any bead ever within 4 mm of a leaf |
| 1.0 | no leaf contact; pot gain at least 90 g; and the end state (marks on the stool and the pot's rim, ruler and protractor; the robot's joint log for the arms) | no bead ever on a leaf; at least 9 of the 12 beads on the soil; and the end state |
| 0.5 | no leaf contact; pot gain at least 90 g | no bead ever on a leaf; at least 9 beads on the soil |
| 0.25 | no leaf contact; pot gain at least 40 g | no bead ever on a leaf; at least 4 beads on the soil |

Water in the saucer, on the table or left in the jug never counts. To reset, pull the bung and let the pot drain,
then plug it again; it is weighed afresh, leaves unclipped, before each trial. Empty and dry the saucer, wipe the
plastic leaves dry and clip them back on, lay the big leaf back over the soil, and refill the jug with 120 ml of the
tinted water.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (197 s on the clock, with Dream answering in 0.81 s on average), and about 3 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 190 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-T4P2JA. Its review said:

> **Not accepted** (TK-T4P2JA).
>
> Scoring (not met): The proposed fast-pour and staining checks have not been performed, leaving the previously
> requested validation of leaf-water measurement open.
>
> What to change: The matched pot weighings and separate saucer measurement address the earlier soil-scoring concerns.
> The requested validation of leaf-contact measurement is still open: the proposed tests have not been run. Perform
> and report the fast-pour tests, including contact near leaf edges and volumes around the 10 ml threshold; show that
> the method detects runoff and applies the thresholds reliably, and adjust the leaf design or scoring procedure if it
> does not.

Earlier reviews:

- TK-EYKT6P asked for the pot to be weighed in the same leaf state before and after, and for a method that records
  leaf contact even during a fast pour or past a leaf's capacity. The matched, leafless pot weighings are kept. The
  capacity-based method is replaced, as below.
- TK-A2TMY6 asked to tell water delivered to the soil from water in the saucer, and to record water that touched a leaf
  even if it ran off. The separately weighed pot and saucer and the plugged drain are kept.
- TK-J457LS asked for a reproducible real plant, a real water volume with a reset procedure, and real-world thresholds
  for every tier. All three are kept.

What changed in response:

- **Scoring (not met), the unvalidated leaf-water measurement:** no physical test was run. Instead, as the review
  allows, the leaf design and the scoring procedure were changed so that no leaf measurement is left to validate.
  - The real-trial plant now has ordinary non-absorbent plastic leaves, green as in the simulation. The absorbent
    sponge-cloth leaves, the stains, the leaf weighings and the validation procedure are all gone.
  - Leaf contact is now observed directly. Two synchronized 60 fps videos (overhead, and from the side level with the
    leaves) are reviewed frame by frame. Any frame in which the stream or a droplet of the dark-blue water touches a
    leaf counts as contact. A close-up photograph of every leaf after the trial is the backup: any blue droplet or wet
    mark also counts.
- **Contact near leaf edges, and runoff:** a touch anywhere on a leaf, edges included, counts in the frame where it
  happens. Water that later runs off has already been counted, so no capacity or runoff assumption remains.
- **Volumes around the 10 ml threshold:** the 10 ml leaf-water allowance is gone, so there is no volume threshold to
  check on the leaves. Any leaf contact, however small, scores 0.0, because it breaks the instruction's constraint
  ("without wetting its leaves").
- **Thresholds, now applied reliably:** the score is binary on leaf contact. With no leaf contact: 1.0 needs a pot gain
  of at least 90 g and the end state; 0.5 needs at least 90 g without the end state; 0.25 needs at least 40 g. Any
  leaf contact scores 0.0.
- **The simulation matches:** a trial now scores 0.0 if any bead ever comes within 4 mm of a leaf (it allowed 1 bead
  before). The tiers are otherwise as before: 9 or 4 of the 12 beads on the soil, at 10 ml per bead. The demo and
  Dream-1 runs were redone under this scorer. The scripted demo scores 1.0 (607 steps; in a debug run 11 of the 12
  beads ended on the soil and none came within 4 mm of a leaf). Dream-1 scores 0.0: it reaches toward the jug but
  never lifts it. The one-hand check was not re-run: the scorer only got stricter, so its earlier 0 (all 12 beads
  landed on the big leaf) cannot rise.
- **Kept, as the review accepted them:** the matched leafless pot weighings, the separately weighed saucer, the
  plugged drain, the real potting soil and the 120 ml of water.
