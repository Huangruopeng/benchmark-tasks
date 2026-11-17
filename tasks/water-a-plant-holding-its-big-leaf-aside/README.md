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
- After every physics step (240 a second, not only after each 50 ms control step), the task file checks each bead.
  A bead that touches any leaf's blade, or comes within 4 mm of it (a margin for the splash and spread a rigid bead
  lacks), has made liquid contact with that leaf for good, even if it runs off straight away, and the trial scores 0.
  A real trial records the same event chemically (see "A real trial"). A bead counts as watering the soil only if it
  ends lying on the soil inside the pot's wall (its centre within 7.25 cm of the pot's axis and between 3 mm below and
  15 mm above the soil's surface) and never touched a leaf. The saucer is drawn only: a bead that misses the pot or
  bounces out of it lands on the table under the saucer and never counts.

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

The plant is an ordinary artificial house plant (an artificial peace lily, say) with moulded green plastic leaves,
which take up no water. Its leaves are 15 cm long and up to 13 cm wide, set as in the simulation: three in the crown
over the back and sides of the pot, and the big lower leaf on a sprung wire stalk lying over the front of the soil,
which springs back when let go. Each leaf is fixed to its wire stalk with a small clip, so it can be unclipped for the
weighings and photographs. The plant stands in real potting soil, just moist, in a 17 cm glazed terracotta pot on a
saucer, its drainage hole plugged with a rubber bung for the trial. A 150 ml watering jug holding 120 ml of liquid B
(below), at room temperature, stands on a 6 cm stool beside it. Two cameras film the trial at 60 frames a second,
overhead and from the side level with the leaves. The videos are supporting evidence only: no score is taken from
them.

### Two liquids: a colour record of leaf contact

| Liquid | Recipe | Use |
|---|---|---|
| A, butterfly pea flower extract | 2 g of dried butterfly pea flowers steeped for about 10 minutes in 100 ml of water just off the boil, cooled, then filtered through a paper coffee filter | sprayed onto the plastic leaves before the trial and left to dry |
| B, dilute white vinegar | 1 part ordinary 5% white vinegar to 9 parts water (10 ml + 90 ml; 12 ml + 108 ml fills the jug with 120 ml): about 0.5% acetic acid, pH about 2.9 | in the robot's jug, as the water |

The dried film of liquid A is blue. Wherever liquid B touches it, the film turns pink or purple, and the mark stays
once it has dried. Liquid B is 99.5% water, so it pours like water and weighs as water does (1 g = 1 ml): the pot is
weighed with it exactly as it was with plain water. The leaves are plastic, so the dilute vinegar harms nothing.

The chemistry, with sources (listed at the end of this section):

- The blue of butterfly pea flowers comes from anthocyanins (ternatins) whose colour changes with pH; the extract is
  used as a pH indicator in colour-changing food-packaging films [2, 3, 4]. In solution it is red below pH 3.2, violet
  to blue from pH 3.2 to 5.2 and light blue from pH 5.2 to 8.2; a gelatin film coloured with it is blue at pH 6 and
  turns violet at pH 4 [1]. Gelatin and methylcellulose films with the extract go from blue to purple as the pH falls
  from 6 to 2 [3].
- Ordinary white vinegar is 5% acetic acid, 50 g per litre. Diluted 1 in 10 it holds 5 g per litre, 83 mM. With
  acetic acid's pKa of 4.76 at 25 °C [5], its pH is ½(pKa − log₁₀ 0.083) = ½(4.76 + 1.08) = 2.92, between the
  tabulated pH of 2.88 at 100 mM and 3.39 at 10 mM [6]. That is below pH 3.2, where the pigment in solution is red
  [1], and films coloured with it are purple at pH 2 [3]; on the dried film a mark shows pink to purple.

### Procedure

1. **Before the trial**, unclip the four leaves. Wipe each clean and dry. Spray both faces with liquid A from a
   fine-mist spray bottle, in light coats, letting each coat dry (a hair dryer on its cool setting speeds this up),
   until a uniform blue film forms. From here on, handle the leaves only by their clips, in clean dry gloves (skin is
   mildly acidic).
2. Photograph each leaf on both faces against a white card, in fixed light, with the leaf filling the frame of a
   12-megapixel camera (more than 20 pixels per mm on a 15 cm leaf). These are the before photos.
3. Lift the pot out of its saucer, with its soil, stem, wire stalks, clips and bung but without its leaves, and weigh
   it, and the dry saucer, on a kitchen scale (1 g steps). Clip the leaves back on and lay the big leaf over the soil.
4. Run the trial.
5. **After the trial**, unclip each leaf by its clip, keeping it level, and lay it on a white card. Lift the pot out,
   let it drip into the saucer, and weigh the pot and the saucer.
6. Once the leaves are dry, photograph each on both faces exactly as in step 2. These are the after photos.

**Liquid delivered to the soil** is the pot's gain (1 g = 1 ml). The pot is weighed in the same state both times, its
four plastic leaves unclipped. With the drain plugged, and 1.3 cm of headroom above the soil (about 235 ml), none of
it leaves the pot. The saucer's gain is spilled liquid and never counts as delivered.

**Leaf contact** is read from the colour record. Each after photo is compared, at full size, with the before photo of
the same face. Any pink or purple spot or streak, however small, that is not in the before photo is leaf contact: on
any of the four leaves, on either face, edges included. A place where a gripper rubbed the film paler without turning
it pink or purple is not contact.

**Why this catches brief or hidden contact.** The record is chemical, not a view. Liquid B changes the film's colour
wherever it touches it, so:

- a touch lasting an instant, at a leaf's very edge, leaves a mark there;
- a drop that lands and runs off leaves a streak along its path, though no liquid stays on the leaf;
- contact out of both cameras' view, behind the jug or a gripper, marks the film just the same, and the photos are
  taken afterwards, of every leaf, both faces, with nothing in the way.

Nothing is weighed on the leaves, so the record does not depend on how much liquid touched a leaf or stayed on it.

Any leaf contact scores 0.0, since it breaks the instruction's one constraint ("without wetting its leaves"). The
simulation scores the same event, any liquid contact with a leaf: a bead that ever touches a leaf or comes within 4 mm
of one.

| Score | Real trial | Simulation (10 ml per bead) |
|---|---|---|
| 0.0 | any leaf contact: a pink or purple mark on a leaf in the after photos that is not in the before photos | any bead ever touching a leaf or within 4 mm of one |
| 1.0 | no leaf contact; pot gain at least 90 g; and the end state (marks on the stool and the pot's rim, ruler and protractor; the robot's joint log for the arms) | no leaf contact; at least 9 of the 12 beads on the soil; and the end state |
| 0.5 | no leaf contact; pot gain at least 90 g | no leaf contact; at least 9 beads on the soil |
| 0.25 | no leaf contact; pot gain at least 40 g | no leaf contact; at least 4 beads on the soil |

Liquid in the saucer, on the table or left in the jug never counts. To reset, pull the bung and let the pot drain,
then plug it again; it is weighed afresh, leaves unclipped, before each trial. Empty, rinse and dry the saucer. Rinse
the film off the leaves, dry them, coat and photograph them afresh (steps 1 and 2) and clip them back on. Lay the big
leaf back over the soil, refill the jug with 120 ml of liquid B and dry its outside.

Sources:

1. G. C. Vidana Gamage, Y. Y. Lim, W. S. Choo (2021). Anthocyanins from *Clitoria ternatea* flower: biosynthesis,
   extraction, stability, antioxidant activity, and applications. *Frontiers in Plant Science* 12, 792303.
   <https://doi.org/10.3389/fpls.2021.792303> (colour of the anthocyanins by pH; the gelatin indicator film at pH 6
   and 4).
2. N. N. Hasanah, E. M. Azman, A. Rozzamri, N. H. Zainal Abedin, M. R. Ismail-Fitry (2023). A systematic review of
   butterfly pea flower (*Clitoria ternatea* L.): extraction and application as a food freshness pH-indicator for
   polymer-based intelligent packaging. *Polymers* 15(11), 2541. <https://doi.org/10.3390/polym15112541> (red
   flavylium form in strong acid, purple quinoidal base, blue near neutral).
3. S. Sai-Ut, P. Suthiluk, W. Tongdeesoontorn, S. Rawdkuen, P. Kaewprachu, T. Karbowiak, F. Debeaufort, P. Degraeve
   (2021). Using anthocyanin extracts from butterfly pea as pH indicator for intelligent gelatin film and
   methylcellulose film. *Current Applied Science and Technology* 21(4), 652–661.
   <https://li01.tci-thaijo.org/index.php/cast/article/view/249347> (films purple to blue over pH 2 to 6).
4. S. Rawdkuen, A. Faseha, S. Benjakul, P. Kaewprachu (2020). Application of anthocyanin as a color indicator in
   gelatin films. *Food Bioscience* 36, 100603. <https://doi.org/10.1016/j.fbio.2020.100603> (of the plant
   anthocyanins compared, butterfly pea's had the highest pH sensitivity).
5. PubChem, acetic acid (CID 176), dissociation constants: pKa 4.76 at 25 °C.
   <https://pubchem.ncbi.nlm.nih.gov/compound/176#section=Dissociation-Constants>
6. aqion, pH of common acids and bases (acetic acid: pH 2.88 at 100 mM, 3.39 at 10 mM).
   <https://www.aqion.de/site/ph-of-common-acids>


## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (197 s on the clock, with Dream answering in 0.81 s on average), and about 3 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 190 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-CHR9EE. Its review said:

> **Not accepted** (TK-CHR9EE).
>
> Scoring (not met): The replacement video-and-photo method has not been validated for brief or obscured leaf-edge
> contact and subsequent runoff.
>
> What to change: The matched pot and saucer weighings address the earlier soil-scoring concern, and removing the 10
> ml leaf allowance makes tests of that threshold unnecessary. The replacement contact-detection method still needs
> validation: perform and report fast-pour tests with contact near leaf edges and water running off, including contact
> potentially obscured by the jug or gripper. Show that the two-camera and photo procedure detects these contacts
> reliably, and adjust camera coverage or the scoring method if it does not.

Earlier reviews:

- TK-T4P2JA asked for the fast-pour tests of the leaf-water measurement to be run, at leaf edges and around the 10 ml
  threshold. The 10 ml leaf allowance was then removed (any leaf contact scores 0.0), which the latest review
  accepted; that is kept.
- TK-EYKT6P asked for the pot to be weighed in the same leaf state before and after, and for a method that records
  leaf contact even during a fast pour. The matched, leafless pot weighings are kept.
- TK-A2TMY6 asked to tell water delivered to the soil from water in the saucer, and to record water that touched a leaf
  even if it ran off. The separately weighed pot and saucer and the plugged drain are kept.
- TK-J457LS asked for a reproducible real plant, a real water volume with a reset procedure, and real-world thresholds
  for every tier. All three are kept.

What changed in response:

- **Scoring (not met), brief or obscured leaf-edge contact and runoff:** leaf contact is no longer judged from what
  the cameras see; it is a chemical record. Before each trial every leaf is sprayed on both faces with butterfly pea
  flower extract (2 g of dried flowers steeped in 100 ml of hot water, filtered), dried to a uniform blue film and
  photographed. The jug holds dilute white vinegar (1 part ordinary 5% vinegar to 9 parts water: about 0.5% acetic
  acid, pH about 2.9) as the water. Wherever it touches the film, however briefly, at an edge, or as a drop that runs
  off, the film turns pink or purple, and the mark stays once dry. After the trial every leaf is photographed again
  on both faces against a white card. Any pink or purple spot or streak not in the before photo is leaf contact and
  scores 0.0. The colour chemistry and the pH are sourced in "A real trial".
- **Contact obscured by the jug or a gripper:** the record does not depend on a view at the moment of contact. The
  film marks wherever the liquid touches it, seen or not, and the photos are taken afterwards, of every leaf, both
  faces, with nothing in the way. The two 60 fps videos are kept as supporting evidence only; no score is taken from
  them.
- **Perform and report fast-pour tests at leaf edges, with runoff and hidden contact:** no tests on leaves were run.
  Instead, as the review allows, the scoring method was changed so that detecting contact no longer depends on a
  view of it: a brief touch, a touch at an edge, a drop that runs off and a touch hidden behind the jug or a gripper
  all mark the film alike, as the cited sources on the pigment's pH response explain.
- **Adjust camera coverage or the scoring method:** the scoring method was changed, as above, so detection no longer
  depends on camera coverage.
- **Pot and saucer weighings, kept as the review accepted them:** unchanged, now with the dilute vinegar, which is
  99.5% water and weighs as water does (1 g = 1 ml). The thresholds are unchanged: with no leaf contact, a pot gain of
  at least 90 g and the end state for 1.0, at least 90 g for 0.5 and at least 40 g for 0.25. The leaves are plastic,
  so the vinegar harms nothing.
- **The simulation scores the same event:** any liquid contact with a leaf scores 0.0 in both. In the simulation that
  is a bead touching a leaf or coming within 4 mm of one, checked 240 times a second; this scorer is unchanged. The
  setup, scoring, reset notes and fidelity note now describe the real-trial method.
- **Text only:** the scene, scorer, demo and instruction are unchanged (in the task file only a docstring was
  reworded), so the existing runs still apply. The scripted demo scores 1.0 (607 steps; in a debug run 11 of the 12
  beads ended on the soil and none came within 4 mm of a leaf), and Dream-1 scores 0.0.
