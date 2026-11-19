# Water a house plant at its roots without wetting its leaves

A house plant in a terracotta pot spreads its broad leaves low over the soil, each on a springy stalk: three in the
crown over the back and sides, and one big leaf in front, lying over the front of the soil. A blue watering jug with
water in it stands on a stool beside the pot. One hand pushes the big leaf round on its stalk, aside, and holds it
there; the other lifts the jug, carries it over the front of the pot and pours the water onto the soil the leaf
covered, without wetting any leaf. The jug goes back on its stool, the leaf is let go back over the soil, and both arms
go back to where they started. The task is one task file,
[`water_a_plant_holding_its_big_leaf_aside.py`](water_a_plant_holding_its_big_leaf_aside.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests: holding a sprung part out of the way while the other hand works where it was; pouring a light stream
into a target without spilling it on what is around it; leaving the plant as it was.

Why two hands: the big leaf springs back over the soil as soon as nothing holds it, and the soil it covers is the only
soil not under a leaf, so water poured with the leaf in place lands on the leaf. With one hand (the left pouring as in
the demo, the right never moving) the demo scores 0: the jug presses on the big leaf, which lies over the soil, its wet
lip touches it, and all 12 beads land on it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres; the bodies here are made of several shapes each, from
`bodykit.py`.

- The pot, its soil and the stem are one fixed body. Each leaf is a thin rigid blade (seven boxes giving its oval
  outline) 15 cm long, joined to the stem by a hinge about the vertical (`Hinge` from `bodykit.py`) with a spring that
  holds it where it lies and springs it back when let go.
- The crown's three leaves point straight back, back-left (60 degrees round from straight back) and right (78 degrees
  round the other way); their stalks turn up to 30 degrees either way against 0.004 N m per degree.
- The big front leaf lies over the front of the soil, its spring 0.0016 N m per degree; pushed round 85 degrees it
  pushes back with 0.14 N m, so a hand holding it 10 cm out from the stem presses on it with about 1.4 N.
- The water is 12 rigid beads in the jug, which is a cup built from a floor and 16 wall panels; its pouring lip and
  handle are drawn only.

The score rests on two rules that each score 0, then on the water delivered and the end state.

- **Water on a leaf.** After every physics step (240 a second, not only after each 50 ms control step), the task file
  checks each bead against every leaf. A bead that touches a leaf's blade, or comes within 4 mm of it (a margin for the
  splash and spread a rigid bead lacks), has wetted that leaf for good, even if it runs off straight away. Once any
  bead has left the jug, the jug's lip and the top 1 cm of its rim are wet: those parts within 4 mm of a leaf's blade
  wet it too. The 4 mm margin is the simulation's counterpart of the real trial's hidden-frame rule (a hidden gap under
  2 cm counts as contact).
- **A leaf disturbed.** At the end, any leaf's tip more than 3 cm from where it lay (for the big leaf, a turn of about
  11.5 degrees); or, at any time, a contact force above 5 N on any leaf from the jug or from any link of either arm,
  averaged over each 50 ms control step (so a single-step solver spike does not count). The force is read after every
  physics step from a PhysX rigid-contact view, with each finger counted separately, so a pinch is not cancelled out. A
  rigid leaf cannot crease or tear, so 5 N (a design value) stands for the press that would damage one; holding the big
  leaf aside takes about 1.4 N, and the demo's hand peaks at 1.7 N.
- **A dry touch is allowed.** The jug or a hand touching a leaf, with no water on it, leaving it undamaged and in place,
  does not lower the score.
- A bead counts as watering the soil only if it ends lying on the soil inside the pot (its centre within 7.6 cm of
  the pot's axis, the inside of its wall, so a bead resting against the wall counts; and between 3 mm below and 15 mm
  above the soil's surface, so a bead on the rim, 16.5 mm up, does not) and never touched a leaf. The saucer is drawn
  only: a bead that misses the pot or bounces out of it lands on the table under the saucer, at least 8.85 cm from the
  axis, and never counts.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the jug, every leaf and every bead still) has held
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

Set `PLANT_DEBUG=1` to print what the score is built from twice a second: how far round each leaf is and how far its
tip is from where it lay, the largest contact force each leaf has taken and from what, how many beads lie on the soil
and how many have touched a leaf, whether water has left the jug and whether its wet lip has touched a leaf, how close
the jug has come to a leaf, where the beads off the soil are, where the jug is and how far it is tipped, whether the
jug is back, whether both hands are clear, and where each TCP is.

## A real trial

The plant is an ordinary artificial house plant (an artificial peace lily, say) with moulded green plastic leaves,
which take up no water, so any water that touches a leaf stays on it as drops or a wet film. Its leaves are 15 cm long
and up to 13 cm wide, each on a bendable wire stalk, set as in the simulation: three in the crown over the back and
sides of the pot, and the big lower leaf lying over the front of the soil, which springs back when let go. Each leaf
is fixed to its stalk with a small clip, so it can be unclipped for the weighings and photographs. A strip of tape on
the table under each leaf's tip marks where it lies. The plant stands in real potting soil, just moist, in a 17 cm
glazed terracotta pot on a saucer, its drainage hole plugged with a rubber bung for the trial. A 150 ml watering jug
holding 120 ml of plain tap water at room temperature stands on a 6 cm stool beside it. The robot's fingers wear dry
nitrile finger covers.

### Cameras and lamp

- **Two fixed cameras**, recording 1080p at 60 frames a second or more (two smartphones on tripods do), started
  together and synchronized on a hand clap that both see and hear:
  - **top:** 1 m above the pot, looking straight down;
  - **side:** level with the leaves, 60 cm to the left of the pot, looking across it.
  Between them they see the pour path, the jug and every leaf, from two directions at right angles.
- **A grazing lamp:** a desk lamp clamped to the table, its light falling across the leaves at about 15 degrees, and a
  phone on a small stand for close-up photographs. Light at a grazing angle makes any drop or wet film on a plastic
  leaf glint against the matt dry surface.

### Procedure

1. Unclip the leaves. Lift the pot out of its saucer, with its soil, stem, wire stalks, clips and bung but without its
   leaves, and weigh it, and the dry saucer, on a kitchen scale (1 g steps). Clip the leaves back on and lay each with
   its tip on its tape mark.
2. Start both cameras and clap.
3. Run the trial.
4. Before anything is touched, photograph each leaf's upper face close up under the grazing lamp. Then unclip each
   leaf in dry gloves, turn it over and photograph its underside the same way. Look each leaf over for a crease, kink
   or tear.
5. Lift the pot out, let it drip into the saucer, and weigh the pot and the saucer.

### Reading the result

**Water on a leaf** is read by direct observation. Any one of these is leaf contact:

1. **Video, frame by frame.** In either view, any frame where the stream, a drop or a wet part of the jug touches a
   leaf, or a drop is seen on a leaf. The jug's wet parts are its lip and rim from the moment water first reaches
   them, and any part with water on it.
2. **Hidden frames.** Any frame in which the stream or a wet part of the jug is within 2 cm of a leaf (judged against
   the leaf's 15 cm length) and the gap between them is hidden from both views (behind the jug, a gripper or another
   leaf) counts as contact. So every frame is either seen or counted: a contact that one camera cannot see is seen by
   the other, or the frame counts as contact by this rule.
3. **Photos.** Any visible water on a leaf in the after-trial photographs: a drop, a streak or a wet sheen, on either
   face or an edge. A brief touch or a drop that runs off leaves water behind on the plastic leaf, and the photographs
   are taken with nothing in the way.

The jug or a hand touching a leaf while dry is not leaf contact, and is allowed as long as the leaf is left as it was.

**A leaf disturbed** is any leaf's tip more than 3 cm from its tape mark in the top camera's last frame (compared with
its first), or any leaf creased, kinked or torn, or its stalk bent out of shape.

**Water delivered to the soil** is the pot's gain (1 g = 1 ml). The pot is weighed in the same state both times, its
four leaves unclipped. With the drain plugged, and 1.3 cm of headroom above the soil (about 235 ml), none of it leaves
the pot. The saucer's gain is spilled water and never counts as delivered.

Water on a leaf scores 0.0, since it breaks the instruction's one constraint ("without wetting its leaves"), and so
does a leaf disturbed, since the instruction says to leave the leaves as they were.

| Score | Real trial | Simulation (10 ml per bead) |
|---|---|---|
| 0.0 | water on a leaf (video, hidden frame or photo); or a leaf tip over 3 cm from its tape mark, or a leaf creased, kinked or torn | a bead within 4 mm of a leaf, or the wet lip and rim within 4 mm of one; or a leaf tip over 3 cm from where it lay at the end, or over 5 N on a leaf |
| 1.0 | neither; pot gain at least 90 g; the jug back on its stool (marks on the stool), both hands clear and the arms back (the robot's joint log) | neither; at least 9 of the 12 beads on the soil; and the end state |
| 0.5 | neither; pot gain at least 90 g | neither; at least 9 beads on the soil |
| 0.25 | neither; pot gain at least 40 g | neither; at least 4 beads on the soil |

Water in the saucer, on the table or left in the jug never counts. To reset, pull the bung and let the pot drain, then
plug it again; it is weighed afresh, leaves unclipped, before each trial. Empty and dry the saucer. Dry any wet leaf
with a lint-free cloth, and swap a creased or torn leaf for a spare of the same plant. Clip the leaves back on with
each tip on its tape mark, wipe the finger covers and the jug's outside dry, refill the jug with 120 ml of tap water,
and restart both cameras with a clap.

**Rig protection.** The only liquid is plain tap water. The finger covers keep the grippers' pads dry and come off
after the session; the jug's outside is dried before each trial.

## Third-party assets

None. Every body is built from boxes, cylinders and spheres in the task file and `bodykit.py`.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 1,200 steps (494 s on the clock, with Dream answering in 0.81 s on average), and about 5 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 305 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-SYSZJA. Its review said:

> Scoring (not met): The assembled contact detector lacks the requested real-world validation, and dry jug-to-leaf
> contact newly triggers zero even when the leaves remain dry and undamaged.
>
> What to change: The requested validation is still open. Using the proposed card-covered leaves and plain water, run
> and report fast-pour tests covering brief edge contact, runoff, contact obscured by the jug or gripper, and
> previously gripped areas; include before-and-after photos of the dried cards and adjust and retest the method if it
> misses contact. A new point not raised before: remove the zero-score penalty for dry jug contact with a leaf when the
> leaf is left undamaged and in its original position; it does not violate the instruction to keep the leaves dry.

What changed in response:

- **The contact detector is removed; leaf wetting is read by direct observation.** The water-sensitive cards, their
  fitting and the 80% RH limit are gone from the task. Every validation request so far (a home-made film, then the
  cards) was about a detector, so the task now uses none. The real trial reads water on a leaf from what is seen, and
  the rule itself makes the read complete:
  - **Video:** two fixed, synchronized cameras at 60 fps or more, top and side at right angles, read frame by frame.
    Any frame where the stream, a drop or a wet part of the jug touches a leaf, or a drop is seen on a leaf, is contact.
  - **Hidden frames:** any frame in which the stream or a wet part of the jug is within 2 cm of a leaf, with the gap
    hidden from both views, counts as contact. Every frame is either seen or counted, so contact behind the jug or a
    gripper cannot go unscored.
  - **Photos:** right after the trial, close-ups of both faces of every leaf under a grazing lamp, which makes drops
    and wet sheen glint. Any visible water counts. The leaves are plastic and take up no water, so a brief edge touch
    or a drop that runs off leaves water behind.
  - Where: setup, scoring, reset_notes, fidelity, "A real trial". Check: in simulation, the stream grazing the held
    leaf's edge scores 0.0 (A1 below).
- **Dry jug contact no longer scores 0 (the new point).** The simulation's rule that zeroed the score for any part of
  the jug within 4 mm of a leaf is removed. A dry touch by the jug or a hand that leaves the leaf undamaged and in place
  does not lower the score, in simulation and in a real trial. Where: task file (`_points`), scoring, "How it is
  modelled". Check: A2 below, 1.0.
- **A leaf disturbed scores 0**, so "leave the plant's leaves as they were" and "undamaged" are scored in place of the
  removed touch rule. Simulation: any leaf's tip more than 3 cm from where it lay at the end, or, at any time, a contact
  force above 5 N on a leaf (design value, averaged over each 50 ms control step) from the jug or any link of either
  arm, read after every physics step from a PhysX contact view with each finger counted separately. Real trial: any
  leaf's tip more than 3 cm from its tape mark in the top camera's last frame, or a leaf creased, kinked or torn. Where:
  task file (`_tip_moves`, `_leaf_forces`), setup, scoring, fidelity. Checks: A3, A4 and A5 below, each 0.0; the demo's
  hand peaks at 1.7 N on the big leaf.
- **All four leaves on sprung stalks.** The crown's three leaves were fixed; each now turns up to 30 degrees either way
  against 0.004 N m per degree and springs back, as a wire-stalked leaf does, so a brush moves a leaf and a leaf held
  out of place shows in its tip. Where: task file (`_crown_leaf`, `crown_stalk_*`), setup, fidelity.
- **The water rule mirrors the hidden-frame rule.** A bead within 4 mm of a leaf (kept) and now also the jug's lip and
  the top 1 cm of its rim within 4 mm of a leaf, once water has left the jug (the real trial's "wet part of the jug").
  Where: task file (`_wet_points`, `_watch_leaves`), scoring, fidelity. Check: in the demo, the wet lip and rim stay
  at least 24.3 mm from every leaf.
- **Beads against the pot's wall count.** A bead counts as delivered with its centre within 7.6 cm of the pot's axis
  (the inside of the wall; it was 7.25 cm, which missed beads lying on the soil against the wall). A bead outside the
  pot lies at least 8.85 cm out, and one on the rim is above the 15 mm height band. The real trial weighs the whole pot,
  so it is unchanged. Check: the demo delivers all 12 beads.
- **Adversarial checks (simulation, scratch copies of the task file with only the demo's motion changed):**
  - A1, the stream grazing the held leaf's edge (the jug's lip 3.5 cm toward it): 5 beads touch the leaf and 7 reach
    the soil, with the jug 8.7 mm clear; score 0.0 (0.25 without the water rule);
  - A2, the dry jug, before pouring, swept 4.2 cm sideways into the left crown leaf and back: the jug touches the leaf
    (2.1 N), which turns 2.6 degrees and springs back; 12 beads on the soil, every end-state condition met; score 1.0;
  - A3, the dry jug swept 10 cm into the left crown leaf, driving it to its 30 degree stop, then the demo carried on
    and met every other full-score condition (12 beads on the soil, jug back, arms home, every tip back): 13.6 N
    averaged over a control step (31.8 N in a single physics step), score 0.0 (1.0 without the force rule);
  - A4, a clean pour, then the jug held against the left crown leaf to the end, turning it 26 degrees: its tip 68 mm
    out of place, pressed with 4.3 N (under the 5 N limit, so the tip rule alone decides), score 0.0 (0.5 without the
    new rule);
  - A5, a clean pour, the jug back and the left arm home, but the right hand still holding the big leaf aside at the
    end: its tip 199 mm out of place, with no press over 5 N; score 0.0 (0.5 without the new rule);
  - A6, one hand (the right arm never moves): the jug presses on the big leaf, which lies over the soil, its wet lip
    touches it, and all 12 beads land on it; score 0.0.
- **Kept, as the reviews accepted them:** the matched pot and saucer weighings, the plugged drain, the real plant and
  water, and the thresholds (90 g and the end state for 1.0, 90 g for 0.5, 40 g for 0.25).
- **Runs:** the scripted demo scores 1.0 (607 steps; all 12 beads on the soil, no bead within 4 mm of a leaf, every
  leaf back in place); Dream-1 scores 0.0 (1,200 steps).

Earlier reviews:

- TK-7FFS8K asked for fast-pour tests of the home-made indicator film; the film was replaced by water-sensitive cards,
  and now no detector is used at all (above).
- TK-CHR9EE asked to show that the two-camera and photo read catches brief, edge, hidden and runoff contact, or to
  adjust the method; the read now has the hidden-frame rule and grazing-light photos of both faces (above).
- TK-T4P2JA asked for tests around a 10 ml leaf allowance; the allowance was removed (water on a leaf scores 0.0), and
  that is kept.
- TK-EYKT6P asked for the pot to be weighed in the same leaf state before and after; the matched, leafless weighings are
  kept.
- TK-A2TMY6 asked to tell water on the soil from water in the saucer; the separately weighed pot and saucer and the
  plugged drain are kept.
- TK-J457LS asked for a reproducible real plant, a real water volume with a reset procedure, and real-world thresholds
  for every tier; all three are kept.
