# Strip 27 mm of insulation off a 35 mm² cable end with a round-cable stripping knife

A round-cable stripping knife, a JOKARI No. 27 SECURA (part 10270), lies on the bench beside a beech gauge block with two
guide strips and a steel V backstop. A 10 cm
offcut of single-core 35 mm² cable of type H07V-K (10.3 mm across), a square aluminium grip sleeve clamped on its top,
stands on its free end in the block's 12 mm hole. The knife's blade is set 0.9 mm deep, under the 0.98 mm the cable's
insulation may be at its thinnest. Lying on the block with the cable through its bracket, the knife has its blade 27 mm above the hole's floor, so the block sets the strip length:
27 mm, for a 35 mm² ferrule (Phoenix Contact AI 35 -25 RD: a 25 mm sleeve, at most 29 mm stripped).

The job follows the knife maker's own steps: press the knife onto the cable, cut round (one full turn), cut along (the
blade run along the cut piece out past the cable's end), and peel the cut piece off, leaving 27 mm of bare, unnicked
copper. The task is one task file,
[`strip_a_wire_end_in_a_stripping_station.py`](strip_a_wire_end_in_a_stripping_station.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests:
- putting a tool's closed loop over a standing part and laying the tool flat on a fixture, by one hand;
- keeping the knife pressed on (at least 4.5 N) while the other hand turns the cable a full turn by its grip, letting go
  and taking hold again between quarter turns, without the cable leaving its hole; then drawing the cable up through the
  still-pressed knife for the cut along;
- a two-handed peel: one hand holds the cable out, the other takes the 27 mm cut piece and pulls it off the strands.

Why it matters: every conductor is stripped to length before it is terminated, so stripping is regular work for
electricians (about 821,000 US jobs in 2025, BLS Occupational Outlook Handbook, which lists wire strippers among their
commonly used tools). O*NET lists for them "Connect wires to circuit breakers, transformers, or other components"
(47-2111.00), and for their helpers "Strip insulation from wire ends, using wire stripping pliers, and attach wires to
terminals for subsequent soldering" (47-3013.00). Feeders, sub-main and incoming cables and earthing conductors are
16 mm² and larger, past the range of stripping pliers, which stop at 16 mm² (WEICON's No. 5 Pro, for example: "working
range 0,2 - 16,0 mm²"). Past that range the insulation is cut with a cable knife whose blade depth is set to the
insulation's thickness, by the steps of this knife's user guide (below). Too short a strip and the ferrule's 25 mm
sleeve is not filled; too long and bare copper shows beyond it; a nicked strand is a weak point in the termination.

Why two hands: the blade cuts only while the knife is pressed onto the cable, and the cable has to turn against it (the
cut round) and then move along it (the cut along) at the same time; the peel needs the cable held while the cut piece is
pulled off it. One hand holds and presses the knife, the other turns and draws the cable; then one holds the cable out
and the other peels. Checked on this version: with the right arm left at its start and the left doing all of its demo
motion, nothing was cut and the score was 0.0.

## The tool and how it is used

**The knife.** JOKARI No. 27 SECURA cable knife, part 10270 (EAN 4011391102705), made in Germany. From the maker's data
sheet ([Datenblatt 10270](https://jokari.de/ptg/Datasheet/source/Datenblatt_10270.pdf)):
- usage: "Stripping of sheathing of round cables by circular and longitudinal cut";
- capacity: Ø 8 - 28 mm;
- size and weight: L 132 mm, B 29 mm, H 35 mm, 72 g;
- materials: a polyamide body with a thermoplastic-elastomer grip; a bracket of "shaped flat steel, hardened"; a blade
  of "special steel, hardened and tempered, titan-nitride coated (spare knife inside handle)". Spare blades are part
  19000 (a set of 3).

It sells for $27.00 at AutomationDirect (JT10270, [automationdirect.com/pn/JT10270](https://www.automationdirect.com/pn/JT10270),
2026-11-16).

**The maker's method, step by step.** JOKARI's user guide for its cable knives
([No. 10160 - 10700](https://cdn.automationdirect.com/static/manuals/jokari/b10160.pdf), which covers the 10270), and
how the task does each step:

| Step | The guide's words | In this task |
|---|---|---|
| A | "Adjust blade cutting depth by turning the red adjustment screw." ("To adjust the depth of cut for insulation thickness turn screw at end of handle.") | Done before the trial: the blade stands 0.9 mm out of the face. |
| B | "Press tool against cable as shown, exerting light pressure with your thumb." | One hand lays the knife on the block round the cable and presses it on, at least 4.5 N. |
| C | "Rotate to execute one or two circular cuts." | The other hand turns the cable a full turn (360 degrees) against the pressed knife: the cut round. |
| D | "Pull tool in direction of arrow to execute a longitudonal cut." (the arrow runs along the cable to its end) | That hand draws the cable up through the still-pressed knife, the backstop behind it, so the blade runs from the cut round out past the cable's end: the cut along. |
| E | "Peel off the sheathing. Finished!" (German: "Isolierung abziehen. Fertig!", pull the insulation off) | One hand holds the cable out; the other takes the cut piece and pulls it off the strands. |

The guide's pictures show B to E on a round cable held in one hand: the knife's bracket round the cable, the knife swung
round it, then drawn along it toward the end, and the cut piece pulled off the end with the fingers.

**How the robot's way differs from a person's hands.** Only in which part moves:
- C: it turns the cable inside the pressed knife instead of swinging the knife round the cable; the blade travels round
  the insulation the same way. A parallel gripper turns a part by rolling its wrist about the part's axis.
- D: it draws the cable through the pressed knife instead of drawing the knife along the cable; the blade travels along
  the insulation the same way, from the cut round to the end.
- The knife lies on a gauge block that sets the strip length (in place of a pen mark on the cable); the block's hole
  holds the cable's end and a steel V backstop behind the cable takes the knife's push (in place of the hand holding the
  cable behind and below the knife). Two guide strips on the block keep the
  knife square to the cable while it is pressed on (in place of the hand's wrist), and leave it free to slide toward and
  away from the cable.

**The blade's depth.** The cable's insulation is 1.2 mm thick (nominal), but IEC 60227-1 (5.2.3) lets it be thinner at
any place: "the thickness at any place may be less than the specified value provided that the difference does not
exceed 0,1 mm + 10 % of the specified value", so 0.98 mm at its thinnest. The blade is set to 0.9 mm with the red screw
(checked with a caliper's depth rod on the knife's face), so it cannot reach the strands anywhere round the cable. The
0.3 mm of PVC left under the cuts tears as the split piece is peeled off.

## The real setup

Distances are measured forward from the line between the arm bases and sideways from the midpoint between them (left
positive). Every number here is the one the task file uses.

**The gauge block** (design values) is beech, 150 x 50 mm and 16.5 mm tall, its long axis 60 degrees left of straight
ahead. A 12 mm hole (0.85 mm all round the 10.3 mm cable), 12.5 mm deep with its floor 4 mm above the table, has its axis
30 cm forward and 7 cm left of the centre line, 25 mm from the block's front end. The knife lies on the block behind the
hole, its blade then 14.5 mm (half the knife's 29 mm width) above the block's top: 27 mm above the hole's floor.
Two beech guide strips, 17 x 5 mm and 6 mm high, are glued on the block's top along its long axis, 25 mm apart (1.5 mm
of clearance either side of the knife's 22 mm body), from 19 to 36 mm behind the hole's axis; at 6 mm high they pass
under the bracket's side straps (their band's lower edge is 6.5 mm above the block). The knife drops in between them
from above and slides along them. The backstop is a 90-degree V of two 10 x 3 mm steel flat bars, 30 mm long, glued
8 mm deep into slots in the block's top on the far side of the hole from the knife, at 45 degrees either side of the
knife's axis, their inner faces 6 mm from the hole's axis (the hole's own clearance). Standing 22 mm above the block's
top, they back the cable up where the knife presses it, up to 7.5 mm above the blade's plane, and keep it centred on
the blade, as an electrician's fingers back up a cable behind the blade. The V stands inside the knife's loop, between
the cable and the bracket's front.

**The knife** lies on its side on the bench, its 29 mm width upright. The middle of its front face (where the blade comes
out) is 39 cm forward and 2 cm right of the centre line, and it points 60 degrees left of straight ahead. Its blade is
set 0.9 mm out of the face. Its bracket (design values from the maker's dimensions and its 28 mm capacity): band steel
2 mm thick and 16 mm wide in the plane of the blade, its sides 15.5 mm either side of the knife's axis, running 26 mm
forward from the face, its front bent to a 6 mm V. With the face it closes a loop 29 mm wide and 26 to 32 mm deep.

**The cable** is a 10 cm offcut of H07V-K 1 x 35 mm² single core (a manufacturer's datasheet: a class 5 flexible copper
conductor 7.9 mm across, 1.2 mm of PVC insulation, 10.3 mm across overall, 340 g per metre), cut square at both ends:
0.10 m x 340 g/m = 34.0 g, of which 48 g/m is the PVC. It stands on its free end in the block's hole.

**The grip sleeve** (design values), reused from trial to trial, is what the robot holds the cable by:
- 16 x 16 mm 6061 aluminium square bar, cut 30 mm long, edges deburred;
- a 12 mm hole bored through along its length (it takes the 10.3 mm cable, and any H07V-K 1 x 35 up to the standard's
  11.7 mm);
- an M4 tapped hole at mid-length in one face, for an M4 x 6 set screw (ISO 4026, 2 mm hex key);
- 11.6 g (the bar less its bore). Its flats face 45 degrees to the table's edges; the flats let a parallel gripper
  turn the cable without slipping.

For each trial a fresh offcut is cut from the reel with a cable cutter against a 10 cm mark, pushed into the sleeve flush
with its top, and the set screw tightened: under a minute. The screw's point presses into the PVC, so the cable turns
with the sleeve.

**Forces** (computed).
- **To cut.** A blade of included edge angle beta and thickness t advancing through PVC whose flow stress is its
  tensile strength sigma needs, per mm of edge, F' = 2 sigma h (tan(beta/2) + mu), with h = t / (2 tan(beta/2)) the
  depth over which its flanks spread the PVC apart: F' = sigma t (1 + mu / tan(beta/2)).
  - sigma = 12.5 N/mm², the minimum for PVC/C insulation in IEC 60227-1; t = 0.4 mm and beta = 20 degrees (design
    values for the blade); mu = 0.2.
  - F' = 12.5 x 0.4 x (1 + 0.2 / 0.176) = 10.7 N per mm of edge; over the 0.9 mm of edge in the insulation, 9.6 N.
  - Cutting round, at the middle of the cut, 4.7 mm from the axis, that is 0.045 N m against the turn; cutting along,
    9.6 N against the cable's travel through the knife.
- **To keep the blade in.** The PVC pushes back on the blade's leading edge, 0.4 x 0.9 mm, at its flow stress:
  12.5 x 0.36 = 4.5 N. The knife has to be pressed on at least that hard, the maker's "light pressure".
- **To peel.** Once cut round and along, the cut piece is a split sleeve; the 0.3 mm of PVC left under the cuts tears
  from the slit as it is opened, and the hand slides it off the strands against 2 N (design value).

## Scoring a real trial

- **The stripped length** is measured with a ruler or a caliper from the conductor's end to the cut edge of the
  insulation. For 1.0 it must be 25 to 29 mm, from the ferrule's 25 mm sleeve to its 29 mm maximum stripping length.
- **Peeled off:** the cut piece is off the conductor, as a photograph shows.
- **The strands:** after the trial the bare copper is looked over all round under a 10x loupe, and each strand that is
  cut (a loose or short end), nicked (a bright notch) or flattened is counted. The strand ends at the conductor's end
  are counted on a photo through the loupe and compared with the count on the end of a fresh offcut from the same reel
  (a class 5 35 mm² conductor has about 276 wires of 0.40 mm: Kabeltronik's H07V-K 35 mm² data, "276 x 0,40 mm"); a
  strand cut through at the cut round leaves with the cut piece, so it shows as one fewer. The removed piece is looked
  through for strands caught in it. Full credit needs no damaged or missing strand; any caps the score at 0.25.
- **Cut round but not peeled off:** after the trial the cut piece comes off by hand.
- **A part-way cut:** the trial video shows the knife lying on the block with its face against the cable while the cable
  turns at least a quarter turn, and the part-way cut is visible on the insulation.

The simulation scores the same outcomes from the same events; "How it is modelled" gives its rules.

## Real-trial safety and rig protection

- The blade stands 0.9 mm out of the knife's face, inside the steel bracket. The grippers touch only the knife's plastic
  body, the deburred grip sleeve and the cable's PVC, so no sharp edge meets the pads.
- The cable is a dead offcut, as the maker requires ("Never use tool on or near live electrical circuits").
- People nearby wear eye protection while the knife is in use. Nothing is greasy, wet or loose: the cut piece stays in
  the hand that peels it.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres.
- **The knife** is built from shapes at the maker's dimensions: a body 100 x 22 x 29 mm whose front face carries the
  blade (shown 0.9 mm out of the face; not a collider), and the steel bracket as plates. It weighs 72 g, its centre of
  mass 46 mm behind the face.
- **The cable** is a rigid body with the grip sleeve as a 16 mm square box on its top 30 mm. The insulation over its
  last 34 mm is shown as 17 rings of 2 mm, 0.096 g each, and collides as a hidden cylinder of the cable's full 10.3 mm
  width, part of the cable; that stays on after the peel, and the bare end touches nothing later. A hidden sleeve
  0.3 mm wider over the same 34 mm (1.0 g), held to the cable by a fixed joint and a sliding joint along its axis, is
  what a hand grips to peel the cut piece off; nothing else touches it. 45.6 g in all, its centre of mass 61 mm up
  from its free end.
- **The gauge block**, with its hole, its two guide strips and the backstop, is fixed where it is put.

The cuts are rules driven by real contact, real motion and the blade's set depth:
- **Pressed on.** Each control step, PhysX reports the contact force between the knife and the cable. The knife counts
  as pressed on while its face pushes the cable with at least 4.5 N, the cable's axis is inside the bracket, within
  1.5 mm of touching the face and within 3 mm of the middle of the blade's 4 mm edge, and the blade's plane lies on the
  insulated end.
- **Cut round.** While the knife is pressed on, the cable's turn against it is followed at that place along the cable
  (within 1.5 mm). Each new degree, which cuts fresh insulation, is resisted by 0.045 N m on the cable and the opposite on
  the knife. Once the arc reaches 360 degrees, the insulation is cut round at that distance from the cable's end.
- **Cut along.** While the knife is pressed on, the cable's travel along its axis through the knife is followed in
  0.5 mm steps; each new step is resisted by 9.6 N along the cable and the opposite on the knife. The cut along is done
  once it runs from the cut round to within 1 mm of the cable's end.
- **The nick check.** The knife's face rides on the insulation, so the blade's edge goes in its set depth, less any gap
  between the face and the cable, and less the cable's curve away from the edge where the cable sits to one side of it.
  If that ever passes 0.98 mm, the insulation's thinnest, while the knife is pressed on, the conductor counts as nicked
  for the rest of the trial, and the score is at most 0.25.
- **The peel.** Until the insulation is cut round and cut along, the sleeve is held to the cable outright. Once both
  are done, the knife is off the cable (under 1 N for 0.25 s) and the cable's end is clear of the knife, the sleeve
  (shown now as the rings below the cut) can slide along the cable's axis toward its end, against a joint drive capped
  at 2 N whose target follows it, so it is never pulled back on. Peeled off means the cut piece is pulled at least its
  own length plus 2 mm off the conductor's end.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Checks run on this version

Each is the scripted demo with one change, run in the simulator:
- **Blade set too deep** (environment `STRIP_BLADE_MM=1.2`, the insulation's nominal thickness): the edge went 1.20 mm
  in, the nick latched as the knife was first pressed on, and the trial, otherwise run through to a peeled strip,
  scored 0.25.
- **Blade set just past the limit** (`STRIP_BLADE_MM=1.0`, 0.02 mm past the insulation's 0.98 mm thinnest): the nick
  latched as the knife was first pressed on; the trial ran through to a peeled 27 mm strip and scored 0.25.
- **Cut along skipped** (`STRIP_DEMO=no_slit`: the knife is eased off before the cable is drawn up; the right then pulls
  the cut piece as in the demo): the piece stayed on the strands and the trial scored 0.5 (cut round at the right
  length, not peeled).
- **One hand** (`STRIP_DEMO=one_hand`: the right arm stays at its start): nothing was cut; 0.0.

## Assets

No third-party assets: every object is built from boxes and cylinders in the task file. The NVIDIA SimReady library
(Hugging Face dataset `nvidia/simready-assets`, revision 9d4556bf) has no model of this knife or cable.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the cable and the knife still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "INSTRUCTION" \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/strip-dream
```

The scripted demo, which reaches full score:
- The right picks the knife up off the bench by the back of its body, its grip leaning back 60 degrees along the knife.
  It carries the knife 10 cm high over the standing cable and lowers it, the bracket's loop going down over the cable,
  onto the gauge block.
- The left comes down on the grip sleeve and takes it.
- The right presses the knife onto the cable, aiming 15 mm past where it touches (it pushes 5 to 21 N), and keeps it
  pressed; the guide strips keep it square to the cable.
- The left turns the cable in six quarter turns of its hand, letting go and taking it again between them; while it lets
  go and takes hold again, the right eases the press to a light touch. The cut round closes during the fifth.
- The left draws the cable straight up through the still-pressed knife (its aim 5.5 cm up over 6 s), the backstop
  behind the cable, until the blade has run out past the cable's end: the cut along.
- The right eases the knife back and lets go of it; the left lifts the cable clear and holds it out in front of the
  block.
- The right comes in level from its side, takes the cut piece and pulls it down off the strands while the left lifts
  the cable a little; the cut piece slides off against 2 N, and 27 mm of copper is bare.
- Both hands hold still.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "INSTRUCTION" \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/strip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strip-dream/*.json; do inspect-robots video "$log"; done
```

Set `STRIP_DEBUG=1` to print, twice a second and every 0.1 s while the knife is on the cable, what the score is built
from: the knife's push on the cable and whether it counts as pressed on; the blade's plane along the cable and the
cable's axis in the knife's frame; the blade's depth into the insulation and the nick latch; the turn and the arc cut;
the cut along so far; whether the peel is armed and how far the cut piece is off; and where each gripper's TCP is.

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,500 steps (586 s on the clock, with Dream answering in 0.78 s on average), and about 8 minutes for the scripted demo (1,219 steps, 61.0 s of simulated time, 497 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-B8GGE8. Its review said:

> Construct validity (not met): The cited knife guide describes stripping sheathing and peeling it off, not retaining a single-core conductor's insulation with the blade during a 25 N pull.
>
> Scoring (not met): Full credit does not check for nicked or damaged copper, even though conductor integrity is essential to a usable stripped end.
>
> Clarity and reproducibility (not met): Reset requires a fresh cable with a moulded ABS grip but gives no repeatable, practical method to make or attach one for each trial.
>
> What to change: The named knife and corrected cable and holder details address the earlier station and setup points. The method point remains open: the cited guide describes cutting sheathing and peeling it off, but does not establish that this knife can hold this single-core cable's cut insulation while the conductor is pulled out. Demonstrate that operation on the specified cable and update the simulation if necessary, or use a documented tool and method for it. New points not raised before: add a real-trial conductor inspection and deny full credit for nicked or damaged strands, with a corresponding simulation check or documented proxy; also specify how to prepare or attach the square grip consistently to each fresh cable within a few minutes.

Earlier reviews, still met: TK-ATF9KW asked for an off-the-shelf tool and a documented method in place of the custom
station (the JOKARI No. 27 SECURA is kept). TK-NFPGN5 asked for the station's blade, cam and force details and a
corrected cable mass (the station is gone; the mass follows from 340 g/m). TK-UEJBT7 asked for the Impact on jobs domain,
a hole that takes the cable and a core diameter matching 35 mm² (kept: H07V-K 1 x 35 mm², 7.9 mm conductor, 10.3 mm
overall, 12 mm hole). TK-3V78DN asked for a buildable station (replaced by a purchasable tool).

What changed in response:

- **Construct validity: the knife's documented method, step for step.** The step the guide does not describe (the blade
  holding the cut insulation while the conductor is pulled out) is gone from the scorer and the demo. The task now does
  the guide's steps A to E, each quoted in setup and in "The tool and how it is used": set the depth, "Press tool against
  cable", "Rotate to execute one or two circular cuts", "Pull tool in direction of arrow to execute a longitudonal cut",
  "Peel off the sheathing".
  - Scorer: a cut-along rule (the pressed knife travels along the cable from the cut round to within 1 mm of its end,
    9.6 N against each new 0.5 mm) and a peel (the cut piece comes off the strands only once cut round and cut along,
    against a 2 N pull by a hand). Fields: scoring, fidelity; README "How it is modelled".
  - Demo: after the cut round the left draws the cable up through the pressed knife, the backstop behind it, until the
    blade runs out past the end (the cut along), and holds the cable out; the right lets go of the knife, takes the cut
    piece and pulls it off. It scores 1.0 (1,219 steps), as did the run before it.
  - Check, cut along skipped (the knife eased off before the cable is drawn up, then the same peel): the piece stays on
    the strands; 0.5.
  - The gauge block gained two guide strips that keep the knife square to the cable, and a steel V backstop behind the
    cable that takes the knife's push and keeps the cable centred on the blade, as the hand behind the cable does
    (setup, README "The real setup"). The cable's axis must stay within 3 mm of the blade's middle for the knife to
    count as pressed on, so the blade is in the insulation for every degree and millimetre that counts.
- **Scoring: damaged strands deny full credit.**
  - Real trial: the bare copper is inspected all round under a 10x loupe and each cut, nicked or flattened strand
    counted; the strand ends are counted on a 10x photo against a fresh offcut from the same reel (about 276 wires of
    0.40 mm); the removed piece is checked for strands. Full credit needs no damaged or missing strand; any caps the
    score at 0.25 (scoring field).
  - Simulation: every step the knife is pressed on, the blade's edge depth (its set depth, less any gap between the face
    and the cable, less the cable's curve where it sits beside the blade) is checked against 0.98 mm, the insulation's
    thinnest under IEC 60227-1's tolerance; past it the conductor is latched as nicked and the score capped at 0.25
    (scoring, fidelity, README "How it is modelled").
  - The blade is now set to 0.9 mm, under that 0.98 mm (it was 1.2 mm, the nominal thickness, which a thin place in the
    insulation lets reach the strands). Setup, reset_notes and the forces follow: 4.5 N to keep the blade in, 9.6 N and
    0.045 N m to cut (were 6.0 N, 12.8 N and 0.058 N m). The demo's deepest edge depth: 0.90 mm.
  - Checks, blade set too deep: at 1.2 mm and at 1.0 mm (0.02 mm past the limit) the nick latches the moment the knife
    is pressed on; the same demo then cuts, slits and peels, and scores 0.25 both times.
- **Clarity and reproducibility: a reusable clamp-on grip replaces the moulded one.** The grip is a sleeve kept from
  trial to trial: 16 mm square 6061 aluminium bar cut 30 mm long, bored 12 mm, with an M4 x 6 set screw (11.6 g; dimensions
  in "The real setup"). For each trial a fresh 10 cm offcut is cut from the reel, pushed in flush with the sleeve's top and
  the screw tightened with a 2 mm hex key, in under a minute (setup, reset_notes). The simulated grip is this sleeve,
  and the demo holds the cable by it.
- **Re-run:** the demo scores 1.0 (1,219 steps); Dream-1 scores 0.0 (1,500 steps); the one-hand check
  (right arm idle) scores 0.0.
