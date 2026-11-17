# Strip 27 mm of insulation off a 35 mm² cable end with a round-cable stripping knife

A round-cable stripping knife, a JOKARI No. 27 SECURA (part 10270), lies on the bench beside a beech gauge block. A
short single-core 35 mm² cable of type H07V-K (10.3 mm across) stands on its free end in the block's 12 mm hole, with a
plug on its other end. The knife's blade is set 1.2 mm deep, the thickness of the cable's insulation. Lying on the block
with the cable through its bracket, the knife has its blade 27 mm above the hole's floor, so the block sets the strip
length: 27 mm, for a 35 mm² ferrule (Phoenix Contact AI 35 -25 RD: a 25 mm sleeve, at most 29 mm stripped).

The job: put the knife's bracket over the cable and lay the knife on the block; press it onto the cable so that its
blade stays in the insulation, while the cable is turned against it more than a full turn (the ring cut); then pull the
cable out of the still-pressed knife. The blade holds the cut insulation back, it drops into the hole, and 27 mm of
copper comes out bare. The task is one task file,
[`strip_a_wire_end_in_a_stripping_station.py`](strip_a_wire_end_in_a_stripping_station.py), for the `yam_arena`
embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same
folder.

What it tests:
- putting a tool's closed loop over a standing part and laying the tool flat on a fixture, both by one hand;
- keeping the knife pressed on (at least 6 N) while the other hand turns the cable more than a full turn by its plug,
  letting go and taking hold again between quarter turns, without the cable leaving its hole;
- a two-handed pull of 25 N against the pressed knife.

Why it matters: every conductor is stripped to length before it is terminated, so stripping is regular work for
electricians (about 821,000 US jobs in 2025, BLS Occupational Outlook Handbook, which lists wire strippers among their
commonly used tools). O*NET lists for them "Connect wires to circuit breakers, transformers, or other components"
(47-2111.00), and for their helpers "Strip insulation from wire ends, using wire stripping pliers, and attach wires to
terminals for subsequent soldering" (47-3013.00). Feeders, sub-main and incoming cables and earthing conductors are
16 mm² and larger, past the range of stripping pliers, which stop at 16 mm² (WEICON's No. 5 Pro, for example: "working
range 0,2 - 16,0 mm²"). Past that range the insulation is cut round with a cable knife whose blade depth is set to
the insulation's thickness, the method in this knife's user guide (below), and the cut piece is taken off. Too short a
strip and the ferrule's 25 mm sleeve is not filled; too long and bare copper shows beyond it.

Why two hands: the blade cuts only while the knife is pressed onto the cable, and the cable has to be turned against
it at the same time; pulling the cable out of the knife takes 25 N, and the cut insulation only stays behind while the
knife is still pressed on. One hand holds and presses the knife, the other turns and then pulls the cable. Checked on
this version: with the right arm left at its start and the left doing all of its demo motion (taking the plug, turning
the cable 540 degrees in six quarter turns, pulling it out of the hole), nothing was cut and the score was 0.0.

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

**The maker's method.** JOKARI's user guide for its cable knives
([No. 10160 - 10700](https://cdn.automationdirect.com/static/manuals/jokari/b10160.pdf), which covers the 10270):
- A: "Adjust blade cutting depth by turning the red adjustment screw." The guide also says: "To adjust the depth of cut
  for insulation thickness turn screw at end of handle."
- B: "Press tool against cable as shown, exerting light pressure with your thumb."
- C: "Rotate to execute one or two circular cuts."
- D: "Pull tool in direction of arrow to execute a longitudinal cut." A 27 mm end does not need it.
- E: "Peel off the sheathing. Finished!"

**This tool on this cable.** The H07V-K 1 x 35 mm² is a round, PVC-insulated cable 10.3 mm across, inside the knife's 8
to 28 mm range. Its insulation is its only layer, so with the blade set to its 1.2 mm thickness the circular cut goes
through to the copper.

**How the robot's method differs from a person's hands.**
- It turns the cable inside the pressed knife instead of swinging the knife round the cable: the blade turns the same
  way against the insulation. A parallel gripper turns a part by rolling its wrist about the part's axis.
- It takes the cut insulation off by drawing the conductor out of the pressed knife, whose blade holds the cut piece
  back, instead of peeling it off with the fingers.
- The knife lies on a gauge block that sets the strip length (in place of a pen mark on the cable), and the block's hole
  holds the cable's end (in place of the hand holding the cable below the knife).
- The lead is short, so the knife's bracket goes on over the plug end.

## The real setup

Distances are measured forward from the line between the arm bases and sideways from the midpoint between them (left
positive). Every number here is the one the task file uses.

**The gauge block** (design values) is beech, 150 x 50 mm and 16.5 mm tall, its long axis 60 degrees left of straight
ahead. A 12 mm hole (0.85 mm all round the 10.3 mm cable), 12.5 mm deep with its floor 4 mm above the table, has its axis
30 cm forward and 7 cm left of the centre line, 25 mm from the block's front end. The knife lies on the block behind the
hole, its blade then 14.5 mm (half the knife's 29 mm width) above the block's top: 27 mm above the hole's floor.

**The knife** lies on its side on the bench, its 29 mm width upright. The middle of its front face (where the blade comes
out) is 39 cm forward and 2 cm right of the centre line, and it points 60 degrees left of straight ahead. Its blade is
set 1.2 mm out of the face. Its bracket (design values from the maker's dimensions and its 28 mm capacity): band steel
2 mm thick and 16 mm wide in the plane of the blade, its sides 15.5 mm either side of the knife's axis, running 26 mm
forward from the face, its front bent to a 6 mm V. With the face it closes a loop 29 mm wide and 26 to 32 mm deep.

**The cable** is H07V-K 1 x 35 mm² single core, 10 cm long (a manufacturer's datasheet: a class 5 flexible copper
conductor 7.9 mm across, 1.2 mm of PVC insulation, 10.3 mm across overall, 340 g per metre). Its top 3 cm is moulded into
a square grey ABS plug 16 mm across, turned 45 degrees to the table's edges.
- 10 cm of cable at 340 g/m weighs 34.0 g.
- The plug is 16 x 16 x 30 mm of ABS (1.05 g/cm³) less the 10.3 mm cable through it: 5.4 g.
- In all, 39.4 g. Of the 340 g/m, 48 g/m is the PVC.

**Forces** (computed).
- **To cut.** A blade of included edge angle beta and thickness t advancing through PVC whose flow stress is its
  tensile strength sigma needs, per mm of edge, F' = 2 sigma h (tan(beta/2) + mu), with h = t / (2 tan(beta/2)) the
  depth over which its flanks spread the PVC apart: F' = sigma t (1 + mu / tan(beta/2)).
  - sigma = 12.5 N/mm², the minimum for PVC/C insulation in IEC 60227-1; t = 0.4 mm and beta = 20 degrees (design
    values for the blade); mu = 0.2.
  - F' = 12.5 x 0.4 x (1 + 0.2 / 0.176) = 10.7 N per mm of edge; over the 1.2 mm of edge in the insulation, 12.8 N.
  - At the middle of the insulation, 4.55 mm from the axis, that is 0.058 N m against the turn.
- **To keep the blade in.** The PVC pushes back on the blade's leading edge, 0.4 x 1.2 mm, at its flow stress:
  12.5 x 0.48 = 6.0 N. The knife has to be pressed on at least that hard, the maker's "light pressure".
- **To pull.** Strip forces, the force to pull a conductor out of a set length of cut insulation, are "typically in the
  range of 5 to 25 pounds (22 N to 111 N)" (Anixter, Wire Wisdom: Wire Strippability, Wiring Harness News, Jul/Aug
  2021), on test lengths commonly 25.4 mm. For 27 mm that is 23 to 118 N; the task uses 25 N.

## Scoring a real trial

- **The stripped length** is measured with a ruler or a caliper from the conductor's end to the cut edge of the
  insulation. For 1.0 it must be 25 to 29 mm, from the ferrule's 25 mm sleeve to its 29 mm maximum stripping length.
- **Pulled off:** the cut insulation is off the conductor (in the hole or the knife), as a photograph shows.
- **Cut all round but not pulled off:** after the trial the cut insulation slides off the conductor by hand.
- **A part-way cut:** the trial video shows the knife lying on the block with its face against the cable while the cable
  turns at least a quarter turn, and the part-way cut is visible on the insulation.

The simulation scores the same outcomes from the same events; "How it is modelled" gives its rule.

## Real-trial safety and rig protection

- The blade stands 1.2 mm out of the knife's face, inside the steel bracket. The grippers touch only the knife's plastic
  body and the cable's plastic plug, so no sharp edge meets the pads.
- The cable is a dead offcut, as the maker requires ("Never use tool on or near live electrical circuits").
- People nearby wear eye protection while the knife is in use. Nothing is greasy, wet or loose: the cut insulation stays
  in the hole or the knife.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres.
- **The knife** is built from shapes at the maker's dimensions: a body 100 x 22 x 29 mm whose front face carries the
  blade (shown 1.2 mm out of the face; not a collider), and the steel bracket as plates. It weighs 72 g, its centre of
  mass 46 mm behind the face.
- **The cable** is a rigid body of 37.8 g, its centre of mass 56.5 mm up from its end. The insulation over its last 34 mm
  is 17 rings of 2 mm, 0.096 g each (39.4 g in all), each held to the cable by a fixed joint and shown but not
  colliding. For contact, that insulated end is one hidden cylinder of the cable's full 10.3 mm width, part of the cable
  itself; it stays on after the cut, and the bare end touches nothing later in the task.
- **The gauge block** is fixed where it is put.

The cut is a rule driven by real contact, the turn and the blade's set depth:
- **Pressed on.** Each control step, PhysX reports the contact force between the cable and the knife. The knife counts as
  pressed on while its face pushes the cable with at least 6.0 N, the cable's axis is inside the bracket and within
  1.5 mm of touching the face, and the blade's plane lies on the insulated end.
- **The turn.** While the knife is pressed on, the cable's turn against it is followed at that place along the cable
  (within 1.5 mm). Each new degree, which cuts fresh insulation, is resisted by 0.058 N m on the cable and the opposite
  on the knife.
- **The ring.** Once the arc turned reaches 360 degrees, the insulation is cut all round at that distance from the
  cable's end; the rings below the blade become the cut piece, held in the knife.
- **The pull.** While the knife stays pressed on (at least 3 N), a joint locked along the cable, anchored to the gauge
  block, holds the conductor in its cut insulation; PhysX breaks it at 25 N, so the robot has to pull past 25 N. Once the cable's end is 2 mm above the
  blade, it is stripped and the cut piece drops into the hole. If the knife is let off (under 3 N for 0.25 s) before
  that, the cut insulation stays on the conductor.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

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
  --instruction "Strip 27 mm of insulation off the free end of the cable with the cable knife." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/strip-dream
```

The scripted demo, which reaches full score:
- The right picks the knife up off the bench by the back of its body, 90 mm behind the face, its grip leaning back 60
  degrees along the knife (which keeps its wrist camera clear of the left hand). It carries the knife 10 cm high over
  the standing cable and lowers it, the bracket's loop going down over the plug and the cable, onto the gauge block.
- The left comes down on the cable's plug and takes it.
- The right presses the knife onto the cable, aiming 12 mm past where it touches (it pushes 7 to 20 N, 11 N on
  average), and keeps it pressed.
- The left turns the cable by its plug in six quarter turns (540 degrees), letting go and taking it again between them.
  The cut closes all round during the fifth.
- The left pulls the cable straight up out of the still-pressed knife, aiming 7 cm high: the joint gives at 25 N, the
  cable comes out with 27 mm of copper bare, and the cut piece drops into the hole.
- The right eases the knife back 3 cm along the block, and both hands hold still.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=strip_a_wire_end_in_a_stripping_station.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Strip 27 mm of insulation off the free end of the cable with the cable knife." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/strip-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/strip-dream/*.json; do inspect-robots video "$log"; done
```

Set `STRIP_DEBUG=1` to print, twice a second and every 0.1 s while the knife is on the cable, what the score is built
from: the knife's push on the cable and whether it counts as pressed on; how far the blade's plane is from the cable's
end, and where the cable's axis is in the knife's frame; the cable's turn against the knife and the arc cut so far;
whether it is cut all round, held and stripped; and where each gripper's TCP is from the plug and from the knife.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 1,500 steps (301 s on the clock, with Dream answering in 1.10 s on average), and about 3 minutes for the scripted demo (783 steps, 39.1 s of simulated time, 155 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-ATF9KW. Its review said:

> Importance (not met): Electricians and electrical equipment assemblers regularly strip wire, but the proposed custom manual station is not established as a tool they use for this cable.
>
> Construct validity (not met): The submission reports finding no comparable manual benchtop stripper for 35 mm² cable, leaving the specified manual cut-and-pull method unsubstantiated as real workplace practice.
>
> Clarity and reproducibility (not met): The custom stripper's rack, pinion, blade carriers and cam lack the drawings or build files needed to reproduce its main mechanism.
>
> What to change: The earlier station-reproducibility point is still open: provide dimensioned fabrication drawings or build files for the complete blade carriers, guides, rack, pinion and cam, or identify a suitable off-the-shelf tool. New point not raised in the earlier reviews: the submission says no comparable manual benchtop stripper for 35 mm² cable was found, so please document that this manual method is regularly used in the stated work, or replace the station with a documented tool and method used for this cable and update the simulation and demonstration accordingly.

The review before that (TK-NFPGN5) asked for the custom station's blade, cam and force details and a corrected cable
mass; the station is now replaced by an off-the-shelf knife, and the cable's mass follows from 340 g/m (39.4 g for
10 cm with its plug). The review before that (TK-UEJBT7) asked for the Impact on jobs domain, a hole that takes the
cable and a core diameter reconciled with 35 mm²; that is kept (H07V-K 1 x 35 mm², 7.9 mm conductor, 10.3 mm overall,
12 mm hole). The first review (TK-3V78DN) asked for a buildable station; it is replaced by a purchasable tool.

What changed in response:

- **Clarity and reproducibility: an off-the-shelf tool replaces the custom station.** The lever station (blade
  carriers, rack, pinion and cam) is gone. The cable is now stripped with a JOKARI No. 27 SECURA cable knife, part
  10270 (EAN 4011391102705), $27.00 at AutomationDirect (JT10270). The maker's data sheet gives its size (L 132, B 29,
  H 35 mm), its 72 g, its 8 to 28 mm capacity and its materials; spare blades are part 19000. The only other fixture
  is a plain beech gauge block, fully dimensioned: 150 x 50 x 16.5 mm with a 12 mm hole 12.5 mm deep.
- **Importance: the tool is an electrician's cable knife, used on this cable.** The 10.3 mm H07V-K 35 mm² is inside
  the knife's 8 to 28 mm range. Stripping pliers stop at 16 mm² (WEICON No. 5 Pro: "working range 0,2 - 16,0 mm²"), so
  conductors of 16 mm² and more (feeders, sub-mains, earthing conductors) are cut round with a cable knife set to the
  insulation's thickness, as the maker's guide instructs ("To adjust the depth of cut for insulation thickness turn
  screw at end of handle").
- **Construct validity: the method is the maker's documented one.** The steps follow JOKARI's user guide: set the blade's
  depth (1.2 mm, the insulation's thickness); "Press tool against cable ... exerting light pressure"; "Rotate to execute
  one or two circular cuts"; take the cut insulation off. The strip length, 27 mm, is for a real 35 mm² ferrule, Phoenix
  Contact AI 35 -25 RD (3200713): a 25 mm sleeve and a 29 mm maximum stripping length, so 1.0 needs 25 to 29 mm. The
  README lists how the robot's way differs from a person's hands: it turns the cable in the pressed knife rather than
  swinging the knife round; it draws the conductor out of the pressed knife rather than peeling the cut piece off; a
  gauge block sets the strip length in place of a pen mark.
- **Simulation and demonstration rebuilt around the knife.**
  - The knife is modelled at the maker's dimensions and mass; its bracket closes a loop round the cable.
  - The cut needs real contact, a real turn and the real force. The knife's face must push the cable with at least
    6.0 N (PhysX contact force; computed from the PVC's flow stress on the blade's 0.4 x 1.2 mm edge), and the cable
    must turn a full 360 degrees against it at one place along it, each new degree resisted by 0.058 N m (computed).
  - Taking the cut insulation off needs the knife kept pressed while the cable is pulled out past 25 N, the low end of
    published strip forces for that length.
  - The new demo: the right lays the knife's loop over the standing cable onto the gauge block and presses it on
    (7 to 20 N); the left turns the cable 540 degrees in six quarter turns, the cut closing all round during the fifth,
    then pulls the cable out past 25 N. The cut piece stays in the knife and drops into the hole, and 27 mm of copper
    is bare.
- **Re-run:** the demo scores 1.0 (783 steps), three runs in a row, each cut 27.0 mm from the end; the one-hand check
  scores 0.0; Dream-1 scores 0.0 (1,500 steps).
