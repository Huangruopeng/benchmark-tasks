# Link two GPUs with the NVLink bridge that fits their slot spacing

A GPU workstation being built up on an open-air test bench before it goes into its case. On a blue antistatic mat stands
a Streacom BC1 V2 open benchtable: an aluminium plate on four short legs carrying an ATX workstation motherboard, with a
bracket bar along the back edge. Two NVIDIA RTX A6000 cards (dual-slot, 267 x 111 mm) stand in the board's first and
fourth x16 slots, three slot pitches apart, screwed to the bar; their NVLink connector covers have been taken off and
lie on the mat. To the right of the bench lie the two RTX A-series NVLink bridges a builder has in the kit: the 2-slot
one and the 3-slot one. The job is the real one: pick the bridge whose spacing matches the cards, push it down onto both
cards' NVLink connectors until both sockets are seated, and leave the other bridge where it is. The task is one task
file, [`link_two_gpus_with_the_nvlink_bridge_that_fits_their_slot_spacing.py`](link_two_gpus_with_the_nvlink_bridge_that_fits_their_slot_spacing.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder, and [`cad/`](cad), the STL of the printed 2-slot bridge replica used in real trials.

The instruction gives the goal and its constraints only ("Link the two GPUs with the NVLink bridge that fits them, and
leave the other bridge on the mat. Finish with both arms back where they started."). It names neither bridge size: the
policy has to see that the cards have an empty slot between them (a 24 mm air gap) and take the longer bridge.

What it tests: placing a flat part over two connectors at once and pushing it home at both ends together and evenly.
Each socket takes 18 N to start over its fingers, and a bridge cocked 4 degrees with an end on its fingers levers the
contacts and fails the trial, so the two pushes have to stay balanced all the way down: this is force and position
control across two hands, not a single press until a click. Before that the bridge has to be chosen to fit: the cards
stand three slots apart, so only the 3-slot bridge (sockets 60.96 mm apart) reaches both connectors, and the 2-slot one
(40.64 mm) can't link them. The job ends cleanly: the other bridge left alone and the arms back home.

Why two hands: the bridge goes onto its connectors only when both ends are pushed down together and level. Each socket
takes 18 N (36 N for the bridge), the two sockets are 61 mm apart, and a socket that runs ahead cocks the bridge, which
then binds and levers the contacts. A person does it with a thumb on each end. Two YAM wrists can't sit 6 cm apart, so
in the demo the hands lean 45 degrees out to either side and each presses one end with its closed fingertips (18 to 25 N
each). With one hand (the right doing exactly its part of the demo, the left never moving) the score was 0.25: under the
push its fingertip slid in along the bridge's top (from 33 to 15 mm from the middle), so its push was split about 12 and
11 N between the two sockets; neither moved and the bridge stayed on the lead-ins. Two other one-arm attempts failed
too. Pressing the middle, the fingertip slid 17 mm off-centre, the near end ran 3 mm ahead and the bridge stuck cocked
2.9 degrees (0.25). Pressing both ends at once with the fingers held open 61 mm apart, aimed 12.6 cm deep, seated one
end and left the other 3.4 mm short with the bridge cocked 3.2 degrees (0.0).

## Real-world basis

- **Bridges:** NVIDIA's RTX A-series (Ampere) NVLink bridges come in 2-slot (900-53651) and 3-slot (900-53669) spacing
  (Puget Systems' NVLink compatibility chart lists both for the RTX A6000; the A6000 datasheet: "NVIDIA NVLink: Connects
  two NVIDIA RTX A6000 GPUs", 112.5 GB/s, its NVLink connector with "a shorter Z height"). NVIDIA's NVLink bridge quick
  start guide (176-0434-000 r11) shows both and says: "Connect your NVLink Bridge into the labeled NVLink ports on the
  two GPUs", then check with `nvidia-smi nvlink -i 0 -s` that each link reads "active". The bridge is a flat bar: a
  glossy black cover on a circuit board, with a grey socket housing under each end that goes over the card's NVLink
  fingers (maker's product photo). Socket spacing is the slot spacing: 2 x 20.32 mm = 40.64 mm and 3 x 20.32 mm = 60.96
  mm (computed; Puget lists the 4-slot bridge at 81.26 mm). Sizes: 48 mm along the cards (from the maker's photo), 47.6
  mm (2-slot) and 68 mm (3-slot) across (computed: socket spacing plus one 7 mm housing), 11.2 mm tall (design value);
  29 g for the 2-slot (listed, Puget) and 36 g for the 3-slot (computed from the 2-slot by size).
- **Cards:** RTX A6000, "4.4" H x 10.5" L, dual slot, full height" (maker's spec), one NVLink connector on the north
  edge near the bracket. The A40 product brief (PB-09976-001, the same board) gives the bridge's fit: "The clearance
  above the card's north edge should meet or exceed 2.5 mm. The backside clearance (from the PCB's rear surface) should
  meet or exceed 2.67 mm", and 4 links of 4 lanes per bridge (maker's spec). The seated bridge's top stands 2.5 mm proud
  of the north edges, its sockets reaching down round the fingers; they travel 6 mm from the fingers' lead-in to seated
  (design value).
- **Slots:** PCI Express slots are 0.8 in (20.32 mm) apart (standard). Dual-slot cards in slots 1 and 4 are three
  pitches (60.96 mm) apart with one empty slot between them, the spacing the 3-slot bridge is made for; side by side
  (slots 1 and 3) they would take the 2-slot bridge.
- **Bench:** a Streacom BC1 V2 open benchtable (ST-BC1 V2): 5052 aluminium, 360 x 270 mm, "supports all form factor
  motherboards, plus up to four VGA cards", the board on standoffs 25 mm above the desk (maker's listing, QuietPC). It
  stands on an antistatic mat with its ATX power supply behind it; the cards' north edges are then 14 cm above the table
  (computed).
- **Why a bench, not a case:** builders fit and test GPU pairs on open benches like this before a system goes into its
  case, and the BC1 V2 is a standard product for it. A tower lying on its side does not fit the YAM's work area: its
  rear, top and bottom walls (16 to 23 cm tall) would stand between the arms and the cards, and its depth (about 46 cm)
  reaches back to the arm bases, which puts the connectors out of reach.
- **Seat force and binding:** each socket starts over its fingers at 18 N, 36 N for the bridge (design value: a firm
  two-thumb press). The socket's slot holds the 1.6 mm board with about 0.4 mm of play, so with 6 mm of it engaged it
  jams near 4 degrees of tilt (computed: atan(0.4/6) = 3.8 degrees); the model binds at 5 degrees and counts 4 degrees
  on the fingers as prying (design values).
- **Impact on AI timelines:** pairs of 48 GB RTX A6000 cards joined by an NVLink bridge (96 GB of GPU memory, 112.5 GB/s
  between them, maker's spec) are a standard build for labs and small teams that train and run models too big for one
  card, and the A40 data-centre card takes the same kind of bridge. Fitting it is a step in every such build, upgrade
  and repair, done by hand today. What makes it hard is physical: setting one part onto two connectors at once and
  pushing both ends home evenly, with the cock angle held under 4 degrees; choosing the bridge that fits the slot
  spacing comes first.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The mat, the bench (with the motherboard, its slots,
CPU cooler, memory, bracket rail and power supply) and both cards are kinematic, fixed where they are put; the two
bridges are loose rigid bodies made of shapes from `bodykit.py`, at their real sizes and masses. No NVIDIA SimReady
model matches these parts (the SimReady dataset has no GPU, motherboard, server chassis, NVLink bridge or antistatic
mat; its one desktop tower is a closed case), so everything is built from primitives.

- **Connector cut-outs:** each card's shroud has a 1.5 mm recess under the bridge and a 47 x 9 mm cut-out, 11 mm deep,
  round the gold NVLink fingers, with 45-degree lead-ins at its mouth, so a socket let down within a few millimetres of
  the fingers slides in and comes to rest on them.
- **Seating:** under each socket's place sits a hidden stop on a vertical slider (a PhysX prismatic joint) from its card,
  held up by a stiff spring. The socket rests on it at the fingers' lead-in, 6 mm above seated. After every control step
  the task file moves the spring's hold point down to wherever the stop has been pushed more than 1 mm below it, so a
  socket slides down over its fingers only while it is pushed harder than 18 N, and stays where it is left (stick-slip).
  At the bottom the socket is seated and stays.
- **Binding and prying:** each end slides on its own, so a push on one end, or off-centre, runs that end ahead and the
  bridge cocks. Once it is cocked 5 degrees the lower socket binds on its fingers and goes no further. Cocked 4 degrees
  or more while an end is on its fingers (pushed past the lead-in), the bridge levers that card's contacts: pried, and
  the trial fails.
- **Wrong bridge:** the 2-slot bridge's sockets are 20 mm too close together. One of them can go into one card's
  cut-out, but the other then hangs over the gap between the cards: it can't link them, and pushing it in cocks it.
- **Arms:** the right hand takes the bridge by its two ends; both hands press with their closed fingertips.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

Latched for the rest of the trial: 0.0 once either bridge has been cocked 4 degrees or more (one end of the 3-slot
bridge 4.3 mm lower than the other) while one of its sockets is pushed onto a card's fingers (its contacts pried), or
has fallen onto the motherboard. Otherwise judged at the end of the trial, on live states:

- 1.0: the 3-slot bridge seated on both cards (both sockets within 0.5 mm of fully down, level within 1 degree), the
  2-slot bridge left on the mat all along (never moved 3 cm or lifted 2 cm) and both arms back where they started
  (each gripper's fingertip centre within 5 cm of its start);
- 0.75: the same with an arm not back;
- 0.5: the 3-slot bridge seated on both cards;
- 0.25: the 3-slot bridge sitting square in both connectors' cut-outs (level within 3 degrees), not yet pushed down;
- else 0.0.

The 2-slot bridge can't score: its sockets can't reach both cards. In a real trial: seated means no gap between either
socket and its card's connector (a close photo at each end) and both ends at the same height above the cards (calipers,
within 0.5 mm). The cards carry no GPU and are never powered, so the photo and the calipers decide. Pried means a side
video shows one end 4 mm or more lower than the other while the bridge is on the fingers. The 2-slot bridge must not
have been moved; the arms are checked against marks at their start.

## Real trial: parts and cost

The trial needs no power and no working GPU. It needs the parts the task acts on: the 3-slot bridge, two cards'
NVLink fingers held at the slot pitch, and a 2-slot bridge to leave alone. Every part that carries the task's mechanics
is the real part; the rest is cheap.

- **3-slot bridge (the one fitted): the real NVIDIA part,** 900-53669 (sold as PNY RTXA6000NVLINK3S-KIT). Its two
  sockets are the connectors the task is about.
- **Cards: two identical no-core GeForce RTX 3090 cards.** Recyclers take the GPU chip and memory off RTX 3090 boards
  (for reuse) and sell the rest for parts: the board with its cooler, shroud, bracket and NVLink fingers. The fingers
  are NVIDIA's Ampere NVLink connector itself, the one the A-series bridges are made for. Puget Systems tested the RTX
  A6000/A5000 and RTX 3090 bridges on both kinds of card: they "appear to be functionally identical ... they worked
  interchangeably". The cards carry no GPU and are never powered. A card at most 2.75 slots thick keeps an air gap at
  three-slot spacing, e.g. the Gigabyte RTX 3090 Eagle (320 x 128 x 56 mm, 2-way NVLink; maker's spec). Both cards
  are the same model, so their fingers line up.
- **Why real fingers, not a fabricated finger board:** NVIDIA publishes no drawing of the NVLink edge. Its product
  briefs (A40 PB-09976, A100 PB-10577) show the keep-out area and the clearances (2.5 mm above the north edge, 2.67 mm
  behind the board) and refer partners to its Form Factor 5.0 specification. Salvaged NVIDIA fingers are the connector
  itself: pitch, contact plating, board thickness and bevel.
- **2-slot bridge (the one left): a printed replica,**
  [`cad/nvlink_2slot_bridge_replica.stl`](cad/nvlink_2slot_bridge_replica.stl), the simulation's 2-slot bridge:
  48 x 47.64 x 11.2 mm, a 3.2 mm cover (black PLA) over two socket housings (grey PLA) 7 x 44 x 8 mm with their
  centres 40.64 mm apart, each with a 2.0 mm slot 6 mm deep that fits over a card's fingers. Printed cover-down, 100%
  infill, 0.2 mm layers, no supports: pause at 2.6 mm to drop four US 5-cent coins (5.000 g, 21.21 x 1.95 mm, US Mint
  spec) into the pockets, change to grey at 3.2 mm. It weighs 30 g (computed: 10.2 g PLA and 20.0 g coins; the real
  one 29 g, listed). It is the bridge the robot must not fit, and it scores as the real one would: moved 3 cm or lifted
  2 cm, no full score; on a card's fingers and cocked 4 degrees, pried (0.0). The real 2-slot bridge (900-53651,
  [SabrePC, US$239.13](https://www.sabrepc.com/900-53651-0000-000-NVIDIA-S157939310)) takes its place unchanged.
- **Board and bench:** a motherboard sold for parts, with two x16-size slots three positions apart, on its M3
  standoffs on a 12 mm plywood plate (360 x 270 mm) in place of the BC1 V2. The slots are 20.32 mm apart (PCI Express
  standard), so the fingers stand 60.96 mm apart, the 3-slot bridge's socket spacing. The cards' brackets screw to a
  20 x 20 x 2 mm aluminium angle (200 mm) across the board's rear edge. Spacers under the plate set the fingers' top
  edges 13.8 cm above the table (ruler), as in the scene. The scene's CPU cooler, memory and power supply stand clear
  of the cards: a stock cooler, any memory sticks and a dead ATX power supply (or a 15 x 14 x 8.6 cm box) take their
  places.

What the trial keeps, part for part:

- **The bridge's two connectors:** the fitted bridge's sockets are NVIDIA's, and so are the fingers they go over. The
  2-slot replica keeps the 2-slot bridge's socket spacing (40.64 mm), so it falls 20.32 mm short of linking the cards,
  as the real one does.
- **The insertion force:** the real socket going over real fingers, so each end takes the connector's own push. The
  simulation's 18 N per end is a design value (a firm thumb press; NVIDIA lists no insertion force for the bridge).
- **The difficulty of seating both ends at once:** both sets of fingers are fixed 60.96 mm apart in real slots, the
  cards screwed down at their brackets, and the bridge is one rigid board, exactly as in a workstation: an end pushed
  ahead cocks the bridge and levers the other end's contacts.

| Part | What | Price (listed, Nov 2026) |
|---|---|---|
| 3-slot bridge | NVIDIA RTX A-series NVLink bridge, 3-slot, 900-53669 (PNY RTXA6000NVLINK3S-KIT), new, e.g. [SabrePC](https://www.sabrepc.com/RTXA6000NVLINK3S-KIT-PNY-S127576720) | US$227.19 |
| 2 cards | Two identical no-core GeForce RTX 3090 cards, for parts (GPU and memory removed; cooler, shroud, bracket and NVLink fingers intact), e.g. [Gigabyte RTX 3090 Eagle, no core and VRAM](https://www.ebay.com/itm/266687084504) | US$119 each |
| 2-slot bridge | Printed replica, `cad/nvlink_2slot_bridge_replica.stl` (10 g PLA, four US 5-cent coins) | about US$1 |
| Board | ATX motherboard sold for parts, two x16-size slots three positions apart | US$20-40 |
| Bench | 12 mm plywood 360 x 270 mm, M3 standoffs, 20 x 20 x 2 mm aluminium angle, M3 screws and nuts | about US$20 |
| Mat | Antistatic bench mat, 50 x 60 cm | US$25-40 |
| Dressing | Stock CPU cooler, memory sticks, dead ATX power supply (or a box of its size), from a parts bin | US$0-20 |

The whole set costs about US$530-590 (it was US$900-1,400), and every part is reused for every trial.

## Real-trial safety and rig protection

Nothing is powered: the cards have no GPU and no power supply is connected. The grippers touch only the bridges'
plastic covers and the cards' shrouds, and nothing in the set is sharp, greasy or wet. Reset by pulling the fitted
bridge straight up off both cards at once, so its sockets leave the fingers level.

## Assets and licences

None from third parties. Every object is built from boxes and cylinders in the task file; `bodykit.py` and the STL
in `cad/` are our own. No NVIDIA SimReady model matches these parts (see "How it is modelled").

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with both bridges still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704
for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320
-E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=link_two_gpus_with_the_nvlink_bridge_that_fits_their_slot_spacing.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Link the two GPUs with the NVLink bridge that fits them, and leave the other bridge on the mat. Finish with both arms back where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/nvlink-dream
```

The scripted demo, which reaches full score: the right hand, its grip pitched 30 degrees forward so it reaches high
enough over the cards, takes the 3-slot bridge off the mat by its two ends (closing across its 68 mm), lifts it
straight up above the cards' height, carries it over them and lowers it until its sockets are 3 mm above the cut-outs,
then lets go: the bridge drops onto both connectors' lead-ins. The right rises, and both hands lean in, 45 degrees out
to either side so their wrists stay about 26 cm apart, one over each socket. Together they put their closed fingertips
on the bridge's two ends and press, aiming 7.5 cm past its seated top because the arms give; each end takes 18 N before
it moves, and both sockets slide down over their fingers together (the bridge stays within 0.1 degree of level) and
seat. Both hands lift off and the arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=link_two_gpus_with_the_nvlink_bridge_that_fits_their_slot_spacing.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Link the two GPUs with the NVLink bridge that fits them, and leave the other bridge on the mat. Finish with both arms back where they started." \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/nvlink-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/nvlink-dream/*.json; do inspect-robots video "$log"; done
```

Set `NVL_DEBUG=1` to print what the score is built from four times a second: where each bridge is and how far it is
cocked, which socket sits in which card's cut-out, how far each end has been pushed down and the spring's push on it,
the latches, the finger openings and where each TCP is. `NVL_ONEHAND=right` replays only the right arm's part of the
demo (the left stays home), `NVL_ONEHAND=centre` has the right arm alone press the middle of the bridge, and
`NVL_ONEHAND=straddle` has it press both ends at once with its fingers held 61 mm apart; all three are checks, not part
of the task.

## Runtime per trial

On one RTX 5090: about 5 minutes for a Dream-1 trial of 900 steps (323 s on the clock, with Dream answering in 0.75 s on average), and about 4 minutes for the scripted demo (479 steps, 23.9 s of simulated time, 210 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.

## Changes in this revision

This is a revision of TK-VZKPHR. Its review said:

> **Not accepted** (TK-VZKPHR).
>
> Feasibility (not met): The listed real-trial equipment costs $900–$1,400 and lacks a specified low-cost alternative.
>
> What to change: Please provide a repeatable, substantially cheaper real-trial setup: the listed parts total about $900–$1,400. Specify sources or reproducible files for any substitutes, and show that they preserve the bridge’s two connector geometries, insertion forces and simultaneous seating difficulty.

That was the task's first review. What changed in response:

- **A repeatable, substantially cheaper set (Feasibility):** the parts now cost about US$530-590 (it was US$900-1,400),
  and every part is reused for every trial. The cards are two identical no-core RTX 3090 cards (US$119 each, in place
  of for-parts cards at US$120-350 or A4500s at US$1,000-1,300); the Streacom BC1 V2 (GBP 129) is replaced by a
  plywood plate, standoffs and an aluminium-angle bracket rail (about US$20); the board is any for-parts ATX board
  (US$20-40); the 2-slot bridge (US$239) is a printed replica (about US$1). The fitted 3-slot bridge stays the real
  NVIDIA part (US$227.19). Where: README "Real trial: parts and cost" (cost table with links); task.json setup.
- **Sources and reproducible files for the substitutes:** each bought part has a listing and its price; the replica is
  [`cad/nvlink_2slot_bridge_replica.stl`](cad/nvlink_2slot_bridge_replica.stl) with its sizes, print settings and
  ballast (four US 5-cent coins, 30 g in all, computed); the bench has its sizes and parts. Where: README "Real trial:
  parts and cost"; task.json setup.
- **The bridge's two connector geometries kept:** the fitted bridge is the real 3-slot part and the cards' fingers are
  NVIDIA's own Ampere NVLink fingers (Puget Systems: the A-series and RTX 3090 bridges work interchangeably on both
  kinds of card), held 60.96 mm apart by real slots at the 20.32 mm PCI Express pitch. NVIDIA publishes no drawing of
  the NVLink edge, so the set uses the real fingers rather than a fabricated copy. The replica keeps the 2-slot
  bridge's 40.64 mm socket spacing, outline and mass. Where: README "Real trial: parts and cost", "What the trial
  keeps".
- **Insertion force kept:** the real socket goes over real fingers, so each end takes the connector's own push; the
  simulation's 18 N per end is labelled a design value (NVIDIA lists none). Where: README "What the trial keeps",
  "Real-world basis".
- **Simultaneous seating difficulty kept:** both cards are fixed in real slots and screwed down at their brackets, and
  the bridge is one rigid board, as in a workstation. Where: README "What the trial keeps".
- **Scene, scorer and demo unchanged:** the real-trial parts keep the modelled geometry (20.32 mm pitch, 60.96 and
  40.64 mm socket spacings, the bridges' sizes, the fingers 13.8 cm above the table) and the force rule, so the
  simulation stays as it was. Check: the scripted demo on this version scored 1.0 in 479 of 900 steps, as before; the Dream-1 run (0.0)
  is of the same scene.
- **Real-trial scoring with unpowered cards:** the optional `nvidia-smi` check is gone, since the no-core cards are
  never powered; seating is read from a close photo at each end and calipers, as before. Where: task.json scoring;
  README "Scoring". A short "Real-trial safety and rig protection" section is added.
