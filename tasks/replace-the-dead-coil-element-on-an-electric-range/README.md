# Replace the dead coil element on an electric range's right front burner

The cooktop of a 30-inch plug-in coil electric range (GE/Hotpoint style). Its right front burner, the 8-inch one,
has stopped heating, and a new 8-inch plug-in element (GE WB30M2 type) lies flat on the deck between the two front
burners, terminals toward the left. The job is the owner's standard fix, done the way the makers' manuals give it:
lift the dead coil's far edge over the drip pan's lip, draw its terminals straight out of the receptacle, take it out
of the well and set it aside; then bring the new coil over the well, push its two terminal blades through the notch
in the pan into the receptacle under the deck, and lower it so it rests evenly. The task is one task file,
[`replace_the_dead_coil_element_on_an_electric_range.py`](replace_the_dead_coil_element_on_an_electric_range.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

The instruction gives the goal and its constraints only ("The right front burner's coil element is dead. Replace it
with the new one without levering the receptacle, and finish with both arms back where they started."). It says
neither how to hold the coils nor where to put the dead one.

What it tests: a blind, horizontal plug-in. The receptacle sits under the deck's edge, out of sight of every camera;
its two channels open 3 to 13.5 mm below the deck surface, toward the well. The blades only go in held at that
height and pushed straight along the channels. Meanwhile the coil's far edge has to be held up over the pan's raised
lip: until the blades are home the coil sits 24 mm short of its seat, where its outer turn would land on the lip.
Once the blades are home, the far edge is lowered and the coil rests flat on its feet. The order is fixed by the
physics: the dead coil fills the well and the receptacle until it is out.

Why two hands: the coil is 19 cm across and weighs 0.45 kg, and the only stiff thing to grip near its terminals is
the 25 mm steel terminal bracket at its edge. Held there alone, its weight acts about 10 cm from the grip (0.44 N m,
computed); in the simulator a coil lifted by its bracket alone pitched 10 to 35 degrees in the grip, its blades
pointing up and away from the channels. Whirlpool's instructions say to hold the coil element as level as possible
while pushing the terminal in, and a repair guide tells people to grab the coiled element firmly with both hands to
pull it out. In the demo the left hand holds the terminal bracket (it pushes and pulls the blades) and the right
hand pinches the coil's outer turn at the far rim (it sets the far edge's height); the wrists stay about 20 cm
apart. With one hand the job fails. The left arm alone, doing exactly its part of the demo with the right at home,
drew the dead coil 14 mm out before its outer turn caught on the pan's lip; the coil slid back in and stayed plugged
(score 0.0). The right arm alone, doing its part with the left at home, lifted the dead coil's far edge and dragged
it, but the coil swung about the pinch and was left tilted 13 degrees across the pan's lip, 6 cm from the well's
centre (0.0).

## Real-world basis

- **The method (makers' manuals):** GE's owner's manual for its plug-in coil ranges (JSP28 series, "Surface Units
  and Drip Pans"): "Lift the surface unit about 1 inch above the drip pan and pull it out. Do not lift the surface
  unit more than 1 inch. If you do, it may not lie flat on the drip pan when you plug it back in. Repeated lifting of
  the surface unit more than 1 inch above the drip pan can permanently damage the receptacle." To replace: "Make sure
  opening in the pan lines up with the receptacle. Insert the terminals of the surface unit through the opening in
  the drip pan and into the receptacle. Guide the surface unit into place so it rests evenly." GE's support page
  (Electric Range - Removing Plug-In or Tilt-Lock Coil Burners) gives the same steps and ends: "Push the burner in and
  down so it rests evenly on the cooktop." Whirlpool and Amana (How to Insert the Coil Element): "Holding the coil
  element as level as possible, slowly push the coil element terminal into the receptacle. When the terminal is pushed
  into the receptacle as far as it will go, push down the edge of the coil element opposite the receptacle." Repair
  Clinic: lower the element "so the metal support rests in the notch in the drip pan". A repair guide
  (stepbystep.com, drip pans): "Grab the coiled element firmly with both hands and pull backwards and away from the
  connection."
- **Element:** GE WB30M2, the 8-inch plug-in Calrod element for GE and Hotpoint ranges, US$47.23 (AppliancePartsPros
  listing; "Simple plug-in replacement", fixes "Element will not heat"). Its replacement SUPCO CH30M2 is listed as
  8-inch, 6 turns, Y-frame support, 2350 W at 240 V and 1765 W at 208 V (Mill Supply listing). GE's 30-inch coil
  ranges put the 8-inch element at the right front and a 6-inch at the left front (retailer spec sheets of the JBS360
  and JB256 models).
- **Drip pan:** Range Kleen Style B 8-inch chrome drip pan, made for GE, Hotpoint and other plug-in ranges (retailer
  listings). Its opening lines up with the receptacle (GE manual, above), and the element's terminal end rests in it.
- **Receptacle:** GE WB17T10006 surface burner terminal block. It snaps into a metal holder under the cooktop, held
  by a single screw (AppliancePartsPros repair guide), so it does not move when a coil is pushed in or pulled out.
- **Range:** GE's 30-inch free-standing electric ranges stand 36 in (91 cm) to the cooktop and are 30 in (76.2 cm)
  wide (GE JBS160 spec sheet).
- **Sizes used in the scene:** coil 19.1 cm across its outer turn, 6 turns of 7.5 x 6.5 mm sheath, terminal bracket
  25 x 16 x 16 mm, two blades 25 mm long, 2 mm thick, 13 mm apart (design values at the part's size); 0.45 kg
  (computed: about 2.2 m of sheath plus the bracket and frame). Drip pan lip 22.4 to 24.8 cm across, its ledge 12 mm
  below the deck, bowl 30 mm deep; notch 44 mm wide (design values).
- **Societal relatability:** plug-in coil ranges are the low-cost electric range in many homes and rentals, and
  swapping a coil is the fix their owners do themselves when one burner stops heating; the drip pans come out the
  same way for cleaning. A clip of a coil being lifted out of its burner and a new one plugged in is recognised at
  once.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The range top is one kinematic body, fixed where
it is put; the two coils are loose rigid bodies made of shapes from `bodykit.py`, at their real sizes and mass.

- **Range top:** a white enamel deck 76.2 cm wide, 35 mm above the table, its front 7.5 cm from the line between the
  arm bases; the backguard with four knobs (all off) and a clock at the back. The front burners' centres are 22.5 cm
  from the front edge and 38 cm apart; the rear burners are scenery.
- **Right front well (8-inch):** a chrome drip pan: a lip ring 22.4 to 24.8 cm across, 2 mm proud of the deck, with
  a 44 mm notch toward the range's centre; inside it a ledge 12 mm below the deck that the coil's feet and terminal
  bracket stand on; then the bowl, 30 mm deep. A coil's outer turn clears the lip by 15 mm all round, so a coil can
  only come out of the well, or go into it, with its far edge lifted over the lip.
- **Receptacle:** a porcelain block under the deck at the notch, its two blade channels opening toward the well: each
  4 mm wide (the blade is 2 mm) and 10.5 mm tall (the blade is 6 mm), 25 mm deep, with 45-degree chamfers at the
  mouth (the wall between the channels comes to an edge, the outer walls are chamfered 6 mm, the floor 4 mm). The
  channels' roof is the deck's underside, 3 mm below the surface, so the blades have to come in under the deck's
  edge. Seated, the blades are 24 mm in.
- **Coils:** six concentric turns of flattened sheath (7.5 mm wide, 6.5 mm tall) out to 19.1 cm, on a Y-frame with two
  feet at its far ends and a chrome hub; the two sheath ends run out to the steel terminal bracket (25 x 16 x 16 mm),
  from which the two blades stick out 25 mm. The gap between the outer turn and the next is 8.5 mm. 0.45 kg, friction
  1.0. The dead coil's sheath is weathered grey-brown, the new one's black.
- **Left front well (6-inch):** the same kind of pan with its cold 6-inch coil plugged in (fixed).
- **Lever check:** after every control step the task file works out each coil's tilt and how far each blade is in its
  channel. Tilted 10 degrees or more (its far edge about 33 mm, 1.3 in, out of level) with a blade 3 mm or more in,
  the coil levers the receptacle: strained, and the trial fails.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

Latched for the rest of the trial: 0.0 once either coil has fallen off the range top, or has been tilted 10 degrees
or more (its far edge about 33 mm, 1.3 in, out of level) while one of its blades is 3 mm or more into the receptacle
(the receptacle levered: strained). Otherwise judged at the end of the trial, on live states:

- 1.0: the new coil seated in the right front well (both blades 20 mm or more into the receptacle, the coil level
  within 3 mm across its 19 cm and its centre within 6 mm of the well's centre), the dead coil out of that well (its
  centre 15 cm or more from the well's centre, its blades out of the receptacle) and both arms back where they started
  (each gripper's fingertip centre within 5 cm of its start);
- 0.75: the same with an arm not back;
- 0.5: the dead coil out and the new coil's blades in the receptacle (3 mm or more), not seated;
- 0.25: the dead coil out;
- else 0.0 (the dead coil still in the well or still plugged in).

The dead coil may be set down anywhere on the range top outside the right front well. A trial ends at the run limit,
or once the full score has held for 1 s with both coils still. In a real trial (range unplugged at the wall): seated
means the terminal bracket's face is within 5 mm of the receptacle's face, seen through the pan's notch (a close photo
with a ruler: blades 20 mm or more in), the coil's feet sit on the pan, its rim level within 3 mm (a straightedge laid
across the coil and a ruler at both ends), and the gap between the outer turn and the pan's lip even all round within
6 mm (a photo from above with a ruler). Blades in at all: the bracket within 22 mm of the receptacle's face (ruler).
Strained: the trial video shows a plugged-in coil's far edge lifted more than 33 mm (1.3 in) above the pan, judged
against a mark on a card at the well's far side. Fallen: the video shows a coil landing off the range top.

## Checks

Each check is the demo changed in one way (task file, `COIL_ONEHAND` or `COIL_CHECK`), run in the simulator:

| Check | What happened | Score |
|---|---|---|
| Full demo, five runs | new coil seated (blades 23.5 to 23.8 mm in, level within 0.1 mm, centred within 3 mm), dead coil on the left front burner, arms home | 1.0 each |
| Demo with the plug-in aim moved 3 mm nearer or further, or with no allowance for arm sag (three runs) | new coil seated (blades 23.5 to 23.9 mm in, centred within 3 mm) | 1.0 each |
| Left arm alone (`COIL_ONEHAND=left`) | the dead coil's outer turn caught on the pan's lip after 14 mm; it slid back in and stayed plugged | 0.0 |
| Right arm alone (`COIL_ONEHAND=right`) | the dead coil swung about the pinch and was left tilted 13 degrees across the lip, 6 cm from the well's centre | 0.0 |
| Far edge lifted 45 mm while plugged in (`strain`) | strained at 10.1 degrees, latched | 0.0 |
| New coil without taking the dead one out (`noremove`) | the new coil came to rest on the dead one, 16 mm off the well's centre; the dead one still plugged in | 0.0 |
| Dead coil taken out, then plugged back in (`putback`) | dead coil seated again; the new coil came to rest on it | 0.0 |
| New coil pushed to 8 mm short of home, then lowered (`short`) | blades 7 mm in, the coil's far side on the lip, 7.6 mm out of level | 0.5 |

The left front well holds its own plugged-in coil, so the new coil can't be plugged in at the wrong burner. A coil
let go with its far edge up drops onto the lip or its feet; it counts as seated only with its blades 20 mm or more in
and its rim level within 3 mm.

## Real trial

- **Range:** any used 30-inch GE or Hotpoint free-standing range with plug-in coil elements (8-inch right front),
  unplugged at the wall for the whole trial. Its cooktop stands 36 in (91 cm) high; the arms' platform puts their
  table 35 mm below the deck, with the range's front 7.5 cm ahead of the line between the arm bases. Equivalent: the
  cooktop assembly lifted off a scrapped range (deck, pans, receptacles and wiring in place) and set on blocks at that
  height.
- **Elements:** two GE WB30M2 8-inch elements (US$47.23 each, listed): one plugged into the right front well at the
  start, one laid on the deck. The "dead" one is any WB30M2 with an open circuit, or a new one tagged as dead; the
  trial never powers it.
- **Pans:** the range's own chrome pans (Range Kleen Style B 8-inch and 6-inch for GE/Hotpoint, if replaced).
- **Measuring:** a ruler, a 30 cm straightedge, a card marked at 33 mm stood at the right well's far side, and the
  overhead and side cameras.
- **Safety and rig protection:** the range stays unplugged, so nothing is live or hot; the elements' steel sheath and
  bracket have no sharp edges, and nothing is wet, greasy or powdered. Reset as in the reset notes: lift the new
  coil's far edge about an inch, draw it out, lay it back; plug the dead coil back in.

## Assets and licences

None from third parties. Every object is built from boxes and cylinders in the task file; `bodykit.py` is our own.
The NVIDIA SimReady set has no range, cooktop, coil element or drip pan, so everything is built from primitives at
the parts' sizes.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with both coils still) has held for 1 s, otherwise
`score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x
704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=replace_the_dead_coil_element_on_an_electric_range.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "The right front burner's coil element is dead. Replace it with the new one without levering the receptacle, and finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/coil-dream
```

The scripted demo, which reaches full score: both hands take the dead coil, the left by its terminal bracket
(fingers closing across it), the right by its outer turn at the far rim (fingers closing radially, the inner one in
the gap to the next turn, pitched 25 degrees forward to reach). The right lifts the far edge 18 mm over the pan's lip
(the coil tilts about 5 degrees about its blades), both draw it 30 mm straight away from the receptacle until the
blades are out, lift it out of the well level and set it down on the cold left front burner. Then they take the new
coil off the deck the same way, carry it over the right well with its far edge up, lower its blades to the
receptacle's mouth, and the left pushes them straight in, aiming 6 mm further forward and 12 mm further in than
the seat because the arms fall short under the load, while the right follows; with the blades 23 mm in, the right
lowers the far edge and the coil drops flat onto its feet. Both let go and the arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=replace_the_dead_coil_element_on_an_electric_range.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "The right front burner's coil element is dead. Replace it with the new one without levering the receptacle, and finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/coil-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/coil-dream/*.json; do inspect-robots video "$log"; done
```

Set `COIL_DEBUG=1` to print what the score is built from four times a second: each coil's position, tilt, level across
its rim, distance from the right well's centre and how far each blade is in its channel; the latches; the finger
openings and where each TCP is. `COIL_ONEHAND=left` (or `right`) replays only that arm's part of the demo (the other
stays home). `COIL_CHECK=strain`, `noremove`, `putback` and `short` run the scoring checks listed under "Checks". All
of these are checks, not part of the task.

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 2,400 steps (343 s on the clock, with Dream answering in 0.78 s on average), and about 2 minutes for the scripted demo (669 steps, 33.5 s of simulated time, 118 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
