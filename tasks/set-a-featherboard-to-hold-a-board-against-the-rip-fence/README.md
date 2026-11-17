# Set a featherboard in the mitre slot to hold a board against the rip fence

A 10 in contractor table saw is set up for a rip: its blade is wound fully down under the throat plate, its paddle
switch is off and its cord is unplugged, and its rip fence is locked 100 mm from the blade. A 1x6 pine board lies on
the top beside the fence, 15 mm off it, its front end 1 in short of the blade. A mitre-slot featherboard (an orange
polymer body with a row of flexible fingers and a shorter setup feather, on an aluminium expanding bar in the left
mitre slot, held by two T-knobs) sits loose in the slot, slid up beside the blade where it was left. The job is the
whole real setup: get the board flat against the fence, bring the featherboard back so it is entirely ahead of the
blade, press it in to the maker's tension (the setup feather just touching the board's edge), lock both knobs at that
tension, and bring both arms back. The task is one task file,
[`set_a_featherboard_to_hold_a_board_against_the_rip_fence.py`](set_a_featherboard_to_hold_a_board_against_the_rip_fence.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

## What it tests

- **A spring preload set to a reference by feel.** The fingers are springs and so is the setup feather. Pressed in
  until the setup feather just touches the board's edge, the fingers are flexed 3 mm and press with 10.5 N, the
  maker's tension. Full score needs the fingers flexed 2 to 4 mm (about 7 to 14 N): pressed too little, the fingers
  barely touch the board; pressed too hard, the setup feather bends with them and the board would bind in the feed.
  The arm has to stop on contact with a light, springy part and hold a force inside that window.
- **Keeping that preload through the locking.** With its knobs loose the featherboard slides on its bar and in the
  slot against only 1.5 N, so the fingers' 10.5 N pushes it straight back off the board the moment it is let go. It
  stays set only if it is held in at the preload while a knob is tightened (the makers' instruction: "Place slight,
  equal pressure on the featherboard toward the stock and fence and secure the knobs").
- **A position measured from the blade.** The featherboard starts beside the blade, where it would press the cut
  pieces against the rear teeth. It has to go entirely ahead of the blade, its leading finger 1 to 7.5 cm in front of
  the blade slot (the board's end is 2.5 cm in front of it).
- **The board left flat against the fence.** The board starts 15 mm off the fence; it has to end flat against it
  (1 mm feeler at both ends), still on its mark and short of the blade.

Why two hands: holding the featherboard in at the preload and turning a knob are two hands' work. The knobs sit on
bolts anchored in the bar, so a hand turning a knob pushes on the bar, not on the body, and cannot hold the fingers'
preload. With one hand (the left doing exactly its part of the demo, the right never moving; `FEATHER_ONE_HAND=1`) the
score was 0.5: the left slid the featherboard ahead of the blade and pressed it in until the setup feather touched (fingers
flexed 2.95 mm, which also pushed the board flat against the fence), but once it let go to reach a knob, the
featherboard sprang back until the fingers were flexed only 0.66 mm, and the front knob then locked it there (0.64
mm, the setup feather 2.4 mm off the board); the rear knob, the right hand's part, stayed loose.

## The real tools and the job

- **Saw.** A 10 in contractor saw such as the Delta 36-725T2 (cast-iron top 27.13 in front to back, maker's spec, with
  steel extension wings; T-square rip fence). The modelled top is 27 in (686 mm) front to back, 85 cm above the floor
  (most cabinet and contractor saws are 86-91 cm). The arms stand on a 76 cm bench, so the top stands 9 cm above it;
  the saw's stand is drawn as a short cabinet. Standard 3/4 x 3/8 in (19.05 x 9.5 mm) mitre slot, 125 mm left of the
  blade (design value).
- **Featherboard.** Modelled on the Bench Dog Single Feather-Loc (Rockler 29799, part 10-005, 0.65 lb, maker's spec):
  orange polymer body, ergonomic T-knobs, a built-in setup feather that "makes setting the tension fast, easy and
  repeatable", and the maker's mitre slot adapter, a 7-1/2 in aluminium bar for 3/4 x 3/8 in slots on 1/4-20 bolts
  (Bench Dog 10-012). The body's size (27 x 14.5 cm, 1.7 cm deep), its 14 fingers and the 3 mm setup-feather offset
  are scaled from the maker's photos by the 7-1/2 in bar. The fingers' stiffness (3,500 N/m for the row, 10.5 N at
  the setup feather's 3 mm) and the setup feather's (300 N/m, one finger of the row, a little stiffer for being
  shorter) are design values for the light pressure the makers describe.
- **Board.** A 1x6 pine board (19 x 140 mm), 18 in long, 0.55 kg (eastern white pine, about 450 kg/m3: computed).
- **Procedure.** Grizzly's table-saw manuals: "Place the workpiece evenly against the fence, making sure it is 1 in.
  in front of the blade", "slide the featherboard miter bar into the miter slot", "Slide the featherboard toward the
  blade until the first finger is nearly even with the end of the workpiece", "Then secure the featherboard to the
  table". Woodsmith: "The featherboard should be positioned entirely ahead of the saw blade"; with a setup finger,
  "position it against the workpiece so the shorter finger just touches the edge. As the workpiece passes the
  remaining fingers, they flex slightly to provide the intended pressure"; "Too little pressure won't provide much
  benefit"; "Too much pressure can make the workpiece difficult to feed and may cause it to bind." Fulton's
  featherboard instructions (Peachtree 11550): "disconnect machine from power source before making any adjustments"
  and "Place slight, equal pressure on the featherboard toward the stock and fence and secure the knobs."
- **Who does it.** Carpenters (O*NET 47-2031.00, 889,700 US workers in 2025): core task "Shape or cut materials to
  specified measurements, using hand tools, machines, or power saws"; O*NET's tools list for the occupation includes
  table saws and push sticks. Cabinetmakers and bench carpenters (O*NET 51-7011.00, 81,800): "Set up or operate
  machines, including power saws, jointers, mortisers, tenoners, molders, or shapers"; their tools list includes
  table saws.

Sources: [Rockler: Bench Dog Single
Feather-Loc](https://www.rockler.com/bench-dog-reg-single-feather-loc-reg-multi-purpose-featherboard), [Rockler: Bench
Dog Miter Slot Adapter 10-012](https://www.rockler.com/bench-dogreg-miter-slot-adapter-10-012), [Woodsmith:
Featherboard basics](https://www.woodsmith.com/article/featherboard-basics/), [Grizzly G0690 manual,
featherboards](https://www.manualslib.com/manual/1044096/Grizzly-G0690.html?page=54), [Grizzly G0661 manual,
featherboards](https://www.manualslib.com/manual/1964717/Grizzly-G0661.html?page=60), [Peachtree 11550 featherboard
instructions](https://www.ptreeusa.com/PDF/11550%20Deluxe%20Featherboard%20Single.pdf), [Delta 36-725T2 instruction
manual](https://pdf.lowes.com/productdocuments/e8f42fe9-5915-41ca-a3a8-e87749ffb311/47969393.pdf), [O*NET
47-2031.00](https://www.onetonline.org/link/summary/47-2031.00), [O*NET
51-7011.00](https://www.onetonline.org/link/summary/51-7011.00), O*NET 30.1 database, "Tools Used" table.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The saw (top, mitre slot, throat plate, locked rip
fence) is one kinematic body built from boxes at real sizes; the board, the featherboard's bar, its body, its finger
row, its setup feather and its two knobs are loose rigid bodies made of shapes from `bodykit.py`, at real sizes and
masses (bar 68 g, body 160 g, fingers 30 g, setup feather 10 g, knobs 25 g each: 0.32 kg in all). No third-party
assets are used: everything is built from primitives in the task file.

- **Bar in the slot.** The bar is held in the mitre slot by a prismatic joint along it. While the rear knob is loose,
  the joint's drive is a spring capped at 1.5 N whose anchor the task file moves to follow the bar once it is pushed
  past what that holds (stick-slip friction): the bar slides along the slot against 1.5 N and stays where it is left.
- **Body on the bar.** The body slides across the bar (along its two knob slots, 3.5 cm of travel) on a second
  prismatic joint, also against 1.5 N while both knobs are loose. Its turn about the bar is locked.
- **Fingers.** The 14 fingers are one body on a sprung prismatic joint in the body (3,500 N/m, damped): pushed back by
  the board's edge, they press it toward the fence. They lean 30 degrees toward the blade and collide along their
  tips.
- **Setup feather.** The setup feather, 3 mm shorter, is a body of its own on a second sprung joint in the body
  (300 N/m). Until it touches the board it carries no load; pressed past touching, it bends with the fingers, so the
  preload keeps rising with the push (3.8 N for each further mm).
- **Knobs.** Each T-knob turns on a revolute joint on the bar (its bolt). It is held by a capped spring whose anchor
  the task file moves (stick-slip): it turns against 0.03 N m while loose and 0.3 N m once snug (40 degrees on), is
  tight from 75 degrees and stops at 100, finger-tight (turned back past 70 it is loose again). A tight knob clamps
  the body to the bar (its slide is then held by a stiff spring capped at 300 N); the rear knob, whose flat-head bolt
  spreads the bar's split end, also locks the bar in the slot. The knobs' wings (58 x 14 mm) are gripped across, so a
  wrist turn drives them like a T-handle.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Scoring

All judged at the end of the trial, on live states, never latched.

- **1.0:** the featherboard ahead of the blade (its leading finger tip 1 to 7.5 cm in front of the front end of the
  throat plate's blade slot) with its fingers flexed 2 to 4 mm against the board's edge (about 7 to 14 N: the setup
  feather within 1 mm of the board's edge, or bent by it at most 1 mm); the board flat, its edge against the fence
  (gap 1 mm or less at the saw's front edge and at the board's front end), its front end still short of the blade
  slot and within 5 cm of where it lay; both knobs tight; and both arms back where they started (each gripper's
  fingertip centre within 5 cm of its start).
- **0.75:** the same with an arm not back.
- **0.5:** ahead of the blade, fingers on the board's edge (flexed at least 0.2 mm), and the board against the fence
  as above, with the preload outside 2 to 4 mm or a knob not tight.
- **0.25:** ahead of the blade with its fingers on the board's edge.
- **0.0:** anything else, including a featherboard left beside the blade.

In a real trial: a ruler from the front end of the blade slot to the leading finger tip; a 1 mm (0.04 in) feeler
blade must not pass between the board and the fence at the saw's front edge or at the board's front end; the fingers
touch the board's edge (no light between); calipers from the board's edge to the body's finger-root wall, at the
setup feather, read 2 to 4 mm less than the fingers' rest length (measured once on the featherboard before the
trials); each knob's wing is within 25 degrees of its finger-tight mark, and a 20 N push (a luggage scale) along the
slot or away from the board does not move the featherboard; the board's end against a tape mark; the arms against
marks at their start.

Before the trials, once: set the featherboard, tighten each knob finger-tight and mark each wing's position on the
body (its finger-tight mark); measure the fingers' rest length (finger-root wall to finger tips) with calipers. Reset
for each trial: turn each knob back 100 degrees from its mark, slide the featherboard up beside the blade to its tape
mark and pull it back on its bar, and lay the board on its tape mark 15 mm off the fence.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the board, the bar, the body, the fingers, the setup
feather and both knobs still) has held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or
1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial
starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to
320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=set_a_featherboard_to_hold_a_board_against_the_rip_fence.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Set the featherboard so it holds this board against the fence for the rip. Finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/featherboard-dream
```

The scripted demo, which reaches full score: the right hand pushes the board's edge, midway along the part lying on
the saw, flat against the fence (aiming 1.8 cm past it, as the arm gives) and goes back. The left takes the rear
knob's wing and slides the featherboard back along the slot until its leading finger is 4.5 cm in front of the blade
slot, lets go, and pushes the body's back edge in toward the board (aiming 4.6 cm past where the setup feather
touches, as the arm gives), keeping it pressed in. The right takes the rear knob's wing and turns it clockwise (105
degrees commanded; it is tight at 75 and stops at 100): the bar spreads in the slot and the body is clamped with the
fingers flexed 2.9 mm. The right opens wide, rises clear of the wing and goes home; the left lets go of the body,
takes the front knob and turns it tight the same way, opens, rises and goes home:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=set_a_featherboard_to_hold_a_board_against_the_rip_fence.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Set the featherboard so it holds this board against the fence for the rip. Finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/featherboard-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/featherboard-dream/*.json; do inspect-robots video "$log"; done
```

Set `FEATHER_DEBUG=1` to print what the score is built from four times a second: each knob's angle and whether it is
tight, how far the bar has slid along the slot and the body across the bar, how far the fingers and the setup feather
are flexed, how far the leading finger is in front of the blade, the board's gaps to the fence, its pose, tilt and
front end, where each TCP is, and the score. Set `FEATHER_ONE_HAND=1` to run the demo's left-hand part alone (the
right arm stays where it started).

## Runtime per trial

On one RTX 5090: about 6 minutes for a Dream-1 trial of 2,400 steps (333 s on the clock, with Dream answering in 0.81 s on average), and about 2 minutes for the scripted demo (788 steps, 39.4 s of simulated time, 115 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
