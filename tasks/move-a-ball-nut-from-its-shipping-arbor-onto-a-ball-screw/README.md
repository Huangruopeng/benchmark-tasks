# Move a ball nut from its shipping arbor onto a ball screw

A HIWIN rolled ball screw, 16 mm across with a 10 mm lead, cut to 400 mm with both ends machined for BK12/BF12
supports, lies loose in a matched pair of V-blocks on a granite surface plate. Its floating-side journal (10 mm across,
11 mm long) points toward the left arm, past the near V-block. Its ball nut, a HIWIN R16-10K3-FSCDIN, lies beside it in
its open box, still on the white plastic assembly tube (the arbor) it ships on. The tube keeps the nut's balls in
until the nut is on the screw. A red pen line on the thread marks 50 mm from the thread start, and a white lint-free
cloth lies under the screw's end. The job is the maker's transfer: put the tube's bore over the journal and butt it
against the thread start, slide the nut up to the thread, screw it on past the pen line, then take the tube off and
put it back in its box. The task is one task file,
[`move_a_ball_nut_from_its_shipping_arbor_onto_a_ball_screw.py`](move_a_ball_nut_from_its_shipping_arbor_onto_a_ball_screw.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

What it tests:

- **Blind coaxial alignment.** The tube's 10.5 mm bore must go over the 10 mm journal and butt against the thread
  start. The journal disappears inside the tube, so the last millimetres are done by feel.
- **Keeping the joint closed while seating another part.** While any of the nut's ball track is between the tube and
  the thread, the tube must stay butted and in line. If they part, the balls drop out and the trial scores 0.
- **A constrained order.** The tube comes off only once the nut is fully on the thread.
- **Precise turning under a hold.** The nut goes on five and a half turns, in wrist strokes with a regrip after each,
  while the other hand keeps the loose screw from turning or sliding.

## The real procedure

HIWIN's "Assembly Instructions Ballscrews" (BS-04-3-EN, section 4.2.2, "Assembling nut on ballscrew shaft",
pp. 13-14; [PDF](https://damencnc.com/userdata/file/HIWIN_Ballscrew_Assembly_and_Maintenance_Instructions_2024_BS-04-1-EN-1812-MA.PDF))
says:

> Unassembled ballscrew nuts are usually supplied on an assembly tube, which can be used for correct assembly.

> Place assembly tube on face end of ballscrew shaft. This makes it easier for the nut to be aligned with the shaft.
> During assembly, the nut must be aligned concentrically and flush with the shaft.

> Slide nut to start of thread and screw onto ballscrew shaft with a little pressure and a rotating movement.

> The nut must be fully on the thread before the assembly tube is removed.

Section 4 (p. 11) gives the failure: "Even if the ballscrew nut only runs off the ballscrew shaft's thread at some
points, there is a risk of balls coming out of the ballscrew nut and the function no longer being guaranteed."
HIWIN makes the tube "0.1 – 0.2 mm smaller than the core diameter of the ballscrew" and "around 20 mm longer than the
nut".

Ball screws drive the linear axes of gantry and Cartesian robots, robot seventh axes and the linear actuators of
robot arms. Builders of these axes buy rolled screws as cut-to-length shafts with machined ends, and the nuts
separately on their tubes. HIWIN: "Ballscrews are delivered either fully assembled or with ballscrew nuts and shafts
supplied separately."

## The parts (all real, at real size)

- **Ball screw.** HIWIN T7 rolled ballscrew, 16 mm, 10 mm lead (outer 14.7 mm, core 12.5 mm), art. 5020 at DamenCNC,
  €114 per metre plus €5 cutting. It is cut to 400 mm and machined with a BK12 fixed end (€75: a 12 mm bearing seat
  with an M12 x 1 thread, then a 10 mm drive spigot, 53 mm in all) and a BF12 floating end (€35: a 10 mm journal,
  11 mm long, with a snap-ring groove; DamenCNC drawings 8112 and 8212). Mass 0.43 kg. The thread is 336 mm long. It
  ships preserved with grease.
- **Ball nut.** HIWIN R16-10K3-FSCDIN, €177 at DamenCNC (HIWIN catalogue BS-09-2-EN, table 4.4). Flange 48 mm across
  and 10 mm thick, with a flat at 40 mm and six 5.5 mm holes on a 38 mm circle. Body 28 mm, 45 mm long overall,
  0.20 kg. It carries 3.175 mm (1/8 in) balls, the size HIWIN's rolled 16 x 10 nuts use.
- **Assembly tube.** White plastic, 12.3 mm outside (0.2 mm under the core, HIWIN's rule), 10.5 mm bore, 55 mm long
  including a 20 mm collar 3 mm long at its rear. Its bore's mouth is chamfered. The nut sits against the collar,
  7 mm back from the tube's front end.
- **V-blocks.** A matched pair of hardened steel V-blocks, 124 mm across, 89 mm tall and 70 mm along the V, with a
  90-degree V 54 mm wide (Penn Tool 57-060-004, $249.95 the pair). The screw lies in them loose, as a finished ball
  screw is laid down: its hardened raceway is not gripped in a vise.
- **Surface plate.** Black granite, 12 x 18 x 3 in, grade B (Standridge B0-12x18x3, $219). Any rigid flat block
  76 mm thick will do.
- **Box.** The nut's open cardboard box, 90 x 82 x 20 mm, with a grey foam insert. Ridges under the flange, the tube's
  front end and its collar hold the nut level with its body free.
- **Cloth.** A white lint-free wiping cloth, folded flat under the screw's end. It catches any ball that falls.

## Why two hands

The screw lies loose. It rolls in its V-blocks with about 0.01 N m and slides along them with about 1.3 N (computed
from its weight with the nut on it, and oiled steel at friction 0.15). The nut's wipers and grease drag about 0.04 N m
against the screw (design value). Turned with nothing holding the screw, the nut takes the screw round with it and
never threads on. One hand holds the screw while the other aligns the tube and turns the nut.

The one-hand check, measured: the left hand did exactly its part of the demo and the right never moved. The trial
scored 0.
- Pushing the tube on, and following the strokes, slid the screw 59 mm along its V-blocks.
- The nut caught at the thread start, but in ten strokes it went on only 0.3 mm: the screw turned with it.
- Drawing the tube off then lifted the screw out of its V-blocks, and it fell.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The plate, the V-blocks, the cloth and the box are
kinematic, fixed where they are put. The screw, the nut and the tube are loose rigid bodies made of shapes from
`bodykit.py`, at their real sizes.

- **The tube on the journal.** This is real contact. The tube's wall is twelve staves, so the journal can go into its
  bore, and its mouth is chamfered at 45 degrees. The journal's tip is stepped down 0.8 mm. The tube only reaches the
  thread start once its bore is over the journal.
- **The nut on the tube.** A joint keeps the nut on the tube's axis and lets it spin freely and slide along it. The
  wipers' drag on the tube is a spring that holds the nut where it is left. It slips at about 1 N (design value), so
  the nut stays put while carried level. The tube's collar stops the nut at the rear. At the front nothing stops it:
  a nut pushed more than 4 mm past the tube's front end with no thread there has lost its first balls.
- **The thread.** A hidden 60 g carrier sits inside the nut (the nut's shell is 0.14 kg, so the whole nut is 0.20 kg).
  The nut spins on the carrier against the thread's drag: a stick-slip spring capped at 0.04 N m. The thread catches
  once the nut's front end reaches the thread start with the tube butted and in line. A slide joint then holds the
  carrier on the screw's axis, 10 mm along it per turn of the nut against the screw, so the nut goes on only as it
  turns. Once the nut's rear end passes the thread start, it is off the tube and the tube's joint lets go.
- **Ball loss (a rule).** The rule latches a score of 0 if, while any of the nut's ball track lies between the tube
  and the thread (from the catch until the nut's rear end passes the thread start), any of these happens:
  - the tube's front end moves more than one ball diameter (3.175 mm) from the thread start;
  - its axis moves more than half a ball (1.6 mm) off the screw's axis;
  - it tilts more than 5 degrees.

  It also latches 0 if the nut runs off the tube's front end with no thread there, or is screwed back off the thread
  with no tube butted. The balls are not simulated: a spill drops eight steel balls onto whatever is below.
- The screw and the V-blocks are lightly oiled steel (friction 0.15, handbook value).
- The tube is modelled at 10 g (it weighs about 3 g) so that its joint to the nut stays stable. The flange's flat is
  not modelled: the flange is round.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose, and it lets the task file act on the simulation after every control step.

## Scoring

Latched failures score 0.0 for the rest of the trial:

- the balls were lost (the rule above);
- the screw fell off its V-blocks onto the plate or the bench;
- the nut fell onto the plate, the cloth or the bench.

Otherwise the score is judged at the end of the trial, on live states:

- **1.0:** the nut's front end is at or past the pen line (50 mm onto the thread, so its rear end is 5 mm past the
  thread start), the tube is off the screw and lying in its box, the screw is lying in both V-blocks, and both arms
  are back where they started (each gripper's fingertip centre within 5 cm of its start).
- **0.75:** the nut is at or past the pen line.
- **0.5:** the nut has been screwed at least one full turn onto the thread (its front end 10 mm past the thread start).
- **0.25:** the nut has caught at the thread start, with the tube butted against the screw's end.
- **0.0:** anything else.

The trial ends at the run limit, or once the full score has held for 1 s with the screw, the nut and the tube still.

In a real trial:

- Lost balls: count the balls on the cloth, and watch the video for a nut that jams or that ran off its tube. Any ball
  means 0.
- The gap and the alignment while the nut crosses: the video. The tube must stay butted against the thread start.
- The nut's position: a ruler from the thread start (the tube's end) to the nut's front end, or the pen line.
- The tube: in its box at the end.
- The arms: checked against marks at their start.

## Rig safety

The screw ships preserved with grease. Before each trial, wipe the thread and the journal with a lint-free cloth
moistened with white spirit (HIWIN p. 11 names "petroleum, thin oil or white spirit"), then wipe them with a clean
cloth, leaving only a thin oil film. Fit the gripper pads with nitrile finger cots (Tech-Med, 144 for $21.95) and
change them every ten trials. The grippers touch only the nut's outside, the tube and the wiped screw. The nut's
grease stays inside it.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` has held for 1 s, otherwise `score()` at the end of the episode
(0, 0.25, 0.5, 0.75 or 1.0). `success()` is the full score with the screw, the nut and the tube still.

`-E object_jitter=0` keeps every object exactly where the task file puts it. The cameras render at 1280 x 704 for
sharp videos. Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=move_a_ball_nut_from_its_shipping_arbor_onto_a_ball_screw.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Move the ball nut from its tube onto the screw, its front end past the red pen line, and put the tube back in its box. Finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ballnut-dream
```

The scripted demo, which reaches full score:

1. The right hand holds the screw by its thread beyond the far V-block.
2. The left hand takes the nut by its body from behind, its gripper pointing along the screw. It lifts the nut out of
   the box level and carries it to the screw's end.
3. It reads where the tube's front end is (scripted demos may read the simulator's state) and corrects twice, 6 mm and
   then 2 mm out, so the bore lines up with the journal.
4. It pushes the tube over the journal until it butts, and on until the nut has slid along the tube to the thread
   start.
5. It screws the nut on in strokes of 220 degrees of wrist turn (about 5.8 mm each). After each stroke it lets go,
   turns back and takes the nut again where it now is, following it along the screw. It does this until the nut's
   front end is past the pen line.
6. It takes the tube by its rear end, draws it off the journal and lays it in the box.
7. The right lets go of the screw, and both arms go home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=move_a_ball_nut_from_its_shipping_arbor_onto_a_ball_screw.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Move the ball nut from its tube onto the screw, its front end past the red pen line, and put the tube back in its box. Finish with both arms back where they started." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/ballnut-demo
```

`BN_ONE_HAND=1` makes the same demo with the right hand never moving, for the one-hand check.

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/ballnut-dream/*.json; do inspect-robots video "$log"; done
```

Set `BN_DEBUG=1` to print what the score is built from four times a second:

- the phase (on the tube, threading, on) and any loss;
- how far the nut is onto the thread, the thread's target and the turns;
- the tube's gap, offset and tilt at the joint;
- the poses of the screw, the nut and the tube;
- both TCPs and the finger openings;
- the drop flags, and whether the tube is in its box and the screw in its V-blocks.

## Runtime per trial

On one RTX 5090: about 8 minutes for a Dream-1 trial of 2,400 steps (497 s on the clock, with Dream answering in 1.27 s on average), and about 6 minutes for the scripted demo (1,319 steps, 66.0 s of simulated time, 357 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed. With `-E object_jitter=0` the layout is fixed. The scripted demo follows fixed waypoints, plus corrections
it reads from the simulator's state at the moments it lines the tube up and takes the nut again.

## Sources

- HIWIN, "Assembly Instructions Ballscrews" (procedure, tube rule, cleaning):
  https://damencnc.com/userdata/file/HIWIN_Ballscrew_Assembly_and_Maintenance_Instructions_2024_BS-04-1-EN-1812-MA.PDF
- HIWIN ballscrew catalogue BS-09-2-EN (R16-10K3-FSCDIN dimensions and mass):
  https://www.mmintercom.com/upload/publish/411/hiwin-ballscrews_68fb1c834c2c9.pdf
- HIWIN 16 x 10 rolled shaft, art. 5020:
  https://www.damencnc.com/en/hiwin-t7-ballscrew-16mm-diameter-10mm-pitch-for-fscdin-ballnut-price-m/a3341
- R16-10K3-FSCDIN nut: https://damencnc.com/en/hiwin-ballnut-r16-10k3-fscdin/a3342
- BK12 and BF12 end machining: https://damencnc.com/en/bk-fk-12-ballscrew-end-machining-fixed-side/a3754 and
  https://damencnc.com/en/bf-ff-12-ballscrew-end-machining-floating-side/a3586 (drawings 8112 and 8212 on those pages)
- V-blocks: https://www.penntoolco.com/precise-precision-v-blocks-clamps-set-matched-pair-1-4-2-1-8-capacity-4-7-8-x-3-1-2-x-2-3-4-57-060-004/
- Surface plate: https://www.judgetool.com/products/standridge-black-granite-surface-plates-grades-aa-a-b-1/
- Finger cots: https://firstaidsuppliesonline.com/first-aid-products/exam-gloves/finger-cots/finger-cot-nitrile-large-144-box/

## Assets

No third-party assets: every object is built from boxes, cylinders and spheres in the task file. No SimReady model of
a ball screw, ball nut or V-block exists in nvidia/simready-assets.
