# Load a flask onto a rotary evaporator: seat its socket on the tilted cone, clamp it, lower it into the bath

A benchtop rotary evaporator (Heidolph Hei-VAP Core type, motor lift, vertical condenser) stands on the lab bench in
front of the arms, its lift up and its EASY CLIP flask clamp open. A 100 mL round-bottom flask with an NS 24/40 socket,
holding 40 mL of extract, stands in a cork ring beside it. The job is the step every chemist does by hand before each
evaporation, done the maker's way: put the flask on the vapour tube's ground cone, press the clamp's lever over the
flask's collar until it clicks, then lower the flask into the water bath with the lift. The task is one task file,
[`load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py`](load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

Why it matters (category Other, laboratory research automation): a rotary evaporator is standard equipment in
synthetic, medicinal, natural-product and analytical chemistry labs, used to strip the solvent after nearly every
reaction work-up, extraction and column; the flask goes on by hand before every run. Loading an instrument the way its
maker says, with the instrument's own clamp and lift, is the kind of wet-lab step that research-lab automation has to
cover.

The instruction gives the goal and its constraints only ("Put the flask of extract on the rotary evaporator, clamped,
and lower it into the water bath, ready to run; then return both arms to where they started."). It says neither how to
hold the flask, nor which way the joint goes on, nor where the lift's keys are.

What it tests: an insertion along a tilted axis, a fixed order, and a hand-off of the load. The vapour tube's cone
points toward the arms and down at 30 degrees. The flask's socket goes onto it only moved straight along that axis,
lined up within a few millimetres and degrees; the flask has to be turned from standing upright in its ring to lying
along the slope on the way. Once home it is not yet carried: a dry ground joint does not hold a flask on that slope,
so it slides off the cone and falls into the bath as soon as it is let go. The clamp closes, with its click, only
when its hook can drop behind the flask's collar, that is, once the socket is fully home. Only a clamped flask may be
let go, and only a clamped flask may go down into the bath.

Why two hands: one hand has to keep the flask pressed home on the tilted cone while the clamp is shut, and the clamp's
lever lies on top of the joint, where the holding hand can't press it. Let go first and the flask slides off; press the
lever first and it springs back open (no flask home to click over). The demo's right hand holds the flask by its neck,
from the side; the left presses the lever, square to the joint, and then works the lift's key; the wrists stay more
than 20 cm apart. With one hand the job fails. The right arm alone, doing exactly its part of the demo with the left
at home, slid the socket home on the cone; when it let go, the unclamped flask slid off the cone and fell into the bath
(score 0.0). The left arm alone pressed the lever, which sprang back open with nothing to click over, and ran the lift
down; the flask never left its cork ring (0.0).

## Real-world basis

- **The procedure (maker's manual):** Heidolph's operating instructions for the Hei-VAP Core, section "Mount/remove
  evaporation flasks": the evaporation flask is fixed to the vapor tube with a special retort clamp (EASY CLIP). Move
  the flask lift to the upper end position; "Place the flask in the EASY CLIP so that the ground joints of the flask
  and the vapor tube are fully aligned next to each other in a straight line and press the clamping lever of the EASY
  CLIP over the flask collar until the clamping lever audibly clicks into place"; then move the flask lift to the lower
  end position. The same section warns: "In the event of improper handling, the evaporation flask can fall to the
  floor." The motor-lift variant moves the lift while its arrow key is held ("The movement stops as soon as you release
  the button!"); the immersion depth is set with a stop on the lift base, and the lift moves down only to it.
- **The machine (maker's data):** Hei-VAP Core: 739 x 845 x 432 mm with motor lift and G3 glassware, about 15 kg
  without glassware, 10 to 280 rpm, flask lift stroke 155 mm at 25 mm/s, heating bath 253 mm across inside and 291 mm
  outside, 20 to 100 degrees C with water; EASY CLIP flask clamps for NS 24 and NS 29 joints come with it (manual,
  technical data and scope of delivery). The product page gives the flask's inclination as adjustable from 20 to 80
  degrees, a 4.5 L bath, and NS 24/40 joints on the North American models.
- **The joint:** NS 24/40 is the standard taper: the cone is 24 mm across at its wide end and 40 mm long, tapering
  1:10 to 20 mm (computed).
- **Other makers clamp the same way:** BUCHI's Rotavapor R-300 page names "fast flask handling with Combi-Clip"; the
  flask goes on the vapour duct's joint and a clip holds it.

Sources (all read for this task): Heidolph Hei-VAP Core operating instructions,
https://www.heidolph.com/Documents/Operation%20manuals/rotary%20evaporator/Operation-Manual-Rotary%20Evaporator-Hei-VAP-Core.pdf
(English part); Heidolph Hei-VAP Core product page,
https://www.heidolph.com/america/en/hei-vap-core-hand-lift-model-with-g3-vertical-glassware~p227; BUCHI Rotavapor
R-300 product page, https://www.buchi.com/en/products/instruments/rotavapor-r-300.

## How it is modelled

- **Scene, at real size.** The station table is cut out where the lab bench is: the bench top is 60 mm below the
  arms' deck, as if the arms stood on a 60 mm plinth at the bench. That puts the flask's working height, with the lift
  up, inside the arms' two-handed reach. The evaporator's base unit, lift column and condenser are built from boxes and
  cylinders, the condenser's top about 80 cm above the bench (the maker gives 845 mm overall); the bath stands in front
  of the base unit rather than beside it, so that the flask hangs facing the arms. The heating bath is the maker's
  253/291 mm pot with 3.0 L of water (the water is a surface, drawn only). The head on the lift carries the drive
  housing, the glass vapour tube, the EASY CLIP ring and the NS 24/40 cone, its axis 30 degrees below horizontal.
- **The flask** is one rigid body: a 64 mm bulb, a hollow neck (25 mm bore, 30 mm outside), a 34 mm bead round its
  mouth; 0.10 kg with its 40 mL of extract, the centre of mass 8 mm below the bulb's centre.
- **The ground joint.** The cone's collision core (14 mm square, its last 12 mm a 9 mm square tip) fits inside the
  socket's 25 mm bore, so a socket pushed at the cone well off its axis meets the cone's tip and does not go on. Once
  the socket's mouth is within 5.5 mm of the cone's axis and its axis within 10 degrees of the cone's (the 20 mm tip
  inside the 24 mm mouth, with the hand's give), and the cone is at least 8 mm inside it, a PhysX D6 joint
  takes over: the flask slides along the axis and turns about it (the joint's taper keeps it centred) and nothing
  else. Unclamped, nothing but gravity and a little joint friction (3 N s/m) acts along the axis, so a flask let go
  slides off and falls: in the no-clamp check it was off the cone and in the bath within a second of being let go.
  Home means within 4 mm of fully on.
- **The clamp.** The EASY CLIP's lever is its own body on a hinge across the joint, sprung open (0.004 N m per degree,
  about 5 N on its thumb pad to shut it). Its hook collides with the flask's collar, so with the socket short of home
  the lever can't reach its click. Pressed to within 3 degrees of shut with the socket home, it
  clicks: a stiff spring then holds it shut, its hook behind the collar keeps the socket from sliding out, and the
  joint's drive draws the socket against the end of the cone.
- **The lift and its keys.** The head rides on a vertical slide driven at 25 mm/s (the maker's figure) while the DOWN
  key (or UP) is held in past its switch point (2 mm of its 3.5 mm travel; each key is a 10 g cap on a 6 N preloaded
  spring), stopping at the immersion stop 95 mm down, where the bulb's centre is 12 mm under the water.

## Scoring

The score follows `score()` in the task file; `task.json` has the same text.

- Latched for the rest of the trial: **0.0** once the flask has been dropped: off the cone and fallen below 35 mm
  above the deck outside its cork ring (into the bath, onto the bench or the deck).
- Otherwise judged at the end of the trial, on live states:
  - **1.0**: the flask home on the vapour tube's cone (its mouth within 4 mm of the cone's base), the EASY CLIP clicked
    shut over its collar, the lift down at its immersion stop (within 4 mm), so the bulb sits in the bath's water, and
    both arms back where they started (each gripper's fingertip centre within 5 cm of its start);
  - **0.75**: the same with an arm not back;
  - **0.5**: the flask home and clamped, the lift not down;
  - **0.25**: the flask on the cone (its socket onto it), not clamped;
  - else **0.0**.
- The trial ends at the run limit, or once the full score has held for 1 s with the flask, the head and the lever
  still.

In a real trial each tier is read with simple checks. Home: no frosted cone shows between the flask's collar and the
clamp ring (a close photo with a ruler: the gap within 4 mm). Clamped: the lever lies flat on the joint with its hook
behind the collar and stays shut when let go (photo; the click is heard). Lift down: the lift's carriage against its
immersion stop and the bulb inside the bath below its rim (photo). On the cone: the socket over the cone (photo). Dropped: the trial
video shows the flask off the cone, falling. The arms are checked against marks at their start.

## Checks

Each check is the demo changed in one way (`RV_ONEHAND`, `RV_CHECK` or an aim offset), run in the simulator:

| Check | What happened | Score |
|---|---|---|
| Full demo, three runs | the socket met the cone 1.3 and 1.9 mm off its axis (the two runs with debug output), slid home and was drawn 0.7 mm onto the cone's end by the clamp; clamp clicked, flask lowered 95.9 mm into the bath, arms home | 1.0 each |
| Grip aimed 2 mm or 4 mm off the axis sideways, or 3 mm low (three runs) | the socket met the cone 1.3 to 4.3 mm off its axis and went on; home, clamped, lowered | 1.0 each |
| Right arm alone (`RV_ONEHAND=right`) | slid the socket home on the cone and let go: the unclamped flask slid off and fell into the bath | 0.0 |
| Left arm alone (`RV_ONEHAND=left`) | pressed the lever (it sprang back open: no flask home to click over) and ran the lift down; the flask never left its cork ring | 0.0 |
| No clamp (`RV_CHECK=noclip`) | socket home, then let go: off the cone and in the bath within a second | 0.0 |
| Lever pressed first (`clipfirst`) | the lever, pressed shut with no flask on the cone, sprang back open; the flask then went home and was let go: it slid off into the bath | 0.0 |
| Socket held 50 mm out, never slid on (`noseat`) | the socket never went on the cone; the lever sprang back open; the open hand dragged the loose flask away with it | 0.0 |
| Socket pushed only to 8 mm short of home (`short`) | the socket caught but stopped 16 mm short; the lever stopped at 4 degrees, no click; let go, the flask slid off into the bath | 0.0 |
| Socket aimed 8 mm off the axis (`RV_AIMY=0.008`) | the socket went 10 mm onto the cone's narrow tip 7 mm off its axis and stuck there, never caught; the lever didn't click; let go, the flask fell into the bath | 0.0 |
| Lift never lowered (`nolower`) | flask home and clamped, left up above the bath | 0.5 |

The socket can't be pushed onto the cone from the side: off its axis it meets the cone's tip. A flask set down
anywhere else, or let go before the clamp clicks, ends on the bath's floor or the bench, which is a drop.

## Real trial

- **Evaporator:** a working lab's rotary evaporator (a Heidolph Hei-VAP Core with motor lift, or any model whose
  flask clamp clicks over the flask's collar and whose lift runs on keys), with its vacuum, rotation and bath heating
  off; tilt set to 30 degrees, the immersion stop 95 mm below the lift's top position, the bath left dry (the lift's
  immersion stop and the bulb inside the bath's rim are the check). The arms stand on a 60 mm plinth at the bench, the bath's centre 29 cm in front of the
  line between their bases. The trial needs no vacuum, heat or rotation, so any lab with an evaporator can run it as it
  stands; a lab without one can borrow the instrument for the session.
- **Flask:** a plastic-coated 100 mL round-bottom borosilicate safety flask with an NS 24/40 socket (any maker; a
  dropped one does not break), holding 40 mL of water tinted with food colouring, in a cork ring for 100 to 250 mL
  flasks.
- **Measuring:** a ruler, the overhead and side cameras, and marks for the arms' start poses.
- **Safety and rig protection:** nothing is hot, under vacuum or turning; the grippers hold the flask by its dry neck
  and the bath is dry; the 40 mL of tinted water stays in the flask's bulb. Reset as in
  the reset notes: lift up, lever open, flask eased off the cone along its axis and stood back in its ring.

## Assets and licences

None from third parties. Every object is built from boxes, cylinders and spheres in the task file; `bodykit.py` is our
own. The NVIDIA SimReady set has no rotary evaporator, round-bottom flask or cork ring, so everything is built from
primitives at the parts' sizes.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the flask, the head and the lever still) has held
for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`).
`-E object_jitter=0` keeps every object exactly where the task file puts it, so every trial starts from the same
layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before
sending it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the flask of extract on the rotary evaporator, clamped, and lower it into the water bath, ready to run; then return both arms to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/rv-dream
```

The scripted demo, which reaches full score: the right hand comes in from the right, in front of the evaporator, and
takes the flask by its neck just above the bulb, from the side (fingers closing front and back). It lifts the flask out
of the cork ring and carries it over to the bath, rolling its wrist 60 degrees on the way so that the neck comes to lie
along the vapour tube's slope, the mouth below the cone. It brings the socket in line with the cone 50 mm out, slides
it straight up the cone along the axis (aiming 10 mm past home for the arm's lag) and keeps it pressed home. The left
hand comes down on the clamp's thumb pad with its closed fingers square to the joint (the gripper tilted 30 degrees)
and presses the lever down past its click, then lifts off. The right hand opens and draws back off the neck; the
clamp holds the flask. The left hand presses the lift's DOWN key and holds it for 5 s while the lift takes the flask
95 mm down into the bath, to its stop, then lets go. Both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_a_flask_onto_a_rotary_evaporator_and_lower_it_into_the_bath.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the flask of extract on the rotary evaporator, clamped, and lower it into the water bath, ready to run; then return both arms to where they started." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/rv-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/rv-dream/*.json; do inspect-robots video "$log"; done
```

Set `RV_DEBUG=1` to print what the score is built from four times a second: the flask's position, the socket's offset
from the cone's axis and its distance from home, whether the joint has caught, the lever's angle and whether it has
clicked, the lift's position, the DOWN key, the finger openings and where each TCP is. `RV_ONEHAND=left` (or `right`)
replays only that arm's part of the demo (the other stays home). `RV_CHECK=noclip`, `noseat` and `nolower` run the
scoring checks listed under "Checks". All of these are checks, not part of the task.

## Runtime per trial

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,600 steps (596 s on the clock, with Dream answering in 0.72 s on average), and about 5 minutes for the scripted demo (754 steps, 37.7 s of simulated time, 318 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
