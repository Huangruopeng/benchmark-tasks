# Bow a clean two-second open A on a full-size violin held up in the other hand

Anyone recognises this in a second: a violin held up, a bow drawn across it near the bridge, one steady note. School
orchestras, street players, weddings and concert halls all start from it, and every beginner's first lesson is exactly
this: the open strings, one long even bow on each. The task asks a robot to do it on a real full-size student violin:
pick the violin up off the cloth it lies on, play its open A string with the bow, one clean steady note of at least two
seconds, and lay both back. The task is one task file, [`bow_a_clean_open_a_on_a_violin.py`](bow_a_clean_open_a_on_a_violin.py),
for the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it
imports from the same folder.

The instruction gives the goal only ("Pick up the violin and play its open A string with the bow: one clean, steady
note of at least two seconds. Then lay the bow and the violin back in their places on the cloth."). It says nothing
about how to hold either, where on the string to bow, how hard or how fast.

## What it tests

A bowed string speaks with a clean tone (Helmholtz motion: one sharp kink running round the string, the hair gripping
and slipping once a period) only inside a narrow window of three bowing parameters that the player sets with the arm:

- **Bow force against bow-bridge distance and bow speed.** For a relative bow-bridge distance beta (the contact's
  distance from the bridge over the string length) and bow speed v, the force must lie between a minimum, below which
  the string slips more than once a period (a whistling, "surface" sound), and a maximum, above which the hair drags the
  string into a raucous crunch. This is Schelleng's playability diagram (Schelleng 1973, as given in the paper below). Schoonderwaldt, Guettler and
  Askenfelt measured both limits with a bowing machine on a violin (Acta Acustica 2008): on the violin's open D string
  at 10 cm/s the upper limit is 91 mN / beta and the lower limit 0.69 mN / beta^2; the upper limit grows with bow speed,
  the lower one does not. At beta = 1/10 and 10 cm/s that leaves 0.07 to 0.91 N; closer to the bridge the window moves
  up and narrows.
- **One string only.** The four strings cross the bridge on an arc (here 42 mm radius, strings 11 mm apart), so each
  string is played with the bow tilted to its own angle across the bridge, 15 degrees from the next. Tilted a few
  degrees too far, the hair also catches the D or the E string and two notes sound.
- **Steady, square motion.** The bow moves along its own length across the string, square to it, at 5 cm/s or more,
  between the bridge and the fingerboard (beta 1/30 to 1/6, the range the measurements cover).

The hair's give is what makes the force controllable: horsehair under tension gives across its length like a spring,
so the force follows from how far the hand lowers the bow onto the string. The robot has to hold all of this for two
seconds while the bow travels 20 cm or more, and the violin it bows is held up by its other hand.

## Why two hands

A violin can't be played lying down: the note only counts while the violin is held up clear of the cloth (its lowest
point 2 cm or more above it), as it is when a person plays it. Held up, a violin has nothing to stand on, so one hand
holds it while the other draws the bow. In the demo the left hand holds it from above by the fingerboard's two edges
where the board runs out over the body, and the right hand holds the bow by its frog; their fingertip centres stay 14
cm or more apart. The one-hand runs bear it out. The right arm alone, doing its part of the demo on the violin where it lies (the
left at home), drew a stroke on the A string that met every other condition (0.65 to 0.14 N, beta 0.10 to 0.16, 9
cm/s), but the violin lay on the cloth: 0.0. The left arm alone lifted the violin and held it, but nothing bowed it:
0.0.

## Real-world basis

- **Bow-force window (the scorer's limits).** Schoonderwaldt, Guettler and Askenfelt, "An Empirical Investigation of
  Bow-Force Limits in the Schelleng Diagram", Acta Acustica united with Acustica 94 (2008) 604-622. With a bowing
  machine on a violin's open D string (and on a monochord copied from a violin, string length 325 mm, a 58 g bow with
  a 10 mm hair ribbon) they fitted the limits Fmax = c_upper / beta and Fmin = c_lower / beta^2: on the violin at
  10 cm/s, c_upper = 91 mN and c_lower = 0.69 mN (their Table VII). "The minimum bow force was found to be independent
  of bow velocity", while the upper limit rose with speed (monochord D string: 66, 86, 125 and 166 mN at 5, 10, 15 and
  20 cm/s, Table I). Their grid covered beta "about 1/30 to 1/6" and bow speeds of 5 to 20 cm/s. The paper gives
  Schelleng's formulas, Fmax = 2 Z0 vB / ((mu_s - mu_d) beta) and Fmin = Z0^2 vB / (2 R (mu_s - mu_d) beta^2). No A
  string was measured; the task uses the open D values for the A, its neighbour (the E string's limits were lower, the
  upper by 19% and the lower by 64%).
- **Real players' force on the open A.** Mizuho and Sugiura (arXiv:2411.05126) had players draw full bows on the
  open A string: "Seven of the eight experienced players had a minimum bow pressure of 0.5 N or more".
- **Bowing square to the string, one string by the bow's inclination.** Jansson, Acoustics for Violin and Guitar
  Makers (KTH), chapter 8: "The tone of the violin is generated by the bow pulled perpendicularly across the string",
  with the bow force "selected within a permitted working range to obtain ... a Helmholtz motion". Schoonderwaldt and
  Altenmuller (PLoS ONE 2014): the bow's inclination has the "main function [of] selecting the string"; string
  crossings are made by "pivoting it about the axis of the string(s)".
- **The violin.** Full-size (4/4) dimensions: body 356 mm (Wikipedia, "Violin"; Hans Johannsson's measurement chart:
  body 356 mm, string length 327 mm, ribs 30 mm, stop 195 mm, neck 130 mm); 59 cm from scroll to end pin (Violin
  Lounge size chart); 400 to 500 g (Violinspiration). Bow: 75 cm with the screw, 55 to 64 g (German Wikipedia,
  "Bogen (Streichinstrument)"). A string: Thomastik Dominant 131 A, medium, 5.5 kg tension (Long & McQuade listing).
- **The outfit and the tuner for the real trial.** Stentor 1400 Student I 4/4 violin outfit, US$268.99 (Guitar
  Center); Stentor: "The outfit includes a reliable wood bow and well-designed lightweight case ... and rosin", "Good
  student wood bow with ebony frog". Snark SN-5 clip-on tuner, US$17.03 (StewMac): "both tuners sense the vibrations
  of the instrument they are attached to" (Snark instructions); green means in tune. D'Addario NS Micro, US$14.99,
  "Accuracy: 0.3 cents" (MusicRadar review).

## Scene (as in the screenshot; the same in every trial)

A table in front of two arms (bases 61 cm apart), seen by one overhead camera and a camera on each wrist. Distances are measured forward from the line between the arm bases and sideways from the midpoint between them.

1. A dark green wool felt cloth, 5 mm thick, from 12 to 90 cm forward and from 47 cm right to 34 cm left, fixed.
2. A full-size (4/4) student violin, 0.45 kg, balance point 10.5 cm into the body from the neck: spruce top and maple back and ribs under orange-brown varnish, body 356 mm long (bouts 168, 112 and 208 mm wide, pointed corners, f-holes, purfling), ribs 30 mm, plates arched 14-15 mm; maple neck and scroll with four ebony pegs; ebony fingerboard 270 mm long, 24 to 42 mm wide, running out 140 mm over the body; maple bridge 195 mm from the body's top end, its top on a 42 mm arc with the strings 11 mm apart; nut to bridge 327 mm; four strings G3 D4 A4 E5 (the A aluminium-wound on a synthetic core); ebony tailpiece with an E fine tuner, ebony chinrest, end button; 59 cm long in all. A black clip-on tuner is clipped to the scroll. The violin lies on its back on the cloth across the robot's view: scroll to the left, the E string on the far side, the middle of its body 40 cm forward and 12.6 cm right, its neck joining the body 5.2 cm left of the centre line.
3. Its bow, 75 cm long and 60 g, balancing 20.5 cm in front of the frog's middle: a round pernambuco-brown stick tapering from 8.4 to 5.8 mm with a slight camber, an ebony frog 46 x 14 x 26 mm with a pearl eye and silver ferrule, a head with an ivory-white face, and a white horsehair ribbon 10 mm wide and 66 cm long, tensioned and rosined. It lies hair down on the cloth, pointing forward, frog toward the robot: the frog's middle 16 cm forward and 40 cm right.

Both arms start at their home pose. The layout is the same in every trial, as in the screenshot.

Real trial: the violin and bow of a Stentor 1400 Student I 4/4 outfit (US$268.99), the A string tuned to A4 (440 Hz); a Snark SN-5 or D'Addario NS Micro clip-on tuner (US$15-17) on the scroll; a felt cloth with tape marks round the violin's outline and at the bow's frog and head; a strip of masking tape on the violin's top beside the A string, marked 11 mm and 55 mm from the bridge; a camera with a microphone facing the tuner. Rosin dust stays on the hair and strings; the grippers touch only the frog and the fingerboard's edges.

## Scoring

Latched for the rest of the trial (score 0.0): the violin or the bow falls (drops faster than 0.5 m/s), or a gripper's fingertips go into the violin's body. The A string sounds at a step when all of these hold: the violin is held up (its lowest point 2 cm or more above the cloth); the bow's hair touches the A string and no other (a string within 0.4 mm of the hair counts as touched); the contact is 1/30 to 1/6 of the string length from the bridge (11 to 55 mm); the bow moves along its length across the string at 5 cm/s or more, square to it within 15 degrees; and the hair presses on the string with a force F between 0.69 mN / beta^2 and 0.91 N s/m x speed / beta, beta being the contact's distance from the bridge over the string length (the bow-force limits measured on a violin's open string, Schoonderwaldt, Guettler and Askenfelt 2008: 91 mN / beta at 10 cm/s, rising with speed).

A clean open A: the A string sounds for 2 s with no break longer than 0.1 s (a bow change fits in that) and no other string touched in that span.

1.0: a clean open A of 2 s at some point, and at the end the violin lying back in its place (its origin at the neck joint within 3 cm of its start, turned less than 10 degrees, flat on the cloth within 10 degrees, not held), the bow lying back in its place (the frog's middle within 4 cm of its start, turned less than 10 degrees, on the cloth, not held), and both arms back where they started (each gripper's fingertip centre within 5 cm); 0.75: the 2 s note played, the end state not met; 0.5: a clean open A of 0.5 s or more at some point; 0.25: at some point the hair touched the strings while the violin was held up; else 0.0.

The trial ends at the run limit or once the full score has held 1 s with the violin and the bow still.

Real trial: the tuner on the scroll shows A (440 Hz) with its needle steady in the green, and the microphone records one steady tone; the 2 s and any break are timed on the video. A crushed tone (too much force), a whistling or dying one (too little force, too slow) or two strings sounding shows as no steady A on the tuner and is heard on the audio. The video shows the hair on the A string alone, between the tape marks 11 and 55 mm from the bridge, the bow moving, and the violin clear of the cloth. Falls, a gripper striking the body, the end positions against the tape marks and the arms against their start marks are judged from the video and the end photo.

The score is computed by `score()` in the task file, every control step; `success()` adds the stillness of the violin and the bow over 1 s. The demo uses fixed waypoints only.

## Checks

Every check is the scripted demo with one change (environment switches in the task file's `demo()`), run in the
simulator with the same seed as the demo. Forces are the hair-string contact force; beta and speed as the scorer
measures them.

| Run | What changes | Score | Why |
|---|---|---|---|
| Demo | (none) | 1.0 | A sounded 2.95 s without a break: 0.49 falling to 0.08 N, beta 0.094 to 0.156, 8.7 to 9.5 cm/s; D and E 1.3 mm clear of the hair; violin and bow back in place |
| Demo, batch run | (none) | 1.0 | the same; the trial ended at 587 steps |
| Right arm alone, violin on the cloth | the left stays home; the bow plays the violin where it lies | 0.0 | the stroke met every other condition (0.65 to 0.14 N, beta 0.10 to 0.16, 9 cm/s), but the violin was never held up |
| Left arm alone | the right stays home | 0.0 | the violin was lifted and held; nothing bowed it |
| Wrong tilt | the bow tilted 9 degrees toward the D string | 0.25 | the hair lay on the D string (0.1 mm) with the A 0.5 mm clear: the D, not the A |
| Too slow | the same stroke time, 9 cm of bow instead of 27 | 0.25 | 2.9 cm/s, under 5 cm/s, although force (0.4 N) and beta (0.10) were in range |
| Pressed hard | the hair aimed 25 to 30 mm into the string | 0.25 | 2.2 to 3.6 N, the hair on its stop against the stick, far above the upper limit; the violin was pushed down to the cloth |
| Pressed harder, within reach of the arms' give | the hair aimed 11 to 14 mm into the string | 1.0 | the arms give: 0.83 falling to 0.37 N, still inside the window; the note sounded (the task is not sensitive to a centimetre of aim) |
| Near the bridge | aimed 9 mm from the bridge (beta 0.028), a 1.3 s stroke | 0.25 | beta 0.033 to 0.055 with 0.15 to 0.25 N: under the minimum force so close to the bridge (0.36 N at beta 0.044); no note |
| Over the fingerboard | aimed at beta 0.20 | 0.25 | beta 0.21 to 0.28, past 1/6; the strings sit closer together there and the hair lay across three or four of them, pressing up to 4.4 N; no note |
| Short note | 7 cm of bow in 0.9 s | 0.5 | a clean A of 0.75 s |
| Not put back | violin and bow kept in hand, held up | 0.75 | note played (2.95 s); end state not met |

A fall of the violin or the bow, or a gripper's fingertips inside the violin's body, scores 0.0 for the rest of the
trial whatever was played before.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The cloth is one fixed body; the violin, the bow's
stick and the bow's hair are loose rigid bodies built from shapes in `bodykit.py`, at their real sizes and masses, with
extra visual shapes added when they are spawned.

- **Violin:** the body is drawn from a generated outline (upper bout 168 mm, C-bouts 112 mm, lower bout 208 mm, 356 mm
  long, pointed corners): an arched spruce top and maple back (14-15 mm arching), ribs 30 mm, purfling, f-holes; the
  neck, fingerboard (a tapered, cambered ebony mesh), nut, pegbox, four pegs and scroll; the bridge with its arched top;
  tailpiece, fine tuner, chinrest, end button; the strings from nut to bridge to tailpiece, and the tuner on the scroll.
  It collides as cylinders and boxes under the drawn body (back at 44 mm below the top's edge), boxes for the neck,
  heel, pegbox and bridge, three tapered plates for the fingerboard, a cylinder for the scroll, and, where the bow can
  reach them (from the fingerboard's end to the bridge), the four strings as 1.2 mm bars along their real lines: G D A E
  11 mm apart at the bridge on a 42 mm arc (the A and D 48 mm above the top's edge, the G and E 45 mm), 5.5 mm apart at
  the nut 8.7 mm up. 0.45 kg, centre of mass 105 mm into the body and 10 mm below the top's edge (design value from
  the parts' weights). Friction 1.2.
- **Bow:** the stick (54 g with the frog, button and head; centre of mass placed so that the whole bow balances 20.5 cm
  in front of the frog's middle) and the hair (6 g), a ribbon 10 mm wide, 2.5 mm thick in collision, 66 cm long, its own
  body on a PhysX prismatic joint to the stick across the ribbon: a spring of 300 N/m with 2.5 N s/m damping, 8 mm of
  travel until it meets the stick, resting on its stop when nothing presses it. The hair is slick (friction 0.35) and
  never collides with its own stick.
- **Bow force:** the contact force between the hair and the violin (PhysX contact report, each step). Over the demo's
  stroke it agrees with the spring's deflection times 300 N/m to within 0.05 N (0.1 N at the stroke's first step).
- **Which strings touch:** each step the scorer finds where each string passes the hair ribbon and measures the gap
  between them; a gap under 0.4 mm is a touch. The contact point on the A string gives beta; the bow's speed is how far
  that point moves along the hair between steps; the skew is the angle between the hair and the square to the string.

The left hand holds the violin by the fingerboard's two edges where the board runs out over the body, 70 mm into the
body, the strings running untouched between the fingers. This is the hold that works for a parallel gripper. Held by
the neck near the heel, the violin's weight acts some 13 cm from the grip (0.56 N m). In the simulator a grip across
the fingerboard's edges there let the violin turn 12 degrees in the jaw until its tail rested on the cloth, and a grip
across the fingerboard and the back of the neck carried the weight but rolled the neck 13 degrees in the jaw until the
upper finger lay flat on the board, where it would stop all four strings. Near the balance point the turning load is
small: the violin hangs within 4 degrees of level and does not move in the grip.

## Fidelity

Rigid-body simulation (Isaac Sim, PhysX, 20 Hz), real sizes and masses. Strings are rigid 1.2 mm bars; the hair is a rigid ribbon on a sprung slide on the stick (300 N/m, 8 mm travel, design values). The tone is computed each step from contact, bow-bridge distance, speed, squareness and force against the published bow-force limits for a violin's open D string (none are published for the A); no sound is simulated. The violin collides as boxes and cylinders. Cameras 1280x704.

## Assets

All geometry is built in the task file from boxes, cylinders, spheres, capsules and generated meshes (the violin's
body outline, arched plates, ribs, purfling and fingerboard); no third-party meshes, scans or textures are used.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score with the violin and the bow still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5, 0.75 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=bow_a_clean_open_a_on_a_violin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the violin and play its open A string with the bow: one clean, steady note of at least two seconds. Then lay the bow and the violin back in their places on the cloth." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/violin-dream
```

The scripted demo, which reaches full score: the left hand comes down over the fingerboard where it runs out over the
body, leaning 30 degrees along the violin toward the scroll, closes across the board's two edges (the strings run
between its fingers) and lifts the violin clear of the cloth, aiming 3.5 cm higher than the lift because the arm
gives under the load. The right hand picks up the bow by its frog (fingers across the frog), rolls it 30 degrees and
brings it over the A string, square to the string and tilted along the bridge's arc, lowers the hair onto the string
29 mm from the bridge, and draws a 3 s down-bow, 27 cm long, toward the frog, aiming the hair's edge 2.2 mm below the
string's top at the start and 5.6 mm at the end because the arm rises slightly as the frog comes in. The bow is aimed
at the violin where it hangs in the left hand (measured in a debug run: 3.5 degrees rolled, 4 degrees tail down). The
right lays the bow back on the cloth, the left lays the violin back, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=bow_a_clean_open_a_on_a_violin.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the violin and play its open A string with the bow: one clean, steady note of at least two seconds. Then lay the bow and the violin back in their places on the cloth." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/violin-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/violin-dream/*.json; do inspect-robots video "$log"; done
```

Set `VLN_DEBUG=1` to print what the score is built from ten times a second: the violin's pose and lowest point, each
string's gap to the hair, the hair's give and the contact force, beta, bow speed and skew, whether the A sounds and
for how long. The checks below use the demo's environment switches (`VLN_ONEHAND`, `VLN_PRESS`, `VLN_TILT`,
`VLN_STROKE`, `VLN_BETA`, `VLN_NORETURN` and others listed in `demo()`).

On one RTX 5090: about 10 minutes for a Dream-1 trial of 1,600 steps (605 s on the clock, with Dream answering in 0.63 s on average), and about 4 minutes for the scripted demo (587 steps, 29.4 s of simulated time, 261 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.
