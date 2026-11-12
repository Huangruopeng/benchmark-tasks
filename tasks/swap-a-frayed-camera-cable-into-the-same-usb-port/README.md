# Swap a robot camera's frayed USB cable for the new one, plugged into the very port the old one came out of

A 7-port USB hub lies loose on a slick white desk riser. Four of its ports hold right-angled USB plugs whose cables run
off the riser's front edge. One of them is the robot camera's cable, and its jacket is split open next to the plug,
with the braid and wires showing. The robot's software finds each camera by the port it is plugged into, so the new
cable has to go into exactly the same port. The new cable's plug is parked upright in a grey cable dock to the right of
the hub. A blue bin stands on the desk in front, and the robot computer's status box on the riser shows a CAMERA lamp:
green while the camera is on its port, red otherwise.

The job:
1. Hold the hub down with one hand while the other pulls the frayed cable's plug straight up out of its port.
2. Drop the frayed plug in the blue bin.
3. Take the new cable's plug out of the dock and push it all the way into the same port the frayed one came out of.
4. Take both arms back to where they started.

Two things fail the trial outright:
- putting a plug into any other port, even part way;
- taking the new plug out of its dock before the frayed one is in the bin, so the old plug can't be left by the port
  as a marker.

The task is one task file,
[`swap_a_frayed_camera_cable_into_the_same_usb_port.py`](swap_a_frayed_camera_cable_into_the_same_usb_port.py), for
the `yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports
from the same folder.

**What it tests:**
- **Memory.** Which port the camera uses depends on the trial's seed, and so do the three other occupied ports. Once
  the frayed plug is out, its port looks like the other empty ones: its light goes out, and nothing else marks it. By
  the time the new plug is in hand, the old one is in the bin. A policy that sees only the current frame has to guess
  among four empty ports.
- holding a light part down while pulling a stiff plug out of it;
- carrying a plug and pushing it home into a small socket;
- an enforced order: the old part in the bin before the new one is picked up.

**Why two hands:** a plug's latch holds it in its port with about 3 N, three times the hub's weight (100 g). Pulled with
the hub unheld, the plug lifts the whole hub off the riser and never comes out. Held by the other hand next to the plug,
the hub stays put and the plug slides out. Holding the hub far from the plug is not enough: in a trial run with the
hub pressed down at its other end, 20 cm away, the hub tipped up about that end and the plug stayed in.

The one-hand check scored 0. The right hand did its part as in the demo while the left never moved. The hub came up
with the plug and was dragged about, and the plug never came out.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The riser, the dock, the status box and the bin are
kinematic bodies made of shapes from `bodykit.py`. The hub, the frayed plug and the new plug are rigid bodies.

- **The hub** has real slotted ports: each is a 20 by 11 mm opening, 17 mm deep. The three other cables are part of the
  hub's body and stay plugged in.
- **The latch** is a `Thread` joint from `bodykit.py` along the port's axis. It keeps the plug on that axis, with 1.5 mm
  of play and 4 degrees of tilt, and pulls it toward a target with a spring of 2,000 N/m. After every control step the
  task file checks how far the plug sits from the target. Pushed or pulled more than 1.5 mm past it, the plug slips:
  the target jumps to where the plug is, so the latch holds with about 3 N and lets go once that is exceeded.
  - Pulled 19 mm up, the frayed plug is out of the port and the joint is switched off.
  - The new plug catches the camera's port when it is set in the port's mouth: within 9 mm of the port's axis, upright
    within 12 degrees, its connector's bottom 4 to 20 mm above the port's floor. Pushed to within 3 mm of the floor, it
    is home.
  - Collisions between the plugs and the hub stay on, so a plug can't pass through the hub's top.
- **The layout** is chosen from the seed when the scene is built. The hub's shapes (the other cables and their lights),
  the frayed plug's start and the latches' port are set before the first reset builds the scene.
- **The lights:** while the frayed plug is in its port, the camera port's blue light is lit and the CAMERA lamp is
  green. Once the plug is out, the light goes dark and the lamp turns red. Both come back when the new plug is home.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena. It gives
`score()` each body's own pose and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`. It gives 1.0 when `success()` has held for 1 s. `success()` means full score, which
includes both arms back where they started, with the hub and both plugs still. Otherwise it gives `score()` at the end
of the episode: 0, 0.25, 0.5 or 1.0 (see `task.json`).

- `-E object_jitter=0` keeps every object exactly where the task file puts it.
- The cameras render at 1280 x 704 for sharp videos. Dream-1's client scales every frame to 320 x 176 before sending
  it, so `-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=swap_a_frayed_camera_cable_into_the_same_usb_port.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "The robot's camera cable (the one with the frayed jacket) is plugged into the USB hub. Swap it for the new cable parked in the grey dock. Hold the hub down with one hand while the other pulls the frayed cable's plug straight up out of its port, and drop it in the blue bin. Then take the new cable's plug out of the dock and push it all the way into the same port the frayed one came out of: the robot's software finds the camera by its port. Putting a plug into any other port, even part way, fails, and so does taking the new plug out of the dock before the frayed one is in the bin. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/usb-dream
```

The scripted demo, written for the default seed (the camera in the 1st port from the left), reaches full score:
1. The left hand comes in from its side along the hub, tilted 20 degrees down, and closes its fingers on the front and
   back of the hub's left end, right next to the camera's port.
2. The right hand comes down on the frayed cable's plug, its fingers across the overmould's thin way, and pulls the plug
   straight up out of its port against the latch.
3. The right carries the plug out over the bin and lets it drop in. The CAMERA lamp has turned red.
4. The right takes the new plug out of the dock and carries it along in front of the hub to the same port. It lowers
   the plug into the port and pushes it home. The port's light and the CAMERA lamp come back on.
5. Both hands let go and go back to where they started.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=swap_a_frayed_camera_cable_into_the_same_usb_port.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "The robot's camera cable (the one with the frayed jacket) is plugged into the USB hub. Swap it for the new cable parked in the grey dock. Hold the hub down with one hand while the other pulls the frayed cable's plug straight up out of its port, and drop it in the blue bin. Then take the new cable's plug out of the dock and push it all the way into the same port the frayed one came out of: the robot's software finds the camera by its port. Putting a plug into any other port, even part way, fails, and so does taking the new plug out of the dock before the frayed one is in the bin. Then take both arms back to where they started." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/usb-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/usb-dream/*.json; do inspect-robots video "$log"; done
```

Set `USB_DEBUG=1` to print the trial's seed and layout at reset, and four times a second what the score is built
from:
- whether the frayed plug is still latched, its height in its port and the latch's hold, whether it has come out and
  whether it is in the bin;
- whether the new plug has caught in the camera's port, its height and how far it is off the port's axis;
- the "another port" and "too early" flags;
- where the hub and both TCPs are, and each TCP against each plug.

## Runtime per trial

On one RTX 5090: about 7 minutes for a Dream-1 trial of 1,000 steps (402 s on the clock, with Dream answering in 0.77 s on average), and about 4 minutes for the scripted demo (608 steps, 30.4 s of simulated time, 243 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

The trial's seed picks the layout: which of the seven ports holds the camera's cable, and which three others hold
cables (140 layouts). The default run (`inspect-robots run` with its default `--seed 0`) resets with seed 1415158438,
which puts the camera in the 1st port from the left and the other cables in the 3rd, 4th and 6th. The scripted demo
follows fixed waypoints written for that layout, so it reaches full score only with the default seed. The recorded
Dream-1 run used the default seed too. To try another layout, pass `--seed N`. The layout is set when the scene is
built at the first reset; later resets in the same run keep it.
