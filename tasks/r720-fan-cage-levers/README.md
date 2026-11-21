# Refit a 2U server's cooling-fan assembly: lower it level onto its two hidden guide pins, then cam both blue levers down

Keeping AI servers cooled is hands-on work that grows with them: the IEA projects data-centre electricity use to rise about 15% a year to 945 TWh by 2030, and that of accelerated (AI) servers 30% a year. A Dell PowerEdge R720's six-fan cooling assembly comes out whenever a fan, the cabling or the board beneath needs work, and goes back by Dell's install steps: "Align the cooling-fan assembly slots with the guide pins on the chassis. Slide the cooling-fan assembly into the chassis. Lock the cooling-fan assembly into the chassis by rotating the blue release levers downward until firmly seated."

**What is hard.** The assembly is as wide as the chassis (42 cm) and is carried by its two levers, one in each hand: by estimate, from one lever it tips. Its two guide pins are hidden under it, and each slot clears its pin by 0.5 mm a side, so it has to come down level and square. On its pins its weight does not push the six fan plugs home; each lever's cam presses its own end the last 3.8 mm, so closing one lever mates only three fans.

**How the demo does it.** Both hands take a lever each, lift the assembly over the drive cage, carry it to the bay, level it and square it by feedback on its pose, and lower it onto the pins in small steps. They let go on the pads, come over each lever with the fingers closed and closing along x (18 mm across, so they fit the lever's 32 mm groove), and push each lever's free end round its hinge, aiming a few degrees ahead of the lever's observed angle each step, until it passes over centre and its cams drive it onto its stop. It reads the two lever angles and the assembly's pose (a policy gets the cameras). Layout, for reach: the chassis' front is at x 23.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder.

## Run a trial

From this folder, the scripted demo (the README command) and the lazy-route checks:

```bash
python demokit.py task.py OUT                                                # the full-score reference, filmed (640 x 360)
python demokit.py task.py OUT --no-record --mode none,drop,drop15,drop5,drop3,drop5close,one,nocam,press         # lazy-route checks, below
python demokit.py task.py OUT --mode full --jitter 0.01 --seed 1             # jittered run
```

A policy run:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --seed 1 \
  --instruction "Refit the cooling-fan assembly in the server and lock it in."
```

`FAN_DEBUG=1` prints the fingertips in the assembly's frame every 10 steps; each push step prints the lever angles, the arm's joint errors, the push and the cam's resisting moment (estimates). No seed changes the scene.

**Result** (scripted, GPU physics on RTX 4070 Ti SUPER, 20 Hz): 992 steps (49.6 s of simulated time) to 1.0, against a limit of 2,400.

| Run | Setting | Score |
|---|---|---|
| README command, 3 runs | no jitter, 640 x 360, three RTX 4070 Ti SUPER runs | 1.0, 1.0, 1.0 (992 steps each; no black frames in any camera) |
| Jittered, 4 runs | `--jitter 0.01 --seed 1..4` | 1.0 on all 4 (the assembly is a mechanism, which the harness's jitter does not move, so these repeat the plain run) |

At 1.0 both ends sit 0.31 mm over seated, the frame is level to 0.04 degrees, the plugs are 0.2 mm off their sockets' axes and both levers are on their 92-degree stops. Lazy routes, GPU physics, the same scorer:

| Check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `drop` | carried and aligned, let go about 3.4 cm over the pin tops | 0 (it falls in) |
| `drop15` | let go 15 mm over its rest height | 0 |
| `drop5` | let go 5 mm over its rest height | 0 |
| `drop3` | let go 3 mm over its rest height | 0 |
| `drop5close` | let go 5 mm over its rest height, then both levers closed as in the demo | 0 |
| `press` | pushed down by the levers' tops, levers not closed | 0.25 |
| `nocam` | lowered onto the pins, levers left up (any lever angle under 88 degrees) | 0.25 |
| `one` | as the demo, but only the left lever pushed down | 0.5 (the right end stays 4.5 mm up) |

The slowest 30 steps took 14 s at 640 x 360 on an RTX 4070 Ti SUPER (about 0.47 s a step): with Dream-1's 8 s idle limit, send 10 or fewer steps per request.

## Building it for a real trial

- **Server:** a Dell PowerEdge R720 (12th generation, 2U; fan modules 60 x 60 x 38 mm, 12 V, Dell part numbers WPVP9, 3RKJC, KVN8J or WCRWR) with its cover off and its cooling-fan assembly (six such modules in the R720's two-lever cage) removed, on the bench, front towards the robot. Power off.
- **Start:** the assembly on a foam mat in front of the chassis, both blue levers up.
- **Score:** after the run, the levers' photo and the server's fan report at boot (six fans present); 1.0 needs both levers flat and the assembly still for 1.5 s on the video. 0.25 (on both pins, a lever not flat): a ruler across the bay at each end shows the frame top no more than 3.7 mm above the chassis wall top (the seated frame top sits 1.3 mm below it), with a square-on photo of the bay. 0: the video shows the hands open while the assembly is above where it rests and it then drops onto the pins, or it is let go with its lower end more than 2 cm up (for example set on the drive cage to regrip). The simulator reads these as a fall of more than 2 mm after the last fingertip contact within 1 s, a release more than 20 mm over the seat, and a fall faster than 0.3 m/s.

| Value | In the simulator | How to measure it on the real setup |
|---|---|---|
| Chassis width and height | 444 x 87.3 mm (Dell) | A tape |
| Assembly | 420 x 60 x 70 mm, 1.0 kg (estimates) | Calipers and a kitchen scale |
| Guide pins, slots | 5 mm pins, 12 mm tall; slots 0.5 mm a side wider (estimates) | Calipers |
| Plug resistance | 12 N an end at rest, 20 N seated (estimate; 3 N a plug) | A push gauge on one end with its lever up |
| Cam | lift 3.8 mm from 49 degrees on, peak at 86, stop at 92 (estimates) | A dial gauge on the frame's end as the lever closes |
| Lever | 75 x 20 x 18 mm, hinge 30 mm above the base (estimates) | Calipers |

## Modelled, not sourced from CAD

Dell publishes no drawing of the fan assembly or the bay, and no CAD was found. The chassis section is built from Dell's published width and height; the drive cage, bay, heat sinks, assembly, levers, cams and brackets are boxes and discs at estimated sizes. Each lever's cam is two discs that straddle the lever and bear on two brackets, one off the drive cage's back, one on a post. The second disc per lever is an addition for the simulator, not from Dell: with a single cam the rigid frame pitches 2.1 degrees as it seats (the score needs 0.5), because the cam sits 17.5 mm off the end's middle. It does nothing for the robot: it carries, lowers and pushes the levers exactly as before. Dell's manual (pp. 57-58) gives the steps only, so the real number of cams per lever is not known. The six plugs' resistance is four spring pads under the frame, preloaded; the plugs are chamfered pins in socket gaps.

## Real items

- **Fan module:** Dell R720 12 V 60 x 60 x 38 mm fan, part numbers WPVP9, 3RKJC, KVN8J or WCRWR: https://altatechnologies.com/products/dell-wpvp9_new
- **Six-fan cage with two blue levers (a like part, not the R720's own):** Dell K6L00 (listed for the R720xd, the sister 2U server; no R720 cage number was found, and Dell's manual warns the two servers' fans are not interchangeable): https://www.itcreations.com/product/60565 . Its shape, six fan bays in one frame with a blue lever at each end, is what the task models.

## Safety for a real trial

Use a spare server, powered off and unplugged. Keep people clear of the arms while the robot runs and keep an emergency stop in reach.

## Sources

- Dell PowerEdge R720 and R720xd Owner's Manual, pp. 57-58, removing and installing the cooling-fan assembly, Figure 28: https://pcserverandparts.com/content/Spec%20Sheets/Servers/Dell/Dell%20PowerEdge%20R720/Dell%20PowerEdge%20R720%20Owner%27s%20Manual.pdf
- Dell PowerEdge R720 Technical Guide (2U, 87.3 mm high, 444 mm wide): https://pcserverandparts.com/content/Spec%20Sheets/Servers/Dell/Dell%20PowerEdge%20R720/Dell%20PowerEdge%20R720%20Technical%20Guide.pdf
- Dell KB 000234366, cooling fan assembly for the R760, R760xa and R750xa: https://www.dell.com/support/kbdoc/en-us/000234366/poweredge-how-to-remove-and-install-cooling-fans-and-assembly
- R720 fan module part numbers (WPVP9, 3RKJC, KVN8J, WCRWR; 60 x 60 x 38 mm, 12 V): https://altatechnologies.com/products/dell-wpvp9_new and https://www.techforless.com/products/servers_server-options_fans_heatsinks_dell_kvn8j ; Dell community thread on R720 fan identification: https://www.dell.com/community/PowerEdge-Hardware-General/Fan-part-number-identification-compatible-with-R720/td-p/7803225
- IEA, Energy and AI (2025), "Energy demand from AI": data-centre electricity use about 15% a year to 945 TWh in 2030, accelerated servers 30% a year: https://www.iea.org/reports/energy-and-ai/energy-demand-from-ai
- O*NET 15-1232.00 Computer User Support Specialists: https://www.onetonline.org/link/summary/15-1232.00
- O*NET 49-2011.00 Computer, Automated Teller, and Office Machine Repairers: https://www.onetonline.org/link/summary/49-2011.00

No third-party meshes, textures or fonts: every part is built in `task.py`. The robot and station assets are the harness's own.

Dell and PowerEdge are trademarks of Dell Inc. This task is not made, endorsed or sponsored by Dell.

## Licence

MIT
