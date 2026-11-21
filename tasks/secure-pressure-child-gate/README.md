# Secure the pressure-mounted child gate at the foot of the stairs

Block a child's path to a stair flight by setting the removable gate into all four cups in the jambs and locking it under pressure. The Toddleroo Supergate Secure can look like a barrier when it is only standing across the floor. An unsecured gate can shift under a push and open the route to the stairs. The shell and latch are fitted from the adult side. [North States model 8714 setup and warnings, pp. 2–3](https://northstatesind.com/content/manuals/8615-8727_Instruction-Manual_111824.pdf) · [Stock gate](https://northstatesind.com/toddleroo-by-north-states-supergate-secure/)

## Hardware and reset

Use a stock Supergate Secure model 8714, four matching wall cups and a 700 mm-wide timber test opening at the foot of a mock staircase. Use the manufacturer's fitting instructions to position the cups against solid jambs. The kit supports openings from 660 to 1,067 mm. The gate is 660 mm high. Use a dry elevated bench or carpentry mock-up with at least a 1.16 × 1.52 m landing platform to recreate the relative floor and robot geometry. Keep the test area clear of people while running it. The pressure-mounted mode is for a lower landing. The manufacturer's separate hardware-mounted procedure applies at the top of a real stairway.

| Part | Location and dimensions |
|---|---|
| YAM bases | Two rigid bolted mounting plates at X = 0, Y = ±305 mm and Z = 750 mm from the lab floor. The robot on the left has positive Y. Work direction is increasing X. |
| Lower landing | Rigid level surface at Z = 460 mm. Construct a visible stair flight behind the jambs starting approximately 516 mm ahead of the bases. Test both arms with a level landing and rigid jambs. |
| Jambs | Solid wood, inner faces Y = ±350 mm, centered at X = 374 mm. Jamb faces should be vertical and rigid. The common pad centerline of all four attached U cups is X = 374 mm in simulation. Use the stock pad spacing to mark the cup positions in the real opening. |
| Gate | Two molded polypropylene panels, combined bare mass approximately 2.5 kg. The lower rail sits on the lower landing. Reset both panels fully retracted for this opening, with the simulated pad line at X = 355 mm and the hinge-side handle raised. Place the gate so all four pad faces are parallel with their jambs and the handle is visible on the adult side. Assist with placement by hand only at reset. |

With the gate loose, the cups should be visible from both front corners. From the front, the four bumpers should align vertically with their cups while remaining forward of their seats. Use the same measured depth and handle angle for each reset. The reference uses one gripper to guide and steady the lower part of the full-width gate, then the other to rotate the pressure handle. Either arm can do either action. The test requires an installed and released barrier.

## Score

Check all four cup locations. Each pad must be centered in the U recess and touching its seat under pressure. A side view of each jamb and the handle, together with a ruler, is sufficient to distinguish a pad on the rim from a pad inside its cup. Use a spring scale or a force gauge to push the center rail about 5 N toward the stair side for one second after the trial. The gate must stay captured in all four cups. A ruler at the nearby jamb makes movement visible.

- **0** if the lock is open, at least one pad misses its cup or a pad remains unloaded.
- **0.5** if all four cups carry their pads under compression and the lock is closed, before the one-second hands-off hold.
- **1.0** if the gate stays seated with the handle locked and both grippers clear for a full second.

The simulator checks the center of every pad within 6 mm of its cup center in depth and height, 1–8.5 mm of compression independently, and handle rotation in the closed band at its stop. An eccentric cam and follower represent the chosen pressure handle setting. The cam, pads, jambs, floor, fingers, and cup walls engage through native contacts. The manufactured hinge/tab details are abstracted to the mechanically equivalent over-center compression action for this opening.

## Run and score locally

Activate the Isaac Lab-Arena environment, then install the task from this directory. The default seed is 1415158438. The environment runs 240 Hz physics and a 20 Hz control loop. Each trial has at most 410 actions.

```bash
python -m pip install -e . --no-deps
python run.py --device cuda:0 --physics-device cpu --output /tmp/gate-demo
python run.py --device cpu --physics-device cpu --no-cameras --policy hold --output /tmp/gate-hold
python run.py --device cpu --physics-device cpu --no-cameras --policy loose --output /tmp/gate-loose
```

The `pressure_gate` Inspect Robots embodiment exposes the camera views and joint positions. The scripted demo executes normal joint actions. The `hold` control waits at reset. The `loose` control moves the panel between the cups and withdraws with the handle open. Every trial writes a trace and score receipt. Camera trials retain the reset frame and each successive action in three 640 × 360 recordings.

## Assets

The simulated gate, stairs, and mechanism are original procedural geometry under the repository MIT license. The pinned robot package supplies robot and camera assets. [Robot license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt) · [Camera license](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE)
