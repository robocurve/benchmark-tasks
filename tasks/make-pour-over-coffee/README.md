# Make a mug of pour-over coffee

> Make a mug of coffee with the pour-over.

## An everyday skill

Making a mug of filter coffee by hand is a daily routine in homes, offices and cafes. The paper filter drains far
more slowly than a kettle pours, so you pour, watch the coffee rise in the cone, tilt the kettle back before it
reaches the rim (over it, water and grounds run round the paper into the mug), wait for it to drain, and pour again,
keeping the stream on the grounds. The coffee still in the cone keeps dripping after you stop, so you stop pouring a
little before the mug is full. Pouring is the robot's own process: it starts the flow, has to watch what it is doing
to the cone, stop, wait and resume, and end at the right amount. Nothing happens in the trial that the robot doesn't
cause.

## Scene (station frame: x forward from the arms, y to the robot's left, cm; heights above the table top)

| Object | Details |
| --- | --- |
| Glass mug | clear heat-resistant glass mug without handle, 8.0 cm across, 8.5 cm tall, 6 mm base (about 340 mL to the brim), a thin black line drawn round it at 220 mL (5.7 cm above its underside); base centred at (38, 0), on the drip tray |
| Dripper | Hario V60 size 01 plastic dripper, white (VD-01W; 11.5 x 10 x 8.2 cm), sitting on the mug, its handle tab to the robot's left |
| Filter and coffee | one Hario V60 01 paper filter (VCF-01) holding 15 g of Folgers Classic Roast ground coffee (medium grind, for drip makers; from a freshly opened canister), levelled with one shake |
| Kettle | Hario V60 Drip Kettle Air (VKA-35-TB, 350 mL, clear resin; 17.7 x 7.8 x 12.8 cm), filled to its 350 mL mark with room-temperature water; base centred at (34, -12), spout pointing away from the right arm (28 deg to the left of straight ahead), handle towards the right arm |
| Drip tray | grey plastic tray, 24 x 16 cm, 5 mm thick, centred at (40, 1) |
| Real trials only | an absorbent pad over the table under the tray and the kettle; a nitrile finger cover on each gripper pad |

Both arms start at the ready pose. One fixed layout, as in the screenshot.

## Scoring (task.py)

- **0 (latched for the trial):** the water rises over the cone's rim; water lands outside the dripper (more than
  about 1 mL, some 20 drops, on the tray, the mug's rim, the pad or the table: checked on the video and with a paper towel
  after the trial); the dripper is
  lifted or knocked off the mug while it holds water; or the mug tips over (past 12 deg).
- Otherwise the brew is weighed once the cone has stopped dripping (after the trial, outside the run limit; mug minus
  its tare, on a 1 g kitchen scale). In simulation that is the mug's content plus the water still standing in the cone.
  - **1.0** if it is 200 to 240 g and, at the end of the trial, the kettle stands upright (within 10 deg) on the table, released (no
    gripper within 7 cm of its handle), for 2 s;
  - **0.5** if it is 150 to under 200 g, or over 240 g, or 200 to 240 g with the kettle not put down;
  - **0.25** if it is 40 to under 150 g;
  - **0** otherwise. Doing nothing scores 0.

Pouring at the kettle's full flow without stopping overflows the cone before a mugful has gone in (in the model, at
about 190 g brewed), and so does a steady slower pour (about 205 g); filling the cone once and stopping gives about
170 g (0.5). Any pacing that keeps the cone below its rim is allowed: pouring in fills, or pouring and easing off.

## Running it

    source <IsaacLab-Arena>/yam.env
    python make_assets.py                        # writes assets/*.usda (pxr only)
    python demo.py task.py out                   # scripted full-score demo
    python demo.py task.py out --variant full    # control: pours at full flow without stopping
    python demo.py task.py out --variant once    # control: fills the cone once to near its rim, then stops
    python demo.py task.py out --variant idle    # control: does nothing

A policy run (Dream-1) uses the same task file through Inspect Robots:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 --policy dream \
        --max-steps 4800 --instruction "$(python -c 'import task; print(task.INSTRUCTION)')" --scorer task_score

Run limit: 4,800 steps (4 minutes at 20 Hz); the demo takes 2,441. A trial takes about 10 minutes of wall time
on one GPU. No randomness.

## Results (simulation)

| Run | Score |
| --- | --- |
| demo | 1.0 in 2,441 steps (three pours; the cone peaked at 111 mL, its rim is at 158 mL; 250 mL poured, 220 mL brewed) |
| full: pours at the kettle's full flow without stopping | 0 (over the cone's rim at 31.4 s, 191 mL brewed by then) |
| once: fills the cone once to near its rim, then stops | 0.5 (172 mL brewed) |
| idle | 0 |
| Dream-1 (dream-1-beta, one trial, 4,800 steps) | 0 (brings a gripper to the kettle and reaches over the dripper, never lifts the kettle; nothing poured) |

The demo: the right gripper takes the kettle by its loop handle, pitched 12 deg down, fingers either side of the
handle's bar; the left arm stands off to the side with its wrist camera on the glass mug. The right arm lifts the
kettle and brings the spout tip 6 cm above the cone, just short of its middle, and tilts the kettle until the water
runs at about 9 mL/s. When the cone is about two-thirds full (105 mL standing) it tilts the kettle back, waits until
the cone has drained to 60 mL, and pours again. It stops once the mug and the cone together hold 220 mL (counting the
stream still falling), and puts the kettle back where it stood. It reads the levels from the task state (the top
camera sees the coffee in the cone, the side camera the coffee in the clear mug). The arm's soft joints let the
kettle sag in the hand, so the demo tracks the kettle's measured pose and corrects its command.

## Fidelity

Rigid bodies, 240 Hz physics, 20 Hz control. Water is a model (`brew.py`): the kettle's outflow follows the head of
water over its spout tip as it is tilted (with the kettle's real shape, the flow starts near 14 deg and needs about
50 deg once it is two-thirds empty); the stream falls from the tip and counts as in the cone only if it crosses the
rim's plane inside the rim; the grounds first soak up 30 mL; water standing above them drains at a rate proportional
to its depth (about 2.7 mL/s with the cone full: a full cone drains to half in about 40 s); the cone holds about
158 mL above the grounds. Water, the stream and the coffee are drawn as shapes whose size follows the model; the mug's
glass is drawn as its edges. The kettle is drawn opaque, and its mass is fixed at its mid-pour value (0.35 kg).
Real trials use room-temperature water: the pour and the drainage are the same, and no hot water is near the robot.

## Files

`task.py` (scene, monitor, scoring), `brew.py` (the water model), `demo.py` (scripted demo and controls),
`make_assets.py` (USD generator), `assets/` (generated USD), `dynkit.py` and `demokit.py` (shared helpers: USD bodies,
kinematic visual bodies, the episode clock, Cartesian IK demo runner). All assets are made by this task's code; no
third-party assets.
