# Serve a bowl of soup with a ladle

> Serve a bowl of soup: two ladlefuls.

## An everyday skill

Ladling soup into a bowl is done every day at family dinners, in school and hospital canteens and on every
restaurant line. A full ladle is a deep cup on a long handle: tilted a few degrees it spills, and moved off quickly
it sloshes over. So it is dipped deep enough to fill, lifted out level, tipped back a little over the pot to shed the
brim, held there a moment while the drips fall back into the pot, carried level, and tipped low over the middle of the
bowl so nothing runs onto the rim. Carry it straight across and it drips and sloshes on the counter; pour from high
and it splashes. Nothing happens in the trial that the robot doesn't cause: every drop goes where its handling of the
ladle sends it.

## Scene (station frame: x forward from the arms, y to the robot's left, cm; heights above the table top)

| Object | Details |
| --- | --- |
| Pad | grey absorbent disposable underpad, 70 x 50 cm, centred at (38, -5), under the pot and the bowl; weighed before the trial |
| Pot | 3 qt stainless steel pot, lid off: inside 21 cm across and 8.5 cm deep, two side loop handles along y; centred at (40, -15) |
| Soup | 2.40 kg of room-temperature tap water with orange food colouring, in the pot |
| Ladle | 8 oz stainless steel ladle with a black soft-grip handle: bowl 10 cm across and 4.6 cm deep (about 220 mL to its brim), handle rising at 65 deg from the bowl's rim, grip 24 x 16 mm, about 31 cm overall. It stands in the pot, its bowl on the floor near the middle, leaning 10 deg with its handle on the rim, pointing back and to the right (150 deg from straight ahead) |
| Bowl | white ceramic soup bowl, 16.6 cm across the rim, 7.2 cm tall, about 700 mL to the brim; empty and dry, centred at (34, 7) |
| Real trials only | a nitrile cover over each gripper's finger pads |

Both arms start at the ready pose. One fixed layout, as in the screenshot.

## Scoring (task.py)

- **0 (latched for the trial):** the pot or the bowl is moved more than 3 cm or tipped (past 10 deg), or the ladle is
  dropped (down on the pad or the table outside the pot).
- Otherwise, after the trial: the bowl's gain W is weighed (1 g per mL); the soup anywhere outside it, S, is the pad's
  gain plus what weighed paper towels wipe off the bowl's rim and outside, the pot's outside, the table and the finger
  covers (a 0.1 g kitchen scale).
  - **1.0** if W is 340 to 480 g, poured in exactly two pours, S is at most 3 g, and at the end
    the ladle is back in the pot and released (no gripper within 6 cm of its handle), for 2 s;
  - **0.5** if W is at least 150 g and S at most 15 g (one ladleful, three pours, a messy serve, or the ladle not put back);
  - **0.25** if W is at least 40 g;
  - **0** otherwise. Doing nothing scores 0.
- A pour is soup running from the ladle into the bowl, at least 20 g of it, with at least 1 s without flow between two
  pours (counted on the video; the bowl can be weighed between pours if it is unclear).

## Running it

    source <IsaacLab-Arena>/yam.env
    python make_assets.py                        # writes assets/*.usda (pxr only)
    python demo.py task.py out                   # scripted full-score demo
    python demo.py task.py out --variant rush    # control: carries each ladleful across quickly, no pause for drips
    python demo.py task.py out --variant one     # control: serves one ladleful
    python demo.py task.py out --variant idle    # control: does nothing

A policy run (Dream-1) uses the same task file through Inspect Robots:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 --policy dream \
        --max-steps 3600 --instruction "$(python -c 'import task; print(task.INSTRUCTION)')" --scorer task_score

Run limit: 3,600 steps (3 minutes at 20 Hz); the demo takes 1,267. A trial takes about 15 minutes of wall time on one
GPU. No randomness.

## Results (simulation)

| Run | Score |
| --- | --- |
| demo | 1.0 in 1,267 steps (430 g in the bowl in two pours; 0.9 g outside it) |
| rush: carries each ladleful across quickly, no pause for drips | 0.5 (402 g served; 13 g sloshed and dripped on the pad) |
| one: serves one ladleful | 0.5 (215 g) |
| idle | 0 |
| Dream-1 (dream-1-beta, 1 trial) | 0: holds the ladle in the pot for 2 minutes, then lifts it out with a part-ladleful and tips it on the way to the bowl (10 g in the bowl, 78 g on the pad), and puts it back in the pot |

The demo: the right gripper takes the ladle's soft handle across its flat faces, 4 cm from its end, pointing down the
handle. It lifts the ladle out of its lean, levels it over the pot, dips it until its rim is under the soup, lifts it
out level, tips it 6 deg to shed the brim back into the pot and holds it there for 4 s while it drips. It carries the
ladle over the gap between the pot and the bowl and down until the lip is 6 cm over the bowl's rim, then tips it,
turning about the lip so the lip stays over the middle of the bowl, holds it there for the last drips, and goes back
for the second ladleful. Then it lowers the ladle back into the pot and lets go. The arm's soft joints let the full
ladle sag in the hand, so the demo tracks the ladle's measured pose and corrects its command. It reads the levels
from the task state (the top camera sees the soup in the pot, in the ladle and in the bowl).

## Fidelity

Rigid bodies, 240 Hz physics, 20 Hz control. The soup is a model (`soup.py`). The ladle's bowl is a spherical cap;
the soup in it is level, perpendicular to gravity leaned by the ladle's acceleration through one slosh mode (2.9 Hz,
lightly damped), and its volume is counted on a grid of points filling the bowl. Soup above the rim's lowest point
runs over it at a rate set by the head over the lip (Q = K h^2), leaves the lip outwards and falls, landing where it
falls: in the bowl, on its rim, in the pot or on the pad. Dipped with its rim under the soup, the ladle fills towards
the pot's level. Its underside carries 3 mL out of the soup (and a share of what runs over the lip), which drips off
its lowest point over about 2 s, straight down. Soup that falls more than 15 cm into the bowl splashes part of itself
onto the rim. The weight of the soup in the ladle is applied to the ladle at the soup's centre. The pot's wall is 24
boxes round its circle, so the ladle goes in and out of it as a real pot; the bowl collides as its outside. Soup is
drawn as shapes whose size follows the model (the surface in the ladle is the circle where the level plane cuts
the ladle's bowl, so it tilts and shrinks with the ladle). Real trials use room-temperature tinted water: nothing hot is near the
robot, and the pad and towel are weighed to measure what lands outside the bowl.

## Files

`task.py` (scene, monitor, scoring), `soup.py` (the soup model), `demo.py` (scripted demo and controls),
`make_assets.py` (USD generator), `assets/` (generated USD), `dynkit.py` and `demokit.py` (shared helpers: USD bodies,
kinematic visual bodies, the episode clock, Cartesian IK demo runner). All assets are made by this task's code; no
third-party assets.
