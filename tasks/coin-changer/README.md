# Give exactly $1.35 change from a four-barrel belt coin changer into a coin tray held under its chute

A cashier's four-barrel steel coin changer (quarters, dimes, nickels, pennies) stands at belt height. Pressing a barrel's
thumb tab forward vends the bottom coin of that barrel out of a slot and off the changer's front lip, one coin per full
press. A loose coin tray lies on the table at the left. "Give the customer $1.35 change from the coin changer into the
tray."

Exact change is five quarters and one dime. The tray has to be under the chute while the tabs are pressed: a coin that
misses it falls 10 cm to the table and is lost. One hand holds the tray under the chute, the other presses the tabs
(the tray can also be set down under the chute; it does not have to be held).

## Run

From the repo root, with Isaac Lab-Arena installed (the Robocurve Brev launchable, `source /data/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/coin-changer/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Give the customer \$1.35 change from the coin changer into the tray." \
      --scorer task_score --max-steps 2400
    python -B tasks/coin-changer/test_task.py          # offline scorer checks, prints "all pass"

The scripted demo (full score, about 50 s, 1,002 steps) and the failure checks, from `tasks/coin-changer/`:

    python demokit.py task.py OUT --mode full            # full score, with a video
    python demokit.py task.py OUT --no-record --fast --mode none,notray   # failure checks: both score 0

`--mode none`: the arms wait (0). `--mode notray`: the tabs are pressed with no tray under the chute, the six coins fall
on the table (0). The scene is deterministic (no seeds, `object_jitter=0`).

In the demo the left hand takes the tray by its long rim and carries it round the changer to under the chute; the right
hand presses the quarter tab until five quarters are in the tray (an occasional press stalls at about 23 mm and leaves its coin half out; the next press finishes it, so it takes six presses) and then the dime tab once, lifting off the tab each time so the pusher springs back.

Files: `task.py` (scene, scorer, demo), `test_task.py`, `mechkit.py` and `demokit.py` (the kits it imports, copied here).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Coin changer | steel, fixed; 15 x 19 x 0.6 cm base plate on four 1.4 cm legs, plate underside 10 cm above the table | plate x 20 to 35, y -9.5 to 9.5; front lip at x 35 |
| Barrels | four vertical tubes, 5 cm tall, 3 mm walls, inside 1.6 mm wider than the coin; floor slot 0.3 mm taller than the coin | centred x 32; y -6 quarters, -2 dimes, +2 nickels, +6 pennies |
| Coins | 8 per barrel, US size and mass: quarter 24.26 x 1.75 mm 5.67 g, dime 17.91 x 1.35 mm 2.27 g, nickel 21.21 x 1.95 mm 5.00 g, penny 19.05 x 1.52 mm 2.50 g | stacked in the barrels |
| Pushers | 1 mm plate, 14 mm wide, 36 mm travel, 60 N/m return spring, thumb tab 1.0 x 1.4 x 1.6 cm | tabs at x about 22, under their barrels' y |
| Coin tray | loose, 70 g, blue: inside 10 x 17 cm, floor 3 mm, walls 2.2 cm tall and 3 mm thick | centre x 19, y 20 on the table |

## Scoring

Coins are counted from the final state, held 1 s (`SCORE_OVER = "end"`). 1.0: exactly 5 quarters and 1 dime in the tray,
no other coin in the tray, no loose coin, tray tilted 20 degrees or less. 0.5: at least 5 quarters and 1 dime in the tray
but an extra coin in it or a loose coin. 0.25: at least one coin in the tray. 0: otherwise. Where the arms end up does
not count.

## Building it for a real trial

No manufacturer drawing of a belt coin changer was found, so every changer dimension in the simulation is a design value,
not a measurement. Retailer listings of the Nadex / Advantus MCG1-type four-barrel steel belt changer give a capacity of
about 38 quarters, 56 dimes, 37 nickels and 49 pennies and say each barrel's lever vends one coin per push. Before a real
trial, measure the actual changer and replace these values in `task.py`:

| Measure | Tool | Simulation's value |
|---|---|---|
| Barrel spacing, thumb-lever width | Calipers | 40 mm pitch, 14 mm wide |
| Lever stroke to vend a coin | Calipers | 36 mm |
| Force to push the lever, and to let it spring back | Push or fish scale | spring 60 N/m, friction 0.05 N |
| Exit slot height of each barrel | Feeler gauges | coin thickness + 0.3 mm |
| Changer height over the counter, and where the coin drops | Ruler | plate 10 cm above the table |

Use real US coins (a quarter, dime, nickel and penny). The real changer's levers act on a ring or plate at the bottom of the
barrel; the simulation models a sliding plate. Count the coins in the tray and on the counter after the run.
