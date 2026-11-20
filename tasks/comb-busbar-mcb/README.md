# Open a row of breaker terminals, lower a comb busbar into all at once, then screw each terminal down onto its pin

A row of DIN-rail miniature circuit breakers is commonly fed by a pin comb busbar. The maker's steps (Contactum consumer unit instructions): "Loosen the
MCB terminal screws. Fit the busbar and tighten the terminal screws." Each breaker's line terminal is a box clamp, shut until its screw is backed out, and
a rigid comb fits only when every clamp in the row is open.

The difficulty is in the hands:

- **Open the whole row.** Each terminal screw is backed out about 3.75 turns with a plain screwdriver, in wrist strokes: the bit kept on the head through
  each turn, lifted off, the wrist turned back, the bit set in again. A clamp left shut stops the whole comb.
- **Insert into all at once.** The comb's pins enter all the open terminals together, in one straight push down from above; skewed, a pin catches.
- **Close the row.** Each screw is driven back down until it bears on its pin and clamps it.

**The arms.** In the demo the right arm works the screwdriver and the left arm lays the comb. The scoring asks for the result only: any division of the
work is allowed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.
  This copy of `mechkit.py` adds a centre-of-mass option to `free()`, used for the screwdriver.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 10000 \
    --instruction "Fit the comb busbar to the three breakers: open their top terminals, lower the comb's pins into all three, tighten the terminal screws, then put the screwdriver down and move both hands away."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                          # the full-score run, with video
python demokit.py task.py OUT_checks --mode closed,behind,loose,hover,leave --fast  # the failure checks
python check_score.py                                  # the scorer on hand-built states, with no simulator
```

The demo is a scripted reference, not a policy. It reads the screwdriver's, the comb's and the terminals' poses from the simulator; all of them are in view
of the cameras. It never reads the scorer.

1. The right hand takes the screwdriver from its stand by the end of its handle, draws it back clear of the breakers and squares it to the row.
2. For each breaker it sets the bit on the terminal screw's head and backs the screw out in strokes of 205 degrees of wrist roll, keeping the bit's tip on
   the head through each turn, until the clamp is open. It parks the screwdriver.
3. The left hand takes the comb by its bar over the free fourth pin, carries it over the row and lowers its pins into all three open terminals, then lets
   go: the comb rests on the breakers' tops.
4. The right hand drives each screw back down in strokes until it bears on its pin, lays the screwdriver back in its stand and backs away.

It scored 1.0 in GPU physics in 234.7 s of simulated time (4,693 steps). A CPU run without cameras takes about four minutes.

Failure checks (`--mode`, CPU physics):

| mode | what the script does | score |
|---|---|---|
| `closed` | lowers the comb onto the row without opening the terminals | 0.0 (no pin seats in a clamp's gap) |
| `behind` | without opening the terminals, lowers the comb with its pins 2.4 mm behind the shut jaws, then pushes it forward onto them | 0.0 |
| `loose` | opens the row and seats the comb, but leaves the screws open | 0.34 |
| `hover` | turns the screwdriver with its tip held 4 mm in front of each head, never on it | 0.0 (the screws do not turn) |
| `leave` | does everything, but sets the bit back on the last screw's head and keeps hold of the screwdriver | 0.64 |
| `none` | does nothing | 0.0 |

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 40 control steps (2 s) in a row, with the pull test inside it, and `success()` is true from then on, so the two agree.

- **0:** once the comb or the screwdriver has fallen off the table, for the rest of the trial.
- **0.03 for each terminal opened:** its screw backed out far enough for a pin (3.7 turns), counted once it has been (0.09 for the row).
- **0.25 more while the comb is home:** all three pins in their clamps (in front of the jaws) and its bar down on the breakers' tops (within 1.5 mm), square to the row in plan
  (within 3 degrees).
- **0.1 more for each screw down,** driven onto its pin and clamping it: 0.64 with all three.
- **1.0:** all three screws down; every fingertip of both hands at least 5 cm from the whole comb and 4 cm from the whole screwdriver; the screwdriver's
  tip at least 2 cm from the screw heads; and a 10 N pull up along the pins, out of the terminals, moving the comb less than 1 mm; all held 2 s. A pull
  that moves the comb more leaves the trial at its partial score.

Real trial: read from the film and checked by hand. A terminal is open when a 3 mm gauge pin drops into its clamp. The bar is down when a 1.5 mm feeler
gauge does not pass under it, and square when a try square laid along the row's fronts meets it within 3 degrees. A screw is down when it turns no
further by fingers. The pull is made with a spring balance hooked under the comb's bar, the comb's rise read on a dial gauge. The hands' distances are read
from the film against a tape on the bench, and the 2 s hold by the film's timer.

## Real trials: the build

- **Board.** A painted steel mounting plate on a wooden board, standing upright at x 38 cm, with a 35 mm DIN rail across it 15 cm up.
- **Breakers.** Three single-pole miniature circuit breakers of one range with box-clamp line terminals in their tops and M5 terminal screws in their
  fronts (for example Schneider Electric Acti9 iC60N 1P: 91 x 18 x 78.5 mm, 0.125 kg, 18 mm pitch; the model is 85 x 18 x 73 mm), clipped on the rail
  side by side with their fronts towards the arms. Back each screw down onto its empty clamp before a trial.
- **Comb busbar.** The same range's comb busbar (for example Acti9 A9XPH112, 18 mm pitch; the maker calls its flat pins teeth), cut to four pins; the fourth stands free beyond the row as the hand's grip (in a real panel it is
  cut off or capped). Measure its pin size (modelled 3 x 6 mm). Lay it on two thin posts at (21.5, 16) cm, its bar 10 cm up.
- **Screwdriver.** A plain #2 Pozidriv screwdriver about 11 cm long with a 32 mm handle, in a V-stand whose rests are at x 24.7 to 28.7 cm, y -15.5 cm, its axis 17 cm up (the screwdriver from 22.5 to 33.5 cm).
- **Safety.** De-energised breakers only, never connected to supply.

## The model behind it

The breakers are one fixed articulation. Each terminal's screw turns on a hinge and its clamp's jaw slides on a slider. The thread is code: the
screwdriver's turn about its own axis turns a screw only while its bit's tip is on that screw's head (within 0.6 mm of its face and 2.5 mm of its centre,
the shaft within 10 degrees of square), either way, as a plain screwdriver does; the jaw follows at 0.8 mm a turn and stops on whatever it closes on. A
screw driven home onto its pin clamps it: its jaw is driven 2 mm past contact on a 20 N/mm drive, capped at 60 N; the jaw sinks into the pin, so it presses with about 10 N, in place of the
maker's 2 N m tightening torque, which is beyond a gripped screwdriver. The comb and the screwdriver are free rigid bodies; the screwdriver's centre of mass is
set in its handle. The screw heads have no collider of their own: where the bit's tip is on a head is read in code.

## Attribution

The task statements quoted in `task.json` are from the O*NET database (O*NET OnLine, task statements for 47-2111.00,
https://www.onetonline.org/link/summary/47-2111.00) by the U.S. Department of Labor, Employment and Training Administration (USDOL/ETA), used under the
CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA. Sources quoted:

- the installation steps: Contactum consumer unit instructions (https://www.tlc-direct.co.uk/Technical/DataSheets/Contactum/Cu_Instructions.pdf);
- the breaker's figures: Schneider Electric product data sheet, Acti9 iC60N 1P (A9F04116W, https://docs.rs-online.com/11bf/A700000015874965.pdf);
- the comb busbar: Schneider Electric product data sheet, Acti9 comb busbar (A9XPH112W, https://docs.rs-online.com/da9d/A700000015872170.pdf).

Contactum, Schneider Electric and Acti9 are trademarks of their owners, named for identification only.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT
