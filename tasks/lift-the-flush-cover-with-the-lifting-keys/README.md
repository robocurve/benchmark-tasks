# Lift a flush access cover out of its frame with twist-lock lifting keys

A grey cover sits flush in a fixed frame with a 2 mm gap all round, too narrow for a fingertip. Two cross-shaped
keyholes go through it: a 14 mm square for the key's 10 mm shaft, with a narrow slot (8 x 30 mm) through it that only
the key's 4 mm crossbar fits. Two lifting keys stand in holders, one beside each arm. A key goes in only with its
crossbar lined up with the slot. Turned a quarter turn under the cover, the crossbar bears on the cover's underside
and lifts it. Pulled up without the turn, it comes straight back out: the shaft can move only 2 mm in the square, so
the crossbar's ends never pass under the solid cover. The robot has to put both keys through, turn
them, lift the cover out, lay it across two blue bearers (which leave room for the crossbars), then turn the keys
back and draw them out.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. It uses `parts.py`, a small helper in this folder that builds multi-shape parts (the frame,
cover, keys and holders are each made of several boxes or a cylinder); it imports only NumPy and `yam_arena`.
`near_misses.py` lists near-successes and wrong outcomes with the score each must get. It checks `score()` and
`success()` in plain Python and needs no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Lift the grey access cover out of its frame with the lifting keys, lay it flat across the two blue bearers, then draw the keys out."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cover-dream

# Scripted demo (reaches 1.0 after about 18 s of simulated time)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cover-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** run with `-E object_jitter=0`, so every object starts exactly where `task.py` puts it and no seed is
  needed. Jitter would move the cover into the frame walls and the keys into their holders; use `object_jitter=0`.

## How the simulator scores it

- "Lying flat" is computed from the cover's pose and true size: some face within 3 degrees of level, the four lowest
  corners within 1.5 mm of the surface (bearer tops at 20 mm, or the table), moving under 1 cm/s and turning under
  0.5 rad/s. The scorer cannot see the grippers. The 1.5 mm band is tighter than the height at which the demo sets the
  cover down, so a cover still hanging on the keys does not count.
- The task is pass/fail: there is no partial credit.
- A key "passes through the cover" when the key's axis, between its crossbar foot and its handle top, crosses the
  plane of the cover's upper face inside the cover's outline (from the key's and the cover's poses). A key lying on
  the cover is parallel to that plane and does not count.
- "Dropped from more than 30 mm" is kept as a record over the whole trial: the trial scores 0 once the cover moves
  down faster than 0.75 m/s (the speed after a free fall of about 29 mm) or ends up more than 25 mm lower than it was
  one or two control steps before. The record is cleared only at a new trial: when every loose object is back within 0.5 mm of its start,
  at rest, having got there by a jump of more than 20 mm in one control step (a scene reset, which the robot cannot
  produce).

## Scripted demo

Each arm pinches its key's handle from above, draws the key out of its holder and carries it over its keyhole with the
crossbar lined up with the slot (the wrist first turns back the 4 degrees the key turned in the fingers when
pinched). It lowers the crossbar through until it is 4 mm under the cover, then turns it a
quarter turn (left arm -90 degrees, right arm +90). Both arms then lift the cover 22 mm, carry it over the bearers and
lower it onto them, go 4 mm lower to unload the keys, turn them back and draw them out. The two plans have the same
timing, so the arms move the cover together. The arms aim off by the tracking error measured in GPU traces (3 mm in
x at the keyholes, about 2 mm in y, 4 mm in z).

Two GPU traces of the same demo without the quarter turn left the cover in its frame. In the first, with the earlier
40 mm straight slot, the keys went in and came out and the cover rose 0.5 mm at most. In the second, with the
cross-shaped keyhole and each key aimed 2.5 mm outward (past the 2 mm the shaft can move in the square), the keys
could not enter: they rested on the cover top and the cover did not move. Within the 2 mm the crossbar's ends stay
inside the slot, so an unturned key has nothing to hook.

Fingers cannot do the keys' job. A finger tip is about 15.5 x 4 mm. One alone fits only on the diagonal of the 14 mm
square, and the closed pair (15.5 x 8 mm) fits nowhere, so the fingers cannot spread inside a keyhole and lift the
cover by pressing on its edges. The 2 mm gap round the cover is narrower than a finger.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The cases include: on the bearers with the keys out (1.0), with a key still through (0), with a key's foot just
below versus just above the cover top, and with a tilted key through the keyhole (0); a key lying on top of the
cover (1.0); 25 mm versus 26 mm off centre towards the robot; 30 mm versus 31 mm side to side; 1.5 mm versus 2 mm
above the bearers; upside down (1.0); tilted 3.5 degrees; still moving; flat on the table (0); lifted 3 mm or tilted
in the frame; propped across the frame wall; a 25 mm versus 26 mm fall within one step; and falling at 0.75 m/s versus
0.8 m/s (a drop that zeroes the trial).
