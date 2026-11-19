# Couple a scaffold transom to a ledger: lay it at its chalk mark, close the cap, swing the bolt in and run the nut down

A scaffolder's everyday step: fix a transom to a ledger with a right-angle coupler. The coupler's lower half is already tightened on the ledger, and its upper
half stands open. The robot lays the transom in the open half with its chalk mark at the ledger, closes the cap over it, swings the T-bolt up into the cap's
slot and runs the nut down until it bears.

It is a sequence of different steps, on a device worked through its own mechanism:

- **The transom.** It is a full-size 48.3 mm steel tube, 70 cm long and 3.1 kg. Its far end lies on the far bearer, so a hand lifts only the near end (about
  15 N) and swings it across while the far end turns on the bearer. The open half only takes the tube square, and the chalk mark must end within 10 mm of
  the ledger's face: a coupler closed off the mark leaves the boards bearing short.
- **The cap.** It lies back on its hinge. A hand swings it up and over the tube; past upright it falls shut and rests shut by its weight.
- **The bolt and nut.** The swing bolt rests outward on the ledger. Pinched by its nut, it swings up and into the cap's slot. The nut then runs down 5
  turns, in wrist strokes with regrips, until it bears on the cap. Until the nut is down, the bolt can drop back out of its slot.

**The arms.** In the demo the right arm lays the transom and works the bolt and nut, and the left arm shuts the cap and holds it. The scoring asks for
the end state only: any division of the work, and either hand or both, is allowed.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`).
- `mechkit.py` and `demokit.py`, in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without modifying them.

## Run a trial

From this folder:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --policy dream -P rows_per_request=20 --scorer task_score --max-steps 4000 \
    --instruction "Fix the transom to the ledger with the coupler at its chalk mark: lay it in, close the cap, put the bolt in the slot and run the nut down."
```

The scene is the same for every seed.

## The demo

From this folder:

```bash
python demokit.py task.py OUT                                          # the full-score run, with video
for m in offmark nocap capfree loose; do python demokit.py task.py OUT_$m --mode $m --fast; done   # the failure checks, each in its own session
```

The demo is a scripted reference, not a policy. It reads the transom's, the cap's, the bolt's and the nut's poses from the simulator; all of them are in
view of the cameras. It never reads the scorer.

1. The right hand takes the transom 4 cm from its near end with a down grip, lifts it 5 cm, and swings the near end round an arc about the far bearer onto the
   coupler's line, the wrist turning with the tube. It lowers the tube into the open half and nudges the mark onto the ledger's face.
2. The left hand pushes the cap's flange round its hinge with closed fingertips, over the top, until it falls shut, and holds it down.
3. The right hand pinches the nut, jaws across it along the bolt's hinge axis, so the pinch acts as a pin, and swings the bolt up into the slot. It then spins
   the nut down in third-of-a-turn strokes, regripping between them, each pinch setting the bolt back upright in its slot.
4. Both hands let go and back away.

It scored 1.0 in GPU physics in 68.8 s of simulated time (1,375 steps): mark 0.5 mm from the ledger's face, cap shut, bolt in its slot, nut within 0.3 mm
of bearing. On a 24 GB RTX GPU, that run took 7.3 minutes with three 640 x 360 cameras. A CPU run without cameras takes under a minute.

Failure checks (`--mode`, CPU physics, each run in its own session):

| mode | what the script does | score |
|---|---|---|
| `offmark` | lays the transom with its mark 35 mm short of the ledger (it ends 21 mm short), then shuts and bolts the coupler | 0.0 |
| `nocap` | never shuts the cap; the bolt swings up and the nut runs down on nothing | 0.25 |
| `loose` | shuts the cap and seats the bolt, but leaves the nut where it started: the bolt drops back out of its slot when let go | 0.25 |
| `capfree` | pushes the cap past upright and lets go of it before the bolt | 1.0 |

`capfree` is not a failure: in this model the cap, once past upright, falls shut and rests shut by its weight, so the left hand's hold steadies it but is
not needed. Whether a real coupler's cap does the same depends on the coupler; the scoring does not ask for the hold.

## Scoring

`--scorer task_score` prints the total. The task file sets `SUCCESS_HOLD_S = 0` and counts its own hold: `score()` is 1.0 only once the 1.0 state has
held 20 control steps (1 s) in a row, and `success()` is true from then on, so the two agree.

- **1.0:** the transom lies in the coupler's upper half, square to the ledger (within 3 degrees), its chalk mark within 10 mm of the ledger's near face
  (an overhang of 150 +/- 10 mm); the cap is shut (within 3 degrees), the bolt is in the cap's slot, the nut bears on the cap (within 0.3 mm); both hands
  are at least 6 cm from the coupler; all held for 1 s (until then, 0.5).
- **0.5:** the transom at its mark with the cap shut and the bolt in its slot, without the 1.0 result (for example, the nut not run down).
- **0.25:** the transom lying in the coupler's upper half at its mark.
- **0:** otherwise; also if the transom falls to the table, or if the coupler is closed with the mark more than 30 mm off.

Real trial: the same lines, read from the video and by an inspector. There, "the nut bears on the cap" is checked by a pull: the coupler grips when a 50 N
pull along the transom does not slide it. If everything else on the 1.0 line holds but the pull slides the transom, the trial scores 0.5.

## Real trials: the build

- **Tube.** Galvanised scaffold tube, 48.3 x 4.0 mm (about 4.4 kg/m): a 60 cm ledger, a 40 cm far bearer and a 70 cm transom (3.1 kg). Mark the transom
  with a white chalk band 15 cm from its near end.
- **Ledger and bearer.** Lay the ledger along y at x 36 cm, its axis 3.5 cm above the table, on two wooden blocks 7 x 5 x 1.1 cm at y -24 and 24 cm,
  clamped to the table. Lay the far bearer along y at x 80 cm, its axis 4.5 cm up, on two blocks 7 x 5 x 2.1 cm at y -15 and 15 cm, clamped.
- **Coupler.** An EN 74-1 class B drop-forged right-angle (double) coupler for 48.3 mm tube, about 1.2 kg. Tighten its lower half on the ledger at y 0, with
  the upper half's cap hinged on the left. Open the upper half: lay the cap back on its hinge, and rest the swing bolt outward on the ledger with its nut 5
  turns short of bearing.
- **Transom start.** Its far end across the far bearer near y 0; its near end at (20, -20) cm, resting 10 cm in from the end in a wooden cradle 3 cm
  along the tube, 6 cm across and 8.9 cm high, with a 90 degree V notch on top, set on the table.
- **Logging.** Film from above and from the side, so that the chalk mark against the ledger's face, the cap, the bolt in its slot and the nut can be seen.
  Then the pull check: 50 N along the transom with a spring balance.

## The model behind it

The ledger, the coupler's lower half and saddle are one fixed articulation's frame. The cap (0.35 kg, on its hinge) and the bolt (on its hinge, the nut
sliding and turning on it) are links. The cap, the bolt and the nut collide with each other and with the transom; the frame does not collide with
the cap or the bolt, which hinge on it. The bolt's root bears on the slot's cheeks and closed end, and the nut bears on the cap. The thread is code: the nut's slide follows its turn at 2 mm a turn
(a simplification: a real M12 coarse thread is 1.75 mm a turn), from 5 turns out to bearing, and at bearing the nut stops turning. The final tightening with a podger (about 50 N m) is left out. The nut is 19 mm
across flats, not the usual 21 or 22 mm of an EN 74 coupler: a 22 mm nut rubbed the cap's ring as it ran down in this model. The bolt's root bears on the
slot's closed end 7 degrees past upright. The transom is a free rigid tube; friction coefficients 0.5 for the tube and 0.4 for the coupler (assumed, steel on steel).

## Attribution

The task statement quoted in `task.json` is from the O*NET database (O*NET OnLine, task statements for 47-2061.00) by the U.S. Department of Labor, Employment and Training Administration
(USDOL/ETA), used under the CC BY 4.0 licence. O*NET is a trademark of USDOL/ETA.

## Assets

Every mesh is generated by `task.py` and `mechkit.py`. There are no textures and no third-party assets.

## Licence

MIT
