# Fill the bottle with beads

A bottle stands in the middle of the bench with a 40 mm mouth. A jug of 9 mm beads stands to one
side of it and a funnel to the other. The beads will not go into the bottle by pouring: a scripted
pour with a perfectly solved aim, from two different heights, puts them on the bench. The funnel has
to be picked up, fitted into the neck and left there before the jug is tipped.

The instruction names neither the funnel nor the method: **"Fill the bottle with beads."**

## Environment

| | |
|---|---|
| `yam-isaac-lab-arena` | `0059d02` |
| Isaac Lab-Arena | 0.3.1 (installed: `isaaclab_arena` 0.3.0) |
| Isaac Sim / Isaac Lab | 6.1.0 / 3.0 |
| GPU used for the runs below | one RTX-class GPU, `cuda:0` |
| Control | 20 Hz, `object_jitter=0` |

Start every shell with `source <ARENA_DIR>/yam.env`.

## Files

| File | What |
|---|---|
| `funnel_beads.py` | the task file: `SCENE`, `OBJECTS`, `success`, `distance`, `score`, `demo` |
| `bench.py` | the scene's geometry and the measurements the scorer takes |
| `simkit.py` | the spawn helper: materials, compound bodies, per-surface friction, smooth motion |
| `shortcuts.py` | four scripted shortcut policies |
| `check_layout.py` | offline checks: the geometry, the clearances, the demo's IK, the pour mechanics |

No asset files ship with the task. Every body is built from Isaac Lab mesh primitives.

## Running it

```bash
cd <this repo>

# the scripted demo (reaches full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/funnel-beads/funnel_beads.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the bottle with beads." --scorer task_score \
  --max-steps 1000 --store-frames --log-dir ~/yam-eval/fb-demo

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/funnel-beads/funnel_beads.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the bottle with beads." --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/fb-dream

# one shortcut policy (hold | tip | high | shove)
FB_SHORTCUT=tip inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/funnel-beads/shortcuts.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the bottle with beads." --scorer task_score \
  --max-steps 1000 --log-dir ~/yam-eval/fb-sc-tip

# offline, no GPU
cd tasks/funnel-beads
python check_layout.py
```

`FUNNEL_TRACE=1` on any run prints the funnel's pose and tilt, the jug's pose and tilt, and the
position of every undelivered bead every twentieth control step; that is how the numbers below were
measured.

**Seeds and determinism.** The layout is fixed: with `-E object_jitter=0` nothing is moved at reset,
so there is one starting state and no seed to set. The scripted demo is open loop, so it replays
identically. PhysX is not bit-deterministic (`deterministic=False` in Arena's default `PhysxCfg`), so
individual beads take slightly different paths between runs; the demo delivers all 24 against a
threshold of 22.

**Runtime per trial.** The longer arm's plan is 45.8 s of motion, 916 control steps at 20 Hz, and the
episode ends on success a little before that. Wall-clock: roughly 1 to 2 minutes per trial once
Isaac Sim is up, and 3 to 6 minutes for the first trial in a fresh process.

## Results

| Policy | `task_score` | Notes |
|---|---|---|
| `yam_scripted` (the demo) | **1.0** | 24 of 24 beads delivered; ends with `termination_reason="success"` |
| `dream` (Dream-1 beta) | **0.0** | ran the full 2,400 steps, no bead delivered |
| shortcut `hold` | 0.0 | both arms hold the ready pose |
| shortcut `tip` | 0.0 | the jug poured straight at the bottle's mouth, funnel left on the bench |
| shortcut `high` | 0.0 | the same pour from the height the real demo pours at |
| shortcut `shove` | 0.0 | both arms lean on the bottle |

## Why the pour fails without the funnel

`tip` and `high` are not sloppy attempts. They run the real demo's jug sequence unchanged -- the same
body grip on the jug's own axis, the same solved aim that puts the jug's mouth over the bottle's
axis, the same slow 80-degree tip -- and differ only in that nothing has been put in the mouth to
catch what comes out. `tip` pours from as low as the jug's rim can go without fouling the neck.

The reason both score zero is that the jug's mouth is not where the hand is. Tipped to 80 degrees the
jug's lip hangs 34 mm below the grip and about 30 mm to one side of it, so what is actually being
aimed is a point a third of an arm's width from the thing the arm is being positioned by, and the
40 mm mouth is smaller than that offset. The funnel replaces a 40 mm target with a 110 mm one.

## The pour mechanics, and why the jug is shaped the way it is

Three measurements decided the design, all of them in `check_layout.py` as checks:

- **A straight-sided cup does not empty.** Tipped to 90 degrees it is a horizontal tube: the beads
  stop resting on the floor, come to rest along the side wall, and that wall is now level. Past 90
  the tube would slope towards the mouth, but 92 degrees is the whole of this arm's reach at the
  bottle's axis, and less than that anywhere else. The jug is therefore a cone inside and a cylinder
  outside: the lining rises at 17.3 degrees, which holds the beads in while the jug is carried at
  5 degrees and points 7 degrees downhill at the mouth once it is 80 degrees over.
- **The tilt has to happen in place.** The lining lets go at about 73 degrees, and tipping while
  still crossing the bench passed through that 70 mm short of the funnel. The demo squares the jug
  over the funnel at carrying tilt first, then tips.
- **The pour height is bounded at both ends.** The jug's rim swings nearly a full radius below the
  hand as it goes over. From 45 mm up that rim was inside the cone and knocked the funnel off the
  bottle; from 130 mm the beads bounced back out. 75 mm leaves 41 mm between the two.

## How the funnel seats

The funnel is carried upright to within a degree, and every early failure happened in the last
20 mm of the descent: it arrived about 11 mm off the bottle's axis -- it swings a little on its
handle -- and the bore had 10 mm of play, so the spout landed on the rim and the 81 mm handle levered
the funnel onto its side. Two changes fixed it, and both are checked offline:

- **A lip.** The top 14 mm of the neck is flared out to 56 mm, so the spout touches down inside a
  lead-in 12 mm wider than itself and is guided into the bore instead of landing on a rim. The flare
  is cut into the neck rather than added on top of it, because the left arm is already at the limit
  of its reach carrying the funnel across at the height of the mouth.
- **A pause.** The demo stops over the bottle for 1.2 s before descending, which lets the funnel stop
  swinging. That alone took the arrival error from about 11 mm to about 2 mm.

A grip on the funnel's own axis would be better still -- that is what cured the jug -- but the only
on-axis surface is the spout, and with the wrist over the bottle the left arm can reach the release
height and nothing above it. The handle buys back the 81 mm of reach that makes the approach
possible.
