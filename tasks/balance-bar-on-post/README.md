# Balance a bar on a post and load both ends

A square wooden post 50 mm wide and 110 mm tall stands fixed on the midline. A loose pine bar (300 x 64 x 19 mm)
lies on the table nearer the arms, and a 40 mm aluminium cube sits on each side. The robot has to set the bar level
and centred across the post top, then put one cube on each end so that the bar stays balanced by itself. Nothing
snaps, clips or hinges: the bar rests on a flat 50 mm top, and it stays level only while the
centre of mass of the bar and its load is over that top. One cube on one end alone tips it (worst case: centre of
mass 11 mm past the post's edge), and on this tall post the tipped end drops to 55-73 deg, where the cube cannot
stay. The demo loads both ends at the same moment, one arm each; steadying the bar with one hand while the other loads,
or bringing a cube out along the bar gradually, are other ways to keep it level.

Instruction: *Balance the wooden bar across the top of the post, then put one metal block on each end of the bar so that the bar stays balanced on the post by itself.*

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `distance`, `score` (`SCORE_OVER = "end"`), `demo` (both arms), and `one_end_first_demo` (a check, not a hook) |
| `task.json` | the submission metadata (`media` is added by the upload script) |
| `test_task.py` | offline checks, NumPy and pytest only |

## Run a trial

Built against the YAM Isaac Lab-Arena embodiment at commit `bd62771`. On the GPU machine with `inspect-robots`
installed, copy this folder to `~/balance-bar-on-post`. Always run with `-E object_jitter=0` (see Seeds).

```bash
export INSTR="Balance the wooden bar across the top of the post, then put one metal block on each end of the bar so that the bar stays balanced on the post by itself."
```

Scripted demo (full score expected). The plan is 23.9 s (478 steps at 20 Hz), including a 2 s final hold; success
should end it about 1.5 s after the two cubes are let go (t = 19.0 s):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/balance-bar-on-post/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 650 --store-frames --log-dir ~/yam-eval/bar-post-demo
```

Dream-1 (the run limit is 2,400 steps, 2 minutes at 20 Hz):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$HOME/balance-bar-on-post/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/bar-post-dream
```

One-end-first check (Verify item 5): play `one_end_first_demo` from a scratch copy kept outside this folder.

```bash
mkdir -p ~/scratch && sed 's/^def demo():/def two_arm_demo():/; s/^def one_end_first_demo():/def demo():/' $HOME/balance-bar-on-post/task.py > ~/scratch/bar_post_one_end.py
```

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$HOME/scratch/bar_post_one_end.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 650 --store-frames --log-dir ~/yam-eval/bar-post-one-end
```

Videos (one MP4 per camera, in each log's `frames/` folder):

```bash
for log in ~/yam-eval/bar-post-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score`: 1.0 when the trial ends in success, otherwise `score()` at the last step.

- **Runtime per trial:** wall clock on an RTX 5090, Isaac start-up included: 52 s for the demo, 209 s for the
  Dream-1 run. Simulated time: about 24 s for the demo, at most 120 s for Dream-1.
- **Seeds:** run with `-E object_jitter=0`; the start layout is then fixed and the seed does not change it. The
  demo is open-loop: with the default 3 cm jitter the bar would be gripped off its middle and land off-centre.
- **Assets:** none. Every object is a `Box` primitive; there is nothing to attribute.

## Scoring

Measured when the robot has stopped (`score()` sees object states only):

| outcome | score |
|---|---|
| bar balanced, one cube resting on each end, everything still (success, held 1 s) | **1.0** |
| bar balanced with cubes on it, but not one on each end (one cube anywhere on the bar; one on each half with at least one not on its end or still moving) | **0.5** |
| bar balanced and empty | **0.3** |
| bar not balanced (on the table, tipped, off-centre beyond tolerance, moving) | 0 |
| both cubes on the same half of the bar (side by side or stacked) | 0 |
| any object on the floor | 0 |

- **Balanced:** the bar's underside within 3 mm above the post top (or up to 2 mm into it: contact penetration),
  its top face within 5 deg of level, its middle within 10 mm of the post's axis along the bar and within 25 mm
  across it, and still (`resting()`: under 1 cm/s and 0.5 rad/s).
- **On an end:** resting directly on the bar's top face (bottom within 3 mm), centre over the bar and within
  50 mm of that end (the outer face no more than 30 mm in from the end; overhanging the end is allowed while the
  centre is on the bar), and still. The demo's spots are 25 mm in from each end, so a cube may land 24 mm either
  way along the bar and still count.
- **On the bar** (for the 0.5 and same-half rules): centre over the bar's top face, from resting on it up to one
  cube height higher, so a cube stacked on the other or held just above the bar counts. Halves are split at the
  bar's middle, in the bar's own frame (any yaw).
- An end cube on its own can only be there while something holds the bar level (it tips the bar otherwise); that
  is a hand steadying the bar while the other loads, real progress, and it earns 0.5. A cube held 6 mm above its
  end does not count as on the end.

`test_task.py` checks every branch: the goal state (1.0 and zero distance) and full score wherever the cubes land
within the end zones, with the bar up to 9 mm off-centre, tilted 4 deg, or the cubes swapped; empty (0.3); one
cube at an end, in the middle or near the post (0.5); one cube per half not at the ends (0.5); a cube still moving
(0.5); five same-half layouts including two stacks (0); a tipped bar at 6 and 50 deg, loaded or not (0); the bar
12 mm off along or 28 mm off across (0); moving (0); lifted 5 mm or pressed 4 mm (0); the load built on a bar lying
on the table (0); and each object on the floor (0).

## Balance: the derivation

A rigid bar on a fixed flat top stays put while the combined centre of mass lies over the top; along the bar
that is |y_G| < w/2. Symbols: post width along the bar w = 50 mm; bar mass M = 165 g (140-200 g for a real pine
board, all checked); cube mass m = 173 g; the bar's middle e off the post's axis along the bar; cubes at e + y_i.

    y_G = (M e + m sum(e + y_i)) / (M + m n)

**Tipping condition for one cube.** A cube at distance d from the bar's middle, on the side the post edge is
w/2 - e away, tips the bar onto that end when its torque about the edge beats the bar's:

    m (d + e - w/2) > M (w/2 - e)        (equivalently  e + m d / (M + m) > w/2)

| case (worst placement) | y_G or torque ratio | result |
|---|---|---|
| empty bar | y_G = e: balances for any \|e\| < 25 mm | the scored 10 mm centring leaves 15 mm to the edge |
| a cube on each end at d = 125 mm, bar e = +10 mm, both cubes 15 mm off toward +y | y_G = e + m(d_1 + d_2)/(M + 2m) = 20.2 mm (20.7 at 140 g, 19.5 at 200 g) | **level**, at least 4.3 mm inside the edge; 0 with everything on its spot |
| one cube at the end zone's inner edge (d = 100 mm), bar 10 mm toward the other end (e = -10 mm) | y_G = 41.2 mm (36.4 at 200 g); torque ratio 1.95 (1.61 at 200 g) | **tips**, 11 mm or more past the edge |
| one cube at the demo's spot, 15 mm short (d = 110 mm), e = -10 mm | y_G = 46.3 mm (41.0 at 200 g) | **tips** |
| one cube at the demo's spot, bar balanced at the very edge (e = -25 mm) | y_G = 39.0 mm (33.0 at 200 g) | **tips** |
| both cubes on one end (d = 100 mm), e = -10 mm | y_G = 57.7 mm | **tips** |

Why 15 mm per cube: the cube grips close along y (yaw 0 is the only cube yaw both arms reach at these heights;
yaw 180 and -90 fail the right arm's joint limits), so the 12-20 mm grip offset runs along the bar. It runs toward
the gripper's -y in both arms, that is, the same way in the world, so it shifts the pair of cubes rather than
splitting them, and the 6 mm place aim toward +y (the GPU rounds' advice) cancels part of it. The worst case above
still takes 15 mm for each, in the same direction. The bar grip closes along x, across the bar, so its own
offset (up to about 15 mm on a 64 mm part) moves the bar across the post, where the post's 25 mm half-width holds
it; along the bar its error is the arm's (about 5 mm at rest), well inside the scored 10 mm.

**Why both ends at once.** Loading one end first tips the bar about the post's edge until its end hits the table,
at asin(H / (L/2 + e - w/2)) with H = 110 mm: 73 deg (e = -10 mm), 61.6 deg (centred), 54.6 deg (e = +10 mm), and
47.2 deg even at e = +25 mm (for e under -15 mm the end cannot reach the table and the bar falls off the post). A
40 mm cube topples off a slope steeper than 45 deg, and slides past atan(1.2) = 50.2 deg, so the first cube falls
off. Even if it stayed, a second cube on the raised end could not right the bar: the loaded bar comes back level
only while tan(tilt) < (2m + M)(w/2 - e) / (2m h_cube + M h_bar), with h measured above the bar's underside
(h_cube = 39 mm, h_bar = 9.5 mm): limits of 27 deg (e = +10 mm), 40 deg (centred) and 50 deg (e = -10 mm), all
below the tipped angle. This is what the post's height is for: on a 50 mm post the tip is 23.6 deg, the cube rides
it, and a second cube on the raised end brings the bar back level (the limit is 40 deg), so a one-at-a-time loader
would recover. The demo therefore lowers both cubes with identical durations and lets go at the same instant.

**Synchrony.** A rough estimate (statics plus rigid-body inertia, not simulated): with one cube alone at its spot
the bar starts to tip at about 38 rad/s^2, so a 20 ms gap between the two cubes landing tilts it 0.4 deg and a
50 ms gap 2.7 deg before the second cube arrives, both far inside the 5 deg tolerance and the righting limit.
A 100 ms gap would reach 11 deg and spin it at 3.8 rad/s, which may carry it over. The demo's two arms run the same
waypoint durations from the carry on and release on the same control step.

**Other routes that also earn 1.0** (not prevented, and physically valid): one hand steadies the bar while the
other loads (hold-and-act; two arms again), or one arm puts a cube near the middle as a counterweight, loads the
far end, then pushes the counterweight out along the bar without lifting it (one arm, but it takes stepwise
centre-of-mass reasoning and a push on a balanced bar). Lifting and placing one end at a time fails.

## Demo

1. Both arms move to park spots at y = +-0.25 m. The left arm (fingers closing along x, across the bar) takes the
   bar at its middle with the pad tips 3.5 mm above its underside, lifts it until its underside is 15 mm above the
   post top, carries it over the post (aimed 6 mm toward +x against the grip offset), lowers it until the underside
   is 3 mm above the post, lets go and rises 3 cm. The pad tips stay above the bar's underside, so they never reach
   the post top.
2. Meanwhile the right arm (fingers along y) picks up its cube and holds it 15 mm above bar height at (0.25, -0.22),
   50 mm beyond the bar's right end while the bar comes in.
3. The left arm turns to yaw 0 over the bar (pad tips 17 mm above it), moves out to its cube and lifts it.
4. From here the two plans have identical durations: both carry their cubes over the ends (aim y = +0.131 and
   -0.119: the 125 mm spots plus the 6 mm aim), lower them until the cube bottoms are 5 mm above the bar, hold
   still 0.5 s, open together, rise 3 cm, move out to (0.15, +-0.28, 0.15 m), clear of the top camera's view of the
   bar and its ends, and hold 2 s.

Heights above the table (TCP): bar 16.5 mm (pick), 141.5 mm (carry), 129.5 mm (release); cubes 25 mm (pick),
169 mm (carry), 159 mm (release), 189 mm (after the 3 cm lift). The probe in the session that built this found the
straight-down grip reaches up to 0.18 m over the post at x = 0.25 m on the midline and 0.20 m over the bar's ends;
the offline IK test checks the actual plan.

## Novelty: nearest tasks and how this one differs

- **Board B80 "Build a DUPLO overhang"** (previous cohort, planned, not reviewed): one arm holds the upper part
  while the other attaches the next brick. The structure there is held by snap-fit studs, and the two-arm need is
  holding while attaching. Here nothing attaches: the only thing keeping the bar level is where the load is.
- **Board "Carry an open tray level"**: keep a held object level while moving it. Here nothing is held at the end;
  levelness comes from the load's placement.
- **The six named sets** (LIBERO, RoboCasa, SimplerEnv, BEHAVIOR, DataCenterBench, RoboDojo): we found no task
  that balances a free beam on a support. RoboCasa `BalancedMealPrep` is a plating task, not a balance.

## Design choices

- **Post 50 mm wide.** The empty bar then balances with its middle up to 25 mm off-centre (a 20 mm post would give
  10 mm, less than the bar grip's along-bar error budget plus margin), and symmetric loading keeps at least 4.3 mm
  in the worst case, while one cube still tips it by 11 mm or more. 50 mm is also the ridge width that stayed level
  under equal loads in earlier simulation runs. The post is square, so a bar turned on the post meets the same width.
- **Post 110 mm tall.** It makes a one-sided tip steep enough (54.6 deg or more within the scored centring) that
  the first cube falls off and the bar cannot be righted from a tip; see the derivation. The heights still fit the
  arm's reach, which the IK test checks.
- **Bar 64 mm wide.** The cubes land up to about 15 mm off the bar's centreline across it (the bar's own grip
  offset), so a 25 mm strip would drop them; 64 mm keeps each cube's centre at least 12 mm inside the bar's edge.
  64 mm is 30 mm under the 94 mm pad opening.
- **Heavy cubes (173 g, aluminium) against a 165 g bar.** A mass ratio near 1 balances the two needs: light
  cubes would leave one cube too weak to tip the bar, and much heavier ones would let their landing errors move the
  centre of mass off the post.
- **Centring tolerance 10 mm** along the bar: physically the empty bar balances to 25 mm, but
  a bar left 10-25 mm off is one small error from tipping under load, and the symmetric-loading margin above is
  derived for 10 mm.
- **0 for both cubes on one half.** On a balanced bar that only happens near the post, where it avoids the task's
  point (load the ends). Two cubes on one end tip the bar anyway.
- **Anything on the floor scores 0**, as a lost part.
- **Category: Other** (physical reasoning about centre of mass and balanced loading). The context names riggers,
  stagehands and warehouse workers, who load supports that tip if loaded one side first.

## Offline tests

```bash
YAM_ARENA_PATH=~/yam-isaac-lab-arena python -m pytest -q tasks/balance-bar-on-post/test_task.py
```

`YAM_ARENA_PATH` points at a clone of the embodiment repo at `bd62771` (not needed where `yam_arena` is installed).
NumPy and pytest only (80 checks): the file loads with primitives only; `task.json` has every field and
a `scores` entry; no affiliation in the shipped text; both arms' plans solve IK at 20 Hz within 0.5 mm and
0.1 deg and inside the joint limits; equal plan lengths, a 2 s still end, fit in the run limit; TCPs over 18 cm
apart; every release followed by a 3 cm lift; both cubes lowered and let go at the same instant with mirrored
timing; the bar on the post and the hand off it before any cube comes over the bar; the right arm's held cube 50 mm
clear of the bar during the bar phase; every scoring branch above; the statics table and the one-end-first
derivation (bar masses 140, 165 and 200 g); the gripper fit; pad tip clearances at every pick and release; carry
clearances; a 15 mm along-bar grip offset still scoring 1.0 and balancing; no start overlaps; loose objects on the
table at the start; the one-end-first check plan.

The physics itself (contact, settling, timing of the two landings, how a tipped bar falls) cannot be tested here.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 52 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 2,400-step run limit; 209 s wall clock.
- Offline tests: 80 passed.
