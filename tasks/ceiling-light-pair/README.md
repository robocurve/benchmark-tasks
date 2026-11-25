# Hang a 4-ft LED wrap light on its four ceiling screws with a partner robot, then tighten the two at your end

Two bimanual YAM stations face each other across a table, one at each end of a plywood panel that serves as the
ceiling, its underside 760 mm above the table. Four #8 x 3/4 in pan-head wood screws are already driven into the
panel, each head left 3/8 in (9.5 mm) below it, as the light's maker instructs. The bare steel housing of a Commercial
Electric WR484080LSL 4-ft LED wrap light (lens and channel cover off, as in the maker's step 1; 1,218 mm long, about
3.7 kg, estimate) rests back up on two padded stands, 30 mm below the ceiling and 15 mm toward station B from its
pencil outline. Its back has the maker's four keyholes, each a round centre hole with a narrow slot running both ways
along the light. Each of the four arms has a push-to-start cordless screwdriver standing on a holder beside it.

Each robot is told, separately:

> With the robot across from you, hang the light on the four screws in the ceiling by its keyholes: lift it off its
> stands, keep it level, move it toward its pencil outline until the round centres of its four keyholes are under the
> screw heads, raise it against the ceiling, slide it on into its outline so each screw goes into a keyhole's narrow
> slot, lower it onto the heads and let go. Then, with the screwdrivers beside you, tighten the two screws at your end
> until your end is held tight against the ceiling.

## What the task tests

Each robot is one of the maker's "two people", at one end. The two policies cannot talk; each sees only its own
three cameras and joint state.
- **Blind overhead engagement of four hidden screw heads.** Each 9.5 mm centre hole has to come under its 8 mm head
  within about 0.75 mm. Once the back is near the ceiling the heads are hidden above it: the wrist cameras see them
  only from below, through the keyholes, or from the side.
- **Agreeing on the slide.** Both ends have to be up, all four heads through their holes, before the light is slid
  10 mm along its length into its pencil outline, one robot pulling while the other pushes. Slid with one end low,
  that end's holes leave their heads. The outline sets the direction, so both can tell which way it goes.
- **Letting go only once it hangs.** An end released before its screws are in the narrow slots falls onto its stand.
- **Overhead power-tool tightening, each robot its own two screws.** Each arm takes a push-to-start screwdriver,
  brings it up into the open housing, seats the bit in a head it sees only from below, and keeps pushing as the screw
  draws the light up flush, until the clutch slips.

Two robots are needed, not only convenient. The far end's screws are about 1.25-1.26 m from a station's nearer
shoulder, beyond the YAM's 750 mm reach. One station alone cannot lift the light flat: its hands reach at most 480 mm
along the walls toward the centre of mass, 669 mm away, where holding half the light would take 1.9 times a shoulder
motor's rated torque (`dev/hold_torque.txt`). In the `alone` check below, A grips there with both hands and tries the
demo's lift: both shoulders (J2) sit at their 27 N m limit and the light rises less than 2 mm.

## Why it matters

- Electricians (821,000 in the US, O*NET 47-2111.00) install fixtures as core work: "Work from ladders, scaffolds,
  or roofs to install, maintain, or repair electrical wiring, equipment, or fixtures."
- The light's Use and Care Guide: "NOTE: Two people are recommended for the installation of this light fixture."
  Step 3: "install the four wood screws (AA) directly into the mounting surface leaving a 3/8 in. gap between the
  screw head and the mounting surface." Step 4: "Guide the heads of the four installed wood screws (AA) into the
  keyhole slots of the fixture (A)." / "Slide the fixture (A) until the wood screws (AA) are secured into the narrow
  ends of the keyhole slots." / "Tighten the wood screws (AA) until the fixture is secured in place." The task follows
  these steps; the screws are set to their gap at reset, as step 3 does before the light is lifted.

## Files

| File | What it is |
|---|---|
| `task.py` | the yam_pair task: objects, the screw model (task code), the contact monitor, `score()`, `report()`, `demo()` |
| `geometry.py` | every size, place and constant; each estimate is marked "estimate" |
| `build_assets.py` | writes `assets/housing.usda`, `assets/driver.usda`, `assets/screws.usda` (needs `pxr` only) |
| `scripted.py` | the scripted demo: one generator script per station, in its own frame |
| `task.json` | the submission fields, with both stations' instructions |
| `yam-pair-kit/` | the four-arm Inspect Robots plugin: `yam_pair`, `dream_pair`, `yam_pair_scripted`, `pair_task_score`, `yam-pair-video` |
| `research/` | the light, its manuals and prices; motor ratings and temperature limits; keyholes; #8 screws (quotes, URLs, dates) |
| `dev/` | `hold_torque.py`/`.txt` (offline statics); `torque_table.py` and `torque_table.md`; `run.sh`, `checks.sh`, `summary.py`, `results.py`, `checks.txt`; `media.sh`, `overlay.py`; `bandscan.py` and `bandscan_run3.txt` (the Dream-1 frame check); `runs/` (monitor and torque logs; `d9r`: d9 replayed for the video) |
| `media/` (uploaded, not in the repository) | `model_run.mp4`, `demo.mp4`, `setup.png` |

All assets are built from scratch in `build_assets.py` and `task.py`; no third-party meshes, scans or textures.

## Run a trial

Install the kit once into the Isaac Lab-Arena venv, then run (`instructions.a` / `.b` in `task.json` both equal
`instruction`):

```bash
source <ARENA_DIR>/yam.env
uv pip install --python "$VIRTUAL_ENV/bin/python" --no-deps -e ./yam-pair-kit
python build_assets.py                      # once
INSTR="$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')"
# Dream-1, one session per robot
inspect-robots run --embodiment yam_pair -E task=task.py -E overview=true --policy dream_pair \
    --instruction "$INSTR" --scorer pair_task_score --max-steps 6000 --store-frames --log-dir logs/dream
# the scripted demo
inspect-robots run --embodiment yam_pair -E task=task.py -E overview=true --policy yam_pair_scripted \
    --instruction "$INSTR" --scorer pair_task_score --max-steps 6000 --store-frames --log-dir logs/demo
yam-pair-video logs/demo demo.mp4
```

The overview camera is for the video only. `success()` is never true: a trial runs to its step limit and is scored on
its end state. `CLP_VARIANT=<name>` runs a scorer check (`dev/checks.sh` runs all); `CLP_TORQUE_LOG=<file.npz>` saves
per-step joint torques; `CLP_SPIKE=0.9` prints substeps where a joint passes 0.9 of rated.

## The scripted demo

`yam_pair_scripted` runs `demo()`: one generator script per station (`scripted.py`), each in its own frame, reading
object poses (privileged) and sharing no state with the other. Both follow the same plan. Times from the demo run:

1. **Take hold** (0-9 s). Each station's two hands come up from below to the rubber trim on the lower edges of its two
   long walls, 80 mm in from its end, and close (approach 35 degrees up, fingers across the wall).
2. **Lift and carry** (9-14 s). The four hands lift the light 12 mm and carry it 5 mm toward A, so the centre holes come
   under the heads. Each hand runs a closed loop on the light's measured height at its grip (the arms give under load).
3. **Line up and raise** (14-19.5 s). Each station waits until its two centre holes are within 0.4 mm of their heads,
   raises its end to 2 mm under the ceiling (the heads go up through the holes), then waits until both ends are up.
4. **Slide and hang** (19.5-25 s). Both slide the light 10 mm toward A, into its outline (all four shanks in their
   narrow slots by 21.1 s), lower it onto the heads, let go and take their hands away (no hand on it from 24.9 s).
5. **Tighten** (25.5-67 s). Each arm takes its screwdriver off its holder, brings it under the screw on its side,
   centres the bit 20 mm below the head, rises and pushes the bit into the head (a 10 N setpoint; the logged per-step
   push peaks at 4.6-7.0 N), so the motor runs. It follows the head up as the screw draws the light flush. Once the
   screw has turned once more after seating, it stops when the screw stops turning (the clutch slips) or the bit
   starts to slide off the head, then lowers the driver (in joint space) and holds it beside the light.

## Results

**Demo: 1.0** in three runs of the final code (d9-d11), identical because the simulation is deterministic: 1,342
steps; all four hung at 21.1 s; no robot touching from 24.9 s; screws tight at 49.4-50.3 s (5.4-5.9 turns in), then
driven about two turns more until the clutch slips at 126 N of clamp; no drop; no contact between the stations.

**Scorer checks** (the demo with one change, `CLP_VARIANT`; table in `dev/checks.txt`, logs in `dev/runs/c_*`):

| Check | Change from the demo | Expected | Score | End state (dropped / hung / tight / hangs on its own) |
|---|---|---|---|---|
| `nothing` | neither robot moves | 0 | 0 | 0 / 0 / 0 / yes |
| `alone` | B does not move; A grips as far toward the middle as it reaches and runs the lift, raise and slide | 0 | 0 | 0 / 0 / 0 / no |
| `drop` | light lifted 14 mm; all four hands open fast and stay: it slides down the open fingers onto its stands | 0 | 0 | 1 / 0 / 0 / yes |
| `unslid` | all hands open with the heads up in the centre holes, before the slide: it rests on the heads' edges | 0 | 0 | 0 / 0 / 0 / yes |
| `set_down` | lifted, lowered back onto its stands by hand, let go | 0 | 0 | 0 / 0 / 0 / yes |
| `let_go_b` | light lifted; B opens its hands fast and keeps them there; A carries on | 0 | 0 | 1 / 2 / 0 / no |
| `one_end_hung` | B keeps its end low (heads never through) and holds it; A hangs its end | 0.25 | 0.25 | 0 / 2 / 0 / no |
| `untightened` | no screw tightened | 0.5 | 0.5 | 0 / 4 / 0 / yes |
| `one_end` | B does not tighten | 0.75 | 0.75 | 0 / 4 / 2 / yes |
| `one_screw` | A's left screw not tightened | 0.75 | 0.75 | 0 / 4 / 3 / yes |
| `slip` | B's right hand closes only to 33.3 mm on the 36 mm driver, which slips | 0.75 | 0.75 | 0 / 4 / 3 / yes |
| `still_held` | A keeps its hands on the light; B tries to drive its two screws (one gets tight) | 0.25 | 0.25 | 0 / 4 / 1 / no |
| `forearm` | A's left screw not tightened; A's left wrist link (link 4) then rests against the light (about 11 N) | 0.25 | 0.25 | 0 / 4 / 3 / no |
| `driver_touch` | all four tight; A's left arm leaves its held driver against the long wall | 0.25 | 0.25 | 0 / 4 / 4 / no |

All 14 give their expected score. In `drop` and `let_go_b` the opened fingers still cradle the edge trim, so the light
slides down them rather than falling clear; an open gripper does not hold it, so it is a drop.

**Media** (`media/`): `demo.mp4` (six station cameras in a 2 x 3 grid plus the overview, with status lines from the
run's log in a black band below the cameras; 67.1 s, 1522 x 494); `model_run.mp4` (the Dream-1 run below, same camera
layout without the band, 300 s, 1522 x 352); both H.264, 20 fps, from step 1. `setup.png`: the overview, a view from
below into A's end of the open housing (two of its keyholes), and both top cameras, at the start.

## Dream-1 run

Dream-1 on both stations (`dream_pair`: one Dream-1 session per robot, each given only its own three cameras, its own
joint state and the instruction), 6,000 steps (300 s, the run limit), on the final code, `media/model_run.mp4`:
**0** (no screw hung).
- **What it did.** The four arms moved low over the table, around the holders, legs and stands, for the whole trial.
  None rose to the light: no gripper touched it at any step (each hand's peak contact force is 0), no other link or
  screwdriver touched it at any logged step (every 10th), and it never moved. All four screwdrivers stayed on their
  holders. No drop.
- **Frames.** The cameras' stored frames were checked every 500 steps to step 6,000, all seven cameras: the share of
  thin black bands on lit surfaces stayed at or below 0.0065 (the corrupted run below reached 0.12; `dev/bandscan.py`,
  output in `dev/bandscan_run3.txt`), and the sampled frames look clean.
- **Joint torques**, with no load in hand: from about 100 s A's two arms reached far out low over the table and stayed
  there. J2's 10 s average reached 1.51 and 1.43 times rated (13.6 and 12.9 N m), and A's arms had a joint over rated
  for almost all of the second half. The highest peak was 1.85 times rated (16.6 N m, J1 of A's right arm), under the
  27 N m peak rating. B's arms stayed under rated. The temperature stop (Motor load) covers this.
- **Earlier runs.** An earlier run on the same instruction and nearly the same scene (the frame's legs have since
  moved 11 mm; also before the last drop-rule change, which cannot change it: nothing touched the light) also scored 0
  with no contact. Another (also 0, no contact) is not used: one of B's arms flung a screwdriver off the table at
  about 30 s and it left view within a second; from 107 s the cameras' ray-traced shadows broke into black bands and
  rings, so the model saw corrupted images. Most likely the driver rolled off the 200 m floor and fell without end
  (its pose was not logged; a driver sent far below the floor reproduces the artifacts). The room rule (Simulation
  fidelity) now stops a flung screwdriver 2 m out.

## Motor load: why the arms can do this safely

**Ratings.** The YAM's joints 1-3 are Damiao DM-J4340 and joints 4-6 DM-J4310, on 24 V (I2RT). Damiao rates the
original DM-J4340 at 9 N m (27 N m peak) and its V1.1 at 12 N m (40 N m); the DM-J4310 V1.1 at 3 N m (7 N m) and its
V1.2 at 3.5 N m (12.5 N m). I2RT does not say which a YAM has, so this task uses the lower figures: **9 N m for J1-J3,
3 N m for J4-J6**. Rated is not an indefinite rating. Damiao's 24 V temperature-rise plots, for the DM-J4340 V1.1 at
12 N m and the DM-J4310 V1.2 at 3.5 N m, reach about 99 °C at the coil (the recommended 100 °C trip) after about 9 and
5 minutes. If a YAM has those versions, 70 % of our figures (6.3 and 2.1 N m) is about half and 60 % of their rated
torque, so a quarter to a third of the plotted heating (heating goes with torque squared). For the older 9 N m and 3 N m
versions no plot is published. Hence the targets here: **any 10 s average at most about 70 % of rated, no worse than
the arms at rest** (yam_arena's READY pose, where every trial starts, holds J3 at 0.72 of rated), and **peaks under
rated**, plus the live temperature stop below.

**How the task keeps the load there.**
- The stations stand at the light's ends, so the four hands share the 3.7 kg light, about 9 N each, with its centre
  of mass between each station's two hands.
- Each hand pinches a long wall's lower edge 80 mm in from its end, 100-150 mm in front of its own base, 680-710 mm
  above the table, where the arm stands nearly upright. What a hand can carry within 70 % of rated falls quickly with
  reach: 13.5 N at 100 mm in front of its base, 11 N at 150 mm, about 5-7 N at 200-300 mm, under 1 N at 400 mm (the
  demo's pinch moved along the wall, `dev/hold_torque.txt`). For the same reason the ceiling is high (760 mm): lower,
  the elbow bends more.
- No hand holds a screwdriver while the light is held, and the light hangs on its screws before anyone fetches one,
  so the loaded hold is short (about 15 s in the demo).
- Each arm tightens the screw on its own side; the other screw at that end is beyond its reach at driving height.

**Offline statics, held indefinitely** (`dev/hold_torque.py`, URDF masses and kinematics, the demo's IK): with 9.1 N on
each hand the worst joint in any hold pose is at 0.65 of rated (0.98 if one hand carried its whole end, a bound); the
screwdriver poses reach at most 0.68.

**In the demo**, logged at every physics substep for all four arms (the drives' applied torque), the highest over the
three runs and four arms, N m (fraction of rated); per arm in `dev/torque_table.md`:

| Joint | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| peak | 6.47 (0.72) | 7.63 (0.85) | 8.55 (0.95) | 2.40 (0.80) | 1.67 (0.56) | 2.18 (0.73) |
| highest 1 s average | 2.54 (0.28) | 6.92 (0.77) | 7.11 (0.79) | 2.04 (0.68) | 1.38 (0.46) | 1.00 (0.33) |
| highest 10 s average | 2.31 (0.26) | 5.00 (0.56) | 6.24 (0.69) | 1.81 (0.60) | 0.96 (0.32) | 0.80 (0.27) |

The 0.95 peak is J3 of B's left arm for 0.3 s at 5.5 s, rising toward the light with nothing in hand. The highest 10 s
average, 0.69, is J3 of A's left arm at 35-45 s, carrying its screwdriver. While the four hands hold the light (9-24 s)
the highest 10 s average is 0.62. (An earlier demo moved each driver away from its screw in a straight line; with the
elbow folded nearly shut the IK jumped and one substep reached about 1.4 times rated; `CLP_SPIKE` showed it,
`dev/runs/old_park_spike.log.gz`, and the move now interpolates the joints.)

**Temperature, live.** Damiao's DM motors report their coil temperature, T_Rotor, in their CAN feedback. A monitor
script on each station's PC reads every motor of its two arms at 10 Hz, shows the hottest coil of each arm on a screen
beside the spotter, and disables both stations' arms if any coil passes 80 °C,
20 °C below Damiao's recommended motor trip. (Damiao's 120 °C trip is the motor driver board's MOSFET temperature, not
the coil's.) Each trial starts only when every coil reads 50 °C or less, after a rest of at least 10 minutes; these
are this task's choices, not maker limits. A run stopped at 80 °C is scored on its state at the stop.

**What a model can do anyway.** The task bounds the load it needs, not the load a policy chooses: a bare arm held out
level needs up to 15.3 N m at J2, 1.7 times rated (`dev/hold_torque.txt`), as in the Dream-1 runs above. The
temperature stop covers this.

## Scoring

Scored on the end state, except that a drop at any time scores 0.

- **Dropped:** at any time an end of the light comes down 10 mm or more onto a stand, the table, the floor or a robot
  while no gripper holds that end and it is not on its screws. A gripper holds an end while it is closed on the light
  and the light does not slip in it: an end lowered by a closed hand onto its stand, or let down onto the screw heads,
  is not a drop; an end that slides down open fingers onto its stand is.
  - Real trial: the video.
  - Sim: checked every control step for each half of the light. Held: a gripper on that half is closed on it (opened
    no more than a quarter, about 23 mm between the pads), has touched it for the last 0.25 s (contact sensors on the
    gripper and both fingers), and its place in the light's frame moved less than 2 mm in that time. The fall is
    measured from the end's highest point in the 0.25 s before it became unheld and off its screws, and the light must
    touch a stand, the table or the floor (contact sensors), any non-gripper arm link, or a screwdriver not in a hand.
- **Hung on a screw:** its shank is in a narrow slot of its keyhole, clear of the round centre hole, with the back above
  the head. Either narrow end counts (a light slid the other way, into the A-side ends, is hung too).
  - Real trial: look up into the open housing at each keyhole.
  - Sim: the shank's axis at least 6.85 mm along the slot from the hole's centre (hole radius plus shank radius),
    within the slot's width, and the head's top under the back (2 mm tolerance).
- **Tight (each screw):**
  - Seated: a 0.5 mm feeler gauge blade does not go between the head and the light's back.
  - Flush: a 2 mm plastic shim, pushed flat against the ceiling at the back's long edge nearest the screw, in line with
    it, does not go in.
  - Clamped (both screws of an end): a 25 N pull across the light at that end moves it less than 0.1 mm. A luggage
    scale on a cord loop pulls the lower edge of a long wall 100 mm from the end, each way; a dial indicator on a
    bracket on the ceiling, against the other long wall near the same end, reads the movement.
  - Sim: the head within 0.3 mm of the back and clamping it with 50 N or more, and that edge within 2 mm of the ceiling.
  - Why 25 N (estimates): each screw carries about 9 N of the light. A clamped end is held by friction at both heads
    and at the ceiling: with each head clamping C, at least 2 [mu1 C + mu2 (C - 9 N)], with mu1 0.15-0.3 (zinc on
    painted steel) and mu2 0.25-0.5 (painted steel on plywood): at least 35 N at the 50 N threshold. An unclamped end
    resists at most 2 x 0.3 x 9 = 5.4 N. But a rigid light can only move at one end by turning about the other: the
    pull, 0.93 m from the far end's screws, applies 23 N m. A near end clamped at 50 N adds at least 35 N x 0.83 m =
    29 N m against it, so it does not move. An unclamped near end adds at most 4.5 N m; the far end's two screws, 190
    mm apart, resist 3.4-6.7 N m of turning at a 50 N clamp but 9.1-18.3 N m at the demo's 126 N, so the old 10 N pull
    (9.3 N m) could be held by a tight far end alone. 25 N (23 N m) exceeds 18.3 + 4.5 = 22.8 N m. An unclamped end
    moves by the shank's play across the slot, about 0.3 mm each way. The pull is calibrated before trials (below).
- **Hangs on its own:** for the last 3 s no robot touches the light: no link of any arm and no screwdriver in a hand.
  - Real trial: the video. Sim: no contact over 0.05 N between the light and any link of the four arms, or between the
    light and a screwdriver that is touching an arm.

| Score | End state |
|---|---|
| 1.0 | hung on all four screws, all four tight, and it hangs on its own |
| 0.75 | hung on all four, hangs on its own, both screws of one end tight, not all four |
| 0.5 | hung on all four, hangs on its own, no end with both screws tight |
| 0.25 | not dropped and hung on at least one screw, but less than 0.5 (one end hung only, or a robot still touches it in the last 3 s) |
| 0 | dropped at any time, or hung on no screw |

**Real trial**, right after the run: (1) the video, for a drop and for the last 3 s; (2) look up into the housing at
each keyhole; (3) the 2 mm shim at each screw's nearest edge; (4) the 0.5 mm blade under each head; (5) at each end
whose screws passed 3 and 4, the 25 N pull each way. A run the spotter stops is scored on its state at the stop.

## Real-world build

Positions in mm from the midpoint between the stations: x from A toward B, y to A's left, heights above the table. A's
arms face +x with bases at x -654, y +305 (left) and -305 (right); B's face -x at x +654, its left at y -305.

| Item | Details | Price |
|---|---|---|
| Light | Commercial Electric WR484080LSL 4-ft LED wrap light (Home Depot 313512347; HD Supply P232165), 47.96 x 14.13 x 2.82 in, 10.4 lb with lens. Lens and channel cover off; bare housing estimated at 1,218 x 320 x 55 mm, 3.7 kg. | $101.67 (Home Depot, 2025-09-01); $139.99 (HD Supply, 2026-02-12) |
| Edge trim | push-on rubber U-channel, 8 mm across, four 80 mm pieces on the long walls' lower edges at the grip spots, 80 mm in from each end; protects the gripper pads | about $8 |
| Ceiling panel and frame | 23/32 in plywood (one 4 x 8 ft sheet): panel 1,400 x 600 mm (x +/-700, y +/-300); offcuts for plugs, feet and holder bases. Two 2x4 rails on edge along the panel's top over its long edges. Four 2x4 legs, 849 mm, at x +/-120, flush against the panel's long edges (y +/-319), wide face along x; each screwed to the panel's edge (two 2 in screws) and to its rail (two 3 in screws), standing on a 150 x 150 mm plywood foot clamped to the table. | about $42 |
| Stands | two 2x4 posts at x +/-300, y 0, each with a 400 mm 2x4 crossbar across the light and 10 mm foam on top (675 mm above the table), on a 200 x 200 mm plywood foot | (2x4s below) |
| 2x4 studs | four 2 x 4 x 8 ft: legs, rails, stand posts, crossbars | 4 x $3.44 |
| Screwdriver holders | four: a 4x4 riser on a clamped plywood base, topped with a 22 mm square hardwood peg, 30 mm tall, its top 350 mm above the table, at (120, +/-450) mm in each station's frame (x forward, y to its left), turned 60 degrees outward so its faces are square to the hand's approach. The driver stands on the peg; the fingers pass either side of it. | 4 x 4 x 8 ft post, about $12 |
| Screwdrivers | 4 x Bosch GO Gen 3 (0 601 9H2 201; a Gen 2 also works): 3.6 V, 306-396 rpm, clutch at its lowest setting (or the lowest that passes the pre-trial check), #2 square bit. Used in push-to-start mode (the motor runs while the bit is pressed on a screw; its button is not used); check push-to-start on the units bought. No US price found. | £75.28 each incl. VAT (Bosch UK, 2026-11-24), about $98; 4 x $98 |
| Screws and plugs | #8 x 3/4 in pan-head square-drive wood screws (box of 100); four 50 mm plugs cut from the panel's plywood, each in a 2 in hole in the panel, flush with its underside, glued to a 100 mm square cap screwed to the panel's top, with a 1/8 in pilot hole at its centre | about $10; hole saws about $25 |
| Gap spacer | a 3/8 in (9.5 mm) hardwood block, 40 x 25 mm, with a 5 mm slot: slid over a shank against the ceiling, it sets the head's gap | offcut |
| Small parts | 10 mm foam strip; eight C-clamps; screws for the frame and caps | about $40 |
| Scoring tools | 0.5 mm feeler gauge; 2 mm shim; luggage scale (0-50 kg) and cord loop; dial indicator (0.01 mm) on a magnetic base, on a steel plate under the panel at each end; kitchen scale; pencil | about $62 |

**Total about $710.** The pencil outline marks the light's hanging place: ends at x +/-609, long sides at y +/-160.

**Keyholes and screws.** The maker's drawings and a Home Depot photo of the back (`research/research_r2.md`) show four
keyholes, about 826 mm apart along the light and 190 mm across, each a round centre hole with a narrow slot running
both ways along the light. Scaled from the drawing: about 0.33 in hole, 0.17 in slot, 1 in overall (low confidence).
`geometry.py` uses 3/8 in (9.5 mm), 3/16 in (4.8 mm) and 25 mm. Estimates; the real slots decide. To place the
screws, hold the light flat against the panel inside its outline and mark the middle of each keyhole's B-side narrow
end (10 mm toward B from its centre hole); cut the plugs' holes there and drill each pilot hole at a plug's centre.
The light comes with Phillips wood screws (AA) of unstated size; the task uses #8 square drive, which cams out less
overhead. That makes no step easier: the robot still finds the head from below, keeps the bit seated and pushes while
the screw draws the light up.

**Before the first trial:**
- **Head through hole:** pass each screw's head through its real centre hole by hand: it must go through with at least
  0.5 mm to spare all round (an #8 pan head is 0.306-0.322 in across; the photo suggests holes of 0.26-0.31 in, the
  drawing 0.33 in). If it does not, use the light's own screws (AA) with a PH2 bit, as the maker intends.
- Clutch: on a spare plug, check that each driver's lowest clutch setting draws the light flush and passes the
  pull test; if not, use the lowest setting that does, on all four.
- Weigh and measure the housing and its keyholes (update `geometry.py`); read each driver's push-to-start force on a
  kitchen scale.
- Calibrate the pull: with both ends' screws driven as the robots drive them, do the 25 N pull at one end with its
  screws clamped and again with them backed off to just touching; the dial indicator must tell them apart. If it
  cannot, raise the pull in 2 N steps, up to 30 N, and use that pull in every trial.
- Log every coil's T_Rotor through one demo-length run, to confirm the rise stays small.

**Safety.** The light is never connected to power; wiring (part of the maker's step 4), the channel cover and the lens
(step 5) are left out. The padded stands stay under the light the whole trial (a drop is about 22-30 mm onto foam). A
spotter stands by, stops the run if the light starts to come down, and watches the coil temperatures (above).

## Simulation fidelity

Rigid PhysX, 240 Hz (12 substeps per 20 Hz control step). Task code in `task.py`: `_substep()` before every physics
substep, `on_step()` after every control step.

- **Screws.** One articulation fixed to the world; each screw is a z prismatic joint (the thread: its target follows
  the spin at 1.69 mm per turn) and a spin revolute. The colliders are on the non-turning z link (PhysX's contact on
  a fast-spinning collider pushes sideways; a real turning head's friction is a couple).
- **Head collider.** A block 8 mm across the slot and 4.2 mm along it, so it never lies over the slot's closed end
  (clamped there, PhysX pushes the light along the slot); the visible head is round. In the 9.5 mm square centre hole
  it has about +/-2.65 mm of play along the slot, the real round head about +/-0.75 mm: alignment along the slot is
  easier in the sim; across it, the same.
- **Keyholes.** The back's collider is boxes with the keyholes cut out (centre hole as a 9.5 mm square, 4.8 mm slot);
  the collider is 1.5 mm thick, the real sheet about 0.6 mm.
- **Driver.** A rigid body (36 mm grip, collar, #2 square bit, 0.31 kg; the bit has no collider). While the bit tip is
  pressed into the head, task code pushes it back (at most 60 N) and centres it as the recess walls do. The motor runs
  only while the bit sits in the head (within 1.5 mm and 6 degrees) and pushes with 3 N or more (estimate), at up to
  360 rpm; the screw needs the thread drag (0.25 N m) plus the clamp through the thread (11.5 % efficiency), up to the
  0.6 N m clutch (estimates). The reaction acts on the driver, held only by finger friction, so it can slip in the
  hand.
- **Tightening.** From the 9.5 mm gap the back draws up flush in about 4.7 turns; then the clamp grows each turn until
  the clutch slips (126 N of clamp in the demo).
- **Contacts for scoring.** Sensors on the light against every link of the four arms (base to fingertips), each
  screwdriver, each screw, the stands, table and floor; and each screwdriver against every arm link.
- **The room.** A screwdriver knocked more than 2 m out, or through the floor, is stopped there on the floor, as a
  room's walls would (`_room()`, logged). Without it a flung driver can leave the 200 m floor and fall without end,
  and the growing scene bounds break the cameras' ray-traced shadows (black bands and rings). Sending a driver 5 km
  down reproduced this; flinging one at 15 m/s with the room on stayed clean (`CLP_VARIANT=far_driver`, `fling_driver`).
- **Not simulated:** electrics, the lens, thread surfaces and the wood. **Estimates:** the housing's size and mass,
  the keyholes, the torques, push forces and clutch, the friction of the edge trim (1.1 / 1.0) and heads (0.3 / 0.25).
- **Scoring differences** from the real trial: "a gripper holds that end" is read from contact sensors, the gripper's
  opening and the light's slip in it, where a judge decides it from the video; and "clamped" is read per screw as a
  clamp of 50 N or more, where the real trial pulls each end with 25 N (chosen and calibrated to tell a 50 N clamp
  from none, above).

## Reset between trials

1. Support the light by hand. Back each screw out about five turns, slide the light 10 mm toward B so the heads are in
   the centre holes, and lower it onto its stands, 15 mm toward B of its outline (marks on the crossbars).
2. Set each gap: slide the 3/8 in spacer over the shank against the ceiling, turn the screw by hand until its head just
   touches the spacer, remove the spacer.
3. Stand each screwdriver on its peg.
4. Rest the arms at least 10 minutes; start when every coil reads 50 °C or less.
5. Replace a plug when its screw spins without drawing up (estimate: about every 20 trials).

About 4 minutes plus the rest.

## Sources

Exact quotes, URLs and fetch dates are in `research/quotes_product.md` and `research/research_r2.md`:
- Commercial Electric WR484080LSL Use and Care Guide (Home Depot, two editions; drawings of the back), spec sheet
  ("10.41lbs", "47.96 in x 14.13 in x 2.82 in"), Home Depot photo of the back, prices (Home Depot, HD Supply).
- O*NET 47-2111.00 Electricians (821,000 employed, 2025).
- ASME B18.6.1 #8 pan-head wood screw: head 0.306-0.322 in across, 0.105-0.115 in high; shank 0.164 in; 15 TPI.
- Damiao product tables, manuals and test curves (ratings above; "Drive over-temperature protection: shutdown at
  120° C"; "Motor over-temperature protection (configurable, recommended ≤ 100° C)"; "T_Rotor: Average motor coil
  temperature"; 24 V temperature-rise plots). I2RT: doc.i2rt.com (3x DM4340, 3x DM4310, 24 V); "750 mm" workspace.
- Bosch GO Gen 2 data sheet (Farnell 3270282): "push machine forward or press 'on' button to begin"; Bosch UK GO Gen 3
  06019H2201: £75.28 incl. VAT on 2026-11-24 (£67.55 on 2026-11-22).

## Licences

The team's own work apart from the short quotations; `yam-pair-kit/` is the team's plugin. Isaac Lab-Arena, Isaac Lab,
yam_arena, inspect-robots and the YAM URDF are used as installed, not included.
