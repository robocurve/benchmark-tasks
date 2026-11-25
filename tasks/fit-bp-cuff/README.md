# Fit a blood-pressure cuff on a training arm: bladder end round first, hook end over it, snug, index on the artery

A life-size training arm lies palm up on a foam arm board. An adult blood-pressure cuff lies open across the board under
the upper arm. The robot wraps the bladder end up round the near side and over the top of the arm, then brings the hook
(Velcro) end up round the far side, over the top and down onto the bladder end's outer face, and presses it shut: snug,
with the cuff's index line over the marked brachial artery and its lower edge 2-3 cm above the elbow crease.

Fitting a blood-pressure cuff is the first step of every manual or automatic reading, a routine duty of about 2.8M US
workers (BLS Occupational Employment and Wage Statistics): nursing assistants (about 1.4M; O*NET 31-1131.00: "Record vital
signs, such as temperature, blood pressure, pulse, or respiration rate"), medical assistants (about 0.8M; O*NET 31-9092.00:
"Interview patients to obtain medical information and measure their vital signs, weight, and height") and LPNs (about
0.63M; O*NET 29-2061.00 lists measuring and recording vital signs, including blood pressure). The AHA places the bladder's
centre over the brachial artery with the lower edge about 2 cm above the antecubital fossa, and names cuff fitting as one of
the leading sources of measurement error (AHA 2019 statement).

## What is distinctive

- **Wrapping a floppy band round a limb with two hands, in order.** The 50 cm cuff has no shape of its own: it has to be
  carried round the arm so it lies flat against it, the bladder end first and the hook end over it (the other way round
  the hook tab ends up against the skin and cannot fasten).
- **Keeping it in place while closing it.** Pulling the second end round drags the whole cuff round and along the arm; the
  index line has to stay within 2 cm of the artery mark and the lower edge within 1.5-3.5 cm of the crease.
- **Snug, then pressed shut.** The hook tab has to be laid and pressed flat on the cuff's outer face, with the cuff close to
  the skin all round.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, scoring (`status`, `score_status`, `hook_touching`, `pressed`, `hands_clear`), `success()`, `self_test()` and the scripted `demo()` |
| `cuffkit.py` | The cuff: a chain of thin plates hinged across its width (adapted from the shared cable helper), with contact reports on |
| `presskit.py` | Reads the simulator's contact forces: the hook tab on the cuff, and the finger pads on each part of the cuff |
| `scenekit.py`, `tiltkit.py` | Fixed shapes and USD bodies for yam_arena task files; the tilted grip used by the demo (shared helpers) |
| `assets/arm.usdc` | The visible training arm (a skin-coloured mesh; the upper arm collides as a capsule in `task.py`) |
| `scripts/make_arm.py` | Builds `assets/arm.usdc` |
| `scripts/naive_cuff.py` | The wrong-order demo (hook end first) |

No third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the table top.

| Object | Where | What |
|---|---|---|
| Arm board | centred (30.0, 6.0) | Grey foam, 56 x 12 x 3 cm |
| Training arm | palm up on the board along x: wrist at x 9, elbow at x 30, shoulder end at x 56 | Life-size vinyl arm; upper arm 10 cm across (31.4 cm round), its underside 4 mm above the board |
| Crease mark | across the top of the arm at x 31.5 | Drawn line |
| Artery mark | along the near (robot's right) side of the upper arm, level with its axis | Red line |
| Cuff | open and flat across the board under the upper arm, along y; lower edge 2.5 cm above the crease (x 34.0 to 50.0); bladder end towards the robot's right (end at y -18), hook end to the left (end at y 32) | Adult reusable cuff, 50 x 16 cm, 0.10 kg, nylon over a 16 x 30 cm bladder (the AHA adult size for 27-34 cm arms) in its first 30 cm; white index line across it 16 cm from the bladder end; 10 cm hook tab on the inner face of the other end; loop fabric on the whole outer face; a 16 x 16 mm stiffened foam hem across each end |

Sources: occupations and counts, O*NET 31-1131.00, 31-9092.00 and 29-2061.00 and BLS Occupational Employment and Wage Statistics (nursing assistants about 1.4M, medical assistants about 0.8M, LPNs about 0.63M); cuff size and placement, AHA recommendations for blood
pressure measurement (bladder width and length for 27-34 cm arms; lower edge about 2 cm above the antecubital fossa).

## Testing in simulation

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the blood-pressure cuff on the arm: wrap the bladder end round first, then the hook end over it, press it shut, snug, with the index line over the artery mark." \
  --scorer task_score --max-steps 2000 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/fit-bp-cuff-dream
```

`task_score` prints the score (1.0 on success, otherwise `score()` at the end of the episode). Seed 0; no random placement.
About 4 minutes per trial including start-up on one GPU. `CUFF_LOG=1` prints, every 0.5 s, how far round the arm the cuff
goes, the hook plates touching the cuff, whether it is fastened, the lower edge's height above the crease, the skew, the index
line's distance from the artery mark, the gap to the skin, whether both hands are clear, and the score.

In simulation the score is computed at every control step from the plates' poses and the simulator's contact forces
(`presskit.py` reads PhysX contact reports): the angle the cuff covers round the arm; fastening, which needs at least 2 hook
plates lying flat (within 25 degrees) on the cuff's outer face while the hook tab presses on the cuff with at least 2 N and a
gripper's finger pads push the tab towards the arm with a net 2 N (a pinch on the hem cancels out), for 0.5 s (proximity alone, or the tab resting on
the cuff under its own weight, does not count); the plates' positions along the arm and round it; the inner layer's gap to the
skin (snug, and no plate pressed more than 1 mm into it); and the hands, which must be off the cuff (no finger pad in
contact with it, every pad at least 10 cm from every plate and hem) for the final 2 s. `python task.py` runs the scoring cases
on hand-made shapes and forces (including a tab lying unpressed, a press without loaded contact, and an over-tight cuff) and
replays the demo's plan through both arms' kinematics.

Scripted demo (`--policy yam_scripted`, same command otherwise): each hand keeps one tilted orientation throughout. The
right hand pinches the bladder end's hem, lifts it straight up above the arm, carries it left over the top, lays it down
the far side and lets go, then moves further left, clear of the cuff. The left hand pinches the hook end's hem, lifts it,
slides it right over the top and a little past the near side to pull it taut, lays it down on the bladder end (the hem
slips from the pinch here, and the closed fingers come down on the tab and push it on), presses it on 2 cm more and holds,
lets go and moves further right. Fastening registers at 30.7 s, with about 10 N between the tab and the cuff and a net
7 N from the finger pads. The run scores 1.0 from 38.2 s (764 steps), once both hands are 10 cm off the cuff; the run limit is 2,000 steps.

Model run: Dream-1 refused the instruction before acting (`DA-4000 ... unsafe instruction, refused: Wrapping and tightening a cuff on a person risks forceful physical contact.`), so there is no model-run video.

### Checks of the scorer

Each row is one simulated run on the fixed starting layout (same command as the demo, with the variable shown).

| Run | What it does | Outcome | Score |
| --- | --- | --- | --- |
| (none) | The full demonstration | Wrapped 472 degrees; fastened at 30.7 s (tab on cuff about 10 N, pads a net 7 N); edge 2.9 cm above the crease, index line 4 mm from the mark, snug; both hands 10 cm clear | 1.00 |
| `CUFF_NEGATIVE=nopress` | Carries the hook end taut over the near side and lets go there, above the cuff; the hand never comes down on the tab | The tab settles flat on the cuff (2 hook plates flat, in contact, placed and snug) but is never pressed; not fastened | 0.25 |
| `CUFF_NEGATIVE=held` | Presses the tab on and keeps holding it to the end | Fastened, placed and snug, but a gripper stays on the cuff | 0.75 |
| `scripts/naive_cuff.py` | Wraps the hook end first and lays the bladder end over it | The cuff ends 296 degrees round the arm and never fastens | 0.00 |

The demonstration was run twice with identical results (1.0 from 38.2 s).

## Testing in real life

Materials: a small luggage scale and a 1 mm plastic card (for the scoring checks), an adult reusable blood-pressure cuff (16 x 30 cm bladder, for 27-34 cm arms) with a 16 x 16 mm foam hem taped
across each end, a life-size mannequin arm (upper arm about 31 cm round), a 3 cm foam board, a marker for the crease and
artery lines, a ruler, a 6 mm dowel, and a phone camera.

Score each trial from the final state with the ruler and photos: the cuff goes once round the arm; the hook tab is pressed on
and holds the cuff closed; the lower edge is 1.5-3.5 cm above the crease all round and the index line within 2 cm of the
artery mark; and the 6 mm dowel cannot be pushed under the cuff's edge at the top or either side. Nothing is consumed and
no liquid is involved; the loads are light (the cuff is 0.10 kg).

## Scoring

The score at the end of the episode counts. The same rule applies in simulation and in real trials.

| Outcome | Score |
|---|---|
| The cuff does not go once round the arm | 0 |
| Wrapped | 0.25 |
| Also fastened: the hook tab laid flat on the cuff's outer face and pressed onto it by a gripper, still holding the cuff closed at the end (real trial: it does not open under a 2 N outward pull on the hook end's hem) | 0.5 |
| Also in place: lower edge 1.5-3.5 cm above the crease, skew round the arm under 2 cm, index line within 2 cm of the artery mark | 0.75 |
| Also snug but not over-tight (the 6 mm dowel cannot be pushed under the edge, the 1 mm card can; in simulation, mean gap to the skin at most 6 mm and no plate pressed more than 1 mm into it), and both grippers off the cuff (no finger touching it, every finger at least 10 cm from it) for the final 2 s | 1.0 |

## Reset

Open the cuff (peel the hook tab off), lay it flat across the board under the upper arm with its lower edge on the 2.5 cm
mark and the bladder end to the robot's right, the ends on the table.

## Fidelity

The arm and board are fixed; the upper arm collides as a 10 cm capsule with skin-like friction, and the visible arm is a mesh
1 mm proud of it. The cuff is 20 rigid 2.5 cm plates hinged across its width with light bending springs; it cannot stretch
or crumple. The hook-and-loop is a rule on the simulator's contact forces: the hook tab must lie flat on the cuff's outer face
and be pressed onto it by a gripper (at least 2 N between the tab and the cuff, and a net 2 N from the finger pads pushing the
tab towards the arm, for 0.5 s); after that only friction and gravity hold the closure.

How fastening is decided, in detail (`pressed` and `press_force` in `task.py`, forces from `presskit.py`):

- **Laid flat on the cuff:** at least 2 of the 4 hook plates have a cuff plate under them, at the same angle round the arm,
  with at most 2 mm between them, and run within 25 degrees of it (not across it or on edge).
- **In loaded contact:** PhysX reports at least 2 N of normal force between the hook tab (its plates and hem) and the cuff's
  other plates.
- **Pressed by a gripper:** the net contact force of all four finger pads on the tab, along the line from the tab to the arm's
  axis, is at least 2 N. Each pad's contact force is taken as pushing away from that pad (a pad can only push), so
  holding the hem in a pinch pushes equally from both sides and adds nothing, and pulling the tab outward or dragging it
  along the cuff gives nothing; only pushing it onto the cuff counts.
- All three hold together, smoothed over 0.1 s, for 0.5 s (10 controls). Proximity alone, the tab resting on the cuff
  under its own weight, or the band's tension pulling it against the cuff without a press, does not fasten.
- **Still fastened at the end:** after the press has fastened it, at the end at least one hook plate still lies flat on
  the cuff and the tab is still in contact with it (at least 0.2 N, smoothed). A tab that was pressed and then came off
  loses the credit.

Over-tightening: the simulated plates are rigid and the band cannot stretch, so the only way the simulated cuff can be
too tight is plates driven into the skin, which the scorer caps at 1 mm. The real trial checks it directly: the 1 mm card
must still slide under the cuff's edge.

The gripper requirement is a simulation-only simplification: in a real trial any press that closes the faces counts, and
the closure is judged by whether it holds against a 2 N outward pull on the hook end's hem. 2 N is a light press. Standard
hook-and-loop strength tests close the faces much harder before measuring them: ASTM D5170 rolls a 4 lb (18 N) roller over
the specimen five times, and the ISO 22776/22777 closing roller applies 1.0 N per mm of width. So the model closes more
easily than real tape; this eases the task. The scripted demonstration follows a fixed plan and never reads the score.
