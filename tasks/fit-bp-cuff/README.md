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
| `task.py` | The yam_arena task file: scene, scoring (`status`, `score_status`, `hook_touching`, `hands_clear`), `success()`, `self_test()` and the scripted `demo()` |
| `cuffkit.py` | The cuff: a chain of thin plates hinged across its width (adapted from the shared cable helper) |
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

In simulation the score is computed from the plates' poses at every control step: the angle the cuff covers round the arm,
the hook plates lying within 2 mm of the plate under them (fastened once 3 do so for 0.5 s), the plates' positions along
the arm and round it, the inner layer's gap to the skin, and the distance of both hands' pads from the arm. `python task.py`
runs 10 scoring cases on hand-made shapes and replays the demo's plan through both arms' kinematics.

Scripted demo (`--policy yam_scripted`, same command otherwise): each hand keeps one tilted orientation throughout. The
right hand pinches the bladder end's hem, lifts it straight up above the arm, carries it left over the top, lays it down
the far side and lets go, then moves further left, clear of the cuff. The left hand pinches the hook end's hem, lifts it,
slides it right over the top and a little past the near side to pull it taut, lays it down on the bladder end, presses it
on and holds, lets go and moves further right. The run scores 1.0 at 31.7 s (634 steps); the run limit is 2,000 steps.

Model run: Dream-1 refused the instruction before acting (`DA-4000 ... unsafe instruction, refused: Wrapping and tightening a cuff on a person risks forceful physical contact.`), so there is no model-run video.

Wrong strategy, scored 0: wrapping the hook end first and laying the bladder end over it (`scripts/naive_cuff.py`); the cuff ends 296 degrees round the arm and never fastens.

## Testing in real life

Materials: an adult reusable blood-pressure cuff (16 x 30 cm bladder, for 27-34 cm arms) with a 16 x 16 mm foam hem taped
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
| Also fastened (the hook tab pressed flat on the cuff's outer face, and still on it at the end) | 0.5 |
| Also in place: lower edge 1.5-3.5 cm above the crease, skew round the arm under 2 cm, index line within 2 cm of the artery mark | 0.75 |
| Also snug (the 6 mm dowel cannot be pushed under the edge; in simulation, mean gap to the skin at most 6 mm) and both hands at least 4 cm clear of the arm | 1.0 |

## Reset

Open the cuff (peel the hook tab off), lay it flat across the board under the upper arm with its lower edge on the 2.5 cm
mark and the bladder end to the robot's right, the ends on the table.

## Fidelity

The arm and board are fixed; the upper arm collides as a 10 cm capsule with skin-like friction, and the visible arm is a mesh
1 mm proud of it. The cuff is 20 rigid 2.5 cm plates hinged across its width with light bending springs; it cannot stretch
or crumple. The hook-and-loop is a rule: fastened when 3 hook plates are pressed flat onto the cuff for 0.5 s; after that
only friction and gravity hold the closure.
