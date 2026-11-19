# Fit lever-lock chain handcuffs on two training wrist forms: each bar ratcheted to a fingertip gap, then double-locked

Two YAM arms put one cuff of a pair of chain handcuffs round each of two fixed wrist forms, close each swinging bar
until a 6 mm rod still fits between cuff and form but a 12 mm rod does not, push both double-lock lever tabs home,
and move both grippers clear. The ratchet only tightens, so a notch too far cannot be undone. A cuff that is not
double-locked keeps tightening. This is a bench drill on rigid forms; no person is involved.

## Requirements

The official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), which provides
`inspect-robots` and the `yam_arena` embodiment. For Dream-1, a Dream API key in `~/.dreamlabs/credentials` and the
Dream-1 bridge on port 8211. Nothing else: `bodykit.py` (multi-shape bodies and joints) and `cuffdemo.py` (the scripted
demo's camera fit and hooks) are in this folder.

## Run and score a trial

From this folder:

```bash
INSTR="Put one handcuff round each wrist form. Close each cuff so a 6 mm rod still fits between cuff and form but a 12 mm rod does not. Double-lock both cuffs. Then move both grippers clear."
ARGS=(--embodiment yam_arena -E task=wrist_form_cuffs.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360
      --instruction "$INSTR" --scorer task_score --max-steps 2400 --store-frames)

# Dream-1
inspect-robots run "${ARGS[@]}" --policy dream -P server_url=http://127.0.0.1:8211 --seed 0 --log-dir ~/yam-eval/cuffs-dream

# The scripted demo, which reaches full score (seeds 0, 1, 2 shown)
inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=3 --seed 0 --log-dir ~/yam-eval/cuffs-demo-s0

# The same demo made wrong on purpose (see Results): loose, tight, nolock, holding, fixed
CUFF_FAIL=nolock inspect-robots run "${ARGS[@]}" --policy yam_scripted -P hold_s=3 --seed 0 --log-dir ~/yam-eval/cuffs-nolock
```

The run prints `task_score` at the end. The trial is scored at its last step (the scripted demo stops itself once
both arms are back at the ready pose). With `CUFF_DEBUG=<file>` set, every control step adds one line to that file:
each cuff's bar angle, notch, lever travel, pose, play and the score so far; `<file>.plan` gets the planned and
measured tool poses. `dev/run.sh <name> <policy> <seed> <steps> [ENV=VAL ...]` wraps the command with these logs.

## Scoring

Tools: luggage scale, 6 mm and 12 mm gauge rods, ruler. All measured at the end of the trial.

| Check | Real trial | In the simulator (`report` in `wrist_form_cuffs.py`) |
|---|---|---|
| Hands off | both grippers at least 50 mm from every cuff part, else the trial scores 0 | finger points from the arms' joint readings against points on every cuff part and chain link |
| Cuff round the form | the form's post passes through the closed loop of exactly one cuff | the form's centre inside the loop outline, the bar's tip at the lock case, the cuff lying flat |
| Latched, 0.20 | a 30 N pull on the bar away from the lock case for 2 s backs it out less than 3 mm (one tooth) | the bar has reached a notch and sits within 1 degree of it (the lock rule lets a latched bar back out only to its notch) |
| Gap, +0.15 | side play of the cuff on the form, taken up with 3 N each way, is at least 6 mm and under 12 mm | play computed from the notch: the cuff's loop outline slid across the form's outline (8.8 mm at notch 8, 10.1 mm at notch 7, 7.0 mm at notch 10) |
| Double-locked, +0.15 | play at least 6 mm, and a 20 N squeeze of the bar toward the double bar for 1 s advances it less than 3 mm | play at least 6 mm and the lever tab has reached 4.8 mm of its 5.5 mm travel |

Score = sum over the two forms, at most 1.0. Outcomes: unlatched 0; too tight 0.20 per form; too loose but
double-locked 0.35 per form; right gap but not double-locked 0.35 per form; right gap and double-locked 0.50 per form.

## Setup

Exactly as in `task.json`. The positions there are reference positions; every trial's seed draws offsets from them
(`layout` in `wrist_form_cuffs.py`): the whole pair moves 15 to 35 mm toward -y, away from the forms, and each cuff
moves and turns on its own within the ranges in the setup text. The framework turns `--seed N` into its own episode
seed. Each trial's draw is written to the first line of the `CUFF_DEBUG` log.

Why the pair shift: the gripper opens 60 mm round a 34 mm lock case and its closing fingers centre and square the case,
so smaller offsets are taken up without looking. With offsets of up to 14 mm only, a plan made for the reference
layout and never looking at the camera scored 1.0 (see Results). Over 3,000 draws, cuff A now starts at least 16 mm in y
off its reference position in 72% of trials.

| Seed | Episode seed | Cuff A rivet (cm), turn | Cuff B rivet (cm), turn |
|---|---|---|---|
| 0 (setup image, video) | 1415158438 | (8.74, -4.62), -0.9 degrees | (33.15, -2.23), -5.2 degrees |
| 1 | 2566101560 | (8.74, -3.60), -1.5 degrees | (31.82, -0.27), -0.2 degrees |
| 2 | 377265627 | (8.91, -3.91), +2.7 degrees | (33.20, -0.65), -1.6 degrees |

## How the scripted demo works

`yam_scripted` with the hooks in `cuffdemo.py`. It uses only what a policy is given: the top camera's RGB image, the
camera's published calibration, the sizes in the setup text, and the arms' own joint and gripper readings. It reads
no simulator object pose and no notch or lever state.

1. With both arms at the ready pose, it looks at the top camera and fits cuff A's known outline (lock case, double
   bar, rivet boss) to the colourless steel pixels. A constant offset of the fit (2 to 3 mm, measured once against
   the true poses on the first frames of three earlier trials) is subtracted. Found minus true rivet position at the
   start of the three demo runs: 0.4, 0.3 and 0.8 mm.
2. The right hand takes cuff A's lock case with a 30 N grip, lifts it 4 mm, lines it up beside the near form and
   slides it round the form. The left fingertip swings the open bar round the form and pushes it in to about
   notch 8, then eases off.
3. The right hand lets go and rests a closed fingertip against the lock case's far side at its end. The left
   fingertip pushes the lever tab home against it, 7 mm past the lever's home position to take up what the cuff
   gives.
4. Both arms go back to the ready pose; the camera looks again and finds cuff B, which the chain has dragged and
   turned (by 24 degrees on seed 0). The same steps fit cuff B on the far form.

## Results

All runs on this version (`wrist_form_cuffs.py` sha256 `8fbecc3e58d831fd`..., `cuffdemo.py` `cec1fa0f23545963`...,
`bodykit.py` `4e76ca4411694aa3`...) unless marked, logs in `runs/` (frames not included), `--max-steps 2400`. The force
check in Fidelity (`runs/m1-a/`) ran on an earlier version of the task file, with one cuff already on its form
(`CUFF_PROBE=m1`); it was not repeated on this version.

| Run | Seed | Score | Steps | At the end |
|---|---|---|---|---|
| demo (the video) | 0 | 1.0 | 1,518 | A: notch 8, play 8.8 mm, lever 5.5 mm; B: notch 7, play 10.1 mm, lever 5.5 mm; grippers 99 mm clear |
| demo | 1 | 1.0 | 1,518 | A: notch 7, play 10.1 mm; B: notch 8, play 8.8 mm; both levers 5.5 mm; grippers 99 mm clear |
| demo | 2 | 1.0 | 1,518 | A: notch 7, play 10.1 mm; B: notch 7, play 10.1 mm; both levers 5.5 mm; grippers 99 mm clear |
| `CUFF_FAIL=fixed`: the same plan made for the reference layout, no camera | 0 | 0.0 | 1,633 | neither cuff on a form, both bars open |
| `CUFF_FAIL=fixed` on the earlier, narrower start (offsets up to 15 mm, no pair shift; `f142bc75c141e06d`, `runs/fixed-narrow-s0`) | 0 | 1.0 | 1,518 | the grip took up a 14 mm offset: the reason for the pair shift |
| `CUFF_FAIL=holding`: cuff A done; cuff B closed, lever not pushed, right hand still holding it | 0 | 0.0 | 1,436 | without the hands-off rule 0.85 (A 0.50, B latched at 10.1 mm 0.35); gripper 0 mm from cuff B |
| `CUFF_FAIL=nolock`: lever tabs never pushed | 0 | 0.7 | 1,272 | both latched at a good fit (A 8.8 mm, B 10.1 mm), levers 0 mm: 0.35 per form |
| `CUFF_FAIL=loose`: bars pushed only to about notch 3 | 0 | 0.7 | 1,568 | both double-locked, too loose (A notch 4, 14.2 mm; B notch 2, 17.2 mm): 0.35 per form |
| Dream-1 (the model video; logs in `runs/h-dream-s0`) | 0 | 0.0 | 2,400 (the limit) | neither cuff on a form, both bars open, no notch reached |

## What is not shown

- Seeds beyond those in the table. The camera fit's constant offset was measured on trials with the same three
  episode seeds under the earlier, narrower start, so the demo seeds are not fully independent of it.
- The ratchet and the double lock working by contact of modelled teeth and blocker (they are rules; see Fidelity).
- Any real-cuff trial.

## Fidelity

- PhysX through the official `yam_arena` simulator, 20 Hz control.
- The ratchet is a rule in the step hook: the hinge's lower stop follows the deepest notch the bar has reached, and a
  spring that rises to 3.5 N at the teeth over one tooth resists the next notch. Checked once by force with one cuff
  on a form: a 30 N pull for 2 s backed the bar out 0.625 mm; a 20 N squeeze (not double-locked) drove it from
  notch 10 to 16, 18 mm, until it met the form.
- The double lock is a rule: once the lever tab has slid 4.8 of its 5.5 mm, the hinge is held at its notch.
- The scorer does not apply the pull and the squeeze at the end; it applies their outcome by the rules above.
- Forms are rigid and fixed to the table. Not from maker data: lever shape and travel, tooth size, pawl spring,
  bar outline.

## Why it matters, and the sources

| Fact | Source |
|---|---|
| Police and sheriff's patrol officers (33-3051): 646,310 employed, May 2023 | https://www.bls.gov/oes/2023/may/oes333051.htm |
| Correctional officers and jailers (33-3012): 351,420 employed, May 2023 | https://www.bls.gov/oes/2023/may/oes333012.htm |

Patrol officers cuff at arrests and before every transport. Correctional officers restrain people for escorts,
transfers and cell moves. The cuffs are sized on the Smith & Wesson Model 100; the lever, teeth and pawl are not
from maker data.

## Real trial and safety

Rigid plastic or wooden forms (54 x 40 mm post, 30 mm tall, with a 70 x 36 x 10 mm cap) screwed to the table, and a
real pair of Model 100 lever-lock cuffs. No person is ever cuffed. Keep the key at the station.

## Files

- `wrist_form_cuffs.py`: the scene, the lock rules, the scorer, the seeded start and the demo's plans.
- `cuffdemo.py`: the demo's camera fit and the hooks into `yam_scripted`.
- `bodykit.py`: multi-shape bodies and joints for `yam_arena`.
- `dev/run.sh`: one trial with logs. `runs/`: the logs behind the Results table (frames not included).
