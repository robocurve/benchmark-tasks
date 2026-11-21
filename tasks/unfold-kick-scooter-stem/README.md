# Unfold a kick scooter's folding stem until its latch pin locks upright, then close the joint quick-release clamp

Instruction given to the policy: `Unfold the kick scooter's stem until it locks upright and close the quick-release clamp, then let go.`

Files: `task.json`, `unfold_kick_scooter_stem.py` (scene, `success()`, partial-credit `score()`, scripted `demo()` and offline self-test), `mechkit.py`, and `README.md`.

## Environment

The program's launchable environment (Isaac Lab-Arena, Isaac Sim, L40S) with yam-arena and inspect-robots.
Before running commands:
```bash
source ~/IsaacLab-Arena/yam.env
```
Dream-1 credentials are in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository:

```bash
python tasks/unfold-kick-scooter-stem/unfold_kick_scooter_stem.py
```

It validates scoring rules on hand-crafted state sequences and verifies arm plans through kinematics at 20 Hz (URDF joint limits, IK error under 0.5 mm, reach within bounds). Expected output:

```
[unfold-kick-scooter-stem] offline self-test: 7/7 checks passed cleanly.
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/unfold-kick-scooter-stem/unfold_kick_scooter_stem.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unfold the kick scooter's stem until it locks upright and close the quick-release clamp, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/unfold-kick-scooter-stem-dream
```

`task_score` is 1.0 when `success()` holds for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/unfold-kick-scooter-stem/unfold_kick_scooter_stem.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unfold the kick scooter's stem until it locks upright and close the quick-release clamp, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/unfold-kick-scooter-stem-demo
```

The deterministic scripted demonstration executes in ~28.2 seconds (~564 steps), well within the 1,200-step limit:
1. The kick scooter deck is an authentic commercial Razor A / Micro Sprite folding kick scooter mounted rigidly in bench vise jaws with rubber protective pads at the rear, supporting the deck securely against tilting during the real push test.
2. The left arm approaches the folded stem, grips the column tube 12 cm above the hinge, and lifts it smoothly along the 90 deg circular arc upright against the vertical stop (0 deg).
3. At vertical (0 deg), the spring-loaded joint latch pin snaps into the locking socket (pin_slide >= 5 mm).
4. While the left arm holds the stem firmly upright against the vertical stop, the right arm approaches the quick-release clamp lever at the base bracket and swings it from 80 deg (open) to 0 deg (clamped flat against the bracket), locking the joint.
5. Both arms release and retreat to clear parking positions (> 5 cm clear).
6. To achieve full score (1.00), the upright stem, seated latch pin, closed quick-release lever, and hands-clear status are all explicit prerequisites alongside the push test. After 2.0 s of stillness, the automated test rig applies a 12 N horizontal push against the upright stem for 1.5 s, verifying stability and latch retention (< 1.0 deg deflection).
