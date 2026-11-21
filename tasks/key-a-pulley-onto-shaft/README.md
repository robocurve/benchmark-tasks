# Key a pulley onto a shaft

Instruction given to the policy: `Fit the pulley to the shaft and lock it.`

The instruction does not mention the key, the hex driver or the torque. The scored steps it leaves unstated: pick the
key off the mat, press it fully into the keyseat (about 6 N), turn the pulley until its keyway lines up with the
key, slide it over the key to the shoulder, take the hex driver from its rack, set it in the set screw, tighten the
screw onto the key to at least 0.4 N*m, take the driver out and put it down. A pulley forced over a key that is not
fully home scores 0.

Files: `task.json`, `key_a_pulley_onto_shaft.py` (scene, `success()`, partial-credit `score()`, the scripted
`demo()` and an offline self-test), `mechkit.py` (joints added to the stock runtime in memory), `README.md`.

## Hardware and fits

| Part | Size | Fit |
| --- | --- | --- |
| Shaft | 20 mm steel, shoulder collar 30 mm | pulley bore 20 mm H7, sliding |
| Key | 6 x 6 x 25 mm, round ends (DIN 6885 form A) | 6.00 mm wide |
| Keyseat (shaft) | 6 mm P9, 3.5 mm deep, 25.6 mm long | press fit, about 6 N; 6.10 mm wide in sim |
| Keyway (pulley) | 6 mm D10, 2.8 mm deep | 6.05 mm wide: 0.05 mm side clearance, 0.3 mm over the key |
| Set screw | M6 x 1.0, 5 mm long, 3 mm hex socket, flat point | point flush with the keyway roof, 0.3 mm clear of the key (108 deg of turn) |
| Hex driver | 3 mm hex tip, 8 mm shank, 20 mm square handle, 46 mm long, 60 g | stands in a rack block with a 12 mm hole |

## The set screw and the tightening check

The screw is one body in the boss. Its joint lets it turn about its own axis and travel along it; the travel is
driven to 1 mm per turn of thread that has slipped, so the point moves down as the screw is turned clockwise and
stops on whatever it meets (the drive pushes it with at most 0.6 N). Clamp force is 4 N for every micron the thread
is run past live bearing contact of the screw's flat point on the seated key's top face; if there is no seated key
under the point (or contact is lost) there is none.
Thread friction resists turning in both directions with 0.02 N*m + 1.2 mm x clamp force (nut factor 0.2 on 6 mm), so
the screw stays where it is left and tightening takes real torque through the gripper's hold on the driver handle.
The hex tip engages the socket (a joint that lets the driver slide along the screw axis and turn with it, at each of
the six positions; driver-screw collision is filtered so the tip enters the socket) only when it is within 0.5 mm,
4 deg of tilt and 10 deg of a hex position, and at least 1 mm in.

Two checks give the locking credit, both reproducible on a real bench:

1. Seating torque: turning the screw further takes at least 0.4 N*m (a torque driver set to 0.4 N*m clicks without
   the screw moving). In sim this is the thread friction torque at the clamp force reached, about 317 N.
2. Proof pull: with the driver out and put down and both grippers 2 cm clear for 0.5 s, the pulley is pulled off the
   shoulder with 20 N for 1 s (a spring scale; in sim a soft spring joint). It may give 0.2 mm. The clamp holds the
   pulley on the shaft with 0.3 x clamp force, so a seated screw holds about 95 N and a screw that only touches
   the key lets the pulley slide.

All joints are authored when the scene is built. The key weld, the six hex-engagement joints, the clamp hold and the
proof pull start disabled and are only switched on; nothing is created during a run.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/key-a-pulley-onto-shaft/key_a_pulley_onto_shaft.py
```

Asserts the scoring rules on hand-built states, the fits and the thread numbers, and replays both arms' demo plans
through the kinematics. Expected:

```
scoring: 22 cases pass
left: 62.7 s plan, worst IK 0.094 mm / 0.016 deg
right: 15.9 s plan, worst IK 0.092 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/key-a-pulley-onto-shaft/key_a_pulley_onto_shaft.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the pulley to the shaft and lock it." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/keypulley-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted`. Open loop and deterministic. The right arm pinches the key, lowers it
into the keyseat and presses it home (the target is 28 mm past home; the arm gives 3 mm per newton and the pad gives
way at 6 N), lets go and parks. The left arm drags the pulley rim round with closed fingertips from 45 deg to within
about 6 deg of the key, pushes the boss front face with closed fingertips until the pulley stops on the key's round
end, leans it sideways while pushing so the round end turns the keyway into line, and pushes it over the key to the
shoulder (the target is 32 mm past the shoulder to make up for the arm's give under load). It then takes the hex
driver from the rack by its handle, lowers the hex tip into the set screw with the wrist at +60 deg, turns the wrist
clockwise to -108 deg in one stroke (the screw meets the key after about 110 deg and seats at about 0.59 N*m), holds,
lifts the driver out and stands it back in the rack. The bench pull then runs and the pulley stays. `KEYPULLEY_DEBUG=1` prints the key, press depth, slide, phase, screw turn, clamp force,
torque, hex engagement, pull and score every second.

Wrong-path checks (same command, `DEMO_VARIANT` in the environment):

- `no_key`: the pulley turned to top-dead-centre, slid home and the set screw engaged with the hex driver and turned
  the full clockwise stroke with no key in the shaft. The screw finds nothing to bear on. Scores 0.
- `hover_key`: the keyseat press pad is pre-depressed with the hex tip first, then the key is held 0.35 mm above the
  seated height and released onto the pre-depressed pad without driving the interference fit. Scores 0.
- `off_key_screw`: the key is seated and the pulley slid over the key but stopped 8.5 mm short of the shoulder so the
  set screw turns clockwise axially ahead of the key's front end. Scores 0.25.
- `snug`: key and pulley fitted, the screw run down until it just touches the key (wrist to -55 deg), not torqued.
  Below 0.4 N*m, so no locking credit; the bench pull then slides the pulley off the shoulder. Scores 0.25.
- `skip_screw`: key and pulley fitted, the set screw never touched. Scores 0.5.

## Videos and screenshot

```bash
for log in ~/yam-eval/keypulley-*/*.json; do inspect-robots video "$log"; done
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz). The demo ends in about 1,300.
- One fixed layout (`-E object_jitter=0`). Dream-1 is hosted and not seeded; the demo is deterministic.
- Roughly 1 minute of wall clock per 400 steps on an L40S, plus Isaac Sim warm-up.

## Reset between trials

Stand the driver back in its rack, back the set screw out until its point is flush with the keyway roof, pull the
pulley back to the shaft end with its keyway 45 deg off the top, tap the key out and lay it on the mat.
