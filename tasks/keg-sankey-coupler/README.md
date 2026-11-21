# Tap a beer keg with a Sankey D coupler

Instruction given to the policy: `Tap the keg with the coupler.`

The instruction does not say how. The coupler has to go onto the valve neck with its ears on the lugs, be turned a
quarter turn clockwise to its stop, and only then have its handle pushed down past over centre, which drives the
probe into the valve. Pushing the handle past 30 deg for 0.25 s before the coupler is turned at least 80 deg on the
valve latches the score to 0.

The keg is free-standing. Nothing clamps or fixes it: it is a full sixtel (5.16 US gal, 26.3 kg) resting on its
bottom chime on the wet counter under its own weight, friction coefficient 0.10. Turning the coupler under the lugs
takes 0.92 N m, and the same torque acts on the keg through the bayonet. The keg's weight resists about 2.7 N m
of turning (0.10 x 26.3 kg x 9.81 m/s2 x 0.105 m, the chime's radius) and about 26 N of sideways push; push harder
than that and it slides or turns, and it can be tipped. Every turn angle in the score is the coupler's angle on the
keg's valve, so a keg that goes round with the coupler earns nothing. Moving the keg costs nothing by itself;
tipping it more than 20 deg or pushing it off the counter fails the episode.

Files: `task.json`, `keg_sankey_coupler.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()`
and an offline self-test), `mechkit.py` (joints, sliders, catches and a revolute catch added to the stock runtime in
memory), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

```bash
python tasks/keg-sankey-coupler/keg_sankey_coupler.py
```

It asserts the scoring rules on hand-built states (among them a keg that has slid and turned, a keg that went
round with the coupler, and a tipped keg) and replays both arms' demo plans and the variants below through the
kinematics at 20 Hz (joints inside the URDF limits, IK error under 0.5 mm).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/keg-sankey-coupler/keg_sankey_coupler.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tap the keg with the coupler." \
  --scorer task_score --max-steps 1400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/keg-sankey-coupler-dream
```

`task_score` is 1.0 when `success()` has held for 1 s, otherwise `score()` at the last step.

## Run and score the full-score demo

Same command with `--policy yam_scripted` (log dir `~/yam-eval/keg-sankey-coupler-demo`). The demo is `demo()`:
open loop and deterministic. The right arm grips the coupler's body, carries it (beer and gas lines trailing) over
the chime turned 45 deg so its ears are over the lugs, lowers it onto the valve neck with a light push and works
it round in a small spiral until it drops on and catches. Still holding the body, it twists the coupler a quarter
turn clockwise to the stop, lets go and clears. Neither arm touches the keg: its weight alone holds it against
the turn. The left arm then rides the handle's top face down past over centre to its stop and clears.

The beer and gas lines are chains of rigid segments on soft spring joints, hung from the coupler's outlet and inlet
and welded at their far ends to a fixed tap tower and regulator (`kit.tie` in mechkit.py, authored with the scene;
no joint is created during a run). Pulling either end more than 5 mm off its fitting latches the score to 0.

Wrong paths, same command with a variable in front (`KEG_DEMO=no_turn inspect-robots run ...`):

- `KEG_DEMO=no_turn`: the coupler is seated and the handle pressed without the quarter turn. Scores 0.
- `KEG_DEMO=no_handle`: the coupler is seated and turned home, but the handle is never pressed. Scores 0.35
  (caught 0.1, turned 0.25; the plan ends before the coupler has been let go and still for 1 s, which is worth
  another 0.1).
- `KEG_DEMO=idle`: neither arm touches anything. Scores 0.

`KEG_DEBUG=1` prints the coupler's state every second, with how far the keg has slid and turned on the counter
(now and at most so far).

## Videos and screenshot

```bash
for log in ~/yam-eval/keg-sankey-coupler-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/keg-sankey-coupler-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,400 steps (70 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`. Dream-1 is hosted and not seeded; the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps, plus Isaac Sim warm-up.

## Reset between trials

Lift the handle back up past centre, turn the coupler back 90 deg and lift it off the valve, stand it on the drip
tray with the handle pointing at the robot and both lines lying in their loops. Put the keg back on its outline
with the lugs near-left and far-right (it may have slid or turned) and leave it standing free: no clamp, strap,
wedge or mat. Keep the surface under it wet, the keg full (water-filled or ballasted to 26.3 kg), both lines
connected, and the CO2 regulator shutoff closed (0 psi).

## Fidelity notes

All parts are rigid-box compounds. Only the top 4 cm of the 26.3 kg sixtel plus its 2 mm bottom chime ring at
10.5 cm radius (`friction = 0.10`, static and dynamic) is modelled, reproducing the ~2.7 N m spin resistance and
~26 N lateral slide resistance of an unclamped full sixtel on wet stainless steel while simplifying the full-height
barrel centre of mass. The bayonet is a revolute catch between the coupler and the free-standing keg (coupler-keg
collision stays enabled) that engages within 11 mm XY, -2 to +3 mm Z, 5 deg tilt and 10 deg lug-entry yaw; a
trailing rest-angle spring applies 0.92 N m turning resistance (transmitted equally to the keg) and 0.4 N m ramp
draw-in over the last 15 deg to the 90 deg stop. The handle over-centre toggle and 11.5 mm probe cam use spring
drives, and each 36 cm hose is twelve 3 cm, 20 g rigid segments on 0.01 N m/rad spring hinges with a 60 N breakable
end weld.
