# Clamp locking pliers onto a tube and leave them there

An aluminium tube, 2 cm across and 16 cm tall, stands fixed upright on the table in front of the right arm, with an
orange band of tape 9 to 11 cm above the table. A pair of steel locking pliers, preset for a 2 cm tube, lies open on
two dark blocks to its right, jaws forward. Both handles are straight and have a uniform 9 by 14 mm rectangular cross-section: no central bulb, enlarged grasp collar or hidden grip collider. The robot must pick the pliers up by their handles, put the jaws around the
tube at the orange band, squeeze the handles until the catch at their ends clicks, and let go: the pliers must stay
clamped on the tube by themselves.

Welders, plumbers, mechanics and machinists clamp locking pliers onto pipes, rods and parts every day, as a stop, a
temporary clamp or a third hand. The task tests tool use that pick-and-place
does not: the tool is carried by the handles that also operate it, its jaws must be placed around a part at a chosen
spot, and it must be squeezed past a lock point with enough force, so that it holds by itself once let go. The cheap
shortcuts fail: let go with the jaws around the tube but not squeezed, the opening spring opens the jaws and the pliers
fall; squeezed shut in the air or on the holder, they lock but are not on the tube; pushed off the holder toward the
tube, they never get onto it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with spring-loaded sliders and hinges, for yam_arena task files |
| `demo.py` | The scripted full-score demo (plans from the published layout and the pliers' pose) |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E device=cpu -E cam_width=320 -E cam_height=176 \
  --instruction "Clamp the locking pliers onto the metal tube at the orange band, level, so that they stay there on their own." \
  --scorer task_score --max-steps 1200 --seed 1 --store-frames --log-dir ~/yam-eval/locking-pliers-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

```bash
python demo.py task.py demo_out                                   # CPU physics, full score: demo.mp4, setup.png, result.json
DEMO_VARIANT=no_squeeze python demo.py task.py out --no-record    # jaws around the tube, let go without squeezing
DEMO_VARIANT=no_tube python demo.py task.py out --no-record       # squeeze shut in the air beside the tube, let go
DEMO_VARIANT=sweep python demo.py task.py out --no-record         # push the pliers off the holder toward the tube
DEMO_VARIANT=idle python demo.py task.py out --no-record          # do nothing
```

Results for the straight-handle task, scored by the standard `task_score` scorer:

| Run | Score | Steps |
|---|---|---|
| Full scripted demo | 1.0 (success) | 498 |
| Dream-1, seed 1 (completed, no errored trial) | 0.0 | 1,200 |
| `no_squeeze`: jaws around the tube, release without locking | 0.0 | 512 |
| `no_tube`: squeeze shut in the air beside the tube and release | 0.0 | 546 |
| `sweep`: push off the holder toward the tube | 0.0 | 282 |
| `idle`: hold both arms at READY | 0.0 | 80 |

A same-process reset after full success also restores the open handles, relaxed jaw pad and catch, and the pliers on
the holder. Twenty subsequent idle actions score 0. The reset hook restores passive joints only during episode reset;
the demonstration moves the tool entirely through robot actions.

## How the lock can be observed

While the gripper squeezes, locked and unlocked handles look the same. The lock shows when the grip opens: locked
handles stay shut (the orange latch is seen over the end of the upper handle). Unlocked ones spring open again:
the handle span at the grasp is about 49.5 mm open versus 36 mm shut, and the jaws open. In the no_squeeze run above the jaws are put around the tube unsqueezed; the pliers
drop off the tube as soon as they are let go.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). Everything is
judged on the final scene with a ruler and a photo.

- 1.0: the pliers are clamped on the tube by themselves (the tube between the jaws,
  the handle ends locked shut by the catch, the pliers at least 2 cm above the table and no gripper on them), the jaw
  centre is on the orange band (9 to 11 cm above the table), and the pliers are level within 10 degrees. The trial ends early
  as a success once they have also been still (within 1 mm) for 1 s.
- 0.5: clamped on the tube by themselves, but off the band or tilted more than 10 degrees.
- 0.25: the tube is between the jaws and the pliers are off the table, but the robot still holds them.
- Otherwise 0.

"Locked shut" means the handle ends are within 2 mm of fully shut, where the catch holds them; the pliers cannot stay
on the tube by themselves unless locked, because the opening spring would open the jaws.

## Runtime and seeds

The 498-action demo takes 24.9 seconds of control time. A full 1,200-action model trial has one minute of control
time; the recorded Dream-1 trial took about four minutes of wall time including simulator startup and model requests.
The physical layout is fixed (`-E object_jitter=0`). The recorded model trial used Inspect seed 1; the scripted demo
uses the fixed layout without a variable task parameter seed.

## Fidelity

The parts are rigid bodies built from boxes and cylinders; the tube and holder are fixed. The pliers are two halves on
a pivot: a fixed half (upper jaw, fixed handle with a rubber grip) and a moving half (lower jaw, moving handle with a
rubber grip), held open against a stop by a preloaded opening spring. Real locking pliers lock with an over-centre
toggle linkage; here that is modelled by an orange spring catch at the end of the fixed handle, which slides back along the handles on a 45
degree chamfer as the moving handle's round pin passes it and springs forward behind the pin, holding the handles shut. The
jaws' give, which stores the clamp force in real pliers, is a sprung pad on the upper jaw. Spring rates and the clamp
force are set low enough for the arm's 50 N gripper to lock the pliers; real locking pliers clamp harder. Opening
spring 0.5 N m per rad (0.55 N m open, 0.72 N m shut); jaw pad 8 N per mm (pressed about 4 mm
on the tube, a 32 N clamp); latch spring 0.3 N per mm. The pliers weigh 160 g. Contacts on
the pliers have an 8 mm minimum torsional patch radius (rubber grips and serrated jaws resist twisting); jaw and grip
friction 1.0, pad 1.2.

## Real-world build

A 20 mm aluminium tube in a hole drilled in a bench block clamped to the table, with orange tape 9 to 11 cm up; small
locking pliers with straight handles, their adjustment knob set to lock on the tube with light force, laid open on two
wooden blocks.


## Third-party assets and licenses

All task-specific geometry, scene code, scoring and the scripted controller are authored for this task and provided
under the MIT License in `LICENSE`. No manufacturer mesh, image or CAD model is bundled.

| Asset or dependency | Use and source revision | License | Bundled here |
|---|---|---|---|
| YAM arm and gripper model | [i2rt](https://github.com/i2rt-robotics/i2rt), revision `120c3c81400171174604e503943f8d1ebc891058`, as distributed in the SDK | MIT (`LICENSE.i2rt` beside the installed asset) | No |
| Standard station floor, table, camera stand, lights and cameras | Shapes generated in code by [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), revision `0059d02a1a91bc6834c1d0846022891df0c78730` | The SDK README states that the repository has no license yet; its YAM assets have the separate MIT license above | No |
| Isaac Lab-Arena | Simulation runtime, revision `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` | Apache 2.0 | No |

The task does not use the SDK station's library objects. The tube, holder, both straight handles, jaws and latch are
primitives authored in this folder. The locking mechanism and lower clamp force are the approximations described
under Fidelity; this is not an exact reproduction of a named commercial model.
