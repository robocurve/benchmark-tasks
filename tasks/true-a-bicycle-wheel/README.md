# True a wobbly bicycle wheel

Use the spoke key to correct the wheel, then release it for a full revolution.
The result is judged by runout, centring and spoke tension, not prescribed turns.
The scripted demonstration scores **1.0** in 89.1 simulated seconds. Dream-1
scores **0.0** after the 120-second limit; it makes no nipple adjustment.

## Run

Use the Isaac/YAM environment pinned in `ENVIRONMENT.json`, with its documented
public asset-root patch. Python 3.12, an NVIDIA rendering GPU and the Dream SDK
are required. Set up Dream credentials through the SDK; never put a key here.
From the repository root, use a new output directory for each run:

```sh
python tasks/true-a-bicycle-wheel/run_quasistatic.py --out DEMO --steps 1782 --plan tasks/true-a-bicycle-wheel/demo-plan.json --tool-servo
python tasks/true-a-bicycle-wheel/run_quasistatic.py --out DREAM --steps 2400 --dream
python tasks/true-a-bicycle-wheel/scoring.py tasks/true-a-bicycle-wheel/evidence/demo-judging.json
```

Reset is fixed, with no object jitter or random seed. Robot control is 20 Hz;
CPU physics is 240 Hz. The demo took 1,136 seconds on a shared A40/9-vCPU
machine. The Dream trial took 1,637 seconds and 296 requests in one session. Runtime
depends on concurrent work and Dream request latency.

The demo uses measured tool and nipple poses to issue bounded ordinary joint
commands. Dream receives the standard camera/proprioception observations and
no scripted feedback. Both use identical geometry and material rules.

## Material approximation

`functional_wheel.py` replaces microscopic spoke/thread dynamics with an
elastic equilibrium model. A physically grasped key must fit the actual nipple
slot and rotate to change its rest length. Rim sectors and nipples follow the
computed geometry. A geometrically qualified rim pinch turns the axle; released
rotation decays with bearing drag. Wrong nipples or turning directions can
worsen the result. Excessive turns or deformation invalidate the model.

Robot and key contacts are native. Spoke tension and runout are computed, not
force-sensor measurements. Key torque reaction, fast vibration and permanent
damage are not calibrated. This tests diagnosis, tool alignment, adjustment and
release, not force control on a particular commercial wheel.

`setup.json` lists the fixed dimensions and positions. `JUDGING.md` defines the
physical checks. `VALIDATION.json` links the recorded results and source hashes.
The numerical review overlay is labelled as simulation measurements and does
not change policy observations or the underlying video timing.
