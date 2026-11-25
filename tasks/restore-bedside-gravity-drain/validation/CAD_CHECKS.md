# CAD fit and recorded YAM trajectories

These are calculations using the shipped STEP geometry and recorded native rigid-body poses. The fit report checks nominal solids, four indexed settings, the withdrawn pin across the rack sweep, 1 mm pin increments and 5° tap increments. Interference inside the elastomer seal is intentional. The trajectory screen loads the YAM URDF collision meshes, applies each measured world pose and checks mesh-surface intersection with the manufacturing solids. A recorded fingertip can meet the lift, brake tab or tap rotor. Any contact between another link and these parts or any link and another CAD solid fails the screen. The 20 Hz capture samples positions between native physics evaluations.

The screen covers the 11 authored STEP parts. Purchased guide hardware, added tubing, the catch assembly and camera housings outside the imported YAM collision meshes need a loaded physical trial. These screens do not establish as-built clearance, fit at all manufacturing tolerances or behavior under load. The score tests use the submitted simulation.

## Repeat on the revision workstation

Run from the public task directory containing `task.py`. The four recorded pose streams are included under `validation/poses/`. Use a separate Python 3.12 environment for the CAD and collision dependencies. Set `YAM_RUNTIME` to the stock checkout that contains `yam_arena/assets/yam/yam_arm.urdf`.

```bash
set -e
export YAM_RUNTIME="${YAM_RUNTIME:-$HOME/research/robocurve/IsaacLab-Arena-YAM}"
TASK_DIR="$PWD"
URDF="$YAM_RUNTIME/yam_arena/assets/yam/yam_arm.urdf"
test -f "$URDF"
python3.12 -m venv .venv-cad
.venv-cad/bin/python -m pip install 'cadquery==2.7.0' 'python-fcl==0.7.0.11' 'trimesh==5.2.2'
CAD_RUN="runs/cad-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$CAD_RUN"
.venv-cad/bin/python "$TASK_DIR/validation/check_cad_fit.py" "$TASK_DIR/physical/cad" "$CAD_RUN/fit.json"
for mode in reference lowered aligned alternate; do
    .venv-cad/bin/python "$TASK_DIR/validation/check_cad_trajectory.py" \
        "$TASK_DIR/physical/cad" "$URDF" "$TASK_DIR/validation/poses/$mode/poses.jsonl.gz" \
        "$CAD_RUN/$mode.json"
done
```

The identical four streams also support the rigid plumbing screen. In the same shell, run:

```bash
.venv-cad/bin/python "$TASK_DIR/validation/check_water_trajectory.py" "$TASK_DIR/validation/poses" "$URDF" "$CAD_RUN/water.json"
```

The reports record the hashes of the STEP files and the URDF and the uncompressed pose-stream hash for each trajectory. The physical build, plumbing, retention tests and tolerance checks are described in [BUILD.md](../physical/BUILD.md).
