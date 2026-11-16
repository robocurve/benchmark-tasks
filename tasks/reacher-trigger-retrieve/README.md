# Retrieve a remote with a handheld trigger reacher

Retrieve a TV remote across a client's table with a trigger-operated reacher and
leave it within a marked pickup area. The reacher is returned to the table with
its jaw open and clear of the remote. The scene contains only the equipment.

Home health aides teach clients and families to live independently and adapt to
disability. This trial isolates operation of a daily-living aid that an aide can
demonstrate while a client remains off camera. [O*NET 31-1121.00](https://www.onetonline.org/link/summary/31-1121.00)
and [31-1122.00](https://www.onetonline.org/link/summary/31-1122.00) describe the
occupations. [BLS May 2025 OEWS, series OEUN000000000000031112001](https://api.bls.gov/publicAPI/v2/timeseries/data/OEUN000000000000031112001)
reports 4,305,810 employees in the combined occupation. The reference
[450 mm reacher](https://www.essentialaids.com/household/reaching-aids-and-grabbers/etac-aktiv-reacher.html)
weighs 111 g, has horizontal and vertical jaw settings, and uses an aluminium
shaft, plastic shell and jaw brackets. The package uses generic visual models
built from the declared collision geometry (original assets, MIT licence).

## Setup and scoring

Set out the objects at the positions in task.json. Use the horizontal jaw
setting and lie the reacher on its side for tabletop access to the trigger.
The shell, trigger, axial link and movable jaw are passive rigid bodies.
Squeezing the trigger mechanically pushes the link and closes the jaw. The
hinges have return springs. The distributor specifies dimensions, mass and
a light grip. Internal geometry and the return spring's 0.55 N·m/rad torque
slope are modelling estimates, with about 7 N spring-only force at the
37 mm trigger lever and 26 degrees of squeeze. The jaw and trigger profiles
are simplified from the exterior reference. A physical trial can use a
spring scale at the contact point and video from above to read the joint
angles and travel.

The observe-only hook measures how far the remote moves under the linked
jaws during the squeeze. For the real task, watch the contact between
the remote and the reacher jaws and measure the remote's travel against a
ruler fixed to the tabletop. Read the task.json scoring thresholds for full
placement and clearance. SUBACTION_END logs displacement, placement and release
at the end of the 1.6-second success hold. The Q6 proxy for the small gripping
interface is the visible remote displacement and final placement accompanied by
joint readouts and close-ups at initial grip and final release.

## Run

From this directory after activating the Isaac Lab-Arena environment:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/reacher_trigger_retrieve.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=1280 -E cam_height=704 --instruction "$instruction" \
  --scorer task_score --max-steps 800 --store-frames --log-dir ./demo
inspect-robots run --embodiment yam_arena --policy dream \
  -E task="$PWD/reacher_trigger_retrieve.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 800 --store-frames --log-dir ./dream
```

Runtime per trial is up to 40 seconds of control time. Seed: none. The
script uses only downward gripper waypoints with yaw about the vertical axis.
Reset the equipment to the task.json positions before each trial.

| Asset files | Origin and licence |
|---|---|
| `assets/*.usda` | Original visual-only geometry and materials, MIT licence |
