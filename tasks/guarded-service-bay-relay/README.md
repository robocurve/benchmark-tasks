# Guarded service-bay relay

A two-arm industrial inspection/service-cell part transfer. Two YAM arms must exchange a 40 mm, 50 g inspection component across a shared transfer bay while keeping their complete collision link envelopes outside a rigid 660 mm equipment keepout fixture and respecting exclusive bay ownership.

The amber solid represents an **equipment-restricted work zone**, not a simulated human. No ISO compliance or contact-force threshold is claimed.

## Task / setup

`guarded_service_bay_relay.py` defines the YAM station, all four object fixtures, object score, arm safety score and executable two-arm reference plan. The task is self-contained in this folder. `arm_collision_model.json` records conservative capsule bounds fitted to each YAM USD collision subpart, in the moving rigid-body local coordinates; `arm_safety.py` reads all 18 current rigid-body transforms directly through GPU PhysX/Torch. Stale USD world transforms are **never** used for scoring.

The scorer reads the two arms at the 20 Hz control rate (not at each 240 Hz physics step). It denies full credit if link-to-fixture geometry is within the 4 mm protected margin, if inter-arm collider envelopes overlap, if both tool midpoints occupy the exclusive transfer bay simultaneously, if the receiving arm enters prematurely, or if a tool crosses into the opposing arm's reserved station. Violations latch at zero. Missing or invalid body state also scores zero.

The **object subscore** is 0, 0.5 for a supported and stable staging set-down after a lift, or 1.0 for a subsequent lift and supported stable final set-down. A transferred cube must be truly close to the pad top; a hover above it cannot earn credit. A direct source-to-goal shortcut does not earn full credit. Actual gripper contact forces are not measured. The model reports geometric contact *candidates*, not measured force or exact mesh contact.

## Reproduce a trial (YAM / Inspect Robots environment)

From inside this folder, with the YAM environment activated:

```bash
SERVICE_BAY_WAYPOINT_PROFILE=sequential_relay_tuned \
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=guarded_service_bay_relay.py -E object_jitter=0 \
  -E control_hz=20 -E headless=true \
  --scorer task_score --max-steps 1100 --seed 42 --no-rerun --no-prompt \
  --instruction "Transfer the component using a supported, exclusive two-arm handoff; avoid the protected fixture."
```

The task runner under the full development package saves the exact task hashes, in-scorer arm samples, independent native physics trace, geometric audit, and H.264 camera replay for validation. Each trial should start in a new YAM process so the event-latched scorer resets.

To run a model with the same task, substitute `--policy dream` and provide the instruction in `task.json`. Media will be uploaded separately through the RoboCurve task board.

## Repeatable scoring behavior

- **1.0** — Supported staged handoff followed by a supported target placement, and no observed protected volume/inter-arm/exclusive access violation.
- **0.5** — Only valid staged set-down and no observed arm safety violation.
- **0.0** — No valid staged set-down, an illegal shortcut, invalid state, or any detected safety violation.

A fixed 4 mm geometric margin is used because the submitted task claims *equipment exclusion*, not measured contact force. All link collision geometry is conservative, derived from the installed YAM USD physics colliders, and can over-report near-contacts. Sampling at control rate can miss shorter transient events; internal development includes a separate 240 Hz full-body physics audit for cross-checking. The geometric grader is not an industrial safety certificate.
