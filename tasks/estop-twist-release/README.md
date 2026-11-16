# estop-twist-release -- emergency-stop function test on a running benchtop machine

The current scripted Isaac/PhysX demo scored **1.0 in 264 control steps** (13.2 simulated seconds), with 265 frames per camera at 20 Hz. The separate green START control and no-auto-restart behavior are included in this run. The same-version Dream-1 attempt was interrupted by `DreamSessionExpired:expired` after 128 control steps. Its last printed trajectory score was 0.0 at step 120; Inspect Robots returned no aggregate score. The attached model video is this actual interrupted attempt, not a completed 600-step evaluation. An earlier connection-interrupted attempt is not used as a model score. No model success or completed zero-score result is claimed.

### Minimal repair

The scorer now observes actual lens position and platen motion before granting stop, no-restart dwell or final-running credit. Platen movement is target-driven rather than teleported, and the hinge spans the full trial without wrapping. A button edge cannot restart while the head is still depressed after twisting. Required missing/nonfinite joints fail closed. Stop tolerance is 0.20 rad/s and running threshold is 0.50 rad/s; lens tolerance is 2 mm.

This task still tests a trajectory with the same initial and final physical state. It has not been redesigned into a diagnostic inspection. The green START control is separate from resetting the e-stop device; see the START-semantics correction below. No safety-compliance claim is made. CPU contact dynamics are not Isaac/PhysX validation.

A single-platen grinder-polisher runs on the bench (platen turning, green RUN lamp lit). Its emergency stop is a
40 mm red mushroom push-to-lock / turn-to-release operator in a yellow 1-hole station; a grey 2-hole station next to
it carries the green START button and the RUN lamp. The robot runs the periodic e-stop function test: press the head
until it latches (the machine stops, the lamp goes out), twist the head clockwise until it unlatches and springs back
out, check that the machine stays stopped, then press START (the machine restarts) and move clear.

Why it matters: an e-stop must latch when pressed and stop the machine; releasing it must not restart the machine;
restart requires a separate deliberate action. Twisting the e-stop head resets that device without initiating motion. The separate green START button provides deliberate restart. The simplified contactor model has no separate safety-relay reset and makes no ISO/IEC compliance claim. Machine owners test the
function periodically; the test is done by the people who run and maintain the machines, e.g. general maintenance and
repair workers (O*NET 49-9071.00; 1,629,700 US jobs in 2024, BLS Occupational Outlook Handbook,
https://www.bls.gov/ooh/installation-maintenance-and-repair/general-maintenance-and-repair-workers.htm).
Unlike twist-only tasks (e.g. zeroing a dial indicator), the skill is the whole safety-circuit test: an axial press
through a 10.5 N latch point, a spring-return release the gripper has to let travel, and the stop / no-restart /
reset order. Unlike safety-device inspections that test a device, judge pass or fail and tag out a faulty machine
(for example a light-curtain test-rod check), nothing here is diagnosed or tagged: the e-stop is sound and the score is
the operating sequence itself -- a latch that stops a running machine, a release that must not restart it, and a restart
only through the separate START.

## Commands

Official runtime: Isaac Lab-Arena + Inspect Robots (`yam_arena` embodiment, station scene). The instruction is the
`instruction` field of `task.json`.

Scripted demo (open-loop waypoints, `320x176` official camera resolution):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/estop_twist_release.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/estop-twist-release-demo
```

Dream-1 (same flags, `--policy dream`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/estop_twist_release.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/estop-twist-release-dream
```

Videos: `for log in ~/yam-eval/estop-twist-release-demo/*.json; do inspect-robots video "$log"; done`.
Offline tests (no simulator): `pytest -q test_estop_twist_release.py`.

Runtime per trial: the scripted demo plan is 12.3 s; the episode ends once the full score has held
`SUCCESS_HOLD_S = 2.0` s with the arm parked: 264 control steps (13.2 s simulated) in the same-version Isaac/PhysX scripted run. Dream-1
runs up to the 600-step limit (30 s simulated). Seeds: none needed -- one fixed layout, every object fixed, no randomised
object or physics parameter.

## Mechanism and scoring

Real-size parts with real contact (labkit joints in `estop_twist_release.py`; values from the cited operator):

| part | joint | value |
|---|---|---|
| mushroom head travel | slide, 0 to -4.5 mm (max operator stroke) | return spring 2625 N/m = 10.5 N push-to-lock force at the 4.0 mm latch point |
| head turn (rides on the travel) | hinge about the vertical, 0 to 40 deg clockwise | torsion spring 0.31 N*m/rad = 0.16 N*m turn-to-reset torque at the 30 deg release angle |
| latch | detent: at 4.0 mm (the direct-opening stroke) the pawl catches; the spring then holds the head at 4.2 mm | pulling the head does not unlatch it; the cam lifts the pawl only after >= 30 deg of turn; it can only catch with the head turned home (within 15 deg) |
| START | slide, 0 to -4 mm, 1250 N/m (5 N at full stroke, declared) | contact closes at 2.5 mm |
| contactor | self-holding: drops out when the e-stop latches; pulls in only on a START press with the e-stop released | RUN lamp and platen (60 rpm) follow it |

Scoring (task.json `scoring`, code `_components()`; the same named constants drive the hook, the score and the log):
+0.35 latched (head turned home, within 15 deg) while the machine ran; +0.20 released by the turn and head back out (within 0.5 mm) at the end;
+0.10 head out and machine still stopped for >= 1.0 s without a break before START (START sooner: 0.8 at most); +0.25 START pressed after the release (never
while latched) and machine running at the end; +0.10 all of these and both gripper centre points >= 100 mm (3D) from
the head-top and START-cap-top centres. Possible scores: 0, 0.35, 0.55, 0.65, 0.8, 0.9, 1.0. Score over the end state (default `SCORE_OVER`): a new latch at any
time clears the release, no-restart and reset parts until they are earned again (undo after success: 0.35 while latched).

## Sub-action end states

Observe-only prints (no effect on actions or scoring): every 10 steps one `ESTOP_TRACE step=... press_mm=...
twist_deg=... reset_mm=... latched=... running=... released=... score=...` line (the event history: latch, release,
reset), and while the full score holds one `SUBACTION_END {...}` JSON line per sub-action plus, on the last step, the
summary `SUBACTION_END latched_once=... released_by_twist=... head_press_mm=... head_twist_deg=...
stopped_after_release_s=... reset_after_release=... reset_while_latched=... running=... button_clear_mm=... score=...`.

| sub-action | end state held at the last step | top camera (320x176) |
|---|---|---|
| press to latch -> machine stopped | event (`latched_once`), shown by the run's ESTOP_TRACE (lamp out, platen still) | head 40 mm |
| twist to release -> head back out | `head_press` within 0.5 mm of rest | head released |
| no restart after the release | event (`stopped_after_release_s` >= 1.0) | lamp off, platen still |
| START after the release -> running | contactor on: RUN lamp lit, platen turning | lamp, platen |
| arm clear | nearest gripper centre >= 100 mm from both buttons | whole work area |

## Assets and provenance

| object | library search (query -> chosen hit) | source | licence | sha256 (first 16) |
|---|---|---|---|---|
| e-stop operator, 1-hole station, legend plate | `search.py "emergency stop button"` -> objaverse `emergency-stop-button-012e4809`, physxverse `9dbf2a60...` (static meshes, no moving head) -> no usable match | builder-made, `assets/build_visuals.py`, to the cited catalogue (below) | task licence | `estop_visuals.usdc` |
| 2-hole station, START, RUN lamp | `search.py "push button control station"` -> no hits | builder-made, same file | task licence | same |
| grinder-polisher | `search.py "polishing machine"` -> physxverse "Pulido" (cc-by-nc, static); `search_cloud.py "fan"` -> car fan shrouds | builder-made, same file | task licence | same |
| caliper | `instrument_caliper_a01` | NVIDIA SimReady, https://huggingface.co/datasets/nvidia/simready-assets | CC-BY-4.0 | `ef4326a552367ae3` |
| solvent spray can | `consumable_spray_solvent_a01` | NVIDIA SimReady, same | CC-BY-4.0 | `7b1099620ca28bf9` |
| box of nitrile gloves | `box_glove_a01` | NVIDIA SimReady, same | CC-BY-4.0 | `1321409ad7afacf1` |

Cited product data (builder-made models are true to these; no maker names or marks on the models):
- E-stop operator: IDEC X-series E-stop catalogue, XA series (16 mm panel, 40 mm mushroom XA1E-BV4..): push-to-lock
  10.5 N, turn-to-reset 0.16 N*m, minimum stroke for direct opening 4 mm, maximum operator stroke 4.5 mm, 40 mm
  button height 30.4 mm, 28 g. https://files.valinonline.com/userfiles/documents/idec-e-stop-catalog.pdf ; reference
  photo: https://www.idec.com/en-in/switches-indicator-lights/switches-pushbuttons/emergency-stop-switches/xa-16mm-estop/xa1e-bv3u02r
  (29 mm sibling of the same series). Release angle (30 deg) is not published -- declared.
- Colours: separate START actuator green, RUN pilot lamp green; labels distinguish their functions.
- Grinder-polisher: generic single-platen 200 mm (8 in) metallographic grinder-polisher, 50-600 rpm class (size class
  of common bench units; dimensions declared, unverified against one model).

## START-semantics correction (supersedes the prior RESET limitation)

The independent actuator now has a green cap and START legend in the authored USD, fallback geometry and task contract. Twist-release resets only the e-stop device. A fresh START button edge with the head physically returned is required to run; holding START before/during release cannot itself restart the machine. Existing `RESET_*`, `reset` joint and contact-history identifiers are retained only for compatibility. No separate safety-relay reset is modeled and no safety-rated circuit is claimed. The corresponding same-version scripted video has been reviewed.

The original measured-output fault controls remain unchanged. A physical release-only demo omits the START press and must remain stopped with only 0.65 partial credit. Unit controls also test START held across release and fresh START only after release.

Primary safety guidance: https://www.pilz.com/en-GB/support/faq/standards/articles/180045 distinguishes resetting the emergency-stop device from the subsequent deliberate restart.

### Decal typeface

Printed legends were rendered with DejaVu Sans Bold. The font is based on Bitstream Vera under its permissive license, with DejaVu changes in the public domain. Font files are not redistributed; the applicable installed copyright/license notice is included in assets/TYPEFACE-LICENSE.txt.
