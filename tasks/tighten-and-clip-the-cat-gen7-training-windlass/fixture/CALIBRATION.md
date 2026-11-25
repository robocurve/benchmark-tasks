# Product readings and further calibration

Current input figures were measured on a C-A-T Gen 7 trainer with a scale by a technician who works with this product, Nov 2026. See `../evidence/TECHNICIAN-READINGS-NOV23.md`. The executable's `technician_phone` mode uses these measurements; the optional `measured` mode below accepts a larger repeat-data file, and its template stays empty.

| Named input | Current value | Provenance |
|---|---:|---|
| WINDING_LEVER_M | 0.065 m | Reported product reading |
| WINDING_FORCE_AT_LOCK_N | 21.57463 N | 2.2 kg-force conversion |
| WINDING_LOCK_DEG | 270 degrees | Reported 3/4 turn |
| CLIP_INSERT_PEAK_N | 7.84532 N | Reported separate 0.8 kg push |
| CLIP_EXTRACT_PEAK_N | 7.84532 N | Reported separate 0.8 kg pull |
| CLIP_MOUTH_M | 0.010 m | Reported product dimension |
| ROD_DIAMETER_M | 0.012 m | Reported product dimension |
| CLIP_HOOK_AZIMUTH_DEG | 156, 204 degrees | Independent geometry estimate |
| ROD_TILT_STIFFNESS_NM_RAD | 0.10 N m/rad | Band model estimate |

The spring interpolates the measured winding endpoint; the pressure mapping is an explicit proxy. Native contact probes must compare insertion and extraction with the measured peaks. They do not measure the physical trainer. The following procedure specifies the optional repeat-data calibration.

Use a NAR blue C-A-T Gen 7 30-0033 on the exact 127 mm diameter training cylinder, with the enrolled band, bladder and chocks. Record the specimen/lot, date, instrument model, resolution and uncertainty, the lever distance, starting band position, cylinder circumference, gauge readings and a side photograph showing the attached factory clip. Keep people and personal details out of reusable evidence.

1. Check the force instrument against a known mass; record the result. Zero it in its measurement orientation. Check the gauge at zero and at known pressure points with a reference manometer.
2. Attach a nonstretch loop at a measured 65 mm lever from the winding axis. Pull tangentially, level with the rod. At 90, 180 and 270 degrees from the start, measure both the slow tightening peak and the static holding force in three independent resets. Record raw newtons, lever metres and pressure; torque is force times perpendicular lever. Do not average tightening resistance down with the lower holding force.
3. Measure pressure at 0, 45, 90, 180, 240 and 270 degrees. Repeat in the opposite winding direction if the task allows both directions. Record the actual curve; do not adjust physical readings to force a linear curve. Revise the task's allowed winding direction or model if the curves differ.
4. Measure insertion and extraction force directly over each factory clip hook using a push/pull force gauge at slow speed, three repeats per hook, at both an early lock and the green-range lock. Subtract the rod's no-clip support force at the same point. Record the full force/displacement curve with a ruler and video, not only a peak. A kitchen scale under the full rig is an alternative only if other applied loads are separately measured and removed.
5. Include raw CSV or a de-identified readable scan in evidence. Hash that file and fill the JSON template with observed values. Copy observed upper peaks, lever and lock angle into the named constants at the top of task.py and mirror them in the JSON constants object. Provide uncertainty, hook-labelled snap rows with travel_m, and at least three repeats per winding angle and per hook. The guard validates provenance, hashes, repeats, constant agreement and pressure enrollment; the spring/caps, clip fit and snap gate derive from those constants. The measured pressure curve is interpolated. The linear winding spring must fit all observed upper torques within 10%; otherwise the loader rejects it and a nonlinear mechanical fit is needed. Tilt remains an explicit independent estimate until measured. Revalidate native insertion AND extraction rather than treating the analytic fit as confirmation. Confirm instrument uncertainty fits the simulated tolerances. No hardware reading has been obtained by running a simulator.
6. Tune both insertion and extraction curves in the contact probe. A single assumed lip stiffness is not a force calibration. Verify reaction on the holding hand and clip supports, clip retention, bimanual handoff and the 1 s release hold in Isaac at the matched loads. Describe the model as hardware-calibrated only after those checks and independent review. The current version uses the measured endpoint figures; the repeat-data procedure above is an optional extension.

No human application or medical efficacy is evaluated. The gauge target is a trainer indicator, not a treatment threshold.

Assessor retention confirmation: after scoring the released hold, pull upward along the clip normal at the clipped rod end with 2 N for 1 s and record any extraction or loss of seating. The fixed 2 N protocol is approximately a 200 g load and about one quarter of the measured 7.84532 N extraction peak; it is a gentle end-state check. The scored robot instruction and native force gates are unchanged.


