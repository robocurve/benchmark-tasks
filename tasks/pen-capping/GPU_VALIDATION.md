# Current original-asset validation

The current scripted demo scores **1.0** at **4,910 actions** in 600.33 seconds
wall time (245.5 simulated seconds). All four capped pens are released in the
correct marked areas and retain full credit through the final hold. The four
measured engagement events occur at 39.35, 100.05, 158.70 and 220.05 seconds.
There are no retention losses or trial errors. The maximum recorded joint
position residual is 0.0277 mm; this is a numerical constraint residual, not a
physical holding-force measurement.

The matching Dream-1 beta run scores **0.0** at **6,000 actions** in 737.58 seconds
wall time (300 simulated seconds), without trial errors. It manipulates loose
caps and an uncapped pen but establishes no cap retention or capped placement.

Both runs use this exact frozen runtime revision:
`ee5269d7fc4721b5eaae64e4656608f4d8d1ce1748182df31957f008cc03f902`.
Their complete frozen source manifests match, and the working runtime files
match those tested inputs. Seed 0, no jitter, 20 Hz control, 240 Hz physics,
CPU PhysX, three 640 × 360 cameras and a common 6,000-action budget are retained.
Headless renderer scheduling is explicitly synchronous in this task process.
Runtime versions, source commits and per-file hashes are in `validation.json`.

The GPU geometry/reset audit passed: four pen colliders, 49 cap colliders,
128 visible noncolliding liner pieces per cap, actual 128/16 solver settings,
joint/insertion/contact/scoring cleanup and resolved-motion-history reset.
All **81 task tests pass**, with no skips. Regeneration tests confirm original
analytic barrel geometry without an external source mesh or USD reference.
The scan-derived barrels are no longer bundled. Authored collision shapes,
masses, transforms and physics properties match the preceding lined-cap assets.

Actual stored frames produced three native 20 fps H.264/yuv420p videos per run:
4,911 frames per demo stream and 6,001 per Dream stream. Videos retain native
640 × 360 observations. The recorded calibration and rig are retained; camera
views are encoded from the same run without cropping or trajectory edits. All
files meet the media size limits.
Setup and final images come from those same recorded frames.

Visual review covered the demo top timeline every two simulated seconds through
245.5 seconds, plus both wrists at one-second intervals around all four engagement
events. Dream review covered the top timeline every five seconds through 300
seconds and both wrists in selected windows across the run. Wrist insertion
interfaces are partly occluded; the measured fit records corroborate engagement.
No identifying names, institutions or paths were visible in the sampled views.

The physical counterpart specifies PP parts, a named Shore A40 silicone and
four mechanically captured headed keys, eliminating reliance on an unspecified
PP/silicone adhesive bond. This remains an untested physical construction.
The visible noncolliding inner silicone zone and geometrically gated fixed
joint approximate compliance/retention. No physical force calibration,
contact-only retention, adhesive performance or calibrated break force is
claimed. See `PHYSICAL_REPRODUCTION.md` for dimensions and the explicitly
unperformed physical qualification procedure.

The old lined-cap/scan-visual results remain in `previous-lined-validation.json`
and `PREVIOUS_GPU_VALIDATION.md`; the older unlined-cap results remain historical.
They are not relabeled as current recordings. Current reviewed station-camera
demo and Dream videos, plus the setup image from the same recorded layout, are
uploaded as **M-CYJ5YA**, matching `task.json`. Their uploaded byte hashes
are recorded in `validation.json`. All synchronized wrist videos and native
evidence remain retained in the evidence package.
