> **Incomplete feedback draft.** No full-score demo or completed model trial is claimed. See DRAFT_STATUS.md.

# Flip, cut the bottom tape, unfold and flatten a used carton

Development prototype: no full-score demonstration or genuine model result exists. The carton starts upright with its top flaps folded back and its bottom closed with tape. The intended robot sequence is turnover, cutting the bottom seam and both taped end folds, parking the guarded cutter, opening every bottom flap outward, then collapsing the intact wall tube flat. No handles or pre-cut tape are added.

## Physical setup

Empty 220 × 180 × 180 mm carton, 3 mm board: four 35 g walls and eight 18 g flaps, joined by native crease hinges. Packing tape is 48 mm wide with 40 mm end-wall tails. The ordinary guarded cutter is 55 g and starts on its pad. Exact poses, colors and shapes are in config.py, scene.py and native.py. The rigid-panel/hinge model approximates cardboard; it is not continuum board or tape-film simulation.

Bottom hinges now permit only outward opening, from 0 to 274.9 degrees. The old negative-angle range allowed flaps to fall into the empty box; it is removed. Sealed outer flaps remain restrained until all 158 sampled seam/end-fold cells have been cut by a valid held blade sweep. Continuous 240 Hz relative-pose tracking prevents inward motion or a wrapped negative angle from counting as opening. Missing native samples and substantial inward motion fail closed.

## Intended score

A revised full run must start upright and establish a measured inverted hold for 0.5 seconds before cutting. Initial bottom-up diagnostic runs do not establish this requirement.

- 0.25: complete valid cutting of all three paths after turnover.
- 0.5: physically open all four bottom flaps outward to at least 150 degrees.
- 1.0: subsequently leave all eight flaps approximately coplanar with their walls (within 15 degrees of 90/270 degrees), with every board vertex inside a 24 mm horizontal slab; lowest height -2 to +8 mm, highest at most 30 mm above the table, carton speed below 10 mm/s, cutter parked and hands clear for one second.

Crease separation above 4 mm, forced sealed outer flaps above 12 degrees, drops below the table by more than 30 mm, bad continuous-angle history or invalid blade motion invalidate the run. Every score input comes from actual native state; controller phases do not earn credit.

## Actual evidence and missing work

The revised unsealed bottom-up gravity check passed 180 controls/9 seconds: all bottom angles remained between -0.0073 and -0.0012 degrees, crease separation below 3 micrometres. This validates the outward stop at closed, not turnover, opening or collapse.

Historical sealed cut011 completed all 158 cut cells and parked the cutter at a 0.25 cutting milestone under the previous bottom-up setup. It is not a completed demo of this revised task. The two-stage physical turnover prototype fails local reach/mesh screening (9/16 poses; a taller-box hypothesis 8/16). No turnover GPU run was authorized. Reliable opening and complete collapse remain unproved, and the current turn-and-cut controller covers only a proposed prefix. No animation is substituted for physical turnover.

## Short native diagnostic

```sh
/data/IsaacLab-Arena/.venv/bin/python run.py --mode setup-check --unsealed --bottom-up-start --no-video --out /path/to/new-outward-stop-check
```

This deliberately unsealed, bottom-up check has no task score. Camera-free outputs preserve the composed scene and actual native actor poses for labelled offline previews, not native RGB. Legacy cutting requires explicit `--bottom-up-start`; it cannot satisfy the revised full-task turnover requirement. Full feasibility, reproducible policy integration and model/media evidence remain unfinished.

## Follow-up turnover candidate (native pending)

The next physical diagnostic uses an ordinary 220×180×180mm carton at XY(250,−100)mm. Mesh-supported lateral fingertip contact permits fixed downward wrists: a low brace and high opposite push, followed by brace withdrawal at25°. The controller withdraws after the measured panel COM passes the leading table-support edge, and waits for actual gravity settling before the second push. It never sets object poses or adds a turnover force. CPU sampling covers122 contact postures,82 acquisition samples and164 withdrawal samples; these are finite geometry checks, not native contact or turnover proof. The changed dimensions invalidate historical native scene equivalence.
