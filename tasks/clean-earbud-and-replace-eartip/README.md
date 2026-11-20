# Clean an earbud and replace its eartip

This is the first build of the investigated Samsung Galaxy Buds+ service task. It intentionally covers the full
procedure: remove and discard the old tip, use the dry brush on the speaker mesh, use the cotton swab on the charging
contacts, seat a fresh tip, and release the work.

## What is implemented

- A 6.3 g earbud with a 22.5 x 17.5 x 19.2 mm compound-primitive envelope, held still in a low silicone service cradle.
- Old and replacement silicone tips with a provisional compliant capture/seat mechanism.
- Separate dry brush and reusable electronics swab. The brush rests on a tray ledge under a light clip that lets go once it is gripped; the fresh tip stands on a presentation peg.
- Cleaning credit that requires the correct tool, surface contact, and accumulated tangential travel. Proximity and
  wrong-tool contact do not count. Individual brown inspection specks disappear as stroke quanta are completed.
- Live disposal, replacement-seat, release, and rest checks; irreversible cleaning history; an offline-tested score
  policy in `scoring.py`.

Run the simulator task from this directory with the repository's normal YAM Arena environment:

```sh
inspect-robots run --embodiment yam_arena -E task=clean_earbud_and_replace_eartip.py \
  --instruction "Remove the worn eartip and put it in the discard cup. Dry-brush the speaker mesh, clean the charging contacts with the cotton swab, then fit the new eartip. Leave the earbud and tools released." \
  --scorer task_score
```

Run the simulator-independent checks with:

```sh
python3 -m unittest test_scoring.py
```

## Validation

The scripted `demo()` reaches 1.0 in YAM Arena (Isaac Sim 6.1, 20 Hz, seed 0, no jitter) in one uninterrupted
episode: worn tip pulled and dropped in the cup, all eight mesh specks brushed off, all six contact specks swabbed
off, fresh tip carried, caught and seated, both tools back on their marks and the arm parked clear. Dream-1 scores
0.0. Silicone resistance, the capture cone, seated gap and contact bands remain uncalibrated against a physical
earbud; a real Buds+ or dimensionally representative mock should measure them.

## Real trial: soil specks

The simulator's 14 specks (8 on the speaker mesh, 3 on each charging contact; about 1 mm dots on the real earbud, 1.1 mm in the simulator) are reproduced on a
real earbud as follows. Preparation takes about two minutes per trial.

- **Material:** blue chalk-line marking chalk, a dry calcium-carbonate powder sold as a refill (for example IRWIN
  Strait-Line 64902). It is dry, contains no oil or liquid, shows on both the black mesh and the gold contacts, and
  comes off with a dry brush or cotton tip. The simulator draws the specks brown; colour is not scored.
- **Stencils:** cut from 0.1 mm laser-printer transparency film with a 1 mm craft hole punch (or drilled with a 1 mm
  pin vice).
  - Mesh stencil: a 7.4 mm disc (the nozzle face), with eight 1 mm holes at 45 degree spacing on a 5 mm circle, and a
    handling tab.
  - Contact stencil: a 14 x 12 mm card with three 1 mm holes per pad, at (-1.2, 0), (+0.8, -1.0) and (+0.8, +1.0) mm
    from each pad centre (x toward the nozzle, y across the shell). These are the same offsets as `CONTACT_MARK_LOCAL`
    in the task file.
- **Apply:**
  1. Wipe the mesh and contacts clean with a dry microfibre cloth.
  2. Lay each stencil on its face.
  3. Dab once through it with a cosmetic sponge pressed into the chalk.
  4. Lift the stencil straight off.
  5. Re-dab any hole that left no dot.
- **Inspect and count:** photograph the nozzle face (10 cm away) and the top contacts (from above) from a fixed phone
  stand under the same ring light, before and after the trial. A speck is cleared when, with its stencil laid back on,
  its hole shows no chalk in the photo or through a 10x loupe.
  - Thresholds are at least 7 of 8 mesh specks and at least 5 of 6 contact specks (80% rounded up, as in
    `scoring.py`).
  - Specks only count if the trial video shows the brush doing the mesh and the swab doing the contacts, matching
    the simulator's correct-tool rule.
- **Shared-rig hygiene:** the grippers only hold the tool handles, eartips and earbud shell. The chalk stays on the
  bristles and cotton tips. Between trials, tap and wipe the tips and wipe the tool handles with a dry cloth so no
  loose powder reaches the gripper pads.
- **Brush ledge clip:** any light plastic tool clip, for example a 19 mm nylon pegboard spring clip, holds the brush on
  the tray ledge. It can be left out on a real station, with the brush simply resting on the ledge.

## Response to review

**TK-C7G68W (2026-11-20), Clarity and reproducibility:** "no material, application method or repeatable placement
for the soil specks". The section above now specifies:
- a safe dry material;
- a two-minute stencil method that places all 14 specks at the simulator's positions;
- before/after photographs from a fixed stand, counted against the 7 of 8 and 5 of 6 thresholds.

The same protocol is summarised in `setup`, `scoring` and `reset_notes` in `task.json`. The simulation, scorer and
recorded media are unchanged.

## Source and audit notes

- Samsung's official cleaning and eartip procedure: <https://www.samsung.com/us/support/answer/ANS10001320/>
- The fresh board audit found no exact task among 719 entries. Closest accepted mechanics were O-ring seating (#1576),
  electric-toothbrush head replacement (#1621), fibre-connector cleaning (#1925), and scrub-the-hub (#1013).
- The charging case is excluded to stay distinct from pending AirPods case-packing PR #2784 and rejected earbud-case
  snap task #1247.
