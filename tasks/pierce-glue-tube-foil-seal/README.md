# Open a new superglue tube: unscrew the cap, flip it and pierce the foil seal with its pin

A superglue-style tube stands in a fixed bench holder. The cap is turned 1.5 turns counter-clockwise (M14 x
1.5 mm), turned over, and its crown pin is driven squarely into the foil that seals the nozzle, then withdrawn and
released, all without pressing on the soft tube body.

## Run

```sh
inspect-robots run --embodiment yam_arena -E task=pierce_glue_tube_foil_seal.py \
  --instruction "Unscrew the glue tube's cap, flip it over, and use the pin on top of the cap to pierce the foil seal on the nozzle without squeezing the tube. Leave the cap and tube released." \
  --scorer task_score
```

Offline checks (no simulator): `python3 -m unittest test_scoring.py`

Run limit 3000 steps (150 s at 20 Hz). Seeds: none; the layout is fixed.

## Layout

- `pierce_glue_tube_foil_seal.py`: scene, thread/foil joints, per-step pose feed, `score()`, `success()`, `demo()`.
- `scoring.py`: pure tracker and scoring ladder (0, 0.25, 0.5, 0.75, 1.0) with whole-episode failure latches.
- `test_scoring.py`: untouched, body squeeze, off-axis shear, foil tamper, dropped cap, axial pull, clockwise turning,
  each tier, and each full-score condition (retraction, release, 1 s stillness).
- `bodykit.py`: vendored composite-body/joint helper.

## Scoring (also in task.json)

Latched for the episode, any one zeroes the score: a finger pad or TCP within 5 mm of the soft body; a pin
engaged on the nozzle at more than 10 degrees tilt; foil displaced 2 mm without an aligned pin contact; cap
below the bench. The thread releases only after 530+ degrees of counter-clockwise turning, so pulling or clockwise
turning cannot detach the cap. Full score needs both grippers 4 cm from cap and nozzle and 1 s of rest.

## Validation

The scripted `demo()` reaches 1.0 in YAM Arena (Isaac Sim 6.1, 20 Hz, seed 0, no jitter) in one episode: four
counter-clockwise wrist strokes free the cap (about 530 degrees of turning), it is tipped onto its side and set down,
re-gripped from above and tilted pin-down, the pin is driven through the foil and withdrawn, and the cap is laid on
its side on the bench with the gripper clear. The foil's 3 N break force, the thread release and the squeeze margin
are uncalibrated estimates; a real tube should be used to measure them.

## Real trial

The simulator's glue-style tube is reproduced with a cheap, sealed, inert stand-in. No glue is used on the rig.

- **Tube (one fresh per trial):** an empty 30 ml collapsible aluminium squeeze tube with a blind nozzle (closed by an
  aluminium membrane) and its own screw cap carrying a piercing pin. These are sold in packs as "empty aluminium
  toothpaste / paint tubes with needle cap", open at the tail for filling: for example the Natural Pigments
  fez-cap collapsible tubes (<https://www.naturalpigments.com/collapsible-aluminum-tube.html>), multi-packs of
  5 ml to 100 ml needle-cap tubes on eBay (e.g. <https://www.ebay.com/itm/225493165655>, 200 x 5 ml for about
  US$42), and 30 ml five-packs (ELYSAID). Use the 30 ml size. Cost is well under US$1 per trial. Sizes (body diameter, cap size,
  thread, membrane tear force) vary by seller, so every pack is checked once before use (pack check below).
- **Pack check (once per pack, about 10 minutes, on one spare tube that is then discarded):**
  1. Measure with calipers: cap outer diameter and height, pin length, neck diameter, body diameter and length
     (typically about 22-28 mm and 100-130 mm for 30 ml), and the
     membrane's depth below the nozzle tip (depth rod).
  2. Unscrew the cap and count the turns until it comes free, using the paint line.
  3. Stand the spare tube nozzle-up on a kitchen scale (1 g resolution) and tare it. Push the cap's pin slowly and
     straight down into the membrane by hand, filming the display with a phone. The peak reading at the moment the
     membrane gives is the piercing force (grams x 0.0098 = N).
  4. Use the pack only if the piercing force is 2 to 5 N (about 200 to 510 g), which brackets the simulator's 3 N.
     The cap must also need 1.5 to 2.5 turns to come free, be 12 to 22 mm across, and have a pin longer than 2 mm plus the membrane depth.
     Otherwise use another pack or seller.
  5. Record all values with the pack's lot or order number in the trial log.
- **Dry fill:** about 25 ml of dry uncooked rice poured into the open tail, which is then folded three times with flat
  pliers. The fill keeps the thin body from buckling under the push and makes it feel like a full tube. Rice grains
  cannot pass a pin hole, so nothing wet, greasy or loose reaches the rig.
- **Holder:** a 70 x 50 x 20 mm block with a 2 mm slot that clamps the folded tail with two screws, so the tube
  stands nozzle-up and cannot turn with the cap. The body is never clamped or touched.
- **Fresh seal:** every tube arrives with its own intact membrane, so a new seal is just a new tube. Preparing one
  takes about 2 minutes and can be done in a batch before the session.
- **Reference marks (paint marker):** a straight line across the cap skirt and the shoulder, so turns can be
  counted; a ring around the pin at 2 mm plus the measured membrane depth from its tip, so the ring is level with
  the nozzle tip exactly when the pin tip is 2 mm past the membrane.
- **Cameras:** two phones on fixed stands 40 cm from the nozzle, one in front and one at the side, both level with
  the nozzle tip, each with a 1 mm grid card standing 60 mm behind the nozzle axis, outside the gripper sweep. At these
  distances parallax scales the grid by about 13%, under 0.2 mm at the 1.5 mm threshold. Lighting and positions stay fixed for the
  session.

How each scored item is checked on a real trial (same thresholds as `scoring.py`):

| Simulator check | Real-trial check |
|---|---|
| 1.5 turns counter-clockwise (530+ degrees) | the paint line on the cap passes the shoulder line at least 1.5 times in the video |
| cap lifted 25 mm | cap rim 25 mm above the neck on the grid card |
| pin within 5 degrees of vertical, tip within 1.5 mm of the nozzle axis | in both camera views, at the first frame the pin touches the nozzle, the pin lies along the grid's vertical line through the nozzle axis, within 1.5 mm (1.5 grid squares) and 5 degrees |
| tip 2 mm past the seal face, seal broken | the pin ring (2 mm past the membrane) reaches the nozzle tip in the side view, and a post-trial loupe photo (10x, from above) shows a hole in the membrane inside the nozzle bore (an outcome check that does not depend on reading the grid) |
| pin driven at more than 10 degrees tilt (fail) | pin more than 10 degrees off the vertical grid line while touching the nozzle |
| seal breached without the pin (fail) | a hole in the membrane but no frame showing the pin entering it |
| gripper within 5 mm of the soft body (fail) | any gripper contact with the tube body in either view, or a new dent in the aluminium body in before/after photos from the fixed stands |
| withdrawn 30 mm, both TCPs 4 cm clear, 1 s still | read from the grid card and the video timestamps |
| cap dropped off the bench (fail) | cap on the floor |

**Shared-rig hygiene:** the grippers only hold the cap. There is no glue, liquid or grease anywhere on the station,
and the dry rice fill cannot leave the tube through a pin hole. Check the pads in the camera view before each
trial.

**Sim-to-real differences (not re-run):** the simulator's body (42 x 14 x 80 mm flat block), neck (14 mm, 1.5 mm
pitch), cap (18 x 22 mm, 3 mm pin), 9 mm foil disc and 3 N tear force are estimates for a large gel-glue tube. They
have not been matched to a purchased needle-cap tube. Matching them would mean changing `BODY_HALF`, `BODY_Z`,
`NECK_R`, `PITCH`, the cap and foil shapes and the foil break force in the task file, and re-recording the runs. The
scored thresholds would stay as they are, except that the 1.5-turn release must be checked against the real
thread.

## Response to review

**TK-M6HR4B (2026-11-20), not accepted.** The feedback and what changed:

- *Scoring:* "The specified 2 mm penetration and 1.5 mm pin-to-axis checks are hidden inside the nozzle and lack a
  practical real-trial measurement method." Each scored item now has a visible real-trial check (table above):
  a paint ring on the pin set 2 mm past the measured membrane depth, two fixed camera views against a 1 mm grid
  card (alignment judged at first contact), and a post-trial membrane photo.
- *Clarity and reproducibility:* "The unspecified glue-style tube and replacement foil lack a repeatable source or
  preparation procedure for the threaded cap, seal, and piercing force." The real station now uses commercially
  sold 30 ml membrane-sealed aluminium tubes with their own needle caps, one new tube per trial, prepared in
  about 2 minutes and held by the folded tail only. The piercing force is controlled by a once-per-pack kitchen-scale check (2 to 5 N accepted), which also
  checks the cap's turns to release (1.5 to 2.5), cap size, pin length and membrane depth.
- *Feasibility:* "The described glue can contaminate the shared grippers, and no inert substitute or pad protection
  is specified." There is no glue, liquid or grease: the tube is filled with dry rice, which cannot leave through
  a pin hole, and the pads are checked between trials.

`setup`, `scoring`, `reset_notes` and `fidelity` in `task.json` carry the same protocol. The instruction, simulation,
scorer and recorded media are unchanged.

## Assets and licence

All geometry is procedural primitives built in this folder (`bodykit.py` is our own helper code). No third-party meshes, textures, scans or other assets are used.
