# Fold a quad-fold game board up in its one working order and lay it in its box on the insert

Instruction: `Fold the game board and put it in its box.`

A games table after play. An open 50 x 50 cm quad-fold game board lies face up on a 16 in (40.6 cm) bamboo board-game
turntable (lazy susan, commonly placed under large tabletop games so every player can turn and reach the board). The
game's rigid box base, fitted with its flat card insert, stands to the right. Putting a board game away ends with
folding the board in its one working order and laying the folded 25 x 25 cm stack onto the insert inside the box.

Unlike generic household object storage (such as BEHAVIOR-1K `putting_away_games`, which moves whole game boxes, or
deformable garment folding), a quad-fold game board is an articulated rigid-panel mechanism whose one-way paper-wrap
hinges and half-slit enforce a strict causal fold order on a freely spinning support, followed by a quarter-turn
placement into a box cavity with 7.5 mm side clearance.

## Why the fold order is forced

The board has four 25.0 x 25.0 cm rigid quarters: near-left (`q_nl`), far-left (`q_fl`), near-right (`q_nr`) and
far-right (`q_fr`), where near (`-x`) is toward the robot. Each hinge is formed by the continuous dark back-wrap paper,
while the printed face paper is cut through along the crease:
- Bent **face-in** (`0` to `180 deg`), the back wrap folds smoothly around the outside of the two plates.
- Bent **face-out** (past `0 deg` the other way), the cut greyboard and face-paper edges butt directly against each
  other and the back wrap goes into tension, stopping the hinge flat (forcing it past `20 deg` tears the wrap).

The two creases are asymmetric:
- **Long crease** (along `y` at `x = 30.0 cm`): **two hinges**, `near-left / far-left` and `near-right / far-right`.
  Each far quarter folds toward the robot (`-x`) face to face onto its near quarter.
- **Short crease** (along `x` at `y = 2.0 cm`): **one hinge** between `near-left` and `near-right`, and a **2.5 mm
  through-slit** separating `far-left` from `far-right`.

Because of that slit and the three face-in hinges, the board closes flat in **one order only**:
1. Fold **both far quarters** (`far-left` and `far-right`) over onto their near quarters (`180 deg` face to face).
2. Turn the **stacked left half** (`near-left + far-left`) over onto the stacked right half (`near-right + far-right`),
   closing the short crease (`180 deg`) into a four-layer `25.0 x 25.0 x 0.88 cm` stack.

If the short crease is folded first (`near-left` turned onto `near-right` while `far-left` is still open), `far-left`
ends up face down on top of `far-right`, and folding it toward the robot would bend its hinge **face-out** against the
butt stop, jamming the board half-open (`50 x 25 cm`, too wide for the `26.5 cm` box).

## What makes it physically demanding

- **Overhanging-edge grasps on a flat board:** the quarters lie flat (`2.2 mm` thick), so the grippers can only pinch
  the `5–15 cm` of board edge overhanging the `40.6 cm` round turntable. Because the `50 cm` span straddles the table,
  only the left arm reaches the far-left edge and only the right arm reaches the far-right edge and the box.
- **Unpinned board on a free-spinning lazy susan:** the board rests unfastened on the bamboo deck, which spins on its
  ball bearing if pulled tangentially. Every lift must follow the live hinge arc rather than dragging sideways.
- **Bimanual half-turn handoff:** the left arm raises the two-layer left half (`180 g`) to vertical (`~88 deg`) and holds
  its upper edge steady while the right arm grasps its near vertical edge (`> 18 cm` wrist clearance), then releases so
  the right arm lays it cleanly onto the right half.
- **Quarter-turn box packing:** the right arm pinches the four-layer stack, lifts it clear, rotates it `90 deg` in yaw
  to clear the box walls and wrist limits, and seats it flat on the insert inside the `26.5 x 26.5 cm` box cavity
  (`7.5 mm` clearance per side, `1.2 cm` below the rim).

## Scene and reproducible hardware

Origin: tabletop directly below the midpoint between the two YAM arm bases (`x` forward, `y` left, `z` up above the
`110 x 130 cm` station table; left base at `(0.0, +30.5 cm)`, right base at `(0.0, -30.5 cm)`, both starting in `READY`
with jaws open).

| Object | Type | Dimensions | Material / commercial part or build recipe | Start pose (cm) |
|---|---|---|---|---|
| Turntable | rotating deck on fixed bearing base | `40.6 cm` (`16 in`) dia, `2.0 cm` bamboo deck on `1.2 cm` bearing base (`3.2 cm` total height); deck `1.20 kg` | **Lipper International 8826 16-in Bamboo Turntable** (`16 in dia x 1.25 in H`, steel ball-bearing ring) fixed to the tabletop with double-sided tape under its base | centred at `(30.0, 2.0)`, flat on table |
| Quad-fold board | four hinged free quarters | `50.0 x 50.0 cm` open; four `25.0 x 25.0 x 0.22 cm` quarters (`90 g` each, `360 g` total); `0.6 mm` long-crease seam, `2.5 mm` far-quarter slit | **Apostrophe Games / BoardGamesMaker 20 x 20 in (`50 x 50 cm`) Blank Quad-Fold Game Board**, or built from four `25.0 x 25.0 cm` panels of `2.2 mm` (`1400 gsm`) greyboard hinged on the underside with `4.0 cm`-wide dark `130 gsm` linen bookbinding paper strips across the two long hinges and near short hinge (`2.5 mm` gap left unbridged between far quarters) and faced per quarter with `150 gsm` coloured matte card (green `nl`, blue `fl`, sand `nr`, coral `fr`, ivory outer border track) | flat (`0 deg`), face up, centred at `(30.0, 2.0)` on turntable (`z = 3.2 cm`); long crease along `y` at `x = 30.0`, short crease along `x` at `y = 2.0` |
| Box base + insert | free rigid open-top box | `27.5 x 27.5 x 5.5 cm` outer, `5.0 mm` walls, `3.0 mm` floor (`26.5 x 26.5 cm` inside); card insert `26.4 x 26.4 x 4.0 cm` (`top 4.3 cm` above table, `1.2 cm` below rim); `250 g` combined | Square rigid set-up game box base (`2.5 mm` greyboard wrapped in red paper, double-turned `5 mm` walls; e.g. `10.5 x 10.5 x 2.2 in` quad-fold game box base) with a `350 gsm` ivory folded card platform insert (`26.4 x 26.4 x 4.0 cm`) glued flat to the interior floor | centred at `(30.0, -44.0)`, resting flat on table, walls square to `x` and `y` |

**Reset procedure:** return both arms to `READY`; align the turntable base to a tape circle at `(30.0, 2.0)` and the
red box base to corner tape marks at `(30.0, -44.0)`; unfold the board flat (`0 deg`) face up on the turntable with its
long crease over `x = 30.0 cm` and its `2.5 mm` cut slit pointing forward (`+x`) along `y = 2.0 cm`.

## Scoring

Evaluated at the end of the trial, with whole-trial disqualifiers latched across every step. Tiers are **mutually
exclusive** (the trial receives the **highest tier met**, not a sum):

- **Score `0.0`** if at any time during the trial:
  - any board quarter, the turntable deck, or the box base falls off the table (centre more than `2.0 cm` below the
    tabletop);
  - any of the three hinges bends face-out past `20.0 deg` (`fold angle < -20 deg`, tearing the wrap);
  - or if at the end neither far quarter is closed within `10.0 deg`.
  *(An undamaged premature fold that is reopened and refolded in the working order is not penalized; only physical
  hinge damage `> 20.0 deg` face-out or an off-table drop zeroes the trial.)*
- **Score `0.15`**: one far quarter closed onto its near quarter (`hinge >= 170.0 deg`, i.e. within `10.0 deg` of `180 deg`).
- **Score `0.30`**: both far quarters closed onto their near quarters (`both long-crease hinges >= 170.0 deg`).
- **Score `0.50`**: all three hinges closed within `10.0 deg` (`long_left >= 170 deg`, `long_right >= 170 deg`,
  `short >= 170 deg`, forming a `25 x 25 cm` stack), when the stack is not resting released inside the upright box.
- **Score `0.75`**: all three hinges closed (`>= 170.0 deg`), the box base resting upright on the table (within
  `10.0 deg` of level, floor within `5.0 mm` of tabletop), every quarter flat (within `10.0 deg` of horizontal) with all
  corners inside the `26.5 x 26.5 cm` box interior (`1.5 mm` tolerance) and resting on the insert (lowest corner between
  `-3.0 mm` and `+5.0 mm` from the insert top and highest corner no more than `2.0 cm` above the insert top), and no
  gripper point (`TCP`, pad tips, or finger tips of either arm) within `1.0 cm` of any quarter or the box base (and not
  held).
- **Score `1.0`**: all `0.75` requirements met, **and** over the final `0.5 s` (`10` control steps at `20 Hz`) every
  gripper point (`TCP`, pad tips, and finger tips of both arms) remained at least `10.0 cm` from every board quarter
  and from the box base's `27.5 x 27.5 x 5.5 cm` outer bounds while moving slower than `2.0 cm/s`.

**Real-trial measurement:** check no off-table drop, no `> 20 deg` backward hinge bend, and `0.5 s` end
stillness (`< 1 cm` motion in the final `0.5 s`) from the overhead and side trial video; check closed stack flatness
(`<= 10 deg`), full containment inside the `26.5 cm` box walls below the `5.5 cm` rim (no edge propped on a wall), and
`>= 10 cm` finger/gripper clearance from the overhead end photo and a ruler/protractor.

## Run a trial

Requirements: `yam-isaac-lab-arena` and Isaac Lab-Arena `0.3.1`.

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1500 \
    --instruction "Fold the game board and put it in its box."
```

Runtime limit: `1,500` steps (`75 s` at `20 Hz`). Nominal layout is fixed (`object_jitter=0`).

## Simulation fidelity

- **Rigid bodies and hinges (`240 Hz` PhysX, `20 Hz` control):** each `25.0 x 25.0 x 0.22 cm` greyboard quarter (`90 g`)
  is a rigid cuboid. The three paper-wrap hinges are revolute joints placed on the mating top surfaces (`z + T` for the
  long crease so quarters fold flat onto `z + T`; `z + 2T` for the short crease so the two-layer left half folds flat
  onto the two-layer right half), each spanning `-2 deg` to `180 deg` (`-2 deg` hard stop models the butting cut face
  edges) with `0.003 N m s/deg` viscous hinge damping (`0.3 N m` torque cap). Collision between the two far quarters is
  filtered across their `2.5 mm` slit so their edges pass cleanly without rigid-corner snagging.
- **Lazy susan bearing:** the `1.20 kg` bamboo deck turns on a vertical revolute joint (`0.004 N m s/deg` viscous
  bearing drag, `1.0 N m` cap) over the fixed `24 x 24 x 1.2 cm` base.
- **Box and insert:** rigid `27.5 x 27.5 x 5.5 cm` box (`5 mm` walls, `3 mm` floor, `250 g`) with a solid flat insert
  block (`26.4 x 26.4 x 4.0 cm`, top at `4.3 cm`); recessed piece wells beneath the board are omitted because a flat
  card insert or recessed tray presents the same perimeter support ledge to the `25 x 25 cm` board.
- **Bounded contact-pinch weld (sim hack):** thin `2.2 mm` card pinched by rigid parallel pads can slip under hinge
  torque in rigid-body contact, so after an arm's jaws have opened (`>= 28 mm`), squeezing an outer edge engages a
  peel-bounded joint **only to the 1 or 2 outer quarters directly contacted** (`tip_left >= 0.5 N` on one outer face
  and `tip_right >= 0.5 N` on the opposite outer face, pad segment `1.0–38.0 mm` inside the `25.0 x 25.0 cm` outline,
  stalled separation `0.5 mm` to `w + 20.0 mm`), releasing when the jaws open or if an external jam stalls the `7 N m`
  wrist past `4.5 cm` TCP lag for `0.15 s`. Uncontacted inner layers (`far-left` and `far-right` inside the 4-layer
  stack) are never welded; they remain retained physically between the two pinched outer covers (`near-left` and
  `near-right`) and their hinges, just as when rubber gripper pads clamp the outside of a real `8.8 mm` folded stack.

## Demo video and shortcut checks

`demo/` contains a scripted two-arm demonstration (planned from live object poses without teleporting or modifying sim
state) reaching `1.0` in `837` steps (`~42 s`):
1. Both arms simultaneously pinch the overhanging side edges of `far-left` and `far-right`, arc them past vertical
   (`110 deg`) about the long crease, and release so both quarters lay flat (`180 deg`) onto their near quarters.
2. The left arm pinches the outer edge of the stacked left half, lifts it to vertical (`~88 deg`), and holds it steady
   while the right arm grasps its near (`-x`) vertical edge (`~18.5 cm` wrist-housing clearance); the left arm then
   releases and clears while the right arm lays the half past `136 deg` onto the right half and releases so all three
   hinges settle closed (`180 deg`).
3. The right arm pinches the four-layer `25 x 25 cm` stack, lifts it `10 cm`, turns `90 deg` in yaw, seats it flat on
   the box insert, releases, and both arms park `> 13 cm` clear with the right wrist camera viewing the packed board.

Scripted shortcut and partial policies checked with `score()`:

| Policy | Behaviour | Score |
|---|---|---|
| Idle | Both arms stay still | `0.0` (no quarters folded) |
| Wrong fold order | Turns the left half onto the right half before closing the far quarters, then tries to fold far quarters and box | `0.0` (far-quarter hinges stay open against the butt stop and turntable; `50 x 25 cm` half-open board cannot close or fit in box) |
| One arm only | Left arm folds `far-left` closed and tries to flip the left half alone without the right arm | `0.15` (`far-right` and `short` stay open, box unreachable) |
| Folded on turntable only | Folds all three hinges closed on the lazy susan and parks without boxing | `0.50` (stack not in box) |
| Held in box | Folds all three hinges and lowers stack into box, keeping jaws shut on the board | `0.50` (still held) |
| Unparked after release | Releases folded stack flat on the box insert, hovering `< 10 cm` above the box | `0.75` (arms not parked `>= 10 cm` clear) |
| Full bimanual method | Folds both far quarters, hands off the stacked half, packs into box, parks clear | `1.0` |
