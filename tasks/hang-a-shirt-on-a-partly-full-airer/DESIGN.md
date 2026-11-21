# Design: hang a baby T-shirt over the free front rail of a partly full airer

Category: **Societal relatability.** Hanging washing on a clothes airer is one of the most familiar household jobs.
A clip of a robot hanging a small T-shirt on an airer, next to cloths already drying, needs no explanation.

## The real job, and what this task tests

Washing is dried on an indoor airer: a frame of parallel rails in one horizontal plane. A garment is hung over one
rail, folded in the middle, one half each side, held up by nothing but its balance. People drape it over a rail of
its own, rather than across two rails or on top of what is already there, so air reaches both faces and it dries.
They also lay it out flat rather than bunched. On a partly full airer this has to be done without pulling the
neighbouring washing off or along its rails.

The robot does exactly that with a small cotton T-shirt. Each of the following is a physical skill, and the score
checks all four:

1. **Over one rail of a row.** The free rail is the front one. The shirt's back half must go down through the 68 mm
   gap between it and the middle rail. A shirt laid on top of the airer, across two rails, or onto the middle rail's
   cloth scores low.
2. **No pegs: balance alone.** The shirt stays up only if its halves are about even (each at least 80 mm below the
   rail). Folded about 2:1 it still holds, by friction over the rail. But a fold more lopsided than that, or one
   dropped onto the rail from a height, slides off. So the fold has to be set down, not thrown.
3. **Spread.** Where it lies on the rail it must cover at least 85 % of its flat width (187 mm) of the rail: it hangs
   open, not gathered into a rope.
4. **Partly full.** The middle and back rails already carry a damp face cloth each, and the front half of the middle
   rail's cloth hangs in that same 68 mm gap. The shirt has to go down beside it without dragging either cloth more
   than 30 mm.

What this adds to the accepted "peg a T-shirt onto a clothesline" task: no pegs, a choice of one rail among parallel
rails with neighbours, balancing the fold, spreading, and leaving neighbouring washing undisturbed. Nothing here is
a spring clamp.

## Real-world protocol (reproducible)

| Item | Specification |
|---|---|
| Airer | Tabletop wooden drying rack, the top deck of a wooden clothes airer cut down to the robot's table (drawing and parts below). Three 12 mm beech dowels, 820 mm long, axes 80 mm apart (68 mm clear between them), their tops 200 mm above the table. They run left-right through two pine A-frame ends 800 mm apart (outer faces). Clear-lacquered. Both feet are G-clamped to the table. |
| Face cloths (2) | 25 x 25 cm cotton terry baby washcloths, about 400 g/m^2: pale blue on the middle rail, pale yellow on the back rail. Each is wetted, wrung by hand and folded over its rail. One half is 8 mm (blue) or 6 mm (yellow) longer than the other. |
| T-shirt | A short-sleeved 100 % cotton single-jersey baby T-shirt, size 6-12 months (EU 74-80): 220 mm across the body laid flat, 280 mm from shoulder to hem, with a rib-knit neck. Washed and spun, so damp (about 30 g). |
| Layout | Distances are forward (x) from the line between the arm bases, and sideways (y) from the midpoint between them. Rail axes at x = 320 (front, free), 400 (middle, blue cloth) and 480 mm (back, yellow cloth), centred at y = 0. The T-shirt lies flat on the table, front up, smoothed, centred at y = 0, its hem toward the arms at x = 185 mm and its neck under the airer. |

**The airer, to build** (about an hour with a drill and a saw, about US$25 of stock timber). All dimensions in mm.
Rail axes are at x = 320, 400 and 480, 194 above the table.

```
 Side view of one end frame (looking along the rails; the arms are to the left)

            x=290   320       400       480   510
              +----( o )-------( o )-------( o )----+  crossbar 220 x 30 x 18, top 209, bottom 179;
              |____________________________________ |  12 mm holes for the rails, centres 194 above the table
             /                                      \
            /  leg 18 x 18, from (250, 18)           \  leg 18 x 18, from (550, 18)
           /   to (310, 179)                          \  to (490, 179)
   +------/----------------------------------------------\------+  foot batten 340 x 45 x 18 (x = 230 to 570)
 ==+=============================================================+== table top

 Front view (looking forward from the arms)

   ||<---------------------- 800 outside the end frames --------------------->||
   ||================= rail, 12 mm dowel, 820 long (10 past each frame) ========||
   ||                                                                          ||
  _||_  end frame (18 thick)                                end frame (18)   _||_
```

| Part | Qty | Material | Size |
|---|---|---|---|
| Rail | 3 | Beech dowel, clear lacquer | 12 dia x 820 |
| Crossbar | 2 | Pine | 220 x 30 x 18 (three 12 mm holes, centres 30, 110, 190 from one end, mid-height) |
| Leg | 4 | Pine | 18 x 18, about 172 long, ends cut at 70 degrees |
| Foot batten | 2 | Pine | 340 x 45 x 18 |
| G-clamp | 2 | Steel | any, one over the far end of each foot batten |

The rails are glued into the crossbars. The legs are glued and screwed to the crossbars and foot battens, as drawn.

**Measuring a trial.** Before the trial, mark each face cloth's two lower corners on a ruler taped upright to the
nearest frame end, or photograph the airer from the side, along the rails, with a ruler in view. Wait 10 seconds
after the run ends. Then photograph from the side along the rails, and from the front.
- **Touches the table:** any part of the shirt touching it, seen in the photo.
- **Rests on a rail:** the shirt lies on top of it, or on top of the cloth over it, in the side photo.
- **Drop:** with a ruler, from the top of the shirt's rail down to the lowest point of each half.
- **Span:** with a ruler along the rail, the length of rail the shirt lies on.
- **Cloth movement:** each cloth corner's height and side-to-side position against the marks.
- **Hands:** check in the video that both are open and clear of the shirt.

**Reset.** Re-wet and re-hang the face cloths with the same offsets. Lay the shirt back flat at its marks. A fresh
damp shirt every 10 trials keeps its moisture similar.

**Safety and the rig.** The fabric is light (under 40 g). The cloths are damp, not dripping: wring them hand-dry so
no water reaches the grippers. Nothing is sharp, hot or heavy.

## Scoring

At the end of the run (simulation and real trial alike):

| Score | Condition |
|---|---|
| 0 | The shirt touches the table, is not held up by the airer, or is still in a hand; or a face cloth came off its rail or touches the table |
| 0.25 | The shirt hangs from the airer, clear of the table, and the hands have let go |
| 0.5 | It rests on one rail only, the free one, and each half hangs at least 80 mm below that rail's top |
| 0.75 | And it lies along at least 187 mm of the rail (85 % of its flat width) |
| 1.0 | And neither face cloth moved more than 30 mm at any corner, and both hands are open and at least 50 mm from the shirt |

Hanging it over a loaded rail (on top of a cloth) stays at 0.25. So does laying it across the airer. Pulling a
cloth off its rail scores 0. In simulation the same quantities are measured on the fabric's vertices (scoring.py,
tested in test_scoring.py).

## Results

| Run | Score | Notes |
|---|---|---|
| Scripted demo (run_demo.py) | 1.0 | Success at step 695 of 900, held 3 s. Halves 131 and 155 mm; lies along 204 mm of the rail; 44 mm clear of the table; cloths moved 3.3 mm; hands open and clear. A previous run of the same plan also scored 1.0 (success at step 690). |
| Dream-1 | 0.0 | 1,200 steps (the run limit). It moved both arms around and over the airer, brushing the blue cloth (it moved 12 mm), but never grasped the T-shirt, which stayed flat on the table. |

## Simulation model and its simplifications

- **Fabric.** The shirt and both cloths are PhysX surface-FEM shells (cloth) with gravity, stretch, shear, bending,
  friction and self-contact.
  - *Shirt:* two layers, front and back, sewn at the shoulders, along the sleeves and down the sides. A 5 mm mesh,
    5,738 vertices: finer than the 12 mm rails, so no rail slips between vertices. 30 g.
  - *Washcloths:* 12.5 mm mesh, 37.5 g each (damp terry), 3 mm thick.
  - *Calibration:* stiffness and friction (cotton on beech 0.5, cotton on cotton 0.6) are set by eye for a limp
    cotton drape, not measured. Check: the shirt laid over the rail about 2:1 (halves of 180 and 88 mm) stays put,
    creeping under 3 mm in 8 s, as a real one does.
- **The pinch.** PhysX cannot hold a thin FEM shell between two position-controlled pads by friction. So a pinch is
  a contact-gated attachment (pinch.py): when an arm's fingers close from open to 6 mm or less, the fabric vertices
  then between its pads are fixed to the finger, and they are released when it opens past 10 mm. The fabric must
  really be between the pads: closing on air, touching it with closed fingers, or the pads pushing it aside gives no
  hold. It is the same for every policy.
- **The airer** is one fixed body. Its feet are clamped in the real trial.
- **Assets.** Everything (the airer, the shirt's sewn pattern, the washcloths) is authored procedurally in this
  folder; no third-party meshes are used.
- **Not modelled:** moisture (the damp mass is in the fabric's density), drying, and fabric tearing.

## The demo (shirtplan.py, run_demo.py)

1. **Pinch the hem.** The hands back off from the airer, then come down over the shirt's two hem corners, fingers
   pointing down and 30 mm apart, closing front to back. They pinch each corner off the table.
2. **Lift and spread.** They lift, turning the fingers to point inward and level, like a person holding a sheet by
   its corners. They draw the hem out to its full width, so the shirt hangs spread, neck down.
3. **Carry and lower.** They carry it high enough to clear the free front rail, to above the gap behind that rail.
   They lower it, so its neck half goes down into the gap beside the blue cloth.
4. **Fold over the rail.** They draw the hem forward over the front rail, down in front of the airer and back under
   the rail, until the halves are about even and the hem side hangs straight. Then they open.
5. **Clear away.** The hands draw back and away.

The policy plays the planned joint targets, corrected for the arms' sag from the observed joint positions.
