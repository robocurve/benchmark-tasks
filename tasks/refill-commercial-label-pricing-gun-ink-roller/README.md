# Refill Commercial Label Pricing Gun Ink Roller

## Description
A commercial handheld label pricing gun (Monarch 1131 / Garvey 22-8) is clamped to a workbench fixture with its front access door latched shut over a spent ink roller. A fresh replacement ink roller stands on the staging area next to a discard waste tray. Replenishing dried ink rollers is regular, recurring operational maintenance across retail, supermarket, grocery, and logistics operations (>3,500,000 workers under BLS SOC 41-2031 Retail Salespersons and BLS SOC 53-7065 Stockers and Order Fillers).

To complete the task:
1. Left arm swings open the front ink access door past its retention detent (>= 55 deg).
2. Right arm extracts the spent ink roller spindle from the carrier cradle into the bench discard waste tray (> 8 cm from carrier and inside tray).
3. Left arm picks up the fresh cylindrical ink roller (12 mm OD x 26 mm L, 12 g), transports it to the pricing gun, and seats it squarely into the carrier journals (< 15 mm horizontal, < 12 mm vertical error).
4. Right arm swings the front ink door shut until the detent latch snaps flush (< 5 deg from closed).
5. Both arms clear (>= 50 mm); the secured assembly must remain in equilibrium for >= 1.0 s.

## Workforce & Domain Significance
Handheld pricing label guns are universal frontline tools used daily across food retail, convenience stores, and department store backrooms. BLS SOC 41-2031 (Retail Salespersons, 3.1M workers) and BLS SOC 53-7065 (Stockers and Order Fillers, 2.9M workers) regularly perform manual roller replacement whenever print becomes faint or illegible.

## Hardware & Physics Realism
- **100% Natural 1:1 Tabletop Scale**: Commercial Monarch / Garvey style price marker (180 x 120 x 45 mm, 380 g rigid ABS) clamped to bench fixture with true internal label cavity and side carrier journals.
- **Physics & Contact Mechanics**: Spent & fresh cylindrical ink rollers (12 mm OD x 26 mm L) with rigid lateral steel axle pins resting in U-channel cradle journals. Front door articulated via USD Physics RevoluteJoint (0.05 N*m/rad stiffness, 0.10 N*m friction). Zero added grasp bosses, zero proximity catch welds.
- **Reversible Multi-Stage Scoring**: 0.20 door open, 0.45 spent roller in waste tray, 0.75 fresh roller seated in journals, 0.90 door latched shut, 1.00 hands clear + continuous hold verified by `yam_arena`.
- **Offline unit tests**: 6/6 checks pass cleanly (`self_test()`).
