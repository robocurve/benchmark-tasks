# Emboss Notary Certificate with Desktop Seal Press

## Overview
- **Category**: Impact on jobs
- **Workforce Relevance**: BLS SOC 23-2011 (Paralegals and Legal Assistants: 352,000 workers),
  BLS SOC 43-6012 (Legal Secretaries: 168,000 workers), BLS SOC 43-6014 (Secretaries and
  Administrative Assistants: 1,980,000 workers). Over 2.5 million legal and administrative personnel
  routinely authenticate deeds, affidavits, diplomas, and corporate certificates using desktop
  seal embosser presses.
- **Physical Task**: Bimanual document alignment and mechanical lever crimping using a heavy-duty
  desktop seal embosser press, stamping a raised official seal impression onto a certificate card.

## Hardware & Environment
- **Countertop**: Standard office desktop counter (Z=0.7475 m).
- **Desktop Seal Embosser Press**:
  - Heavy cast-iron base: 200 x 80 x 24 mm with 4 non-slip rubber bench pads at (0.280, -0.060) m.
  - Structural throat column: 120 mm height with internal depth stop rib at Y = 0.015 m.
  - Lower and upper die platens: brass circular dies (46 mm diameter) at throat opening.
  - Revolute lever handle: chrome steel arm with spherical palm knob, internal spring return
    (stiffness 0.60 N*m/rad, rest pose 35 deg, Coulomb friction 0.15 N*m).
- **Certificate Card**:
  - Stiff 300 gsm cardstock: 120 x 80 x 1.5 mm, ivory finish, gold foil notary seal target patch,
    and rear grip pinch tab, staged on desk at (0.280, 0.130) m.
- **Bimanual Operation**:
  - Left Arm: Grasps card pinch tab, slides card 140 mm into throat stop, and holds aligned.
  - Right Arm: Approaches lever knob from above, depresses handle through 35-degree stroke
    (~25 N compression force) to crimp seal, and releases for spring return.
  - Both arms retract clear to park positions for 20-step (1.0 s) stability hold.

## Scoring Stages
1. **0.20**: Certificate card grasped and advanced >40 mm toward press throat.
2. **0.40**: Certificate seated squarely against internal depth stop (<20 mm tolerance).
3. **0.65**: Lever handle depressed past halfway point (`q < 22.0 deg`).
4. **0.85**: Lever depressed to full crimp stop (`q <= 6.0 deg`) embossing seal impression.
5. **1.00**: Both hands released and retracted clear (TCPs >= 45 mm, link hulls >= 10 mm clear)
   for >= 20 consecutive simulation steps (1.0 s at 20 Hz) with assembly resting still.
- **Defect Latches (0.0)**: Press base sliding off anchor (>40 mm) or non-finite state.
