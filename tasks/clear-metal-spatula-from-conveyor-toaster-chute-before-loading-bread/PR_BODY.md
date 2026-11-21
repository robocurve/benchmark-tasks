### Task Summary: Clear Metal Spatula from Conveyor Toaster Chute Before Loading Bread

Commercial continuous conveyor toasters (e.g. Hatco Toast-Qwik, Belleco commercial toasters) are standard equipment across high-throughput commercial kitchen and breakfast line operations (BLS SOC 35-3023 / 35-2021, >4,350,000 workers). Line cooks frequently rest metal utensils (turner spatulas, tongs) across the inclined feed chute during rushes. If bread is introduced while a rigid conductive utensil is in the chute, the descending bread and continuous wire conveyor belt draw the utensil into the internal motorized drive and open 208V/240V heating elements, causing belt jams, element shorting, and serious kitchen fires.

This task models this occupational safety prerequisite and physical workflow:
1. **Safety Clearance**: Left arm grasps the stray slotted stainless steel turner spatula (165 g) from the inclined feed chute and transfers it safely to the side silicone rest tray (> 15 cm clear).
2. **Bread Loading**: Right arm grasps a sandwich bread slice (40 g) from the staging caddy and feeds it onto the conveyor toaster entry chute plate.
3. **Clearance & Verification**: Both grippers retract clear (> 50 mm) for a sustained stillness hold (>= 1.0 s).
- **Safety Defect Latch**: Feeding bread while the spatula remains on the chute triggers an immediate hazard lock, capping the episode score at 0.0.

### Validation Results
- **Scripted Policy**: Score 1.00 (verified across seeds 0 and 1)
- **Negative Control (No Clear Spatula)**: Score 0.00 (safety defect correctly triggered)
- **Negative Control (No Release / Incomplete Retraction)**: Score 0.75 (latched before 1.00)
- **Dream-1 Baseline**: Score 0.00
- **Corpus Novelty**: Verified 100% novel across all 3,280+ benchmark tasks (0 collisions, 0 precedents).
