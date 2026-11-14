# Physical reconstruction and measurement protocol

Revision for PR #1151, 12 November 2026. This specifies a **proposed aqueous-tracer benchtop reconstruction**. It is not a report of a completed hardware experiment. The submitted videos demonstrate the original simulation; they do not validate the equipment or measurement uncertainty below.

## Nursing preparation scope

This blue-tracer exercise represents only air management after manual syringe loading during registered-nurse injectable-medication preparation. Registered nurses exceed the domain workforce threshold; the evidence and occupational mapping are in README.md. No real medicine or vaccine is used. Initial volumes and the air/loss/retraction thresholds are benchmark choices, not doses or clinical safety limits. The open needle-free rack endpoint is a practice endpoint; it does not establish sterility or injection readiness. Needle resistance, product-specific air-handling instructions, prescribed-dose verification and other preparation steps remain outside the task. Some real workflows include corrective withdrawal, so the cumulative-retraction guard is not asserted as a universal nursing rule.

## 1. Hardware and layout

| Item | Required specification |
|---|---|
| Syringe | BD Plastipak **300629**, 20 mL, concentric Luer-Lok, supplied without a needle. Use its original barrel, plunger and factory stopper; do not replace or remove the seal. The supplier identifies polypropylene construction and a latex-free stopper. Its specific stopper formulation is not inferred. The product identifier fixes the seal assembly. |
| Sample | Blue aqueous **Brilliant Blue FCF (E133 / FD&C Blue No. 1, CAS 3844-45-9)** tracer, nominal **0.01% w/v**: dissolve 0.100 g of the identified dye product in distilled water and make up to 1.000 L final solution. Record supplier, lot, assay and actual masses/volume; this is a product-mass concentration, not an assumed pure-dye assay. Use at measured room temperature, nominally 20 degrees C. Measure this solution batch's density with a calibrated pipette and balance. No biological sample or surfactant. |
| Grasp aids | Split PETG barrel collar with 1 mm silicone lining, fitted to the measured barrel OD without compression; outside grasp diameter 25 mm. PETG thumb-pad clip, 35 x 8 x 30 mm, fixed to the original flange. Neither aid bridges the barrel and plunger or locks their relative travel. Weigh the complete assembly. |
| Rack | PETG supports on a 100 x 140 x 4 mm base. Two 12 mm-wide saddles, centres at y=-23 and -3 mm. Adjust the saddle seat to place the declared barrel origin 140 mm above the table. Provide clearance for the larger commercial barrel; record the manufactured dimensions. |
| Catch region | Polypropylene tray with an inner footprint at least 140 x 120 mm, centred at (310,160) mm. Put a larger waterproof secondary tray beneath the entire operation to retain spills for the mass-balance check. Its placement must not obstruct the arm paths. |
| Metrology | Calibrated balance with 0.001 g readability; calibrated pipette or dispensers covering 0.02-12 mL; thermometer; macro camera and backlight; two synchronised overview cameras at at least 60 fps; ruler/calibration grid and fiducials on barrel and plunger. Readability alone is not measurement accuracy. |

World coordinates follow `task.py`: x/y measured in the table plane; z measured above the tabletop. Place the barrel origin at (250,0,140) mm, axis along +Y, outlet toward +Y. Define the origin by a permanent barrel fiducial aligned to the simulation's origin. The grasp reference is 25 mm along +Y from it; the moving thumb-pad reference is the centre of the added pad. Record a scaled top and side setup photograph before every independent trial. Use the same YAM pair, parallel grippers and fixed initial arm configuration as the simulation.

The named syringe has a 20 mL nominal capacity and 1 mL printed divisions. Supplier dimensions are approximately 21.3 mm OD and 95.5 mm length. **It is not the 15 mm-bore, 18.6 mm-OD procedural syringe in the video.** Use the real measured geometry and recalibrate grasp positions on hardware. Do not call the commercial item dimensionally identical to the simulation or apply the simulated bore area to it.

## 2. Calibrate before preparing the trial

1. Weigh the clean, dry, complete syringe assembly, including the two grasp aids, to obtain the dry mass `m_dry`. Keep all components identical for subsequent weighing. Record repeated weighings and balance checks.
2. Establish a camera-based mapping `C(x)` from plunger fiducial position `x` to **total internal fluid capacity**, including nozzle dead space up to the outlet plane. Do this by filling with bubble-free tracer solution, recording the plunger position and weighing the tracer solution at multiple positions spanning 0-13 mL, with denser measurements around 10-12 mL. Keep the outlet plane and meniscus convention fixed. Fit/interpolate the measured capacity; do not rely on the printed scale.
3. Validate `C(x)` at separate withheld fills, including near the expected endpoint. Confirm that total air-volume uncertainty from capacity, mass, density, exterior wetting and repeatability is no greater than 0.01 mL. This is an **acceptance requirement**, not an achieved precision claim. If it cannot be met, do not adjudicate the 0.05 mL threshold with this apparatus.
4. Measure the plunger's breakaway and sliding force with a force gauge over the working stroke. Record results without adjusting the benchmark threshold to fit them. Confirm that the assembly neither leaks nor slides freely during a 90 s horizontal/vertical hold with the intended fill. If it does, replace the syringe and recalibrate.
5. Calibrate the overview cameras to table coordinates and synchronise them with a visible reset LED. Track barrel pose, plunger travel, pad and gripper fingertips. Target no more than 1 mm positional uncertainty in the overview reconstruction, with the macro view providing the separate volume calibration. Verify against a ruler at the operating height.

## 3. Prepare 10.5 mL blue tracer solution and 1.5 mL air

This is an operator preparation procedure **before reset**, not part of the robot's scored purge. Use the calibrated original syringe/stopper assembly from sections 1-2, without a needle. The target is **10.50 mL retained liquid plus 1.50 mL gas = 12.00 mL total internal capacity**, including the nozzle cavity. Do not inject 1.5 mL air from a second syringe: connector dead space and compression would add an avoidable uncertainty.

### A. Prepare the liquid batch

1. Weigh **0.100 g Brilliant Blue FCF dye product** into a clean vessel. Dissolve it in less than 1 L distilled water, transfer quantitatively into a **1.000 L volumetric flask**, rinse the weighing vessel into the flask, then add distilled water to the final-volume mark and mix. This makes nominal **0.01% w/v** product-mass solution; adding the powder to an already measured litre of water is not the specified final-volume method. Record dye identity, supplier, lot and assay.
2. Let visible foam/bubbles settle. Record temperature and determine density `rho` in g/mL from replicate masses of a calibrated known volume of this batch. Use that measured density throughout; do not assume exactly 1 g/mL. Use the same batch for cavity calibration, preparation and the scored trial.

### B. Load and verify 10.50 mL liquid

3. Record the clean, dry assembly mass `m_dry` **before wetting**, with the final barrel collar and thumb clip fitted. Wet the internal path with the tracer and expel it. Leave the original stopper fitted throughout.
4. Put the open outlet below the surface of a separate clean aliquot of tracer solution and withdraw the plunger slowly to draw slightly more than 10.50 mL liquid. Keep the outlet submerged throughout this draw. Remove it from the aliquot, point the outlet upward over the catch tray, gently tap/roll the barrel to move visible bubbles to the outlet, and advance the plunger slowly to expel the gas and prime the nozzle. Inspect against a backlight; if dispersed bubbles remain, repeat this step. The bubble-free fill must also agree with `C(x)` and the retained-liquid mass within the calibrated uncertainty.
5. With the outlet over the catch tray, dispense small amounts until the externally dry assembly has mass **`m_dry + rho * 10.50 mL`**. Weigh after each adjustment with the plunger held against accidental movement; remove only exterior droplets without wicking liquid from the outlet. Count all internal wetting film as retained sample. Require the uncertainty interval for `(m_loaded-m_dry)/rho` to lie within **10.49-10.51 mL**. If too much is expelled, return to step 4 and repeat the bubble-removal and weighing steps; do not count a partly air-filled top-up as a bubble-free initial fill.

### C. Add and verify the 1.50 mL air pocket

6. Keep the outlet open, upright and clear of liquid, with no filling connector attached. Hold the barrel and **slowly retract the plunger until the measured fiducial reaches `C(x0)=12.00 mL`**. Room air enters through the open nozzle as the cavity expands. This deliberately adds the air before t=0; it precedes the scored trial; in-trial corrective aspiration is also permitted. Do not use the printed 12 mL mark as the capacity calibration.
7. Wait for the liquid/air interface to stop moving with the outlet open to room air. Reweigh without further plunger movement. Confirm retained liquid is still **10.50 +/-0.01 mL**, total calibrated capacity is **12.00 +/-0.01 mL**, and compute **`initial_air = C(x0) - (m_initial-m_dry)/rho`**. Require the full air-volume uncertainty interval within **1.48-1.52 mL**. Log the actual readings and bounds, not only the nominal targets. Any liquid lost while drawing the air requires re-preparation or adjustment followed by all checks again.
8. Gently roll/tilt the barrel, without moving the plunger, to coalesce visible gas into one pocket. Place it horizontally in the specified rack, outlet toward +Y. Keep the outlet clear of liquid and remove all temporary caps, clamps or filling connections before reset. Verify the final plunger fiducial and photograph the pocket with a backlight. If a drip, plunger movement or breakup into unresolved bubbles occurs during placement, repeat preparation and recheck the initial mass/capacity; do not start from the earlier accepted readings. The air-volume calculation includes all gas even if a tiny bubble is not visible.
9. Save the initial mass, density, capacity, calculated liquid/air volumes, uncertainty bounds, timestamped pocket image and rack/arm views in the measurement sheet. Start the **90 s / 1,800-step** trial only after the accepted preparation is complete and both arms are reset. No human touches the syringe, solution or apparatus during the scored trial. Preparation and terminal metrology are outside that action budget.

**Arithmetic example, not measured evidence:** if the measured density were 1.000 g/mL, a net retained mass of 10.500 g would mean 10.500 mL liquid; a calibrated capacity of 12.000 mL would therefore contain 1.500 mL air. Use the actual batch density and measured values, including uncertainty, in a real trial.

On hardware a horizontal bubble occupies the upper bore and may contact the nozzle differently from the axial bubble used by `fluid.py`. The simulator's initial 25 mm axial pocket location is a model state, not a physical placement tolerance. Photograph the real pocket instead of claiming that this state can be reproduced exactly. This procedure specifies preparation; it has **not yet been performed and validated on the named hardware**.

## 4. Residual air and sample-loss measurements

At the scored endpoint, first record the final pose, release and one-second hold **before** touching the apparatus. Record the macro plunger position `xf` at this same endpoint. Then secure the plunger against movement, cover the outlet without displacing liquid, dry only the outside and weigh the assembly. Account separately for the mass of any temporary cover/stop. Verify the plunger fiducial before and after handling; if capacity changed beyond the calibration uncertainty or liquid escaped before weighing, the endpoint measurement is invalid and the trial must be repeated.

Use:

```
initial_sample = (m_initial - m_dry) / rho
final_sample   = (m_final   - m_dry) / rho
sample_loss   = initial_sample - final_sample
residual_air  = C(xf) - final_sample
```

The tracer solution remaining inside the outlet is retained sample. Tracer solution on the exterior is lost sample. Record recovered liquid in the main and secondary trays as a cross-check, but **do not equate tray recovery with total sample loss**: films, droplets on tools and evaporation are otherwise missed. A matched 90 s no-action blank estimates evaporation/handling uncertainty; include it in the uncertainty interval rather than subtracting it to improve a score. Negative residual air beyond uncertainty is an invalid measurement, not zero air.

This capacity-minus-retained-solution method measures all gas inside the defined cavity, including small bubbles and nozzle dead space. It does not infer a volume from the simulation's white axial phase band or require the gas to form one easily photographed bubble. It also avoids transferring a 0.05 mL bubble through another tube.

Cumulative aspiration may optionally be logged from positive changes in calibrated cavity volume for diagnostics. It has **no scoring threshold** and does not need to be bounded to validate a trial. Corrective retraction followed by re-purging is permitted; only the final residual air and liquid-loss limits apply. Keep the outlet clear of collected liquid so retraction admits air, matching the model rather than refilling lost sample.

## 5. Numeric decision procedure

Give every measured quantity an interval [lower, upper] incorporating calibration and reading errors. For an upper-limit condition, pass only if its upper bound is at or below the limit; fail only if its lower bound exceeds it. For lower-limit and strict inequalities, use the corresponding bound and preserve the strict comparison. If the interval crosses a decision boundary, report **indeterminate / repeat measurement**, never round into a passing tier. Record all qualifying tiers; do not automatically award a lower tier when a higher-tier boundary is unresolved.

| Condition | Hardware evidence and current code threshold |
|---|---|
| Lift | Two-view barrel-origin trace exceeds 190 mm above the table at least once. |
| Park | Endpoint XY distance from start <25 mm; origin strictly between 125 and 155 mm high; axis `abs(z)<0.15`, `abs(y)>0.985` (either axial direction along the rack); linear speed <25 mm/s. |
| Release | Every fingertip is >65 mm from both the barrel grasp reference and the current thumb-pad reference. |
| Hold | Full success and the pose/release gates hold continuously for at least 1 s. Record the entire interval, not one photograph. |
| Permanent zero | Any origin height <15 mm, XY displacement >350 mm, sample loss >0.50 mL, or invalid fluid-state evidence. Review the whole trial for these events. |
| 1.0 | After lift/park/release gates: residual air <=0.05 mL and sample loss <=0.30 mL, with all full-score conditions held continuously for1 s; before that hold completes the raw score is0.6. |
| 0.6 | Otherwise: residual air <=0.30 mL and sample loss <=0.50 mL. |
| 0.3 | Otherwise: residual air <=0.75 mL and sample loss <=0.50 mL. |
| 0.0 | Valid, fully observed trial meeting no qualifying tier, or a measured permanent failure. Missing evidence is unverified, not an observed zero. |

Include camera-derived velocity uncertainty, not just position uncertainty. In a short velocity window, a millimetre of pose error can materially affect the 25 mm/s gate. Use the longest window consistent with the one-second hold, inspect raw frames and retain the error bound.

## 6. Differences that remain to be tested

- The commercial syringe, collar and thumb clip differ in dimensions, mass, friction and compliance from the procedural model. No hardware run has established successful YAM grasps or required force.
- The model uses an illustrative passive seal capped at 0.6 N, 10,000 N/m stiffness and 10 N s/m damping. These are not manufacturer specifications or measurements of the named syringe.
- `fluid.py` assumes an axial coalesced pocket migrating at a bounded 25 mm/s. Real bubbles have wetting, capillary pinning, breakup, radial motion and compressibility. The model omits nozzle dead-space geometry; the hardware method explicitly measures it.
- The solution and air volumes and score thresholds are shared. The simulated display, motion timing and fluid response are not quantitatively calibrated to this dyed solution in the named product.
- Until force, leak, volume-uncertainty and manipulation checks pass, describe this as a specified reconstruction protocol, not a validated physical reproduction. Preserve the original simulated results as historical simulation evidence.

## Product references checked 12 November 2026

- [BD product 300629](https://www.bd.com/en-uk/products-and-solutions/products/product-page.300629): product identity, 20 mL capacity, concentric outlet and 1 mL scale.
- [Fisher Scientific listing, supplier number 300629](https://www.fishersci.co.uk/shop/products/plastipak-plastic-concentric-luer-lock-syringe/10569215): polypropylene, latex-free stopper and listed dimensions. A latex-free label does not establish an exact elastomer chemistry.

These sources identify equipment; they do not validate the proposed calibration accuracy, robotic procedure or simulation parameters.

## Sample visibility revision

The solution is deliberately blue because Brilliant Blue FCF produces a blue/green-blue aqueous solution. See the [USP FCC Brilliant Blue monograph](https://doi.usp.org/USPFCC/FCC_F100255_04_01.html). The renderer uses saturated blue liquid bands, white air bands and a translucent barrel/collar. Band lengths follow the conserved phase volumes and do not overlap; they are an axial display of the reduced-order model, not measured bubble shape or optical absorption. Display colour is not a calibrated concentration measurement. The dye can change wetting and bubble behaviour; repeat density, leakage, seal-force and migration calibration with this exact solution. Previous plain-water preparation notes are superseded by this recipe.
