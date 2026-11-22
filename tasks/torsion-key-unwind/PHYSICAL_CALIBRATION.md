# Keyed torsion trainer

## Scope

The task uses a hand-sized passive bench fixture. The operator's access hazard is the output vane sweeping the space beside the withdrawing key when a loaded restraint clears. The spring, friction brake and blocking key transmit the load. The simulation demonstrates that interaction at the dimensions below. Transfer to a particular installed machine, access enclosure or injury threshold requires separate validation. Stored mechanical energy during servicing is the real-world hazard motivating this design. Source: https://www.osha.gov/control-hazardous-energy/

## Mechanical layout and replication

All dimensions below are mm in fixture coordinates, positive Z up. Put the common shaft axis at station X=335, Y=0 on the tabletop. Use the object file for every solid's nominal datum, size and orientation. Values for physical materials below are proposed build choices. The simulation uses rigid bodies, lumped masses and contact coefficients. Physical fabrication and hardware commissioning are pending.

| Assembly | Nominal geometry and load | Candidate physical construction and setup |
|---|---|---|
| Bed and spindle | Bed 245 × 215 × 16 centred at (−27,0,8). Brake face Ø96 × 4 at z=18. Vertical shaft Ø18. | Rigid aluminium plate secured to the bench, coaxial steel spindle and replaceable dry friction faces. The simulated spindle illustration has no collision geometry. |
| Blue input | Ø92 × 22 disc at z=31. The 22 × 19 × 39 grasp feature is centred at (−67,41,60). Total moving input mass 420 g. | Turned aluminium disc, radial lever and bolted grip. Adjust assembled mass with secured trim weights. Use a floating thrust bearing with ±3 axial travel and hard angle stops at −45° and +5°. Gravity supplies the normal force on the brake face. |
| Output | 100 g, axis at z=84, hub Ø60 × 25. Vane 42 × 8 × 12, outer edge 60 from axis. Relative angular limits −10° to +55°. | Coaxial bushed hub and keyed-on rigid vane. Attach opposite ends of a torsion spring to input and output. Set the unloaded relative angle to +36°. Nominal stiffness 0.0009 N·m/degree (0.0516 N·m/rad), damping 0.000033 N·m·s/degree, simulated torque cap 0.15 N·m. |
| Orange restraint | Moving mass 55 g at z=84, blade 46 × 10 × 14 centred at (−66,−9.5,84), grip 21 × 20 × 31 at (−83,−63,93). Travel 0 to 46 along negative X. Reset overlap 17. | Captive keyed slide, bridge and rigid grip, with an end stop at each end. The guide allows translation only. The simulated guide has viscous drag 3 N·s/m capped at 0.15 N. Choose or adjust a physical guide against the measured drag curve. |
| Paint and scale | Dark dots on the vane, common axis and key, white dot on the blue grip. Bed marks for key travel 0, 17 and 40. Red marks correspond to ±11.537° vane sweep. | Paint or transfer the markings to the specified faces at the USD datums. Added marks have no collision shapes. Zero the camera against the inserted key and the two reset hub marks. |

Assemble the brake disc on the floating carrier, then the upper sprung hub, then align the guide with the vane at rotor zero. The key blade must overlap the vane by 17 mm at reset and clear it with 46 mm travel. Use guards around a physical calibration rig and operate it remotely with a test handle. Bench measurements and a competent local hardware review are necessary before any powered-arm hardware trial.

## Commissioning measurements

1. Measure moving masses, axial runout, hub limits, key stops and blade overlap. Record materials, bearing part numbers, actual friction pairing, spring and damper parts, and environment. Dimensional drawings alone cannot establish the friction response.
2. With the test assembly restrained, measure torque versus relative angle with a force gauge at a measured radius. The model predicts 0.0324 N·m at the 36° reset deflection. Record loading and unloading curves, breakaway torque and free-decay response. The modeled static/dynamic friction coefficients are 0.95/0.70. Measure the built disc's resistance with its actual axial load and the guide force at several slide speeds.
3. On the enclosed test rig, establish that the key retains the initially loaded vane, the unloaded hub remains where released, and a loaded-key withdrawal produces a distinct excursion across the witness line. Repeat in both directions where applicable and record variation. A physical build whose responses or uncertainty overlap the pass and fail boundaries needs recalibration.

## Independent scoring with optical measurements

Use a fixed overhead camera, a ruler, a printed protractor and a second view of the hands and guides. Rectify against a ruler or reference points **at the height of each moving dot**. A ruler on the bed alone introduces parallax. Establish the input zero, vane zero and inserted-key X in a reset image. Use the overhead image to measure both yaw angles and the negative-X travel of the key. A 3° change spans about 3.14 mm on a 60 mm radius. The output excursion limit is asin(12/60) = 11.537°. Residual twist in degrees equals 36 + input angle − output angle. No force sensor enters the trial score.

Record through withdrawal and the clear final dwell. For each interval check both signs of peak excursion, residual twist whenever travel reaches 17 mm, and any arm-to-arm or non-hand-to-fixture contact. A final photograph cannot detect a rebound followed by recentering. Use sufficient frame rate and a validated between-frame motion bound, or an independently calibrated passive peak witness plus synchronized optical measurements. If the hands obscure both views or the measurement interval could straddle a boundary, record the result as inconclusive. The standard 320 × 176 policy cameras are adequate for qualitative review. Physical scoring resolution and occlusion coverage must be demonstrated on the built fixture.

The independent worksheet uses columns `time_s,input_deg,output_deg,key_mm,observed,clear,intact,collision`. Flags are 0 or 1. Set observed=1 only for an interval with adequate visibility of the required features, including guide and base integrity. Clear means both hands are off the fixture. Collision means an inter-arm collision or non-hand link striking the fixture. Intact covers the original mounting, joint and guide tolerances in `native/shaft_contract.py` and `native/shaft_bridge.py`. Use a reference jig/dial indicator if the views cannot establish guide integrity. The two error bounds must include calibration, reading uncertainty and possible motion between frames. The worksheet will report uncertain trials. Excessive uncertainty is a real limitation of a proposed physical setup.

```bash
python3 optical_score.py readings.csv --angle-error "$MEASURED_ANGLE_ERROR_DEG" \
  --key-error "$MEASURED_TRAVEL_ERROR_MM" --max-gap "$MAX_FRAME_INTERVAL_S"
```

Full credit requires the key at least 40 mm out, output and residual twist both inside ±3°, and a continuous 0.5 s clear dwell. Output excursion to either limit or disengagement under residual twist at or above that limit latches failure. A damaged or displaced guide, an inter-arm contact, or a non-hand link striking the fixture also latches failure. An unfinished release earns zero. The YAM recording continues for 3.6 s of consecutive action-level success where the action budget permits; the task scorer still applies its 0.5 s physical dwell.
