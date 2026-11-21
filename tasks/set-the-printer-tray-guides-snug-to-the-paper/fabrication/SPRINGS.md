# Custom finished spring specifications: targets, not measured parts

Sources: [Lee Spring material properties](https://www.leespring.com/spring-materials), [custom spring manufacture](https://www.leespring.com/learn-about-custom-springs), [torsion design guidance](https://www.leespring.com/learn-about-torsion-springs). Lee lists ASTM A228 music wire with nominal E=30×10^6 psi and G=11.5×10^6 psi. Those material constants support calculations only. Lee is a possible custom manufacturer, not a contacted supplier or a confirmed quote/stock source. Any competent spring shop may manufacture to the finished dimensions and load tests below. No stock part number is invented.

All dimensions mm. Supply dimensional/load inspection with the parts; surface finish and stress relief appropriate to A228 must be selected by the manufacturer. Do not hand-wind an unqualified substitute and label its rate verified. Finished performance and envelope requirements take precedence over estimated active-coil counts.

## S1: guide pad compression spring, quantity 6

Music wire ASTM A228; wire Ø0.50±0.005; mean coil Ø4.50±0.05 (OD nominal 5.00, ID nominal 4.00); 9 active / 11 total coils; closed, squared and ground ends; free length 15.00±0.10; maximum solid height 5.60; winding direction immaterial for these independent guided compression springs. Guide over Ø3 pins; spring must not rub the guide or its seat.

Required **finished rate 0.750±0.038 N/mm per spring**, 0..5 mm deflection; pair the two springs on each pad so their combined measured rate is 1.500±0.075 N/mm and neither side causes pad skew. Load at 2 mm deflection nominal 1.50 N per spring, at 5 mm 3.75 N. No intentional preload: installed 15.00 mm at the pad's zero stop, with spring contact but initial pair force ≤0.15 N.

Prediction using k=Gd^4/(8D^3Na) is about 0.755 N/mm per spring. This is not measured. At the 5 mm travel stop, installed length is 10 mm, leaving at least 4.4 mm above the specified maximum solid height. This margin is geometric; it is not fatigue/life certification. Calibrate the assembled cartridge, including friction.

## S2: latch return torsion spring, quantity 2, opposite installation handedness

Music wire ASTM A228; nominal wire Ø1.20; mean coil Ø8.00, nominal OD 9.20 / ID 6.80; body length ≤4.0; straight tangential legs 26 and 8 from their tangent points; unloaded included leg angle 90°. Put on a Ø3 pivot with axial spacers giving ≥0.5 clearance each side. Use the finished centreline tangent/anchor coordinates in HINGE.md: moving leg N−23,Z36..62; fixed leg Z40,N−19..−11; both S22.5. Squeeze must wind the coil and reduce its diameter, per the manufacturer's guidance.

Finished incremental torque rate, measured **in the assembled leg/anchor configuration**, 400±40 N mm/rad (0.400±0.040 N m/rad). Reference zero is the fully open latch; residual torque at zero ≤2 N mm. Required nominal torque at 0.10 rad is 40 N mm and at 0.17 rad 68 N mm. No coil binding through 0.20 rad. A sizing estimate is approximately 2.09 effective coils including leg compliance; starting body winding about 1.65 active turns. The manufacturer must adjust body turns/leg forming to meet the finished load points and fit, not treat the approximate turn count as a guaranteed rate.

## S3: pawl return compression spring, quantity 2

Music wire ASTM A228, wire Ø0.20, mean coil Ø3.00, 7 active / 9 total coils, closed ends, free length 6.0, maximum solid height 1.9. Install between the fixed flat cap seat and pawl-lid seat at 4.2 mm locked-position length, as PAWL.md specifies. Select finished springs, or require manufacturer load acceptance, to meet **0.15±0.03 N at 4.2 mm installed length**. There is no adjustable S3 seat screw. Normal operating compression adds at most 0.57 mm: length ≥3.63, above solid. The independent 1.2 mm backup lift stop also remains above solid. Estimated rate 0.084 N/mm; finished rate acceptance 0.07..0.10 N/mm. The load is only a pawl return, not the guide's holding force. Include it in assembled latch calibration.
