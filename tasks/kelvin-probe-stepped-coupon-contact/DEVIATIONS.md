# Deviations and Engineering Notes (`kelvin-probe-stepped-coupon-contact`)

- **Real-world part reference**: Four-point probe head and ceramic XY reference carriage scaled 1:1 from benchtop hardness/resistivity metrology hardware (`204 x 128 x 46 mm` carrier body, `94 mm` overall height with the probe arm raised at `-22 deg` on the `4 mm` ESD mat, `$76` BOM: benchtop metrology reference carriage / precision linear ball slide / stainless steel extension spring equivalents).
- **PhysX tolerances**: Seat alignment tolerance set to `6 mm` (`SEAT_TOL_M = 0.006`) and head tilt to `6 deg` (`ARM_TILT_MAX_DEG = 6.0`) per real-hardware open-loop PD settling margins (`1-4 mm`) rather than idealized sub-millimetre cutoffs.
- **Electrical continuity omitted**: Rigid-body contact geometry verifies physical four-point step seating and polarity keyway alignment; four-wire resistance acquisition is omitted.
