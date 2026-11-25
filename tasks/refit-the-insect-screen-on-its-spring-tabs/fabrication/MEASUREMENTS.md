# PL 7776 bench reading — one specimen

Measured Nov 23, 2026, on one Prime-Line PL 7776 spring; package identity checked.
Tools: digital calipers, kitchen scale with 1 g display increments, ruler.

| Quantity | Reading | Method / scope |
|---|---|---|
| Strip thickness | 0.27 mm | Digital calipers; one specimen |
| Free bow height | 7 mm | Table to top of free bow; ruler/calipers; not installed proud height above a frame rail |
| Force pressing the bow flat at its center | Scale reading 161 g, equivalent to 1.58 N | Kitchen scale; one specimen; instrument calibration uncertainty not supplied |

These are bench measurements on one free spring, not an installed screen assembly. No photographs, repeated cycles,
intermediate-compression readings, unloading curve or mounting/preload measurement were supplied. The display
increment of 1 g corresponds to approximately 0.00981 N; that is resolution, not calibrated force accuracy.

Derived quantities (estimates, not additional readings): 1.58 N / 0.007 m gives a secant stiffness of about
226 N/m **if** linearity is assumed. The simply-supported strip estimate with 200 GPa modulus, 69.85 x 6.35 mm
nominal strip dimensions and measured 0.27 mm thickness gives about 293 N/m. Neither is an installed force curve.

The nominal model uses **400 N/m**, 1.77 times the measured free-part endpoint secant
`1.58 N / 0.007 m = 225.7 N/m`. A separate 0.04905 N drive offset compensates the artificial 5 g crown weight
upright; the constant rest target is -0.122625 mm, applied once after reset. No installed preload was measured;
the nominal model therefore has none. Upright static net axial resistance is `400*q` N with q in metres:
**0.4 N seated, 1.6 N at 4 mm, 2.6 N at 6.5 mm and 2.8 N flat**. Total elastic drive adds 0.04905 N.
At lean a the static axial estimate is `400*q + 0.04905*(1-cos(a))`; it is not a contact-force sensor or a
ceiling-normal measurement. In frictionless static equilibrium with a compressed slider, ceiling-normal reaction
would be axial resistance divided by cos(a), not multiplied by it. Damping and joint-stop reactions are separate.
The equal internal reaction acts on crown and frame; neither gravity nor episode poses are modified.

For every q in 0–7 mm, the nominal linear net resistance is at least the **assumed free-part linear secant**
`225.7*q`; full-compression resistance 2.8 N also exceeds the measured 1.58 N endpoint. This states exactly what
the bound covers. It **does not prove an upper bound on an unknown nonlinear installed curve**, preload or gauge
uncertainty, and it does not assume that a free-part secant measures installed stiffness. Enrol both springs in
their actual corner keys across the executed loading/unloading path. Reject or remodel any assembly whose
measured resistance plus uncertainty exceeds the model, or whose proud height/travel differs materially.

