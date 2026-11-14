# Icing model and limits

The surface law is a shallow-layer Herschel–Bulkley approximation. It uses an
800 Pa shear yield stress, a consistency of 4,000 Pa·s^0.5 and a flow index of
0.5. These are engineering values in the range of the aerated buttercream
measurements in Fernandes et al., *Aeration and rheology of buttercream icings*
(2023), [accepted manuscript](https://www.repository.cam.ac.uk/handle/1810/357900),
[DOI](https://doi.org/10.1016/j.jfoodeng.2023.111775), Figures 8 and 10.
They are not a fitted recipe: Figure 10 gives different fits depending on the
yield criterion, and the authors report substantial batch and aeration effects.
The model omits elasticity, thixotropy and fracture. Checked 2026-11-13.

The former constant slope threshold of 0.75 was inadequate: at a 3 mm deposit
depth, it allowed flow at a driving shear stress of only about 19 Pa. It treated
a thin decorative ridge like a much deeper mound. That model is preserved in
the source snapshots of native trials through batch 029; their results cannot
validate this revised law.

For each neighbouring pair of cells, the driving pressure gradient is the
gradient of gravity head plus pressure beneath the measured nozzle aperture.
For depth `h`, gradient magnitude `G`, shear yield `tau`, consistency `K` and
index `n`, the yielded lower-layer depth is `Y = max(h - tau/G, 0)`. Integrating
the shear rate gives the volume flux per unit width:

```text
q = sign(gradient) * (G/K)^(1/n) *
    [Y^(2+1/n)/(2+1/n) + (h-Y)*Y^(1+1/n)/(1+1/n)]
```

The upper unyielded plug moves with the lower layer. The formula reduces to
Newtonian thin-film flow when `tau = 0` and `n = 1`. Each disjoint neighbouring
cell pair is solved implicitly for its end-of-substep flux, bounded by its
available volume. Axes alternate to reduce splitting bias. The aperture
footprint is fixed within each pair solve, while submergence changes with depth.
This replaces the explicit 24% donor limiter used in batch 030: that limiter
strongly affected spreading under high pressure and was not sufficiently stable
under grid refinement. No new volume or prescribed shell geometry is introduced.

The aperture and whole annular metal face are sampled at the grid scale.
Contact stiffness and damping are area weighted, so adding samples does not
multiply the force. Repeated samples in one grid cell displace its material
only once. This removes holes in the old two-ring metal contact quadrature.
Deposited parcels use conservative bilinear weights rather than snapping to
one cell; material crossing the modeled boundary remains recorded as lost.

Prescribed-input CPU controls at 0.5 mm and 0.25 mm now produce similar shell
heads and long ridges down the tapered shoulders. Their scored decorated lengths
are 75.0 and 73.75 mm for the six-shape diagnostic, both below the 80 mm full-credit
threshold. Maximum deposit heights are 5.27 and 6.06 mm, a remaining 15%
grid sensitivity; this model is not fully converged. These are material
controls, not robot demos. Native batches through
031 precede the full-face contact and bilinear deposition corrections; they
cannot validate the final material implementation.

The independent reservoir law still uses a 6 kPa extrusion threshold, an
effective bulk modulus of 160 kPa and a declared nozzle conductance. Reservoir
pressure is not the same quantity as shear yield stress. Pressure transmission
under a submerged aperture over 2 mm is a local approximation, not a solved
three-dimensional pressure field. It has no access to the target border.

Ballistic parcels sample the star opening and deposit conserved volume onto a
single-valued surface. Pressure transmission onto the supported layer applies
to downward-facing outlets; upward ejection remains ballistic. This representation cannot model overhangs, long strings
or voids. Only the native robot run with matching final photographs can establish
task feasibility. Prescribed-input material probes are diagnostic controls.


## Native boundary motion

From the next trial after087, the external material solver uses the unsmoothed
world displacement of every metal-face sample and nozzle tip over the preceding
native step. The explicit coupling is staggered by one step. Reset primes the
previous pose; no subsequent displacement is clipped, filtered or suppressed.
The native reported velocity remains in the saved trace for diagnostics.
[PhysX documents](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html)
that its split-impulse solver can report velocities different from transform
integration. Holding the endpoint velocity over an entire external material
step incorrectly represented the observed nozzle motion in trial087.

Pressure and contact forces are applied at each body origin, with contact
moments about that same origin. PhysX therefore transfers the wrench to the
actual centre of mass; it is not assumed to coincide with the body origin.
Stationary, translation, rotation and reset controls pass. This correction does
not change the reservoir, material law, nozzle geometry or scoring. Native trial111 demonstrates a complete full-score border with this coupling.
Neither that successful run nor the replay and kinematic controls prove energy
conservation of the coupled approximation.
