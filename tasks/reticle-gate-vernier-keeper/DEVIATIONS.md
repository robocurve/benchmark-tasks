# Deviations from 1:1 Physical Hardware — `reticle-gate-vernier-keeper`

1. **Simulated Rigid Bodies vs Elastic/Micrometer Screw Fine-Adjust**:
   - Commercial comparator reticle stages use a fine-pitch micrometer drum for sub-millimeter vernier adjustment. Because screw threads are excluded from the rigid-body solver, this task models the macro-indexing reticle dovetail slide (`90 mm`), stepped polarity datum (`180 deg` non-reversible stop), X-axis crossing spring beam gate (`0.10 N*m/rad` return spring), and perpendicular vernier locking bar (`54 mm`) with passive slide detents (`0.18 N`).
2. **Rigid Collider Footprint vs Visually Chamfered PBR Geometry**:
   - Physics collision shapes remain strict zero-penetration boxes over a fixed pebbled ESD bench mat (`0.90` friction, no runtime `@on_step` joint drive hacks), while invisible collision primitives are visually dressed in Blender 5.2.3 LTS `.usdc` meshes with procedural metallic/rubber/optical-glass PBR maps (`assets/reticle-gate-vernier-keeper/`).
3. **Top-Down Gripper Caging Posts**:
   - Handles incorporate narrow fore/aft caging shoulders and lower push ledges matched to the dual parallel grippers so open-loop motion trajectories transmit tangent and axial loads purely via normal contact rather than friction approximations.
4. **Excluded Energized Optical Source**:
   - The comparator condenser lamp and projection readout electronics are omitted; the optical illumination port and reticle wafer cell are passive inert glass-effect solids (`OmniGlass`).
5. **Category `Other`, workflow importance**: the task is an automated dimensional-inspection step. The importance argument is a workflow argument (see README `## Importance`), not an occupational headcount claim: no occupation code is asserted for this step.
