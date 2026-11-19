"""Native PhysX PBD liquid and a packed-plane inflow calibration candidate.

The measured pump supplies particles; PhysX alone moves them after birth. No glass
position, capture decision or score enters this module. The packed volume and
inlet remain unvalidated. Carbonation and foam are not modeled.
API pattern: NVIDIA's installed FluidBallEmitterDemo.py and
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/particles/particles.html
"""

import math

import numpy as np

from .scoring import REST_OFFSET, QUANTUM_ML
from .geometry import NOZZLE as NOZZLE_POSITION

SPACING = 2 * REST_OFFSET
LAYER_SPACING = math.sqrt(.75) * SPACING
PARTICLE_ML = QUANTUM_ML
DENSITY_KG_M3 = 1005.
PARTICLE_MASS_KG = DENSITY_KG_M3 * PARTICLE_ML * 1e-6
# The USD particle importer floors restOffset cubed at FLT_EPSILON.
# Normalize its mass-only density; the PBD material density remains 1005 kg/m^3.
# Native metrology must validate this route before task-completion claims.
NATIVE_SET_DENSITY = PARTICLE_MASS_KG / (8 * max(REST_OFFSET**3, float(np.finfo(np.float32).eps)))
NOZZLE = np.asarray(NOZZLE_POSITION)
NOZZLE_RADIUS = .0065
# Clear the finite particle/contact shell at the physical outlet lip.
BIRTH_DROP = .003
MAX_PARTICLES = 300000


def packed_layer(radius, layer):
    """XY offsets of one native TightPack3D lattice plane, in metres."""
    if not math.isfinite(radius) or radius <= 0 or int(layer) != layer:
        raise ValueError('Expected positive finite radius and integer lattice layer')
    extent = math.ceil(radius / LAYER_SPACING) + 1
    sites = [(i*SPACING + ((j + int(layer)) & 1)*SPACING/2, j*LAYER_SPACING)
             for j in range(-extent, extent + 1) for i in range(-extent, extent + 1)]
    return np.asarray([(x, y) for x, y in sites if x*x + y*y <= radius**2], dtype=float).reshape(-1, 2)


def create_fluid(stage, root):
    """Author an empty GPU PBD fluid system inside a task-owned USD root."""
    from omni.physx.scripts import particleUtils, physicsUtils
    from pxr import Gf, Sdf, UsdShade

    system_path = Sdf.Path(str(root) + '/system')
    material_path = Sdf.Path(str(root) + '/material')
    system = particleUtils.add_physx_particle_system(
        stage, system_path, contact_offset=.002, rest_offset=REST_OFFSET,
        particle_contact_offset=REST_OFFSET / .6, solid_rest_offset=REST_OFFSET,
        fluid_rest_offset=REST_OFFSET, enable_ccd=True,
        solver_position_iterations=16, max_neighborhood=96, max_velocity=8.,
        non_particle_collision_enabled=True, global_self_collision_enabled=True)
    # Water-like coefficients are estimates, not measurements of a beer recipe.
    particleUtils.add_pbd_particle_material(
        stage, material_path, friction=.1, damping=0., viscosity=.000002,
        surface_tension=.0074, cohesion=.01, density=DENSITY_KG_M3)
    physicsUtils.add_physics_material_to_prim(stage, system.GetPrim(), material_path)
    # Native sparse-grid surface extraction fails on an empty particle set.
    particleUtils.add_physx_particle_isosurface(
        stage, system_path, enabled=False, grid_spacing=REST_OFFSET,
        surface_distance=REST_OFFSET, max_vertices=1000000,
        max_triangles=2000000, max_subgrids=8192)

    material = UsdShade.Material.Get(stage, material_path)
    shader = UsdShade.Shader.Define(stage, str(material_path) + '/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.72, .30, .025))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.12)
    shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(.72)
    shader.CreateInput('ior', Sdf.ValueTypeNames.Float).Set(1.34)
    shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(system.GetPrim()).Bind(material)

    points = particleUtils.add_physx_particleset_points(
        stage, Sdf.Path(str(root) + '/particles'), [], [], [], system_path,
        self_collision=True, fluid=True, particle_group=0,
        particle_mass=0., density=NATIVE_SET_DENSITY)
    points.CreateDisplayColorAttr([Gf.Vec3f(.72, .30, .025)])
    points.GetPrim().CreateAttribute('physxParticle:maxParticles', Sdf.ValueTypeNames.Int).Set(MAX_PARTICLES)
    UsdShade.MaterialBindingAPI.Apply(points.GetPrim()).Bind(material)


class NativeBeerFluid:
    """Read raw particles and append nozzle births after native USD writeback.

    Caller must enable native updateToUsd/updateParticlesToUsd and synchronize
    completed physics results before every sample or emission. No smoothing is
    authored: Points contains simulation coordinates, not render-smoothed ones.
    Construct a new reader after a complete scene reset; this never resets fluid.
    """

    def __init__(self, stage, root):
        from pxr import PhysxSchema, UsdGeom

        self.points = UsdGeom.Points.Get(stage, str(root) + '/particles')
        if not self.points or not self.points.GetPrim().HasAPI(PhysxSchema.PhysxParticleSetAPI):
            raise RuntimeError('Missing native beer particle set')
        system = stage.GetPrimAtPath(str(root) + '/system')
        if system.HasAPI(PhysxSchema.PhysxParticleSmoothingAPI):
            raise RuntimeError('Beer scoring requires unsmoothed particle coordinates')
        self._surface_enabled = PhysxSchema.PhysxParticleIsosurfaceAPI(system).GetIsosurfaceEnabledAttr()
        self.count = 0
        # Pending inlet volume in particle quanta, below the next whole plane.
        self._remainder = 0.
        self._layer = 0
        self._layers = (packed_layer(NOZZLE_RADIUS, 0), packed_layer(NOZZLE_RADIUS, 1))
        if not all(len(sites) for sites in self._layers):
            raise RuntimeError('Nozzle cannot resolve both packed lattice planes')
        # Exit velocity uses the unchanged physical bore, independent of cell calibration.
        self._area = math.pi * NOZZLE_RADIUS**2
        self.positions()

    @staticmethod
    def _vectors(attribute):
        values = np.asarray(attribute.Get(), dtype=np.float32)
        if values.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        if values.ndim != 2 or values.shape[1] != 3 or not np.isfinite(values).all():
            raise RuntimeError('Invalid native fluid position or velocity readback')
        return values

    def positions(self):
        positions = self._vectors(self.points.GetPointsAttr())
        if len(positions) != self.count:
            raise RuntimeError(f'Native fluid count changed outside emitter: {len(positions)} != {self.count}')
        return positions

    def emit_volume(self, volume_ml, dt):
        """Append measured piston delivery at the fixed outlet boundary."""
        from pxr import Sdf, Vt

        if not (math.isfinite(volume_ml) and volume_ml >= 0
                and math.isfinite(dt) and dt > 0):
            raise ValueError('Expected nonnegative native delivery volume and positive timestep')
        if volume_ml == 0:
            return 0
        rate = volume_ml / (dt*PARTICLE_ML)
        total = self._remainder + rate * dt
        births = 0
        planes = []
        speed = volume_ml*1e-6/(dt*self._area)
        while total - births >= len(self._layers[(self._layer + len(planes)) % 2]):
            sites = self._layers[(self._layer + len(planes)) % 2]
            births += len(sites)
            if self.count + births > MAX_PARTICLES:
                raise RuntimeError('Fluid capacity exceeded; refusing to discard emitted beer')
            event_time = (births - self._remainder) / rate
            age = dt - event_time
            if age < -1e-12 or age > dt + 1e-12:
                raise RuntimeError('Packed inlet birth lies outside the measured interval')
            age = min(dt, max(0., age))
            positions = np.tile(NOZZLE, (len(sites), 1))
            positions[:, :2] += sites
            positions[:, 2] -= BIRTH_DROP + speed*age + .5*9.81*age**2
            velocities = np.zeros_like(positions)
            velocities[:, 2] = -speed - 9.81*age
            planes.append((positions, velocities))
        if not births:
            self._remainder = total
            return 0
        old_positions = self.positions()
        old_velocities = self._vectors(self.points.GetVelocitiesAttr())
        if len(old_velocities) != self.count:
            raise RuntimeError('Incomplete native fluid velocity readback')
        # All sites of a plane share its measured completion time. Native physics
        # alone moves them after this substep-resolved inflow boundary.
        with Sdf.ChangeBlock():
            self.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(
                np.concatenate([old_positions, *(p for p, _ in planes)]).astype(np.float32)))
            self.points.GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(
                np.concatenate([old_velocities, *(v for _, v in planes)]).astype(np.float32)))
            self.points.GetWidthsAttr().Set(Vt.FloatArray([SPACING] * (self.count + births)))
            if self.count == 0:
                self._surface_enabled.Set(True)
        from omni.physx import get_physx_simulation_interface
        get_physx_simulation_interface().flush_changes()
        self.count += births
        self._layer += len(planes)
        self._remainder = total - births
        return births
