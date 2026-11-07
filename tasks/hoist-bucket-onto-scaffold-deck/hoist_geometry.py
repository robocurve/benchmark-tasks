"""Offline geometry shared by the passive spawner, demo and checker (metres/XYZW)."""
import math
from dataclasses import dataclass
import numpy as np
from yam_arena.task_file import TABLE_TOP
from yam_arena.ik import matrix_to_quat_xyzw
from yam_parts import Part, box, cyl
from yam_joints import Mechanism, Link, Hinge

TOP = TABLE_TOP['station']
PITCH, RADIUS, LINK_MASS = .020, .004, .0012
GROOVE_R, PITCH_R, SPAWN_R = .026, .030, .032
# 11 mm cannot give an 8 mm rope 2 mm on EACH side. Use 12.2 mm.
HALF_GAP, FLANGE_R, FLANGE_T = .0061, .038, .002
ARMATURE = 5.e-6


def rotation(q):
    q = np.asarray(q, float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1.e-10:
        return np.full((3, 3), np.nan)
    x, y, z, w = q/np.linalg.norm(q)
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def rope_frame(direction):
    x = np.asarray(direction, float)
    y = np.array((1., 0., 0.))
    return np.column_stack((x, y, np.cross(x, y)))


def wheel(name, axle):
    x, y, z = axle
    # Post is forward of the entire rope/bucket/hand envelope. Cantilever and
    # one-sided hanger stay above/outside the groove. The axle is inside hub.
    base = [box((.140, 0, .235), (.018, .032, .470)),
            box((.080, 0, .460), (.138, .025, .018)),
            box((.020, 0, .430), (.008, .012, .060)),
            cyl((.016, 0, .400), .003, .024, axis='x')]
    shapes = [cyl((0, 0, .400), GROOVE_R, 2*HALF_GAP, axis='x')]
    shapes += [cyl((s*(HALF_GAP+FLANGE_T/2), 0, .400), FLANGE_R,
                   FLANGE_T, axis='x') for s in (-1, 1)]
    return Mechanism(name, (x, y, z-.400), base,
        [Link('sheave', shapes, Hinge('x', (0, 0, .400), -100000, 100000,
              damping=1.e-4, armature=1.e-4), .065,
              color=(.26, .43, .57), friction=(.8, .8))],
        friction=(.8, .8), contact_offset=.001, color=(.42, .44, .46))


@dataclass(frozen=True, init=False)
class HoistRope(Mechanism):
    # Mechanism is a kit Part. Its offline_state protocol is the checker's
    # custom-state extension point; ordinary Parts are reduced to PartState.
    # Only the pail is exposed as shapes. The 53-link GPU articulation is still
    # authored exclusively by hoist_physics, before either kit spawn handler.
    axle: tuple = (.18, -.03, TOP+.400)
    side: int = 1  # load on +y for A; load on -y for B
    bucket_mass: float = .30
    wheel_name: str = 'hoist_a'
    arm: str = 'right'
    segments: int = 53

    def __init__(self, name, pos, *, axle=(.18, -.03, TOP+.400), side=1,
                 bucket_mass=.30, wheel_name='hoist_a', arm='right', segments=53,
                 color=(.60, .62, .65)):
        for key, value in dict(axle=tuple(axle), side=side, bucket_mass=bucket_mass,
                               wheel_name=wheel_name, arm=arm, segments=segments).items():
            object.__setattr__(self, key, value)
        # No kit hinge/slide links: this object's joints use the custom spawner.
        super().__init__(name, pos, pail_shapes(), (), color=color, friction=(.8, .8))

    @property
    def free(self):
        return False

    def _check(self):
        Part._check(self)
        assert self.yaw == 0 and self.side in (-1, 1)
        assert self.segments == 53 and self.bucket_mass > 0
        assert np.allclose(self.pos[:2], (.180, 0.))
        assert math.isclose(self.pos[2], TOP+.037)

    def vertices(self):
        # Six tangent links, all EXACTLY 20 mm. Their centreline apothem is
        # 32 mm: each capsule is 2 mm off the 26 mm groove at its closest point.
        delta = 2*math.atan(PITCH/(2*SPAWN_R))
        angles = (math.pi-5*delta)/2 + np.arange(6)*delta
        arc_centres = np.array(self.axle)+np.column_stack((np.zeros(6),
            self.side*SPAWN_R*np.cos(angles), SPAWN_R*np.sin(angles)))
        directions = np.column_stack((np.zeros(6), -self.side*np.sin(angles), np.cos(angles)))
        start = arc_centres[0]-directions[0]*PITCH/2
        v = [start-np.array((0, 0, PITCH*i)) for i in range(14, -1, -1)]
        for p, d in zip(arc_centres, directions):
            assert np.linalg.norm(v[-1]-(p-d*PITCH/2)) < 1.e-10
            v.append(p+d*PITCH/2)
        for _ in range(18):
            v.append(v[-1]+np.array((0, 0, -PITCH)))
        # Two shallow links ease into a straight 260 mm tabletop tail at +6 mm.
        v.append(v[-1]+np.array((0, -self.side*PITCH/math.sqrt(2), -PITCH/math.sqrt(2))))
        dz = v[-1][2]-(TOP+.006)
        assert 0 < dz < PITCH
        v.append(v[-1]+np.array((0, -self.side*math.sqrt(PITCH**2-dz**2), -dz)))
        for _ in range(13):
            v.append(v[-1]+np.array((0, -self.side*PITCH, 0)))
        return np.asarray(v)

    def bail_offset(self):
        return self.vertices()[0]-np.asarray(self.pos)

    def layout(self):
        v = self.vertices()-self.pos
        return v, (v[:-1]+v[1:])/2, np.array([rope_frame(d/PITCH) for d in np.diff(v, axis=0)])

    def body_names(self):
        return ['bucket']+[f'seg_{i:02d}' for i in range(self.segments)]

    def expected_dofs(self):
        return [f'bail_swivel:{i}' for i in range(3)]+[
            f'bend_{i:02d}:{j}' for i in range(1, self.segments) for j in (1, 2)]

    def bucket_part(self):
        # 60 x 60 x 70 mm open pail; bail is part of the heavy root, with its
        # top touching the swivel anchor. No rope inside the rim or pail walls.
        return Part('bucket', self.pos, self.shapes, mass=self.bucket_mass,
                    color=self.color, friction=(.8, .8), contact_offset=.002)

    def offline_state(self, pos, yaw, joints, *, R=None):
        """Geometric rope at a specified pail pose; never used by GPU reads."""
        from hoist_runtime import nominal_state
        if joints:
            raise ValueError('HoistRope offline joints must be empty; give the pail pose')
        if R is None:
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.)))
        return nominal_state(self, tuple(map(float, pos)), tuple(np.asarray(R, float).ravel()))

    def initial_poses(self):
        _, c, rr = self.layout()
        return {'bucket': (np.asarray(self.pos), np.array((0., 0., 0., 1.))),
                **{f'seg_{i:02d}': (c[i]+self.pos, matrix_to_quat_xyzw(rr[i]))
                   for i in range(self.segments)}}


def pail_shapes():
    shapes = [box((0, 0, -.033), (.060, .060, .004))]
    shapes += [box((s*.028, 0, .002), (.004, .060, .066)) for s in (-1, 1)]
    shapes += [box((0, s*.028, .002), (.052, .004, .066)) for s in (-1, 1)]
    shapes += [cyl((s*.034, 0, .055), .003, .040) for s in (-1, 1)]
    shapes += [cyl((0, 0, .075), .003, .068, axis='x')]
    shapes += [box((s*.032, 0, .033), (.008, .008, .004)) for s in (-1, 1)]
    return shapes


def make_rope(name, axle, side, kg, wheel_name, arm, color):
    provisional = HoistRope(name, (0, 0, 0), axle=tuple(axle), side=side,
                            bucket_mass=kg, wheel_name=wheel_name, arm=arm, color=color)
    start = provisional.vertices()[0]
    return HoistRope(name, (float(start[0]), 0., TOP+.037), axle=tuple(axle),
                     side=side, bucket_mass=kg, wheel_name=wheel_name, arm=arm, color=color)


def capsule_inertia(mass=LINK_MASS):
    r, h = RADIUS, PITCH-2*RADIUS
    vc, vs = math.pi*r*r*h, 4*math.pi*r**3/3
    mc, ms = mass*vc/(vc+vs), mass*vs/(vc+vs)
    axial = mc*r*r/2 + 2*ms*r*r/5
    transverse = mc*(3*r*r+h*h)/12 + ms*(2*r*r/5+h*h/4+3*h*r/8)
    return np.array((axial, transverse, transverse))


def compound_inertia(part):
    """Uniform density on disjoint box/cylinder pieces; matches USD inference."""
    volumes = np.array([np.prod(s.size) if hasattr(s, 'size') else
                        math.pi*s.radius**2*s.height for s in part.shapes])
    masses = part.mass*volumes/volumes.sum()
    com = np.average(np.array([s.center for s in part.shapes]), axis=0, weights=masses)
    inertia = np.zeros((3, 3))
    for s, m in zip(part.shapes, masses):
        if hasattr(s, 'size'):
            a = np.asarray(s.size)**2
            I = np.diag(m*(a.sum()-a)/12)
        else:
            diagonal = np.full(3, m*(3*s.radius**2+s.height**2)/12)
            diagonal['xyz'.index(s.axis)] = m*s.radius**2/2
            I = np.diag(diagonal)
        d = np.asarray(s.center)-com
        inertia += I+m*(np.dot(d, d)*np.eye(3)-np.outer(d, d))
    return com, inertia


def deck():
    timber = (.56, .38, .19)
    steel = (.42, .44, .46)
    shapes = [box((0, y, .134), (.160, .040, .012), timber)
              for y in (-.060, -.020, .020, .060)]
    shapes += [box((x, 0, .122), (.012, .160, .012), steel) for x in (-.060, .060)]
    shapes += [box((x, y, .058), (.012, .012, .116), steel)
               for x in (-.060, .060) for y in (-.060, .060)]
    return Part('deck', (.180, .130, TOP), shapes, friction=(.8, .8), color=steel)
