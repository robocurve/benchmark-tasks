"""Contact representation of a split-hub dishwasher rack roller and axle."""
import numpy as np

import mechkit as m


RADIUS = .017
TYRE_HALF_WIDTH = .0045
STEM_RADIUS = .003
HEAD_RADIUS = .00435
SEATED_Y = .0065
LIP_CENTER = -.0041
LIP_BORE_RADIUS = .00345

LOOKS = {
    'nylon': m.Look(m.rgb(.27, .30, .29), roughness=.62),
    'steel': m.Look(m.rgb(.50, .54, .55), roughness=.49, metallic=.70),
    'witness': m.Look(m.rgb(.94, .65, .14), roughness=.74),
}


def band(inner, outer, start, stop, back, front):
    """Make one convex annular sector around the axle, which runs along Y."""
    a, b = np.radians((start, stop))
    outline = [(r * np.cos(t), r * np.sin(t))
               for r, t in ((inner, a), (outer, a), (outer, b), (inner, b))]
    return m.prism(outline, back, front).aimed('+Y')


def segments(inner, outer, start, stop, back, front, count):
    bounds = np.linspace(start, stop, count + 1)
    return [band(inner, outer, a, b, back, front) for a, b in zip(bounds[:-1], bounds[1:], strict=True)]


def axle_parts(at=(0., 0., 0.)):
    stem = m.cylinder(STEM_RADIUS, .013, (0., -.0025, 0.), axis='y')
    shoulder = m.cylinder(.006, .002, (0., -.002, 0.), axis='y')
    head = m.lathe(((0., .0038), (HEAD_RADIUS, .0038), (HEAD_RADIUS, .0054),
                    (.0028, .0087), (0., .0087))).aimed('+Y')
    return tuple((f'axle_{name}', mesh.moved(at), 'steel')
                 for name, mesh in (('stem', stem), ('shoulder', shoulder), ('head', head)))


def contacts(parts):
    return tuple(m.col_hull(name + '_contact', mesh) for name, mesh, _ in parts)


def roller(at, start_quat=None):
    tire = segments(.0051, RADIUS, 0., 360., -TYRE_HALF_WIDTH, TYRE_HALF_WIDTH, 20)
    frame = tuple((f'tire_{i}', mesh, 'nylon') for i, mesh in enumerate(tire))
    clips = []
    for k, center in enumerate((0., 180.)):
        walls = segments(.0049, .006, center - 40., center + 40., -.0062, -.0007, 5)
        lips = segments(LIP_BORE_RADIUS, .006, center - 40., center + 40.,
                        LIP_CENTER - .0006, LIP_CENTER + .0006, 5)
        parts = tuple((f'clip_{k}_{i}', mesh, 'nylon') for i, mesh in enumerate(walls + lips))
        drop_in = (0., .0026) if k == 0 else (-.0026, 0.)
        clips.append(m.slider(f'hub_lip_{k}', 'X', *drop_in, parts, contacts(parts),
                              spring=2800., damping=.30, effort=15., mass=.0008, armature=1.e-6))
    marker = m.box((.004, .0004, .005), (.011, TYRE_HALF_WIDTH + .0002, .004))
    tread_mark = band(RADIUS + .0001, RADIUS + .00035, 6., 19., -.004, .004)
    marks = (('spoke_witness', marker, 'witness'), ('tread_witness', tread_mark, 'witness'))
    return m.mechanism('replacement_roller', at, clips, LOOKS,
                       frame + marks, contacts(frame),
                       free_mass=.012, frame_inertia=(1.9e-6, 2.2e-6, 1.9e-6),
                       friction=(.61, .45), start_quat=start_quat)


def component_fixture(at):
    steel = axle_parts()
    track = ('level_track', m.box((.100, .036, .008), (0., .004, -RADIUS - .004)), 'steel')
    parts = (*steel, track)
    return m.fixed('original_rack_axle', at, parts, contacts(parts), LOOKS, friction=(.58, .41))


def nominal_clearances():
    return {
        'bare_stem_radial_clearance_mm': 1000. * (LIP_BORE_RADIUS - STEM_RADIUS),
        'head_overlap_mm': 1000. * (HEAD_RADIUS - LIP_BORE_RADIUS),
        'hub_travel_mm': 2.6,
        'axial_shoulder_clearance_mm': 1000. * (.0038 - (SEATED_Y + LIP_CENTER + .0006)),
        'wheel_diameter_mm': 2000. * RADIUS,
        'estimated_radial_spring_load_n_at_head': 2800. * (HEAD_RADIUS - LIP_BORE_RADIUS),
    }
