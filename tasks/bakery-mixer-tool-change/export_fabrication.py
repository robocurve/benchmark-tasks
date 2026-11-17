#!/usr/bin/env python3
"""Export millimetre-scale manufacturing meshes; does not execute physics.

Install fabrication/requirements.txt in a separate environment. Native scene
files are read only. Inspect/ream prints and commission the apparatus as described
in PHYSICAL_BUILD.md; a valid mesh is not a validated physical mechanism.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path

import manifold3d as mf
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import trimesh as tm

ROOT = Path(__file__).resolve().parent


def solid(mesh):
    if not mesh.is_volume:
        raise ValueError("Input mesh is not a closed, consistently wound solid")
    return mf.Manifold(mf.Mesh(np.asarray(mesh.vertices, dtype=np.float32),
                              np.asarray(mesh.faces, dtype=np.uint32)))


def clean(result):
    result.merge_vertices()
    # Boolean seams can collapse to exactly zero-area triangles on STL's float32 grid.
    result.update_faces(result.nondegenerate_faces(height=0))
    result.update_faces(result.unique_faces())
    result.remove_unreferenced_vertices()
    return result


def mesh(model):
    value = model.to_mesh()
    return clean(tm.Trimesh(np.asarray(value.vert_properties)[:, :3],
                            np.asarray(value.tri_verts), process=True))


def shape_mesh(shape):
    kind = shape['type']
    if kind == 'mesh':
        faces = [[f[0], f[i], f[i+1]] for f in shape['mesh']['faces']
                 for i in range(1, len(f)-1)]
        result = tm.Trimesh(np.round(np.asarray(shape['mesh']['points'])*1000, 6),
                            faces, process=True)
    elif kind == 'box':
        result = tm.creation.box(np.asarray(shape['size'])*2000)
    elif kind == 'cylinder':
        result = tm.creation.cylinder(radius=shape['size'][0]*1000,
                                      height=shape['size'][1]*2000, sections=64)
    elif kind == 'capsule':
        a, b = np.asarray(shape['fromto']).reshape(2, 3)*1000
        direction = b-a
        result = tm.creation.capsule(height=np.linalg.norm(direction),
                                     radius=shape['size'][0]*1000, count=[12, 24])
        result.apply_transform(tm.geometry.align_vectors([0, 0, 1], direction))
        result.apply_translation((a+b)/2)
    else:
        raise ValueError('Unsupported authored shape: '+kind)
    if 'orientation' in shape:
        result.apply_transform(tm.transformations.quaternion_matrix(shape['orientation']))
    if 'position' in shape:
        result.apply_translation(np.asarray(shape['position'])*1000)
    return result


def union_shapes(shapes):
    return mf.Manifold.batch_boolean([solid(shape_mesh(s)) for s in shapes], mf.OpType.Add)


def cylinder(radius, height, z=0):
    return mf.Manifold.cylinder(height, radius, circular_segments=96).translate([0, 0, z])


def housing_and_mandrel():
    wall = cylinder(50, 88)-cylinder(48, 90, -1)
    bottom = cylinder(50, 2)-cylinder(40, 4, -1)
    flange = cylinder(65, 12, 12)-cylinder(40, 14, 11)
    pod = wall+bottom+flange
    # First-coupon mandrel only: steel-wire springback must be measured before winding the final spring.
    mandrel = cylinder(8.6, 42)+cylinder(13, 3)
    mandrel -= cylinder(3.1, 44, -1)
    return pod, mandrel


def bearing_housing(length, lower_end, upper_start, flange=False):
    housing = cylinder(13, length)-cylinder(6.2, length+2, -1)
    housing -= cylinder(10.48, lower_end+1, -1)
    housing -= cylinder(10.48, length-upper_start+1, upper_start)
    if flange:
        housing += cylinder(19, 3, length-3)-cylinder(13, 5, length-4)
        for angle in [0, 120, 240]:
            a = math.radians(angle)
            housing -= cylinder(1.6, 5, length-4).translate([16.5*math.cos(a), 16.5*math.sin(a), 0])
    return housing


def spin_pulley(bore):
    """24 mm belt pitch diameter, 2 mm round belt, 4 mm wide wheel.

    The set screw is drilled/tapped after printing; the 12 mm version uses a
    low-profile split shaft collar bolted to the wheel instead of a tiny web.
    """
    wheel = cylinder(12.5, 4)
    groove = tm.creation.torus(major_radius=12, minor_radius=1.05,
                              major_sections=96, minor_sections=24)
    groove.apply_translation([0, 0, 2])
    wheel -= solid(groove)
    wheel -= cylinder(bore/2, 6, -1)
    for x in [-8.5, 8.5]:
        wheel -= cylinder(1.1, 6, -1).translate([x, 0, 0])
    return wheel


def ballast_parts(beater):
    """Captive steel webs match mass without filling or altering the bayonet hub."""
    for y, z in [(15, -105), (25, -105), (-15, -105), (-25, -105), (0, -102.5), (0, -117.5)]:
        beater -= cylinder(1.6, 30).rotate([0, 90, 0]).translate([-15, y, z])
    large = mf.Manifold.cube([6, 20, 80], center=True)
    for y in [-5, 5]:
        large -= cylinder(1.6, 10).rotate([0, 90, 0]).translate([-5, y, 20])
    small = mf.Manifold.cube([6, 20, 25], center=True)
    for z in [-7.5, 7.5]:
        small -= cylinder(1.6, 10).rotate([0, 90, 0]).translate([-5, 0, z])
    assembly = mf.Manifold.compose([beater, large.translate([8, 20, -125]),
                                   large.translate([-8, -20, -125]), small.translate([8, 0, -110])])
    return beater, large, small, assembly


def lift_parts():
    """Matched 20-degree involute pinion/rack, 3.125 mm circular pitch."""
    teeth, pitch, pressure, backlash = 96, 3.125, math.radians(20), .15
    module = pitch/math.pi
    rp = teeth*pitch/(2*math.pi)
    rb, rr, ra = rp*math.cos(pressure), rp-1.25*module, rp+module
    involute_pitch = math.tan(pressure)-pressure
    half_tooth = math.pi/(2*teeth)-backlash/(2*rp)
    outline = []
    for tooth in range(teeth):
        centre = tooth*2*math.pi/teeth
        for side, radii in [(-1, np.linspace(rr, ra, 7)), (1, np.linspace(ra, rr, 7))]:
            for radius in radii:
                angle = math.acos(rb/radius)
                offset = half_tooth+involute_pitch-(math.tan(angle)-angle)
                a = centre+side*offset
                outline.append([radius*math.cos(a), radius*math.sin(a)])
    pinion = mf.CrossSection([outline]).extrude(8)
    pinion -= cylinder(4.1, 10, -1)
    for x in [-15, 15]:
        pinion -= cylinder(1.6, 10, -1).translate([x, 0, 0])
    root_y, top_y = 6-1.25*module, 6+module
    rack = [[0, 0], [150, 0], [150, root_y]]
    half_pitch = (pitch/2-backlash)/2
    for tooth in reversed(range(48)):
        centre = (tooth+.5)*pitch
        root_half = half_pitch+1.25*module*math.tan(pressure)
        tip_half = half_pitch-module*math.tan(pressure)
        rack.extend([[centre+root_half, root_y], [centre+tip_half, top_y],
                     [centre-tip_half, top_y], [centre-root_half, root_y]])
    rack.append([0, root_y])
    return pinion, mf.CrossSection([rack]).extrude(8)


def lift_checks(pinion, rack):
    rp = 96*3.125/(2*math.pi)
    samples = sorted(set([float(a) for a in np.linspace(0, 90, 37)]+[.1, .8, 1.6, 3.2, 5.3, 8.1]))
    volumes = []
    for angle in samples:
        gear = pinion.rotate([0, 0, -angle]).translate([105, 180, 0])
        moving = rack.transform([[0, 1, 0, 105-rp-6],
                                 [1, 0, 0, 80+rp*math.radians(angle)], [0, 0, 1, 0]])
        volumes.append((gear ^ moving).volume())
    if max(volumes) > .02 or not math.isclose(rp*math.pi/2, 75, abs_tol=1e-9):
        raise ValueError('Printed lift gear/rack CAD interference or travel error')
    return dict(angles_degrees=samples, max_intersection_mm3=max(volumes), quarter_turn_travel_mm=rp*math.pi/2,
                circular_pitch_mm=3.125, pressure_angle_degrees=20, tooth_backlash_mm=.15)


def core_checks(beater, shaft):
    # Tests the exported geometry, not simulated contact or an invented receipt.
    checks = {}
    for yaw in [0, -15, -30, -45, -60]:
        candidate = beater.rotate([0, 0, yaw]).translate([0, 0, -1])
        volume = (candidate ^ shaft).volume()
        if volume > .02:
            raise ValueError(f'Insertion/twist CAD interference at {yaw} deg: {volume} mm3')
        checks[f'inserted_twist_{yaw}_intersection_mm3'] = volume
    retained = beater.rotate([0, 0, -60]).translate([0, 0, -6])
    checks['retained_intersection_mm3'] = (retained ^ shaft).volume()
    checks['straight_down_1mm_interference_mm3'] = (retained.translate([0, 0, -1]) ^ shaft).volume()
    if checks['retained_intersection_mm3'] > .02 or checks['straight_down_1mm_interference_mm3'] < .1:
        raise ValueError('Pin/pocket CAD retention test failed')
    probe = cylinder(6.20, 39.8, -19.9)
    checks['hub_bore_probe_intersection_mm3'] = (beater ^ probe).volume()
    if checks['hub_bore_probe_intersection_mm3'] > .02:
        raise ValueError('Manufacturing mesh obstructs the shaft bore')
    return checks


def preview(parts, output):
    canvas = Image.new('RGB', (1600, 1000), 'white')
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 24)
        small = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 18)
    except OSError:
        font = small = ImageFont.load_default()
    draw.text((24, 12), 'Manufacturing meshes — CAD preview, not simulator footage', fill='black', font=font)
    xaxis = np.array([1., -1., 0.])/math.sqrt(2)
    depth = np.array([1., 1., .8]); depth /= np.linalg.norm(depth)
    yaxis = np.cross(xaxis, depth)
    palette = [(56, 117, 164), (36, 139, 128), (186, 122, 42), (110, 96, 160), (125, 130, 139)]
    for index, (name, part) in enumerate(parts.items()):
        m = mesh(part)
        col, row = index % 3, index//3
        left, top = col*530+15, row*465+60
        points = np.column_stack((m.vertices@xaxis, -m.vertices@yaxis))
        lo, hi = points.min(axis=0), points.max(axis=0)
        scale = min(490/max(hi[0]-lo[0], 1), 365/max(hi[1]-lo[1], 1))
        points = (points-(lo+hi)/2)*scale+[left+250, top+205]
        order = np.argsort(m.triangles_center@depth)
        light = np.array([.3, -.4, .86]); light /= np.linalg.norm(light)
        intensity = .45+.55*np.maximum(m.face_normals@light, 0)
        color = np.asarray(palette[index % len(palette)])
        for face in order:
            draw.polygon([tuple(p) for p in points[m.faces[face]]],
                         fill=tuple((color*intensity[face]).astype(int)))
        dimensions = ' x '.join(f'{v:.1f}' for v in m.extents)
        draw.text((left+10, top+395), name.replace('_', ' '), fill='black', font=font)
        draw.text((left+10, top+425), dimensions+' mm', fill='black', font=small)
    canvas.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'fabrication')
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    native = ROOT/'geometry.json'
    descriptions = json.loads(native.read_text())
    beater = union_shapes(descriptions['beater']['bodies'][0]['shapes'])
    beater, large_ballast, small_ballast, ballasted = ballast_parts(beater)
    guard = union_shapes(descriptions['guard']['bodies'][0]['shapes'])
    bowl_body = next(b for b in descriptions['mixer']['bodies'] if b['name'] == 'bowl')
    bowl_shapes = [json.loads(json.dumps(bowl_body['shapes'][0]))]
    # A printable wall/floor needs a shared annular face, not the native edge contact.
    bowl_shapes[0]['size'][0] = .072
    bowl_shapes += [s for s in bowl_body['shapes'] if s['type'] == 'mesh']
    bowl = union_shapes(bowl_shapes)
    shaft_body = next(b for b in descriptions['mixer']['bodies'] if b['name'] == 'shaft')
    checks = core_checks(beater, union_shapes(shaft_body['shapes']))
    pinion, rack = lift_parts()
    gear_checks = lift_checks(pinion, rack)
    pod, mandrel = housing_and_mandrel()
    spindle_housing = bearing_housing(18, 5.5, 12.5)
    orbit_housing = bearing_housing(28, 7.5, 22.5, flange=True)
    motor_pulley, spindle_pulley = spin_pulley(3.1), spin_pulley(12.1)
    parts = dict(beater=beater, guard=guard, bowl=bowl, lift_pinion=pinion, lift_rack=rack,
                 metal_ballast_large=large_ballast, metal_ballast_small=small_ballast,
                 drive_pod=pod, spring_mandrel=mandrel,
                 spindle_bearing_housing=spindle_housing, orbit_bearing_housing=orbit_housing,
                 spin_motor_pulley=motor_pulley, spin_spindle_pulley=spindle_pulley)
    records = {}
    for name, model in list(parts.items()):
        pieces = model.decompose()
        slivers = [p.volume() for p in pieces if abs(p.volume()) < .0001]
        pieces = [p for p in pieces if abs(p.volume()) >= .0001]
        if len(pieces) != 1:
            raise ValueError(f'{name}: disconnected manufacturing solid ({len(pieces)} components)')
        model = pieces[0]
        parts[name] = model
        result = mesh(model)
        if not result.is_volume:
            raise ValueError(f'{name}: invalid manufacturing solid')
        translation = -float(result.bounds[0, 2])
        source_bounds = result.bounds.tolist()
        result.apply_translation([0, 0, translation])
        result.vertices = np.asarray(result.vertices, dtype=np.float32)
        result = clean(result)
        path = output/(name+'.stl'); result.export(path)
        loaded = tm.load_mesh(path, process=True)
        if not loaded.is_volume or not np.allclose(loaded.bounds, result.bounds, atol=.001):
            raise ValueError('STL round-trip failed: '+name)
        records[name] = dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            quantity=2 if name == 'metal_ballast_large' else 1,
            connected_solids=1, removed_boolean_sliver_volumes_mm3=slivers,
            watertight=bool(loaded.is_watertight),
            source_bounds_mm=source_bounds, print_translation_mm=[0, 0, translation],
            dimensions_mm=result.extents.tolist(), volume_cm3=float(result.volume/1000),
            material='mild steel; cut/drill, do not print' if name.startswith('metal_') else 'PETG',
            nominal_material_mass_g=float(result.volume/1000*(7.85 if name.startswith('metal_') else 1.25)))
        print(name, 'verified', np.round(result.extents, 3).tolist(), flush=True)
    preview(dict(beater=parts['beater'], beater_with_steel_ballast=ballasted, guard=parts['guard'],
                 bowl=parts['bowl'], lift_pinion=pinion, lift_rack=rack), output/'mesh-preview.png')
    steel_mass = 2*large_ballast.volume()/1000*7.85+small_ballast.volume()/1000*7.85
    if not 240 <= records['beater']['nominal_material_mass_g']+steel_mass+12 <= 260:
        raise ValueError('Beater ballast drawing does not meet nominal mass target')
    report = dict(scope='Manufacturing mesh checks only; no physics execution or hardware validation',
        source_geometry_sha256=hashlib.sha256(native.read_bytes()).hexdigest(),
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        versions={name:importlib.metadata.version(name) for name in ['trimesh','manifold3d','numpy','pillow']},
        units='millimetres', parts=records, static_coupling_checks=checks, static_lift_checks=gear_checks,
        beater_mass_estimate=dict(printed_body_g=records['beater']['nominal_material_mass_g'],
            steel_webs_g=steel_mass, fasteners_allowance_g=12,
            assembled_g=records['beater']['nominal_material_mass_g']+steel_mass+12,
            target_g=250, acceptance_g=[240, 260],
            note='Density estimate only; scale measurement decides. Physical beater centre of mass is about 110 mm below the hub; native lumped model assumes 80 mm.'),
        fabrication_changes=['Bowl floor outer radius 72 mm joins the 2 mm wall; cavity and floor height unchanged.',
                             'Six 3.2 mm ballast fastener holes are below the beater grip; the hub bore and L-slot are unchanged.',
                             'The bowl STL excludes the separate metal lift cradle and clamps.',
                             'Static draining platforms are cut from wood; do not print their solid native meshes.'],
        limitations='Watertight meshes and static interference checks do not establish print accuracy, strength, friction, drive margin or safe operation. Ream/inspect mating features, add captive ballast, and commission the finished apparatus.')
    (output/'mesh-validation.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
