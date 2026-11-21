"""Produce millimetre machinist masters in the hinge frame. Requires trimesh and manifold3d."""
import json
from pathlib import Path
import numpy as np
import trimesh as tm

ROOT=Path(__file__).resolve().parent
reference=json.loads((ROOT/'holder-faces-mm.json').read_text())
parts={}


def block(xyz, size):
    mesh=tm.creation.box(extents=size)
    return mesh.apply_translation(xyz)


def cylinder(xyz, diameter, length, axis='x'):
    mesh=tm.creation.cylinder(radius=diameter/2,height=length,sections=64)
    if axis=='x':
        mesh.apply_transform(tm.transformations.rotation_matrix(np.pi/2,[0,1,0]))
    elif axis!='z':
        raise ValueError(axis)
    return mesh.apply_translation(xyz)


def join(shapes):
    return tm.boolean.union(shapes,engine='manifold')


def cut(solid,tools):
    return tm.boolean.difference([solid,*tools],engine='manifold')


def register(name,mesh):
    mesh.remove_unreferenced_vertices()
    mesh.export(ROOT/(name+'-mm.stl'))
    actual=tm.load(ROOT/(name+'-mm.stl'))
    if not (actual.is_volume and actual.is_watertight and len(actual.split(only_watertight=False))==1):
        raise ValueError((name,len(actual.split(only_watertight=False))))
    parts[name]=actual


for side,sign in [('source',1),('receiver',-1)]:
    faces=reference[side+'_frame']
    solid=[]
    for name,p in faces.items():
        if name in ('knob_pin','pin_riser','outboard_stop','catch_wall0','catch_wall2'):
            continue
        verts=np.asarray(p)
        # Bring the underside only down to the bed; working surfaces retain nominal levels.
        if name.startswith(('rail','guide_throat')) or name=='inboard_stop':
            verts[verts[:,2]==verts[:,2].min(),2]= -3.70
        solid.append(tm.convex.convex_hull(verts))
    solid.append(block([sign*67,sign*43,-1.2],[8,9,10.6]))
    bores=[cylinder([sign*21,0,0],4.20,15),cylinder([sign*67,sign*43,0],4.,13)]
    if side=='source':
        # Ø2.2 clearance and 90° countersink, inserted from the bottom of the 3 mm bed.
        for x in (-19.6,19.6):
            recess=tm.creation.revolve([[0,-7],[2.41,-7],[1.10,-5.69],[1.10,-3.4],[0,-3.4]],sections=64)
            recess.apply_translation([x,42,0])
            bores.append(recess)
        for x in (-18,18):
            solid.append(block([x,70.,-5.14],[8,9,3]))
            bores.append(cylinder([x,71,-5.14],2.20,5,'z'))
        stop=block([0,70.75,-2.145],[48,6.5,2.99])
        register('source-removable-stop',cut(stop,[cylinder([x,71,-2.145],1.60,5,'z') for x in (-18,18)]))
    else:
        for s in (-1,1):
            # Split bore, retaining shoulder and outer threaded spring seat in a single socket.
            y,z=-55,-2.65
            solid.append(block([50+s*13.1,y,z],[16.2,7.5,6.8]))
            # Wall lugs connect the cartridges to the back bridge but leave the central throat open.
            solid.append(block([50+s*9,y+4.3,-4.9],[8,5,4.6]))
            bores += [cylinder([50+s*5.45,y,z],3.,2.0),
                      cylinder([50+s*10.05,y,z],4.10,7.70),
                      cylinder([50+s*17.7,y,z],4.20,7.7)]
    register(side+'-carrier',cut(join(solid),bores))

base=[block([0,0,-37.5],[110,140,6])]
holes=[]
for s in (-1,1):
    base.append(block([s*42,0,-14.8],[9,10,39.8]))
    holes.append(cylinder([s*42,0,0],4.20,12))
    base.append(block([0,s*58,-20.7],[20,12,28.08]))
    for x in (-48,48):
        holes.append(cylinder([x,s*62,-37.5],4.5,10,'z'))
register('bench-base',cut(join(base),holes))
register('rotating-grip',cut(block([0,0,0],[18,17,16]),[cylinder([0,0,0],4.22,22)]))
# Redrawable moving detent with hemispherical nose, guided shank and captive shoulder.
# Local x grows outwards from the catch centreline. Other side uses the mirror mounting.
ball=tm.creation.icosphere(subdivisions=3,radius=1.4).apply_translation([3.1,0,0])
plunger=join([ball,cylinder([4.71,0,0],2.8,3.24),cylinder([7.2,0,0],3.8,2.)])
register('detent-plunger',plunger)
register('pedestal',block([0,0,12.45],[26,26,24.9]))

def square_pocket_ring(x,y,side,z):
    return [[x+dx*side/2,y+dy*side/2,z] for dx,dy in ((-1,-1),(1,-1),(1,1),(-1,1))]

# Two identical trays. The receiver has unused blind mounting holes on its underside.
outline=[(-25.4,-25.4),(25.4,-25.4),(25.4,22.4),(22.4,25.4),(-25.4,25.4)]
flange=tm.convex.convex_hull([[x,y,z] for z in (-3.5,-1.45) for x,y in outline])
blank=join([flange,block([0,0,-.725],[44.4,44.4,1.45])])
holes=[cylinder([x,0,-2.40],1.60,2.40,'z') for x in (-19.6,19.6)]
for x in (-13,0,13):
    for y in (-13,0,13):
        holes.append(block([x,y,-.725],[10,10,.55]))
        holes.append(tm.convex.convex_hull(square_pocket_ring(x,y,10,-.45)+
                                            square_pocket_ring(x,y,11.10,.10)))
register('stepped-keyed-tray',cut(blank,holes))

if __name__=='__main__':
    print('Checked single closed solids:', ', '.join(parts))
