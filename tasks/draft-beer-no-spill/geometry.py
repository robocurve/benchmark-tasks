"""Original dimensioned CQ reconstruction, hollow glass and portable counter frame."""
import json
import math
from pathlib import Path

import numpy as np
from .scoring import INNER_RADIUS, BASE, RIM, TARGET_ML
from .pump import body_frames, joint_definitions, LEVER_PIVOT
from .cq_assets import neck_path

TABLE = .7475
MOUNT = np.array([.18, -.10, TABLE+.265])
YAW = -.9
ROTATION = np.array([[math.cos(YAW),-math.sin(YAW),0.],
                     [math.sin(YAW), math.cos(YAW),0.],[0.,0.,1.]])
NOZZLE_LOCAL = np.array([-.220,-.040,-.040])
NOZZLE = MOUNT + ROTATION @ NOZZLE_LOCAL
PIVOT = MOUNT + ROTATION @ LEVER_PIVOT
GLASS_START = (.26,.16,TABLE)
GLASS_YAW = -1.5
HANDLE_POINT = np.array([0.,0.,.300])-LEVER_PIVOT


def shape(kind, size, position=(0,0,0), color=(.65,.68,.70,1), **kw):
    return dict(type=kind,size=list(size),position=list(position),color=list(color),**kw)


def part(name, shapes, mass, position=(0,0,0)):
    return dict(name=name,shapes=shapes,mass=mass,position=list(position),
                inertia=dict(diagonal=[mass*.003,mass*.003,mass*.002]))


def qmul(a,b):
    w,x,y,z=a;v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]


def ring_wall(z0,z1,outer,inner):
    walls=[]
    r=(outer+inner)/2
    for i in range(24):
        a=2*math.pi*i/24
        walls.append(shape('box',[(outer-inner)/2,math.pi*r/24,(z1-z0)/2],
            [NOZZLE_LOCAL[0]+r*math.cos(a),NOZZLE_LOCAL[1]+r*math.sin(a),(z0+z1)/2],
            orientation=[math.cos(a/2),0,0,math.sin(a/2)],visible=False))
    return walls


def geometry():
    glass_color = (.75, .88, .94, .22)
    walls = [shape('cylinder', [.040, BASE/2], (0, 0, BASE/2), glass_color)]
    for i in range(48):
        a, b = 2*math.pi*i/48, 2*math.pi*(i+1)/48
        points = [[r*math.cos(t), r*math.sin(t), z]
                  for z in (BASE, RIM) for r in (INNER_RADIUS, .040) for t in (a, b)]
        walls.append(shape('mesh', [], color=glass_color,
            mesh=dict(points=points, faces=[[0,1,3,2], [4,6,7,5], [0,4,5,1],
                [2,3,7,6], [0,2,6,4], [1,5,7,3]]), roughness=.05))
    # A normal glass mug handle provides a grippable feature for parallel fingers.
    walls.extend([shape('box', [.008,.023,.006], (0,.057,z), glass_color)
                  for z in (.045,.100)])
    walls.append(shape('box', [.008,.006,.0335], (0,.074,.0725), glass_color))
    line_z = BASE + TARGET_ML/1e6/(math.pi*INNER_RADIUS**2)
    for i in range(48):
        angle=2*math.pi*i/48
        walls.append(shape('box', [.0015,.001,.0005],
            (.0405*math.cos(angle),.0405*math.sin(angle),line_z),
            (.95,.95,.95,1), collision=False,
            orientation=[math.cos(angle/2),0,0,math.sin(angle/2)]))
    glass=part('glass',walls,.35)
    glass['center_of_mass']=[0.,.010,.056]
    glass['inertia']={'diagonal':[.00060,.00055,.00045]}
    root=Path(__file__).parent
    pieces=json.loads((root/'assets/cq/PARTS.json').read_text())
    shapes={name:[] for name in ('frame','lever','cylinder','piston')}
    for name,data in pieces.items():
        collide=data['body'] in ('lever','cylinder','piston')
        shapes[data['body']].append(shape('mesh',[],mesh_file=data['path'],
            color=data['color'],roughness=data['roughness'],metallic=data['metallic'],
            collision=collide,collision_approximation='convexDecomposition'))
    frame=shapes['frame']
    # Compound hood leaves the actual lever slot open.
    for centre,half in [((-.015,-.0660375,.019),(.105,.0260375,.019)),
                        ((-.015,.0660375,.019),(.105,.0260375,.019)),
                        ((.073,0,.019),(.017,.040,.019))]:
        frame.append(shape('box',half,centre,visible=False))
    # Side plates and clamp jaws; the cylinder moves between the plates.
    for side in (-1,1):
        frame.append(shape('box',[.0605,.0015,.1256],[-.002,side*.06994,-.1316],visible=False))
        frame.append(shape('box',[.062,.006,.009],[.062,side*.078,-.056],visible=False))
    frame.append(shape('box',[.0015,.07144,.1256],[-.061,0,-.1316],visible=False))
    neck=neck_path()
    # Solid capsules stop above the lower straight hollow outlet.
    end=np.flatnonzero(np.linalg.norm(neck-[*NOZZLE_LOCAL[:2],.102],axis=1)<1e-9)[0]
    neck=neck[:end+1]
    path=np.vstack([neck[::3],neck[-1]])
    for a,b in zip(path[:-1],path[1:]):
        frame.append(shape('capsule',[.008,0.],fromto=[*a,*b],visible=False))
    frame.extend(ring_wall(-.034,.102,.008,.0065))
    frame.extend(ring_wall(-.040,-.034,.010,.0065))
    # The original drip tray remains a spill surface, never a credited vessel.
    frame.append(shape('box',[.095,.079,.0015],[-.170,0,-.2385],visible=False))
    for y in np.linspace(-.070,.070,11):
        frame.append(shape('box',[.0925,.002,.0015],[-.170,y,-.222475],visible=False))
    for x in np.linspace(-.260,-.080,13):
        frame.append(shape('box',[.002,.0715,.0015],[x,0,-.222475],visible=False))
    wood=(.23,.075,.03,1)
    # Portable open counter frame: 25 mm top, 265 mm above the station deck.
    frame.append(shape('box',[.075,.165,.0125],[.150,0,-.0125],wood))
    for side in (-1,1):
        frame.append(shape('box',[.1775,.0175,.0075],[.0675,side*.14,-.2575],wood))
        for x in (.100,.205):
            frame.append(shape('box',[.0175,.0175,.1125],[x,side*.14,-.1375],wood))
    bodies=[]
    frames={'frame':dict(position=[0.,0.,0.],orientation=[1.,0.,0.,0.]),**body_frames()}
    mount_q=[math.cos(YAW/2),0.,0.,math.sin(YAW/2)]
    for name,mass in [('frame',7.),('lever',.32),('cylinder',.55),('piston',.08)]:
        f=frames[name]
        body=part(name,shapes[name],mass,MOUNT+ROTATION@np.asarray(f['position']))
        body['orientation']=qmul(mount_q,f['orientation'])
        if name=='lever':
            body['center_of_mass']=[0.,0.,.25]
            body['inertia']={'diagonal':[.006,.006,.0005]}
        elif name=='piston':
            body['center_of_mass']=[0.,0.,-.085]
            body['inertia']={'diagonal':[.00022,.00022,.00001]}
        bodies.append(body)
    return {'glass':dict(name='Glass',bodies=[glass],solver_position_iterations=16,
                         solver_velocity_iterations=4),
            'tap':dict(name='CQ',bodies=bodies,articulation=True,fixed_base='frame',
                       self_collisions=False,solver_position_iterations=16,
                       solver_velocity_iterations=4,joints=joint_definitions())}
