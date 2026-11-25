"""Real-size compound furniture; four free two-interface sliders; no attachment logic."""
import os
import json
from pathlib import Path
import numpy as np
from config import S, F, P, FLOOR_Z, FOOT_CENTERS
from partkit import part

MATERIALS = {k: S['materials'][k] for k in ('foot', 'foam', 'felt', 'plastic', 'floor')}
WHITE = (.83, .84, .82)
FOAM = (.25, .23, .20)
PLASTIC = (.56, .49, .39)
FELT = (.72, .65, .49)


def furniture_inertia():
    """Explicit approximate hollow-product mass allocation, never auto uniform solid density."""
    w, d, h = F['size_m']
    t = F['top_thickness_m']
    m = F['mass_kg']
    mt = m * F['top_mass_fraction']
    ml = (m - mt) / 4
    dimensions = [np.array((w, d, t))] + [np.array((F['foot_width_m'], F['foot_width_m'], h-t))] * 4
    centers = [np.array((0, 0, h-t/2))] + [np.array((x, y, (h-t)/2)) for x,y,_ in FOOT_CENTERS]
    masses = [mt] + [ml] * 4
    com = sum(mi*pi for mi,pi in zip(masses, centers)) / m
    inertia = np.zeros(3)
    for mi, size, pi in zip(masses, dimensions, centers):
        delta = pi - com
        inertia += mi / 12 * (np.sum(size**2) - size**2) + mi*(delta@delta - delta**2)
    return tuple(com), tuple(inertia)


def build():
    w, d, h = F['size_m']
    t, fw = F['top_thickness_m'], F['foot_width_m']
    boxes = [((0, 0, h-t/2), (w, d, t), WHITE, {'material':'foot'})]
    boxes += [((x, y, (h-t)/2), (fw, fw, h-t), WHITE, {'material':'foot'}) for x,y,_ in FOOT_CENTERS]
    com, inertia = furniture_inertia()
    start=(*S['layout']['start_xy_m'], FLOOR_Z + .0002)
    quat=None
    contact_debug=os.environ.get('FURNITURE_SLIDERS_CONTACT_DEBUG')=='1'
    preinstalled=int(os.environ.get('FURNITURE_SLIDERS_PREINSTALLED_PAD_COUNT','0'))
    if contact_debug:preinstalled=1
    if preinstalled not in (0,1,2):raise ValueError('Invalid explicit diagnostic initialization')
    if preinstalled:
        from config import PAD_H
        angle=np.arcsin(PAD_H/F['foot_spacing_m'])
        start=(*S['layout']['start_xy_m'],FLOOR_Z+(F['foot_spacing_m']+fw)/2*np.sin(angle)+.0002)
        quat=(0.,np.sin(angle/2),0.,np.cos(angle/2))
    reset_path=os.environ.get('FURNITURE_SLIDERS_RESET_STATE')
    reset=json.loads(Path(reset_path).read_text())['measurement'] if reset_path else None
    if reset:
        start=tuple(reset['furniture'][:3]);quat=tuple(reset['furniture'][3:])
    furniture = part('furniture', start,
                     boxes=boxes, mass=F['mass_kg'], materials=MATERIALS,
                     center_of_mass=com, diagonal_inertia=inertia,quat=quat)
    # One continuous static mesh, preserving the two stock-mount cutouts.
    # Touching box colliders snagged a pad on their artificial x=0.10 seam.
    from floor_geometry import panel_mesh
    vertices,faces=panel_mesh(S['floor']['center_m'],S['floor']['size_m'])
    floor_boxes=[]
    # Visual-only destination marking: cannot trap, support or funnel feet.
    goal = np.asarray(S['layout']['goal_xy_m']) - np.asarray(S['floor']['center_m'][:2])
    gx, gy = S['layout']['goal_size_m']
    z = S['floor']['size_m'][2]/2 + .0001
    for sign in (-1, 1):
        floor_boxes += [((goal[0]+sign*gx/2, goal[1], z), (.002,gy,.0002), (.05,.45,.16), {'visual':True}),
                        ((goal[0], goal[1]+sign*gy/2, z), (gx,.002,.0002), (.05,.45,.16), {'visual':True})]
    floor = part('floor_panel', S['floor']['center_m'], boxes=floor_boxes, mass=None,
                 body_mass=6, materials=MATERIALS,
                 meshes=[(vertices,faces,(.60,.42,.25),{'material':'floor','approximation':'none'})])
    pads = []
    count = int(os.environ.get('FURNITURE_SLIDERS_PAD_COUNT', '4'))
    if count not in (1,4):
        raise ValueError('Only the one-pad gate or the four-pad task is supported')
    for i, xy in enumerate(S['layout']['pads_xy_m'][:count]):
        if i<preinstalled:
            xy=(start[0]-F['foot_spacing_m']/2*np.cos(angle),(1 if i==0 else -1)*F['foot_spacing_m']/2)
        layers, z0 = [], 0.
        for key, color, radius in [('felt', FELT, P['diameter_m']/2), ('backing', PLASTIC, P['diameter_m']/2),
                                   ('foam', FOAM, P['usable_foam_radius_m'])]:
            height = P[key+'_thickness_m']
            layers.append(((0,0,z0+height/2), radius, height, color, 'Z',
                           {'material':'plastic' if key=='backing' else key}))
            z0 += height
        pad_pose=reset['pads'][f'slider_{i}'] if reset else [*xy,FLOOR_Z+.0002,0.,0.,0.,1.]
        pads.append(part(f'slider_{i}', pad_pose[:3], quat=tuple(pad_pose[3:]), cylinders=layers,
                         mass=P['mass_kg'], materials=MATERIALS))
    return [furniture, floor, *pads]
