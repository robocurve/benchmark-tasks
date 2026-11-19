"""Generate an original, dimensioned Angram CQ reconstruction, in metres.

This is reference-based original CAD, not manufacturer CAD. Published overall
dimensions are approximate; undimensioned cosmetics and internal parts are
estimates. No reference drawings, photographs or brand marks are bundled.
The wood texture is an original procedural material.
Run this file to rebuild the retained OBJ/MTL assets; importing it writes nothing.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import zlib

import numpy as np

ASSET_DIR = Path(__file__).parent / 'assets' / 'cq'
PIVOT = np.array([0., 0., -.035])
CYLINDER_PIVOT = np.array([0., 0., -.230])
CRANK_PIN = np.array([.055, 0., 0.])
NOZZLE = np.array([-.220, -.040, -.040])
TRAY_TOP = -.220975
HANDLE_TOP = .41275


class Mesh:
    def __init__(self):
        self.vertices = []
        self.faces = []

    def add(self, vertices, faces, rotation=None, position=(0., 0., 0.)):
        vertices = np.asarray(vertices, dtype=float)
        if rotation is not None:
            vertices = vertices @ np.asarray(rotation).T
        offset = len(self.vertices)
        self.vertices.extend((vertices + position).tolist())
        self.faces.extend([[offset + int(i) for i in face] for face in faces])

    def lathe(self, profile, position=(0., 0., 0.), rotation=None, sides=96):
        vertices = [[r*math.cos(a), r*math.sin(a), z]
                    for r, z in profile for a in np.arange(sides)*2*math.pi/sides]
        faces = [[i*sides+j, i*sides+(j+1)%sides,
                  (i+1)*sides+(j+1)%sides, (i+1)*sides+j]
                 for i in range(len(profile)-1) for j in range(sides)]
        faces += [list(range(sides-1, -1, -1)),
                  list(range((len(profile)-1)*sides, len(profile)*sides))]
        self.add(vertices, faces, rotation, position)

    def cylinder(self, radius, length, position, rotation=None, sides=64):
        self.lathe([(radius, -length/2), (radius, length/2)], position, rotation, sides)

    def rounded_box(self, size, position, radius=.003, rotation=None):
        x, y, z = np.asarray(size)/2
        bevel = min(radius, z*.8, x*.4, y*.4)
        loops = [rounded_rectangle(x-bevel, y-bevel, max(radius-bevel, .0002)),
                 rounded_rectangle(x, y, radius),
                 rounded_rectangle(x, y, radius),
                 rounded_rectangle(x-bevel, y-bevel, max(radius-bevel, .0002))]
        heights = [-z, -z+bevel, z-bevel, z]
        vertices = [[a, b, h] for loop, h in zip(loops, heights) for a, b in loop]
        n = len(loops[0])
        faces = [[k*n+j, k*n+(j+1)%n, (k+1)*n+(j+1)%n, (k+1)*n+j]
                 for k in range(3) for j in range(n)]
        faces += [list(range(n-1, -1, -1)), list(range(3*n, 4*n))]
        self.add(vertices, faces, rotation, position)

    def ring(self, outer, inner, z0, z1, centre=(0., 0.), inner_centre=(0., 0.)):
        loops = []
        for h in (z0, z1):
            for size, shift in ((outer, centre), (inner, inner_centre)):
                loops.append([[x+shift[0], y+shift[1], h]
                              for x, y in rounded_rectangle(*size)])
        n = len(loops[0]); vertices = sum(loops, [])
        faces = []
        for j in range(n):
            k = (j+1)%n
            faces += [[j,k,2*n+k,2*n+j], [n+k,n+j,3*n+j,3*n+k],
                      [j,n+j,n+k,k], [2*n+j,2*n+k,3*n+k,3*n+j]]
        self.add(vertices, faces)

    def hood(self):
        # The undimensioned clearance is an original estimate, not factory CAD.
        bottom = u_outline(-.120, .090, .092075, .056, .040, .012, .004)
        top = u_outline(-.114, .084, .086075, .060, .044, .006, .004)
        rings = [(bottom, 0.), (bottom, .031), (top, .038)]
        n = len(bottom)
        vertices = [[x,y,z] for loop,z in rings for x,y in loop]
        faces = [[k*n+j,k*n+(j+1)%n,(k+1)*n+(j+1)%n,(k+1)*n+j]
                 for k in range(2) for j in range(n)]
        faces += [list(reversed(f)) for f in triangulate_polygon(bottom)]
        faces += [[i+2*n for i in f] for f in triangulate_polygon(top)]
        self.add(vertices,faces)

    def tube(self, path, outer, inner, sides=48):
        path = np.asarray(path)
        tangent = np.gradient(path, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        # The swan neck lies in XZ; one cross-section axis remains along Y.
        across = np.tile([0., 1., 0.], (len(path), 1))
        radial = np.cross(tangent, across)
        vertices = []
        for radius in (outer, inner):
            for p, a, b in zip(path, across, radial):
                for angle in np.arange(sides)*2*math.pi/sides:
                    vertices.append(p+radius*(math.cos(angle)*a+math.sin(angle)*b))
        n = len(path)*sides; faces = []
        for layer in (0, 1):
            for i in range(len(path)-1):
                for j in range(sides):
                    k = (j+1)%sides
                    f = [layer*n+i*sides+j, layer*n+i*sides+k,
                         layer*n+(i+1)*sides+k, layer*n+(i+1)*sides+j]
                    faces.append(f if layer == 0 else f[::-1])
        for end in (0, len(path)-1):
            for j in range(sides):
                k = (j+1)%sides
                f = [end*sides+j, n+end*sides+j, n+end*sides+k, end*sides+k]
                faces.append(f if end == 0 else f[::-1])
        self.add(vertices, faces)

    def save(self, name, material, texture=None):
        vertices = np.asarray(self.vertices)
        if not np.isfinite(vertices).all() or not self.faces:
            raise ValueError('Invalid original CAD mesh')
        normals = np.zeros_like(vertices)
        for face in self.faces:
            if len(face) < 3 or min(face)<0 or max(face)>=len(vertices):
                raise ValueError('Invalid CAD polygon')
            normal = np.cross(vertices[face[1]]-vertices[face[0]],
                              vertices[face[2]]-vertices[face[0]])
            normals[face] += normal
        length = np.linalg.norm(normals, axis=1)
        if np.any(length < 1e-12):
            raise ValueError(f'Undefined vertex normals in {name}')
        normals /= length[:, None]
        lines = ['# Original reference-based CQ reconstruction; metres; MIT.',
                 '# Not manufacturer CAD.', f'mtllib {name}.mtl', f'usemtl {name}']
        lines += ['v '+' '.join(f'{v:.9f}' for v in point) for point in vertices]
        if texture:
            span = np.maximum(np.ptp(vertices[:, :2], axis=0), 1e-9)
            uv = (vertices[:, :2]-vertices[:, :2].min(axis=0))/span
            lines += [f'vt {u:.8f} {v:.8f}' for u,v in uv]
        lines += ['vn '+' '.join(f'{v:.8f}' for v in point) for point in normals]
        lines += ['f '+' '.join(f'{i+1}/{i+1}/{i+1}' if texture else f'{i+1}//{i+1}'
                               for i in face) for face in self.faces]
        (ASSET_DIR/(name+'.obj')).write_text('\n'.join(lines)+'\n')
        mtl = [f'newmtl {name}', 'Ka 0.05 0.05 0.05',
               'Kd '+' '.join(str(v) for v in material['color'][:3]),
               'Ks 0.6 0.6 0.6', f'Ns {200 if material["metallic"] else 40}', 'illum 2']
        if texture:mtl.append('map_Kd '+texture)
        (ASSET_DIR/(name+'.mtl')).write_text('\n'.join(mtl)+'\n')
        return dict(path='assets/cq/'+name+'.obj', vertices=len(vertices),
                    polygons=len(self.faces), local_bounds_m=[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()],
                    **material)


def rounded_rectangle(x, y, radius, steps=12):
    radius = min(radius, x*.95, y*.95)
    return [[cx+radius*math.cos(a), cy+radius*math.sin(a)]
            for cx,cy,start in [(x-radius,y-radius,0),(-x+radius,y-radius,90),
                                (-x+radius,-y+radius,180),(x-radius,-y+radius,270)]
            for a in np.radians(np.linspace(start,start+90,steps,endpoint=False))]


def u_outline(front, back, half_width, aperture_rear, aperture_half_width,
              outer_radius, inner_radius, steps=12):
    """A single non-self-intersecting hood outline, open toward the nozzle."""
    p=np.array([[front,-half_width],[back,-half_width],[back,half_width],
                [front,half_width],[front,aperture_half_width],
                [aperture_rear,aperture_half_width],
                [aperture_rear,-aperture_half_width],[front,-aperture_half_width]])
    radii=[outer_radius]*4+[.004,inner_radius,inner_radius,.004]
    result=[]
    for i,(vertex,radius) in enumerate(zip(p,radii)):
        incoming=vertex-p[(i-1)%len(p)];incoming/=np.linalg.norm(incoming)
        outgoing=p[(i+1)%len(p)]-vertex;outgoing/=np.linalg.norm(outgoing)
        centre=vertex-incoming*radius+outgoing*radius
        start=vertex-incoming*radius-centre
        angle=math.atan2(start[1],start[0])
        turn=math.copysign(math.pi/2,np.cross(incoming,outgoing))
        result.extend((centre+radius*np.array([math.cos(a),math.sin(a)])).tolist()
                      for a in np.linspace(angle,angle+turn,steps,endpoint=False))
    return result


def triangulate_polygon(points):
    """Ear clip the concave U caps; a triangle fan would fill the aperture."""
    p=np.asarray(points);remaining=list(range(len(p)));faces=[]
    while len(remaining)>3:
        for j,b in enumerate(remaining):
            a,c=remaining[j-1],remaining[(j+1)%len(remaining)]
            if np.cross(p[b]-p[a],p[c]-p[b])<=1e-12:continue
            others=[i for i in remaining if i not in (a,b,c)]
            q=p[others]
            inside=((np.cross(p[b]-p[a],q-p[a])>=-1e-12)
                    &(np.cross(p[c]-p[b],q-p[b])>=-1e-12)
                    &(np.cross(p[a]-p[c],q-p[c])>=-1e-12))
            if np.any(inside):continue
            faces.append([a,b,c]);remaining.pop(j);break
        else:raise ValueError('Could not triangulate original hood outline')
    faces.append(remaining)
    return faces


def bezier(points, count=32):
    p = np.asarray(points); t=np.linspace(0,1,count)[:,None]
    return (1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t**2*p[2]+t**3*p[3]


def neck_path():
    """Estimated off-centre routing from the below-hood case outlet."""
    y=NOZZLE[1]
    return np.vstack([
        np.array([[x,y,-.030] for x in np.linspace(-.064,-.135,20)]),
        bezier([[-.135,y,-.030],[-.151,y,-.030],[-.145,y,.065],[-.166,y,.108]])[1:],
        bezier([[-.166,y,.108],[-.180,y,.142],[-.220,y,.141],[-.220,y,.102]])[1:],
        np.array([[-.220,y,z] for z in np.linspace(.102,-.034,35)])[1:]])


def refined_profile(profile, subdivisions=5):
    # Monotone linear refinement preserves the explicitly authored lathe shape.
    p=np.asarray(profile); result=[]
    for a,b in zip(p[:-1],p[1:]):
        result.extend(a+(b-a)*t for t in np.linspace(0,1,subdivisions,endpoint=False))
    return [*result,p[-1]]


def mahogany_texture(path, size=512):
    """Original deterministic material approximation, not a photograph."""
    y,x=np.mgrid[0:size,0:size]/size
    noise=np.random.default_rng(17).normal(0,.015,(size,size))
    grain=np.sin(2*math.pi*(y*49+.7*np.sin(x*5)+.2*np.sin(x*17)))
    fine=np.sin(2*math.pi*(y*183+.5*np.sin(x*13)))
    brightness=np.clip(1+.12*grain+.035*fine+noise,.65,1.2)
    rgb=np.clip(brightness[:,:,None]*np.array([91,35,17]),0,255).astype('uint8')
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    pixels=b''.join(b'\0'+row.tobytes() for row in rgb)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>2I5B',size,size,8,2,0,0,0))
                     +chunk(b'IDAT',zlib.compress(pixels))+chunk(b'IEND',b''))


def build():
    ASSET_DIR.mkdir(parents=True,exist_ok=True)
    parts={name:Mesh() for name in ['frame_wood','frame_brass','frame_steel',
           'lever_brass','lever_grip','cylinder_shell','cylinder_caps','piston_rod','tray']}
    wood=parts['frame_wood']
    wood.hood()
    steel=parts['frame_steel']; brass=parts['frame_brass']
    panel_rotation=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
    steel.rounded_box((.142875,.251175,.003),(-.061,0,-.1315875),.002,panel_rotation)
    for side in (-1,1):
        steel.rounded_box((.121,.003,.251175),(-.002,side*.0699375,-.1315875),.002)
        steel.rounded_box((.124,.012,.018),(.062,side*.078,-.056),.003)
        steel.rounded_box((.014,.018,.067),(.038,side*.078,-.035),.003)
        steel.cylinder(.005,.060,(.115,side*.078,-.055))
        steel.lathe([(.008,-.004),(.011,-.002),(.011,.002),(.009,.005)],(.115,side*.078,-.030))
        brass.lathe([(.011,-.004),(.014,-.002),(.014,.003),(.010,.006)],(.115,side*.078,-.090))
        for z in (-.021,-.239):
            brass.cylinder(.0055,.004,(-.064,side*.056,z),panel_rotation,6)
    steel.rounded_box((.050,.2159,.015),(.113,0,-.057),.003)
    steel.rounded_box((.085,.180,.005),(.100,0,.0025),.002)
    # Swan neck: connected annular metal wall and annular outlet; no filled disc.
    neck=neck_path()
    brass.tube(neck,.008,.0065)
    brass.tube([[*NOZZLE[:2],-.034],[*NOZZLE[:2],NOZZLE[2]]],.010,.0065)
    brass.lathe([(.010,-.004),(.011,-.002),(.011,.002),(.010,.004)],(-.064,NOZZLE[1],-.030),
                np.array([[0.,0.,-1.],[0.,1.,0.],[1.,0.,0.]]))
    lever=parts['lever_brass']
    lever.lathe(refined_profile([(.030,.075),(.036,.079),(.036,.084),(.029,.090),
        (.025,.100),(.023,.114),(.025,.127),(.031,.135),(.031,.143),(.025,.149),(.017,.158)]))
    lever.lathe([(.017,.157),(.019,.160),(.019,.165),(.017,.168)])
    lever.lathe(refined_profile([(.0165,.425),(.019,.428),(.019,.432),(.011,.437),
        (.007,.440),(.0075,.444),(.004,.447),(.0008,HANDLE_TOP-PIVOT[2])]))
    lever.cylinder(.008,.048,(0,0,0),np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]))
    for side in (-1,1):
        lever.rounded_box((.063,.009,.017),(.026,side*.0135,0),.003)
    lever.cylinder(.006,.042,CRANK_PIN,np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]))
    parts['lever_grip'].lathe(refined_profile([(.0165,.166),(.0165,.172),(.015,.188),
        (.014,.217),(.0135,.250),(.014,.282),(.016,.319),(.021,.355),
        (.023,.381),(.022,.402),(.018,.419),(.0165,.425)],8))
    shell=parts['cylinder_shell']; caps=parts['cylinder_caps']
    shell.lathe([(.037,.008),(.039,.014),(.039,.143),(.037,.150)])
    caps.lathe([(.013,-.016),(.023,-.011),(.038,-.004),(.043,.001),(.043,.009),(.038,.015)])
    caps.lathe([(.038,.143),(.043,.148),(.043,.157),(.022,.162),(.012,.166)])
    for a in np.arange(4)*math.pi/2+math.pi/4:
        caps.cylinder(.0023,.147,(.038*math.cos(a),.038*math.sin(a),.078))
        caps.cylinder(.0045,.005,(.038*math.cos(a),.038*math.sin(a),.157),sides=6)
    caps.cylinder(.0065,.026,(0,.039,.020),np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]]))
    rod=parts['piston_rod']
    rod.cylinder(.006,.173,(0,0,-.0935))
    rod.lathe([(.007,-.012),(.012,-.007),(.012,.005),(.008,.010)])
    tray=parts['tray']
    tray.ring((.101,.085,.012),(.095,.079,.009),-.240,TRAY_TOP-.002,centre=(-.170,0),inner_centre=(-.170,0))
    tray.rounded_box((.190,.158,.003),(-.170,0,-.2385),.009)
    for y in np.linspace(-.070,.070,11):
        tray.rounded_box((.185,.004,.003),(-.170,y,TRAY_TOP-.0015),.001)
    for x in np.linspace(-.260,-.080,13):
        tray.rounded_box((.004,.143,.003),(x,0,TRAY_TOP-.0015),.001)
    for side in (-1,1):
        steel.cylinder(.003,.128,(-.105,side*.066,-.225),np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]]))
    materials={
      'frame_wood':dict(color=[.30,.10,.045,1],roughness=.32,metallic=0.),
      'frame_brass':dict(color=[.70,.52,.19,1],roughness=.18,metallic=1.),
      'frame_steel':dict(color=[.44,.47,.49,1],roughness=.27,metallic=1.),
      'lever_brass':dict(color=[.70,.52,.19,1],roughness=.18,metallic=1.),
      'lever_grip':dict(color=[.87,.80,.63,1],roughness=.23,metallic=0.),
      'cylinder_shell':dict(color=[.43,.28,.11,1],roughness=.30,metallic=.65),
      'cylinder_caps':dict(color=[.64,.56,.34,1],roughness=.24,metallic=1.),
      'piston_rod':dict(color=[.65,.68,.70,1],roughness=.15,metallic=1.),
      'tray':dict(color=[.035,.035,.035,1],roughness=.34,metallic=0.)}
    mahogany_texture(ASSET_DIR/'mahogany.png')
    manifest={}
    for name,mesh in parts.items():
        manifest[name]=mesh.save(name,materials[name],'mahogany.png' if name=='frame_wood' else None)
        manifest[name]['body']=('lever' if name.startswith('lever') else
            'cylinder' if name.startswith('cylinder') else 'piston' if name=='piston_rod' else 'frame')
        manifest[name]['collision']=False
    dimensions=dict(
      provenance='Original dimensioned Angram CQ reconstruction. Not manufacturer CAD; no copyrighted reference images or brand marks bundled.',
      license='MIT',units='metres',source_drawings=[
       'https://www.ukbrewing.com/v/vspfiles/assets/images/schemcq.jpg',
       'https://www.ukbrewing.com/v/vspfiles/assets/images/CQassem.jpg',
       'https://www.ukbrewing.com/v/vspfiles/assets/images/CQcylin.jpg'],
      published_approximate=dict(handle_top_above_bar=.41275,body_below_bar=.257175,
          outlet_to_tray=.180975,clamp_clearance_max=.0508,front_body_width=.142875,front_overall_width=.2159),
      estimated=dict(lever_pivot=PIVOT.tolist(),cylinder_pivot=CYLINDER_PIVOT.tolist(),crank_pin_in_lever=CRANK_PIN.tolist(),
          nozzle=NOZZLE.tolist(),tray_top=TRAY_TOP,neck_outer_radius=.008,neck_inner_radius=.0065,
          neck_routing=dict(case_port=[-.064,-.040,-.030],lateral_offset=-.040,
              basis='The front-view drawing shows an off-centre neck but gives no lateral dimension. The 40 mm offset and below-hood route are original clearance estimates, not claimed factory measurements. The tray remains centred.'),
          hood_depth=.210,hood_width=.18415,hood_height=.038,barrel_outer_diameter=.078,
          hood_clearance_aperture=dict(type='open-front U',rear_x=.056,half_width_y=.040,
              upper_bevel_expansion=.004,
              basis='Original undimensioned clearance estimate for the retained crank and ferrule over the estimated 60 degree stroke; not a claimed factory cutout.'),
          piston_rod_length=.180,pull_angle_degrees=60,clamp_panel_thickness=.025,
          clamp_studs=[[.115,-.078,-.025],[.115,.078,-.025]]),
      frames=dict(frame='bar-surface origin; nozzle points toward -X; width along Y',
          lever='origin at lever pivot; object local +Z points up in rest pose',
          cylinder='origin at lower pivot; local +Z points toward crank pin',
          piston='origin at upper crank pin; same orientation as cylinder; rod extends along -Z'),
      grasp_region=dict(body='lever',product_z_range_m=[.140,.350],diameter_m=[.028,.034],
          note='Actual cream handgrip. The scripted demo grasps near product Z=0.300 m; exact contact geometry is retained in the OBJ.'),
      fidelity_notes=['Functional exterior silhouette follows public drawings; undimensioned profiles and hardware are estimates.',
          'The plain outlet is hollow and has no sparkler. Internal hydraulic valves and invisible spring wires are not visualized.',
          'The open-front U hood aperture is an original swept-clearance estimate; factory cutout dimensions are unavailable. External envelope dimensions are preserved.',
          'The off-centre neck is estimated at Y=-40 mm and joins the case below the hood; the published drawings do not dimension that routing.',
          'Meshes are visual surfaces. Supply matching compound colliders; do not convex-hull the hollow neck, tray or wood slot.',
          'Mahogany is a deterministic original procedural texture, not a photo or manufacturer material.'])
    (ASSET_DIR/'PARTS.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (ASSET_DIR/'DIMENSIONS.json').write_text(json.dumps(dimensions,indent=2)+'\n')
    (ASSET_DIR/'LICENSE').write_text(Path(__file__).with_name('LICENSE').read_text())
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ASSET_DIR.iterdir()
            if p.is_file() and p.name!='SHA256.json'}
    (ASSET_DIR/'SHA256.json').write_text(json.dumps(hashes,indent=2,sort_keys=True)+'\n')
    print(json.dumps({name:dict(vertices=v['vertices'],polygons=v['polygons'],bounds=v['local_bounds_m'])
                      for name,v in manifest.items()},indent=2))


if __name__ == '__main__':
    build()
