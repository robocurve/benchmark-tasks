"""Camera-independent physical input and live body/joint/mass manifests."""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path


def manifest(stage,assets,urdf,physics_dt,control_hz):
    urdf=Path(urdf);xml=ET.parse(urdf).getroot();links={};meshes={}
    for link in xml.findall('link'):
        inertial=link.find('inertial');mass=None if inertial is None else inertial.find('mass')
        links[link.get('name')]={'collision_count':len(link.findall('collision')),
                               'mass_kg':None if mass is None else float(mass.get('value'))}
        for collider in link.findall('collision'):
            mesh=collider.find('geometry/mesh')
            if mesh is not None:
                name=mesh.get('filename');meshes[name]=hashlib.sha256((urdf.parent/name).read_bytes()).hexdigest()
    inputs={'urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'urdf_links':links,
            'urdf_joint_count':len(xml.findall('joint')),'collision_mesh_sha256':meshes,
            'native_asset_sha256':{name:hashlib.sha256(Path(path).read_bytes()).hexdigest() for name,path in sorted(assets.items())},
            'physics_dt_s':float(physics_dt),'control_hz':float(control_hz)}
    # Sensor Cameras and lights carry none of these physical schemas. Do not
    # hash dynamic world transforms/velocities, which are measured separately.
    physical=[];counts={'colliders':0,'bodies':0,'joints':0,'mass_prims':0}
    for prim in stage.Traverse():
        schemas=list(prim.GetAppliedSchemas());kind=str(prim.GetTypeName())
        collision=any('CollisionAPI' in s for s in schemas)
        body=any('RigidBodyAPI' in s for s in schemas)
        joint='Joint' in kind and (kind.startswith('Physics') or kind.startswith('Physx'))
        mass=any('MassAPI' in s for s in schemas)
        if not (collision or body or joint or mass):continue
        counts['colliders']+=int(collision);counts['bodies']+=int(body);counts['joints']+=int(joint);counts['mass_prims']+=int(mass)
        values={}
        for attr in prim.GetAttributes():
            name=attr.GetName()
            if (name.startswith(('physics:','physx','drive:')) and not any(word in name.lower() for word in ('velocity','sleep','contactreport'))):
                value=attr.Get()
                if value is not None:values[name]=str(value)
        relationships={rel.GetName():[str(p) for p in rel.GetTargets()] for rel in prim.GetRelationships() if rel.GetName().startswith(('physics:','physx'))}
        physical.append({'path':str(prim.GetPath()),'type':kind,'schemas':sorted(s for s in schemas if 'Physics' in s or 'Physx' in s),
                         'attributes':values,'relationships':relationships})
    digest=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return {'physical_input_sha256':digest(inputs),'body_joint_mass_sha256':digest(physical),
            'counts':counts,'inputs':inputs,'body_joint_mass':physical,
            'scope':'All URDF collision mesh bytes and native collider assets; live collision/body/joint/mass prims. Sensor Camera/light prims and dynamic world transforms excluded.'}
