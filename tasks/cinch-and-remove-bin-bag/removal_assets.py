"""Shared physical assets for native and Inspect Robots drawstring removal.

Import and build only after Isaac starts. Object transforms are reset geometry.
"""

# Bin: IKEA NYPON 15 cm plant pot (art. 603.956.16): 171 mm outside / 159 mm
# inside diameter, 152 mm tall, modelled as a straight cylinder.
WALL_H=.152                   # m, wall from mat (z 0.750) to rim (z 0.902)
WALL_R=.0825                  # m, wall mid-surface radius
WALL_T=.006                   # m
FLOOR_T=.006                  # m, pot bottom
RIM_Z=.750+WALL_H             # 0.902
BAG_BASE_Z=.758               # liner bottom plane, 2 mm above the pot floor
BLOCK=(.070,.100,.045)        # m, mock-waste block (identical blocks; smallest face 70 x 45 mm, diagonal 83 mm)
BLOCK_MASS=.050               # kg
BLOCK_COUNT=3
# Packing: two blocks standing on a 45 x 100 mm face side by side, one lying
# flat on top (long side along y). Each block is spawned axis-aligned, so its
# edge lengths in world x, y, z are listed per block.
BLOCK_SIZES=[(.045,.100,.070),(.045,.100,.070),(.070,.100,.045)]
def block_positions():
    return [(.22-.0235,0.,BAG_BASE_Z+.036),(.22+.0235,0.,BAG_BASE_Z+.036),(.22,0.,BAG_BASE_Z+.0955)]
LOOP_OUT=.060                 # m, loop bight distance outward from the cut-out
BAG_COLOR=(.92,.92,.90)       # white liner
STRING_COLOR=(.80,.80,.78)    # white drawstring, slightly darker for contrast
FILM_T=.0001                  # m, film thickness (numerical; see fidelity note)
BEND_SCALE=.1                  # film bending stiffness factor
# Exposed drawstring loops: flat film-tape straps (see loop_chain.py). Each
# loop leaves its hem cut-out and lies on the mat as a U, standing on edge as
# the tape lies in the hem channel, its two strands 16 mm apart (centres).
STRAP_W=.020                  # m, flat drawstring tape width (real liners: about 10-20 mm)
STRAP_T=.002                  # m, numerical collision thickness of the doubled tape
STRAP_SEG=.010                # m, link length
STRAP_MASS=.004               # kg per exposed loop (solver value)
STRAND_GAP=.016               # m, strand centre spacing in the laid-out loop
LOOP_ROOT=.020                # m, strand ends this far out from the cut-out
LOOP_LEN=.0635                # m, straight strand length before the bight
LOOP_YAW=30.                  # deg, loops lean toward +x (toward the operator's reach)
STRING_RADIUS=.002
STRING_MASS=STRAP_MASS

def loop_axes(s):
    """Unit direction along loop s (+1 left/+y, -1 right/-y) and its normal."""
    import numpy as np
    a=np.radians(LOOP_YAW)
    u=np.array([np.sin(a),s*np.cos(a),0.])
    n=np.array([np.cos(a),-s*np.sin(a),0.])
    return u,n

def loop_polylines():
    """Authored exposed loops: one U-shaped strap per side, standing on edge on
    the mat, strands running outward from the cut-out (two root ends, then the
    strand, a semicircular bight, the other strand back)."""
    import numpy as np
    from realistic_meshes_v32 import HEM_R
    loops=[]
    z=.750+STRAP_W/2+.0004
    for s in (1,-1):
        u,n=loop_axes(s)
        cut=np.array([.22,s*HEM_R,z])
        r=STRAND_GAP/2
        a=cut+u*LOOP_ROOT;b=a+u*LOOP_LEN
        pts=[a+n*r,b+n*r]
        for k in range(1,12):
            ang=np.pi*k/12
            pts.append(b+u*r*np.sin(ang)+n*r*np.cos(ang))
        pts+=[b-n*r,a-n*r]
        loops.append(np.array(pts))
    return loops

def pinch_point(s,from_bight=.022):
    """Authored pinch site on loop s: across both straight strands, 22 mm short of the bight."""
    import numpy as np
    from realistic_meshes_v32 import HEM_R
    u,_=loop_axes(s)
    return np.array([.22,s*HEM_R,.750])+u*(LOOP_ROOT+LOOP_LEN-from_bight)

def build_assets():
    import numpy as np
    import trimesh
    import isaaclab.sim as sim
    from isaaclab.sim.utils import clone
    from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg, PhysxRigidBodyPropertiesCfg, PhysxCollisionPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    def _box(name,size,pos,color,rot_xyzw=(0.,0.,0.,1.)):
        # Polypropylene pot wall: smooth plastic, friction 0.3 / 0.25.
        return Object(name=name,object_type=ObjectType.BASE,spawner_cfg=sim.CuboidCfg(size=tuple(size),collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=.3,dynamic_friction=.25),visual_material=sim.PreviewSurfaceCfg(diffuse_color=color,roughness=.7)),initial_pose=Pose(position_xyz=tuple(pos),rotation_xyzw=tuple(rot_xyzw)))


    from realistic_meshes_v32 import dirty_mesh
    dv,df,dirty_mouth=dirty_mesh()
    loaded_mesh=trimesh.Trimesh(vertices=dv,faces=df,process=False)
    models={'loaded_bag':loaded_mesh}
    raw={k:np.asarray(v.vertices).copy() for k,v in models.items()}
    centroids={k:np.asarray(v.vertices).mean(0) for k,v in models.items()}
    for k,v in models.items():v.apply_translation(-centroids[k]);v.fix_normals()
    bead_count=20
    from realistic_meshes_v32 import HEM_R,HEM_Z
    bead_positions=np.array([[.22+HEM_R*np.cos(j*2*np.pi/bead_count),HEM_R*np.sin(j*2*np.pi/bead_count),BAG_BASE_Z+HEM_Z] for j in range(bead_count)])
    anchors={}
    from pxr import UsdGeom,UsdPhysics,Gf,Vt
    import omni.usd
    @clone
    def spawn_bag(prim_path,cfg,translation=None,orientation=None,**kwargs):
        name=prim_path.rsplit('/',1)[-1];model=models[name]
        root=_spawn_mesh_geom_from_mesh(prim_path,cfg,model,translation,orientation) or omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
        stage=omni.usd.get_context().get_stage();v=np.asarray(model.vertices)+centroids[name]
        hem=np.asarray(dirty_mouth)
        theta=np.arctan2(v[hem,1],v[hem,0])%(2*np.pi)
        assignments=np.round(theta*bead_count/(2*np.pi)).astype(int)%bead_count
        for j in range(bead_count):
            # A 3 mm seam marker must not rigidly constrain an entire 35 mm
            # arc of film. Attach its nearest mouth vertex; the intervening
            # film remains free to gather and fold under cord tension.
            candidates=hem[assignments==j]
            distances=np.linalg.norm(v[candidates,:2]-bead_positions[j,:2]+np.array([.22,0.]),axis=1)
            ids=candidates[np.argsort(distances)[:1]]
            fast_path=prim_path.rsplit('/',1)[0]+'/hem_%02d'%j
            attach=stage.DefinePrim(prim_path+'/hem_seam_%02d'%j,'OmniPhysicsVtxXformAttachment')
            attach.GetRelationship('omniphysics:src0').SetTargets([prim_path]);attach.GetRelationship('omniphysics:src1').SetTargets([fast_path])
            attach.GetAttribute('omniphysics:vtxIndicesSrc0').Set(ids.tolist())
            local=v[ids]+np.array(translation)-centroids[name]-bead_positions[j]
            attach.GetAttribute('omniphysics:localPositionsSrc1').Set(Vt.Vec3fArray.FromNumpy(local.astype(np.float32)))
            UsdPhysics.FilteredPairsAPI.Apply(root).CreateFilteredPairsRel().AddTarget(fast_path)
        anchors[name]=hem.tolist()
        return root
    def bag(name,pos,color):
        cfg=sim.MeshRectangleCfg(size=(.172,.172),edge_refinement=1.,func=spawn_bag,
            deformable_props=PhysxDeformableBodyPropertiesCfg(self_collision=True,solver_position_iteration_count=64,linear_damping=.5,sleep_threshold=0.,settling_threshold=0.,settling_damping=0.),
            collision_props=[PhysxCollisionCfg(rest_offset=.00005,contact_offset=.0005)],
            physics_material=PhysxSurfaceDeformableBodyMaterialCfg(density=900.,surface_thickness=FILM_T,
                youngs_modulus=2.5e7,poissons_ratio=.3,surface_bend_stiffness=BEND_SCALE*2.5e7/(12.*(1.-.3**2)),
                elasticity_damping=.05,bend_damping=.05,static_friction=.3,dynamic_friction=.25),
            visual_material=sim.PreviewSurfaceCfg(diffuse_color=color,roughness=.65))
        return DeformableObject(name=name,spawner_cfg=cfg,initial_pose=Pose(position_xyz=tuple(float(v) for v in np.array(pos)+centroids[name])))

    from yam_arena import station
    from station_setup import rig as standard_rig, scene_assets as station_assets, setup_manifest, save_views, setup_result
    assets=station_assets()
    centre=np.array([.22,0.,BAG_BASE_Z])
    # Thin static wall segments preserve the hollow interior; round floor.
    for j in range(40):
        a=2*np.pi*j/40
        pos=np.array([.22+WALL_R*np.cos(a),WALL_R*np.sin(a),.750+WALL_H/2])
        assets.append(_box('bin_wall_%02d'%j,(WALL_T,2*np.pi*(WALL_R+WALL_T/2)/40+.001,WALL_H),pos,(.80,.82,.83),rot_xyzw=(0,0,np.sin(a/2),np.cos(a/2))))
    assets.append(Object(name='bin_floor',object_type=ObjectType.BASE,spawner_cfg=sim.CylinderCfg(radius=WALL_R,height=FLOOR_T,axis='Z',
        collision_props=PhysxCollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),visual_material=sim.PreviewSurfaceCfg(diffuse_color=(.75,.77,.78),roughness=.7)),
        initial_pose=Pose(position_xyz=(.22,0.,.750+FLOOR_T/2))))
    assets.append(bag('loaded_bag',centre,BAG_COLOR))
    def marker(name,pos,size,mass,color,collide=True):
        return Object(name=name,object_type=ObjectType.RIGID,spawner_cfg=sim.CuboidCfg(size=size,
            rigid_props=PhysxRigidBodyPropertiesCfg(sleep_threshold=0.),collision_props=PhysxCollisionPropertiesCfg(collision_enabled=collide,contact_offset=.0005,rest_offset=0.),
            mass_props=sim.MassPropertiesCfg(mass=mass),physics_material=sim.RigidBodyMaterialCfg(static_friction=1.,dynamic_friction=.8),
            visual_material=sim.PreviewSurfaceCfg(diffuse_color=color)),initial_pose=Pose(position_xyz=tuple(map(float,pos))))
    for j,pos in enumerate(bead_positions):assets.append(marker('hem_%02d'%j,pos,(.003,.003,.003),.001,BAG_COLOR,collide=False))  # seam force points, no collider
    from loop_chain import chain,chain_asset
    for j,poly in enumerate(loop_polylines()):
        assets.append(chain_asset(chain('loop_%d'%j,poly,segment=STRAP_SEG,width=STRAP_W,thickness=STRAP_T,mass=STRAP_MASS,color=STRING_COLOR)))
    for i in range(BLOCK_COUNT):
        assets.append(Object(name='rubbish_%d'%i,object_type=ObjectType.RIGID,
            spawner_cfg=sim.CuboidCfg(size=BLOCK_SIZES[i],rigid_props=PhysxRigidBodyPropertiesCfg(sleep_threshold=0.),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=.001,rest_offset=0.),
                mass_props=sim.MassPropertiesCfg(mass=BLOCK_MASS),
                physics_material=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.7),
                visual_material=sim.PreviewSurfaceCfg(diffuse_color=[(.6,.35,.16),(.8,.8,.5),(.2,.65,.3)][i%3])),
            initial_pose=Pose(position_xyz=block_positions()[i])))
    return assets, anchors, bead_count

