"""Candidate complete prop layout. YAM integration and reach remain unvalidated."""
import os
import numpy as np
from cloth_mesh import tshirt
from native_cloth import Fabric

TABLE=.7475
LINE_X=.52
LINE_Z=1.09
LINE_HALF_SPAN=.25
SUPPORT_SIZE=(.10,.44,.155 if os.environ.get('ROBOCURVE_SHIRT_RAISED_SUPPORT')=='1' else .10)
SUPPORT_BOARD_THICKNESS=.008
PHYSICS_HZ=480
SUPPORT_CENTER=(.31,0.,TABLE+SUPPORT_SIZE[2]/2)
SUPPORT_TOP=TABLE+SUPPORT_SIZE[2]
HEM_X=.195


def initial_layout(mesh=None):
    mesh=tshirt() if mesh is None else mesh
    p=mesh.rest
    return np.column_stack((HEM_X+p[:,1],-p[:,0],SUPPORT_TOP+.003+p[:,2]))


def support_geometry(board_thickness=None):
    """Exact geometry shared by native authoring and the reset penetration audit."""
    thickness=SUPPORT_BOARD_THICKNESS if board_thickness is None else board_thickness
    props={'Board':dict(kind='box',size=(SUPPORT_SIZE[0],SUPPORT_SIZE[1],thickness),
                       center=(SUPPORT_CENTER[0],0.,SUPPORT_TOP-thickness/2))}
    for ix,x in enumerate((.325,.3475)):
        for iy,y in enumerate((-.185,.185)):
            props[f'Leg{ix}{iy}']=dict(kind='box',size=(.015,.015,SUPPORT_SIZE[2]-thickness),
                         center=(x,y,TABLE+(SUPPORT_SIZE[2]-thickness)/2))
    return props


def rack_geometry():
    props={'Clothesline':dict(kind='cylinder',axis='Y',center=(LINE_X,0.,LINE_Z),
                              radius=.0015,height=2*LINE_HALF_SPAN)}
    for sign in (-1,1):
        label='Left' if sign>0 else 'Right'
        props[label+'Post']=dict(kind='box',size=(.025,.025,LINE_Z-TABLE),
            center=(LINE_X,sign*(LINE_HALF_SPAN+.0125),(LINE_Z+TABLE)/2))
        props[label+'Foot']=dict(kind='box',size=(.15,.07,.015),
            center=(LINE_X,sign*(LINE_HALF_SPAN+.0125),TABLE+.0075))
    return props


def author_support(stage,path='/World/LaundrySupport'):
    """Fixed raised laundry board with clear space beneath the thin top.

    The ordinary support has a 100 x 440 x 8 mm plastic top on four 15 mm
    square legs. Its top is 100 mm above the table; the hem overhangs its near edge by 65 mm. The front edge is at x=.26. There are no attachments.
    """
    from pxr import UsdGeom
    from native_cloth import author_box
    root=UsdGeom.Xform.Define(stage,path)
    for name,spec in support_geometry().items():
        author_box(stage,path+'/'+name,spec['size'],spec['center'],(.78,.82,.80))
    return root


def author_rack(stage,path='/World/Rack'):
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    from native_cloth import author_box
    UsdGeom.Xform.Define(stage,path)
    line=UsdGeom.Cylinder.Define(stage,path+'/Clothesline')
    line.CreateRadiusAttr(.0015)
    line.CreateHeightAttr(2*LINE_HALF_SPAN)
    line.CreateAxisAttr('Y')
    line.AddTranslateOp().Set(Gf.Vec3d(LINE_X,0.,LINE_Z))
    line.CreateDisplayColorAttr([Gf.Vec3f(.78,.78,.78)])
    UsdPhysics.CollisionAPI.Apply(line.GetPrim())
    cp=PhysxSchema.PhysxCollisionAPI.Apply(line.GetPrim())
    cp.CreateContactOffsetAttr(.0005)
    cp.CreateRestOffsetAttr(0.)
    for sign in (-1,1):
        label='Left' if sign>0 else 'Right'
        author_box(stage,path+'/'+label+'Post',(.025,.025,LINE_Z-TABLE),
                   (LINE_X,sign*(LINE_HALF_SPAN+.0125),(LINE_Z+TABLE)/2),(.35,.35,.37))
        author_box(stage,path+'/'+label+'Foot',(.15,.07,.015),
                   (LINE_X,sign*(LINE_HALF_SPAN+.0125),TABLE+.0075),(.35,.35,.37))
    return line


def author_props(stage,with_peg_rests=True,line_pegs=False,contact_refined=False,regular_refined=False):
    from native_cloth import author_cloth,author_peg
    if contact_refined and regular_refined:raise ValueError('Select one garment mesh')
    if regular_refined:mesh=tshirt(spacing=.0025)
    elif contact_refined:
        from refined_cloth import contact_band_mesh
        mesh=contact_band_mesh()
    else:mesh=tshirt()
    author_cloth(stage,'/World/Tshirt',mesh,Fabric(),points=initial_layout(mesh))
    author_rack(stage)
    author_support(stage)
    if line_pegs:
        from line_pegs import author
        author(stage)
        return mesh
    if with_peg_rests:
        from peg_rests import author,peg_positions
        author(stage)
        pegs=peg_positions()
    else:
        pegs={('Left' if sign>0 else 'Right'):(.12,sign*.20,TABLE+.014) for sign in (-1,1)}
    for label,position in pegs.items():
        author_peg(stage,'/World/'+label+'Peg',pos=position,
                   fixed_lower=False,spring=True,open_angle=0.)
    return mesh
