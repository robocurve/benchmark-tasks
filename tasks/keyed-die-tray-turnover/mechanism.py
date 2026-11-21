"""Passive turnover jig with a bolted source tray and a freely removable receiver."""
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import UsdPhysics, Gf
from layout import *
from native import box,body
import training_tray as training
ROOT='/World/envs/env_0/die_turner'
COLORS={'source':(.32,.55,.65),'receiver':(.55,.42,.68)}
def quat(matrix):
    q=R.from_matrix(matrix).as_quat()
    return Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3])))
def axis(stage,name,child,parent=None,parent_anchor=None,limits=None):
    j=UsdPhysics.RevoluteJoint.Define(stage,ROOT+'/'+name)
    j.CreateBody1Rel().SetTargets([child])
    j.CreateAxisAttr('X')
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateCollisionEnabledAttr(False)
    if parent:
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,parent_anchor)))
    else:
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,PIVOT)))
        j.CreateLocalRot0Attr(quat(BASIS))
    if limits:
        j.CreateLowerLimitAttr(limits[0]);j.CreateUpperLimitAttr(limits[1])
    return j
def build(stage,quarter_turn=1):
    from pxr import UsdGeom
    axle=UsdGeom.Cylinder.Define(stage,ROOT+'/axle')
    axle.CreateAxisAttr('Y');axle.CreateHeightAttr(.091);axle.CreateRadiusAttr(.002)
    axle.AddTranslateOp().Set(Gf.Vec3d(*map(float,PIVOT)))
    axle.CreateDisplayColorAttr([Gf.Vec3f(.42,.43,.44)])
    for side in [-1,1]:
        box(stage,ROOT+f'/stand{side+1}',world([side*.042,0,-.023]),[.010,.009,.041],(.47,.48,.50))
    for s in [-1,1]:
        # Both slots rest on physical bench pads at their horizontal endpoints.
        box(stage,ROOT+f'/rest{s+1}',world([0,s*.058,-.02356]),[.012,.020,.03380],(.39,.42,.45))
    box(stage,ROOT+'/foot',world([0,0,-.0375]),[.105,.102,.006],(.39,.41,.43))
    for name,sign in [('source',1),('receiver',-1)]:
        base=ROOT+'/'+name
        leaf=base+'_frame'
        body(stage,leaf,PIVOT,.0505 if name=='source' else .0690,BASIS)
        colour=COLORS[name]
        def part(key,p,size,col=colour):
            box(stage,leaf+'/'+key,p,size,col)
        y=sign*RADIUS
        part('bed',[0,y,-.00364-.0015],[.057,.057,.003])
        if name=='receiver':
            # The apron supports the loose tray before insertion and leaves its side edges accessible.
            part('loading_apron',[0,y+sign*.044,-.00514],[.043,.047,.003])
        # Rails cover the near-axle half. The outboard end remains open for a side pinch.
        # A 0.75 mm retaining lip engages the recessed flange below the inspection face.
        for side in [-1,1]:
            clear_x=.02620
            if name=='source':
                part(f'rail{side+1}',[side*.02655,y,-.0021],[.0022,.054,.00305])
                part(f'lid{side+1}',[side*.02575,y,-.00070],[.0032,.054,.00075])
                continue
            rail_y=y-sign*.0105
            part(f'rail{side+1}',[side*(clear_x+.0011),rail_y,-.0021],[.0022,.029,.00305])
            part(f'lid{side+1}',[side*.0258,y-sign*.014,-.00070],[.0040,.020,.00075])
            if name=='receiver':
                # Two side guides taper from the open mouth to the parallel throat.
                poly=np.array([[clear_x,y+sign*.012],[.033,y+sign*.040],
                               [.034,y+sign*.040],[.034,y+sign*.012]])
                part(f'guide_throat{side+1}',[side*(clear_x+.0011),y+sign*.008,-.0021],[.0022,.008,.00305])
                poly[:,0]*=side
                if np.cross(poly[1]-poly[0],poly[2]-poly[1])<0:poly=poly[::-1]
                training.prism(stage,leaf+f'/guide_entry{side+1}',[(-.00364,poly),(-.0007,poly)],colour)
                part(f'guide_bracket{side+1}',[side*.027,y+sign*.022,-.0051],[.014,.040,.0028])
        part('inboard_stop',[0,y-sign*.02635,-.0021],[.049,.0018,.0031])
        if name=='source':
            part('outboard_stop',[0,y+sign*.0265,-.0021],[.050,.002,.0031])
        # Each leaf meets the common axle at separate hinge knuckles.
        part('hinge_arm',[sign*.021,sign*.0095,-.00565],[.009,.026,.0034])
        part('hinge_post',[sign*.021,sign*.003,-.0031],[.009,.008,.0060])
        part('knuckle',[sign*.021,0,0],[.012,.007,.007])
        part('outrigger',[sign*.049,sign*HANDLE_Y,-.0060],[.047,.008,.0040])
        part('pin_riser',[sign*.067,sign*HANDLE_Y,-.0033],[.007,.008,.0065])
        # Separate the finger grip from the socket sweep. Only the inner 2 mm
        # of the rotating knob encloses the axle end.
        pin_start=.0645
        pin_end=HANDLE_X-.009+.002
        part('knob_pin',[sign*(pin_end+pin_start)/2,sign*HANDLE_Y,0],[pin_end-pin_start,.005,.005])
        axis(stage,name+'_hinge',leaf)
        handle=base+'_knob'
        offset=[sign*HANDLE_X,sign*HANDLE_Y,0]
        body(stage,handle,world(offset),.00655,BASIS)
        box(stage,handle+'/grip',[0,0,0],[.018,.017,.016],(.92,.59,.22))
        axis(stage,name+'_spindle',handle,leaf,offset)
        traypos=world([0,y,-.00014])
        trayrot=BASIS@TRAY_YAW[name]
        if name=="receiver":
            traypos=STAGE.copy()
            trayrot=R.from_euler("z",90*quarter_turn,degrees=True).as_matrix()
            box(stage,ROOT+'/staging_pedestal', STAGE+[0,0,-.01595],[.026,.026,.0249],(.49,.51,.52))
        training.tray(stage,base,traypos,trayrot)
        if sign==1:
            # Two flush screws installed from below retain the loaded source tray.
            # Both carriers still rotate on the hinge. The receiver and all dies are free.
            mount=UsdPhysics.FixedJoint.Define(stage,ROOT+'/source_tray_mount')
            mount.CreateBody0Rel().SetTargets([leaf])
            mount.CreateBody1Rel().SetTargets([base])
            mount.CreateLocalPos0Attr(Gf.Vec3f(0,float(y),-.00014))
            mount.CreateExcludeFromArticulationAttr(True)
            mount.CreateCollisionEnabledAttr(False)
            for i,(ix,iy) in enumerate(CELLS):
                xyz=traypos+trayrot@np.array([ix*training.PITCH,iy*training.PITCH,-training.DEPTH+training.THICK/2+.000015])
                training.chip(stage,ROOT+f'/d{i}',xyz,trayrot)
