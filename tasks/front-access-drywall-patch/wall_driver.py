"""Task-only free collated driver for the full loose-stock wall. Moving parts use native joints."""
import numpy as np
from surface_process_probe import WORLD, box, body
from rotary_driver import joint, cyl
from tool_profile import SHELL_BOXES, TIP, TRIGGER, TRIGGER_SIZE

PITCH = .012
POCKETS = 8

def make_strip(stage, origin):
    from pxr import UsdPhysics
    body(stage, WORLD+'/screw_carrier', origin, .005)
    box(stage, WORLD+'/screw_carrier/feed_lug', (.166,.014,0), (.020,.020,.006), (.39,.34,.23))
    for pocket in range(POCKETS):
        y = -.043 + pocket*PITCH
        box(stage, WORLD+f'/screw_carrier/carrier_{pocket}', (.171,y,.003),
            (.009,PITCH,.004), (.64,.57,.42))
        box(stage, WORLD+f'/screw_carrier/screw_{pocket}', (.184,y,0),
            (.030,.0039,.0039), (.45,.46,.48))
    slide = joint(stage, UsdPhysics.PrismaticJoint, 'feed_slide', 'shoe', 'screw_carrier', [0,0,0], [0,0,0])
    slide.CreateAxisAttr('Y')
    slide.CreateLowerLimitAttr(-PITCH*(POCKETS-1)-.0003)
    slide.CreateUpperLimitAttr(.0003)
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(WORLD+'/screw_carrier')).CreateFilteredPairsRel().SetTargets(
        [WORLD+'/driver', WORLD+'/spindle', WORLD+'/trigger'])
    spring = UsdPhysics.DriveAPI.Apply(slide.GetPrim(),'linear')
    spring.CreateTypeAttr('force')
    spring.CreateStiffnessAttr(950.)
    spring.CreateDampingAttr(2.)
    spring.CreateMaxForceAttr(7.)
    spring.CreateTargetPositionAttr(0.)
    return spring


def build(stage,origin):
    from pxr import UsdPhysics
    body(stage,WORLD+'/driver',origin,1.095)
    spindle_center=origin+[0.,TIP[1],0.]
    rotor=WORLD+'/spindle'
    body(stage,rotor,spindle_center,.035)
    cyl(stage,rotor+'/shaft',.143,.090,.0025,(.35,.37,.38))
    cyl(stage,rotor+'/bit',.190,.005,.0018,(.26,.28,.29))
    bearing=joint(stage,UsdPhysics.RevoluteJoint,'spindle_bearing','driver','spindle',
                  [.14,TIP[1],0.],[.14,0.,0.])
    motor=UsdPhysics.DriveAPI.Apply(bearing.GetPrim(),'angular')
    motor.CreateTypeAttr('force');motor.CreateStiffnessAttr(0.);motor.CreateDampingAttr(.015)
    motor.CreateMaxForceAttr(.65);motor.CreateTargetVelocityAttr(0.)
    # Internal drive and slider are coaxial. Their interaction is represented by their bearings.
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(rotor)).CreateFilteredPairsRel().SetTargets([WORLD+'/shoe'])
    body(stage,WORLD+'/shoe',origin,.115)
    slide=make_strip(stage,origin)
    for name,loc,size,color in SHELL_BOXES:
        if name=='nose':
            for n in range(12):
                t=2*np.pi*n/12
                box(stage,WORLD+'/shoe/rim_'+str(n),(.221,TIP[1]+.007*np.cos(t),.007*np.sin(t)),
                    (.004,.003,.003),color)
            sleeve=joint(stage,UsdPhysics.PrismaticJoint,'shoe_slide','driver','shoe',[0.,0.,0.],[0.,0.,0.])
            sleeve.CreateLowerLimitAttr(-.033);sleeve.CreateUpperLimitAttr(0.)
            spring=UsdPhysics.DriveAPI.Apply(sleeve.GetPrim(),'linear')
            spring.CreateTypeAttr('force');spring.CreateTargetPositionAttr(0.)
            spring.CreateStiffnessAttr(450.);spring.CreateDampingAttr(2.);spring.CreateMaxForceAttr(15.)
        else:
            parent='shoe' if name in ('magazine','feed_guide','strip_envelope') else 'driver'
            box(stage,WORLD+'/'+parent+'/'+name,loc,size,color)
    body(stage,WORLD+'/trigger',origin+TRIGGER,.010)
    box(stage,WORLD+'/trigger/button',(0,0,0),TRIGGER_SIZE,(.66,.21,.13))
    j=joint(stage,UsdPhysics.PrismaticJoint,'trigger_slide','driver','trigger',TRIGGER,[0,0,0])
    j.CreateLowerLimitAttr(-.004);j.CreateUpperLimitAttr(0.)
    d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear')
    d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.)
    d.CreateStiffnessAttr(2000.);d.CreateDampingAttr(1.2);d.CreateMaxForceAttr(8.)
    return motor, slide
