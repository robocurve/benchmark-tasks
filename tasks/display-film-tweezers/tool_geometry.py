"""Passive tools and stands shared by the integrated scene and scorer.

Two otherwise identical tweezers stand side by side. Only the fine-tip pair fits the 5.6 mm
notch between the tab's side bridges; the paddle-tip pair's 7 mm blades land on the bridges.
"""
import numpy as np
from fixture_geometry import TABLE_Z
from geometry import rotation

STAND_X,STAND_Y=.36,-.24
TOOL_SIZE=np.array([.008,.0054,.120])
# Blade box (x width, y thickness, z length); must match the literal 'tip' part in toolkit.py.
TIP_SIZE=(.0024,.0008,.024)
# Returned = tip centre inside the stand cavity. Inner half-widths are 6 mm (x) and 13 mm (y);
# minus the blade half-width (1.2 mm) and the open tips' half-span (4.8 mm) this leaves
# 4.8 mm and 8.2 mm. 4.5 mm keeps the whole tip inside the walls in both directions.
RETURN_XY_MM=4.5
# key: stand centre (x, y), half names, stand part prefix, blade (x, y, z); blades must match toolkit.BLADES.
TOOLS={'fine':dict(stand=(STAND_X,STAND_Y),names=('tweezer_a','tweezer_b'),prefix='stand',blade=TIP_SIZE),
       'paddle':dict(stand=(.36,-.17),names=('paddle_a','paddle_b'),prefix='paddle_stand',blade=(.007,.0008,.024))}
TOOL_NAMES=TOOLS['fine']['names']  # the pair that fits the slot
ALL_TOOL_NAMES=tuple(n for t in TOOLS.values() for n in t['names'])
STAND_FLOORS=tuple(t['prefix']+'_floor' for t in TOOLS.values())


def tool_objects():
    from yam_arena.task_file import Box
    z=TABLE_Z;objects=[]
    for tool in TOOLS.values():
        (x,y),(a,b),prefix=tool['stand'],tool['names'],tool['prefix']
        objects+=[Box(name=a,size=(.008,.006,.12),pos=(x,y-.0044,z+.063),mass=.0075,color=(.4,.45,.5)),
                  Box(name=b,size=(.008,.006,.12),pos=(x,y+.0044,z+.063),mass=.0075,color=(.4,.45,.5)),
                  Box(name=prefix+'_floor',size=(.03,.034,.003),pos=(x,y,z+.0015),color=(.1,.15,.2))]
        for name,size,pos in [('left',(.003,.034,.020),(x-.0075,y,z+.013)),
                              ('right',(.003,.034,.020),(x+.0075,y,z+.013)),
                              ('front',(.018,.003,.020),(x,y-.0145,z+.013)),
                              ('back',(.018,.003,.020),(x,y+.0145,z+.013))]:
            objects.append(Box(name=prefix+'_'+name,size=size,pos=pos,color=(.1,.15,.2)))
    return objects


def root_from_state(state,sign):
    # ObjectState.pos is the transformed local bounds centre, displaced from the
    # root by the asymmetric neck and handle. Never treat it as the root pose.
    return np.asarray(state.pos)-rotation(state.quat_xyzw)@np.array([0,sign*.0023,0])


def return_metrics(objects,tool='fine'):
    names,prefix=TOOLS[tool]['names'],TOOLS[tool]['prefix']
    a,b=(objects[n] for n in names)
    ra,rb=(rotation(s.quat_xyzw) for s in (a,b))
    pa,pb=root_from_state(a,-1),root_from_state(b,1)
    tip=(pa+pb)/2+(ra+rb)@np.array([0,0,-.030])
    target=np.asarray(objects[prefix+'_floor'].pos)+[0,0,.0015]
    return dict(xy_mm=float(np.linalg.norm((tip-target)[:2])*1000),
                z_mm=float(abs(tip[2]-target[2])*1000),
                tilt_deg=max(float(np.degrees(np.arccos(np.clip(r[2,2],-1,1)))) for r in (ra,rb)),
                gap_mm=float((ra.T@(pb+rb@[0,0,-.06]-pa-ra@[0,0,-.06]))[1]-.0008)*1000,
                speed_mm_s=max(float(np.linalg.norm(s.lin_vel)*1000) for s in (a,b)),
                spin_rad_s=max(float(np.linalg.norm(s.ang_vel)) for s in (a,b)))


def tool_returned(objects,tool):
    m=return_metrics(objects,tool)
    return (all(np.isfinite(v) for v in m.values()) and m['xy_mm']<=RETURN_XY_MM and m['z_mm']<=5
            and m['tilt_deg']<=5 and 6<=m['gap_mm']<=10
            and m['speed_mm_s']<10 and m['spin_rad_s']<.5)


def returned(objects):
    """Every tool stands upright and open in its own stand."""
    return all(tool_returned(objects,tool) for tool in TOOLS)


def box_penetration(ca,ra,sa,cb,rb,sb):
    """Exact OBB separating-axis overlap; zero for separated boxes (metres)."""
    axes=[*ra.T,*rb.T]+[np.cross(a,b) for a in ra.T for b in rb.T]
    overlaps=[]
    for axis in axes:
        length=np.linalg.norm(axis)
        if length<1e-10:continue
        axis=axis/length
        extent_a=np.abs(ra.T@axis)@(np.asarray(sa)/2)
        extent_b=np.abs(rb.T@axis)@(np.asarray(sb)/2)
        overlap=extent_a+extent_b-abs(axis@(np.asarray(cb)-ca))
        if overlap<=0:return 0.
        overlaps.append(float(overlap))
    return min(overlaps)
