"""Full-size cold two-slot toaster and real-scale dry bread crumbs, SI units.

No heat or washing is claimed. A consumer empties the removable crumb tray
between deeper cleans, then reinserts it before next use. Dimensions are an
explicit engineering reconstruction of an ordinary 300 x 170 x 190 mm toaster.
"""
import numpy as np
CONTROL_HZ=20
PHYSICS_HZ=960
MAX_STEPS=1100
INSTRUCTION='Empty the cold, unplugged toaster\'s crumb tray into the waste bin, then slide the empty tray all the way back into the toaster.'
TABLE_BOUNDS=(-.10,.91,-.50,.62)
CONTACT_OFFSET=.0001
LINEAR_DAMPING=.01
ANGULAR_DAMPING=.01
BOUNCE_THRESHOLD=.05
TOASTER_CENTER=np.array((.730,.170,0.))
TOASTER_SIZE=np.array((.170,.300,.190))
TOASTER_FRONT=.130 # housing spans -170..+130 mm relative to tray origin.
TRAY_CENTER=np.array((.730,.170,.020))
TRAY_SIZE=np.array((.140,.260,.0004))
TRAY_WALL=.006
HANDLE_LOCAL=np.array((0.,.152,.007))
BIN_CENTER=np.array((.400,.360,0.))
BIN_INNER=np.array((.270,.220))
BIN_FLOOR=.004
BIN_RIM=.120
N_CRUMBS=40
TOASTER,TRAY=0,1
CRUMBS=np.arange(2,2+N_CRUMBS)
LINKS=('toaster','tray')+tuple(f'crumb{i:03d}' for i in range(N_CRUMBS))
# Actual repeatable dried bread chunks, not ball surrogates for dust.
CRUMB_SIZES=np.array([(0.004+0.00045*(i%5),.003+0.0004*((i*3)%5),.002+0.0003*((i*7)%4)) for i in range(N_CRUMBS)])
CRUMB_DENSITY=180. # kg/m^3, dry aerated bread estimate, not calibrated material data.
CRUMB_MASSES=np.prod(CRUMB_SIZES,axis=1)*CRUMB_DENSITY
TRAY_DENSITY=7800.
# 0.4 mm stainless floor and four 6 mm flanges, ordinary ABS pull handle.
TRAY_PARTS=[(TRAY_SIZE,(0.,0.,TRAY_SIZE[2]/2),'steel')]
for x in [-1,1]:TRAY_PARTS.append(((.0004,.260,TRAY_WALL),(x*(.070-.0002),0.,TRAY_WALL/2),'steel'))
for y in [-1,1]:TRAY_PARTS.append(((.140,.0004,TRAY_WALL),(0.,y*(.130-.0002),TRAY_WALL/2),'steel'))
TRAY_PARTS.append(((.024,.012,.006),(0.,.136,.003),'plastic'))
TRAY_PARTS.append(((.044,.020,.014),HANDLE_LOCAL,'plastic'))
_pm=np.array([np.prod(s)*(TRAY_DENSITY if m=='steel' else 1050.) for s,p,m in TRAY_PARTS])
TRAY_MASS=float(_pm.sum())
TRAY_COM=np.average(np.array([p for s,p,m in TRAY_PARTS]),axis=0,weights=_pm)
TRAY_INERTIA=np.zeros(3)
for mass,(size,pos,mat) in zip(_pm,TRAY_PARTS):
 s=np.asarray(size);d=np.asarray(pos)-TRAY_COM
 TRAY_INERTIA+=mass*((np.sum(s*s)-s*s)/12+np.dot(d,d)-d*d)
MASSES=np.r_[1.60,TRAY_MASS,CRUMB_MASSES]
YAW=np.pi/2
RZ=np.array(((0.,-1.,0.),(1.,0.,0.),(0.,0.,1.)))
START_POSES=np.zeros((len(LINKS),7));START_POSES[:,5]=np.sin(YAW/2);START_POSES[:,6]=np.cos(YAW/2)
START_POSES[TOASTER,:3]=TOASTER_CENTER;START_POSES[TRAY,:3]=TRAY_CENTER
for i in range(N_CRUMBS):
 x=-.047+(i%5)*.0235;y=-.100+(i//5)*(.200/7)
 START_POSES[2+i,:3]=TRAY_CENTER+RZ@np.array([x,y,TRAY_SIZE[2]+CRUMB_SIZES[i,2]/2+.0006])
RESET_POSES=dict(zip(LINKS,START_POSES))


def quat_matrix(q):
 x,y,z,w=np.asarray(q,dtype=float)
 return np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))


def ry(t):
 c,s=np.cos(t),np.sin(t);return np.array(((c,0,s),(0,1,0),(-s,0,c)))


def box_corners(size):
 return np.array([(x,y,z) for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*np.asarray(size)
