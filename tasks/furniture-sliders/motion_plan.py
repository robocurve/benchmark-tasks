"""CPU-only conservative joint-path planning around the stock scene.

Link-local bounding boxes enclose every native collision-hull vertex. SAT checks
between boxes are conservative; this is a path screen, not a dynamics guarantee.
"""
from functools import lru_cache
import numpy as np
from yam_arena import analysis,ik
from yam_arena.constants import ARM_JOINTS
from yam_arena.station import camera_stand,top_camera
from scoring import rotation
from config import S,F,FOOT_CENTERS,T


def released_parking_goal(furniture_pose,side):
    """Choose a nearby stock-limit pose with actual hand-to-furniture clearance.

    Native READY is only 46 mm clear of the tilted table after the first quarter
    turn. An outward base-joint rotation provides the required released gap.
    The caller still screens the complete approach against the other arm/scene.
    """
    from yam_arena.constants import READY
    pos=np.asarray(furniture_pose[:3]);r=rotation(furniture_pose[3:])
    w,d,h=F['size_m'];t=F['top_thickness_m'];fw=F['foot_width_m']
    boxes=[((0,0,h-t/2),(w,d,t))]+[((x,y,(h-t)/2),(fw,fw,h-t)) for x,y,_ in FOOT_CENTERS]
    base=(0,.305 if side=='left' else -.305,.75)
    for outward in (0.,.1,.2,.3):
        q=np.array(READY);q[0]=outward*(1 if side=='left' else -1)
        poses=analysis.fk_link_poses_world(dict(zip(ARM_JOINTS,q))|{'joint7':-.04695,'joint8':-.04695},base)
        vertices=np.concatenate([hull.vertices for _,hull in analysis.world_hulls(poses,links=['gripper','tip_left','tip_right'])])
        clearance=min(float(analysis.box_signed_distance(vertices,pos+r@np.asarray(c),r,size).min()) for c,size in boxes)
        if clearance>=T['release_clearance_m']+.005:return q
    raise RuntimeError('No released parking pose with sufficient furniture clearance')


@lru_cache(None)
def bounds():
    result={}
    for name,hulls in analysis.link_hulls().items():
        v=np.concatenate([h.vertices for h in hulls])
        lo,hi=v.min(0),v.max(0)
        result[name]=((lo+hi)/2,(hi-lo)/2)
    return result


def arm_boxes(q,side,grip=1.):
    base=(0,.305 if side=='left' else -.305,.75)
    poses=analysis.fk_link_poses_world(dict(zip(ARM_JOINTS,q))|{'joint7':-.04695*grip,'joint8':-.04695*grip},base)
    return [(name,t[:3,3]+t[:3,:3]@bounds()[name][0],t[:3,:3],bounds()[name][1])
            for name,t in poses.items() if name in bounds()]


@lru_cache(None)
def hull_bounds(name):
    return [((h.vertices.min(0)+h.vertices.max(0))/2,(h.vertices.max(0)-h.vertices.min(0))/2)
            for h in analysis.link_hulls()[name]]


def self_overlap(q,side,a,b,grip=1.):
    base=(0,.305 if side=='left' else -.305,.75)
    poses=analysis.fk_link_poses_world(dict(zip(ARM_JOINTS,q))|{'joint7':-.04695*grip,'joint8':-.04695*grip},base)
    def small_boxes(name):
        pose=poses[name]
        return [(name,pose[:3,3]+pose[:3,:3]@c,pose[:3,:3],h) for c,h in hull_bounds(name)]
    return any(overlap(x,y,margin=0.) for x in small_boxes(a) for y in small_boxes(b))


def link_overlap(q,side,name,obstacle,grip):
    base=(0,.305 if side=='left' else -.305,.75)
    poses=analysis.fk_link_poses_world(dict(zip(ARM_JOINTS,q))|{'joint7':-.04695*grip,'joint8':-.04695*grip},base)
    p=poses[name]
    return any(overlap((name,p[:3,3]+p[:3,:3]@c,p[:3,:3],h),obstacle) for c,h in hull_bounds(name))


def overlap(a,b,margin=.002):
    _,pa,ra,ha=a; _,pb,rb,hb=b
    delta=pb-pa
    if np.any(abs(delta)>abs(ra)@ha+abs(rb)@hb+margin): return False
    axes=np.concatenate((ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(9,3)))
    separation=abs(axes@delta)-(abs(axes@ra)@ha+abs(axes@rb)@hb)
    return not np.any(separation>margin*np.linalg.norm(axes,axis=1))


class World:
    def __init__(self,furniture_pose,other_q,side,grip=1.,narrow=False):
        pos=np.asarray(furniture_pose[:3]); r=rotation(furniture_pose[3:])
        w,d,h=F['size_m']; t=F['top_thickness_m']; fw=F['foot_width_m']
        self.obstacles=[('slab',pos+r@np.array((0,0,h-t/2)),r,np.array((w,d,t))/2)]
        for i,(x,y,_) in enumerate(FOOT_CENTERS):
            self.obstacles.append((f'leg_{i}',pos+r@np.array((x,y,(h-t)/2)),r,np.array((fw,fw,h-t))/2))
        self.obstacles += [('station_table',np.array((.36,0,.7175)),np.eye(3),np.array((1.1,1.3,.06))/2),
                           ('floor_panel',np.array((.46,0,.7525)),np.eye(3),np.array((.72,1.1,.01))/2)]
        for ya,yb in ((-.55,-.415),(-.195,.195),(.415,.55)):
            self.obstacles.append(('panel_back',np.array((.06,(ya+yb)/2,.7525)),np.eye(3),np.array((.08,yb-ya,.01))/2))
        for name,(kind,size,p,q) in camera_stand(top_camera()).items():
            size=np.asarray(size) if kind=='box' else np.array((size[0]*2,size[0]*2,size[1]+size[0]*2))
            self.obstacles.append((name,np.asarray(p),rotation(q),size/2))
        self.obstacles+=arm_boxes(other_q,'left' if side=='right' else 'right')
        self.side=side; self.grip=grip; self.narrow=narrow; self.cache={}; self.last_collision=None

    def clear(self,q):
        key=tuple(np.round(q,7))
        if key in self.cache:return self.cache[key]
        boxes=arm_boxes(q,self.side,self.grip)
        clear=True
        for a in boxes:
            if a[0]=='base':continue
            for b in self.obstacles:
                if overlap(a,b) and (not self.narrow or link_overlap(q,self.side,a[0],b,self.grip)):
                    self.last_collision=(a[0],b[0]);clear=False;break
            if not clear:break
        # Adjacent chain elements and both finger assemblies intentionally touch.
        order={'base':0,'link1':1,'link2':2,'link3':3,'link4':4,'link5':5,'gripper':6,'tip_left':7,'tip_right':7}
        if clear:
            for i,a in enumerate(boxes):
                for b in boxes[i+1:]:
                    if (abs(order.get(a[0],0)-order.get(b[0],0))>2 and overlap(a,b,margin=0.)
                            and self_overlap(q,self.side,a[0],b[0],self.grip)):
                        self.last_collision=(a[0],b[0]);clear=False;break
                if not clear:break
        self.cache[key]=clear
        return clear

    def edge(self,a,b,resolution=.06):
        count=max(1,int(np.ceil(np.max(abs(b-a))/resolution)))
        return all(self.clear(a+(b-a)*t/count) for t in range(1,count+1))


def path(start,goal,world,seed=7,max_iterations=2200,resolution=.06):
    start=np.asarray(start); goal=np.asarray(goal)
    if not world.clear(start):raise RuntimeError(f'Path starts in screened collision: {world.last_collision}')
    if not world.clear(goal):raise RuntimeError(f'Path goal in screened collision: {world.last_collision}')
    # Direct routes need the same final screen as tree-generated routes.
    if world.edge(start,goal,resolution=.017):return [start,goal]
    rng=np.random.default_rng(seed)
    trees=[([start],[-1]),([goal],[-1])]

    def extend(tree,target):
        nodes,parents=tree
        near=int(np.argmin(np.linalg.norm(np.asarray(nodes)-target,axis=1)))
        delta=target-nodes[near]; length=np.linalg.norm(delta)
        new=nodes[near]+delta*min(1.,.35/max(length,1e-12))
        if not world.edge(nodes[near],new,resolution=resolution):return None,False
        nodes.append(new); parents.append(near)
        return len(nodes)-1,length<=.35

    def chain(tree,index):
        nodes,parents=tree;out=[]
        while index>=0:out.append(nodes[index]);index=parents[index]
        return out[::-1]

    found=None
    for iteration in range(max_iterations):
        side=iteration%2
        target=trees[1-side][0][0] if rng.random()<.15 else rng.uniform(ik.LOWER+.02,ik.UPPER-.02)
        ia,_=extend(trees[side],target)
        if ia is None:continue
        for _ in range(40):
            ib,done=extend(trees[1-side],trees[side][0][ia])
            if ib is None:break
            if done:
                a,b=chain(trees[side],ia),chain(trees[1-side],ib)
                found=a+b[-2::-1] if side==0 else b+a[-2::-1]
                break
        if found is not None:break
    if found is None:raise RuntimeError(f'No collision-screened path in {max_iterations} RRT iterations')
    for _ in range(100):
        if len(found)<3:break
        i,j=sorted(rng.choice(len(found),2,replace=False))
        if j>i+1 and world.edge(found[i],found[j],resolution=min(.025,resolution)):found=found[:i+1]+found[j:]
    if not all(world.edge(a,b,resolution=.017) for a,b in zip(found,found[1:])):
        if resolution>.017:
            # Reject the aliased candidate before issuing actions, and replan
            # with the finer screen on every extension and shortcut.
            return path(start,goal,world,seed=seed,max_iterations=max_iterations,resolution=.017)
        raise RuntimeError('Fine-resolution path check failed')
    return found
