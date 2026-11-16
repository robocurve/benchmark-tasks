"""CPU-only URDF check of grasp arcs; not native grip/collision validation."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY
from closure_geometry import GRASPS,flap_target,rx
from carton_scene import layout
from config import CENTER



def gripper_triangles(aperture=.020):
    """Actual pinned URDF/STL collision triangles in the TCP frame."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import ARM_JOINTS,TCP_OFFSET
    from yam_arena.urdf import forward_kinematics,rpy_matrix
    values=dict(zip(ARM_JOINTS,READY));values.update(joint7=-aperture/2,joint8=-aperture/2)
    poses=forward_kinematics(YAM_URDF,values);inverse=np.linalg.inv(poses['gripper']);triangles=[];meshes=0
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        if link.get('name') not in ('gripper','tip_left','tip_right'):continue
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh');data=(YAM_URDF.parent/mesh.get('filename')).read_bytes();count=int.from_bytes(data[80:84],'little')
            if len(data)!=84+count*50:raise ValueError('expected binary STL in pinned YAM collision asset')
            dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            v=np.frombuffer(data,dtype=dtype,offset=84,count=count)['vertices'].astype(float)
            v*=np.array([float(x) for x in mesh.get('scale','1 1 1').split()])
            origin=collision.find('origin');position=np.zeros(3);rotation=np.eye(3)
            if origin is not None:
                position=np.array([float(x) for x in origin.get('xyz','0 0 0').split()])
                rotation=rpy_matrix(np.array([float(x) for x in origin.get('rpy','0 0 0').split()]))
            v=v@rotation.T+position;transform=inverse@poses[link.get('name')]
            triangles.append(v@transform[:3,:3].T+transform[:3,3]-np.asarray(TCP_OFFSET));meshes+=1
    return np.concatenate(triangles),meshes


def triangle_box_count(triangles,half):
    """Triangle/box SAT; reports surface crossings, not convex-hull guessing."""
    triangles=np.asarray(triangles);half=np.asarray(half)
    edges=np.roll(triangles,-1,axis=1)-triangles
    axes=np.concatenate((np.broadcast_to(np.eye(3),(len(triangles),3,3)),
                         np.cross(edges[:,0],edges[:,1])[:,None,:],
                         np.cross(edges[:,:,None,:],np.eye(3)[None,None,:,:]).reshape(-1,9,3)),axis=1)
    projections=np.einsum('naj,nvj->nav',axes,triangles);radius=np.sum(np.abs(axes)*half,axis=-1)
    separated=(projections.min(axis=-1)>radius)|(projections.max(axis=-1)<-radius)
    return int(np.count_nonzero(~np.any(separated,axis=1)))


def check():
    parts,hinges=layout();parts={p['name']:p for p in parts};rows=[];triangles,mesh_count=gripper_triangles();closed_triangles,_=gripper_triangles(.0015)
    for name,(arm,edge,tangent,tilt) in GRASPS.items():
        part=parts[name];q0=part['quat'];from yam_arena.urdf import quat_xyzw_to_matrix
        wr=quat_xyzw_to_matrix(np.r_[q0[1:],q0[0]]);wp=np.array(CENTER)+part['pos']
        hinge=next(h for h in hinges if h['name']==name+'_top_hinge');base=(0,.305 if arm=='left' else -.305,.75);q=np.array(READY)
        for angle in (-175.,-160.,-130.,-90.,-50.,-20.,0.):
            p,r,_=flap_target(wp,wr,hinge,parts[name+'_top']['size'],angle,edge,tangent,tilt)
            if angle==-175.:
                hover=p-r[:,2]*.040;q,pe,re=ik.solve(hover,ik.matrix_to_quat_xyzw(r),q,base_pos=base,iters=200)
                rows.append(dict(flap=name,arm=arm,phase='hover',angle_deg=angle,position_error_m=pe,rotation_error_rad=re))
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),q,base_pos=base,iters=200)
            fr=wr@rx(angle);fp=wp+wr@hinge['anchor_a']-fr@hinge['anchor_b'];local_triangles=(triangles@r.T+p-fp)@fr
            intersections=triangle_box_count(local_triangles,np.asarray(parts[name+'_top']['size'])/2)
            closed_crossings=triangle_box_count((closed_triangles@r.T+p-fp)@fr,np.asarray(parts[name+'_top']['size'])/2)
            rows.append(dict(flap=name,arm=arm,phase='rim',angle_deg=angle,position_error_m=pe,rotation_error_rad=re,open_gripper_board_triangle_crossings=intersections,closed_command_board_triangle_crossings=closed_crossings))
    return dict(kind='cpu_urdf_grasp_reach_only',passed=all(r['position_error_m']<(.004 if r['phase']=='hover' else .002) and r['rotation_error_rad']<.03 and r.get('open_gripper_board_triangle_crossings',0)==0 and r.get('closed_command_board_triangle_crossings',1)>0 for r in rows),samples=rows,
                gripper_collision_meshes=mesh_count,open_aperture_m=.020,
                max_position_error_m=max(r['position_error_m'] for r in rows),max_rotation_error_rad=max(r['rotation_error_rad'] for r in rows),native_contact_tested=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path);a=p.parse_args();result=check();text=json.dumps(result,indent=2)+'\n'
    if a.out:a.out.write_text(text)
    print(text);raise SystemExit(0 if result['passed'] else 1)
