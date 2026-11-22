"""Mechanical endpoint checks using native simulation state."""
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.analysis import link_hulls
from threads import RIM
from specimen import SECTORS, BORE, BEAD, FLANGE, LIP_HALF
from bearing import inspect_pair

SEATING_GAP_MM = .05

def radial_margin(mm):
    angle=np.arctan2(mm[...,1],mm[...,0])
    phase=angle%(2*np.pi/SECTORS)-np.pi/SECTORS
    bore=BORE*np.cos(np.pi/SECTORS)/np.cos(phase)
    return np.linalg.norm(mm[...,:2],axis=-1)-bore


def metrics(nodes,poses,grid):
    mm=1000*np.einsum('fnj,fjk->fnk',nodes-poses[:,None,:3],Rotation.from_quat(poses[:,3:]).as_matrix())
    edge=mm[:,grid[:,BEAD,1]];flange=mm[:,grid[:,FLANGE,:]].mean(2)
    raw=(edge[:,:,2]>LIP_HALF)&(radial_margin(edge)>0)&(flange[:,:,2]<-LIP_HALF)&(radial_margin(flange)>0)
    # For a retained sector its wall must cross the throat plane inside the physical aperture.
    wall=mm[:,grid[:,:BEAD+1,1]];p=wall[:,:,:-1];q=wall[:,:,1:];dz=q[:,:,:,2]-p[:,:,:,2]
    u=-p[:,:,:,2]/np.where(np.abs(dz)>1.e-8,dz,1.)
    xy=p+u[...,None]*(q-p);inside=((np.abs(dz)>1.e-8)&(u>=0)&(u<=1)&(radial_margin(xy)<.3)).any(-1)
    angle=np.arctan2(edge[:,:,1],edge[:,:,0]);d=np.roll(angle,-1,axis=1)-angle
    winding=np.rint(np.arctan2(np.sin(d),np.cos(d)).sum(1)/(2*np.pi)).astype(int)
    return raw,raw&inside&(winding[:,None]==1),mm


def pad_clearance(nodes, poses, paths, indices, tetrahedra):
    """Lower bound between each complete soft-body and each supplied robot collision mesh at observed poses."""
    soft = nodes[indices]
    soft_lo = soft.min(axis=1)
    soft_hi = soft.max(axis=1)
    bounds = []
    for index, path in enumerate(paths):
        pose = poses[indices, index]
        matrix = Rotation.from_quat(pose[:, 3:]).as_matrix()
        vertices = np.vstack([h.vertices for h in link_hulls()[path.split('/')[-1]]])
        world = np.einsum('fij,nj->fni', matrix, vertices) + pose[:, None, :3]
        separation = np.maximum(soft_lo - world.max(axis=1), world.min(axis=1) - soft_hi)
        aabb = np.linalg.norm(np.maximum(separation, 0), axis=1)
        # Bound the complete mesh separation along additional unit axes. A positive
        # separation on any axis proves that the enclosed shapes do not intersect.
        delta = world.mean(axis=1) - soft.mean(axis=1)
        horizontal = delta.copy()
        horizontal[:, 2] = 0
        axes = np.concatenate((matrix.transpose(0, 2, 1), delta[:, None], horizontal[:, None]), axis=1)
        axes /= np.maximum(1e-14, np.linalg.norm(axes, axis=-1, keepdims=True))
        soft_proj = np.einsum('faj,fnj->fan', axes, soft)
        pad_proj = np.einsum('faj,fnj->fan', axes, world)
        axis_gap = np.maximum(pad_proj.min(axis=2) - soft_proj.max(axis=2),
                              soft_proj.min(axis=2) - pad_proj.max(axis=2))
        whole_gap = np.maximum(aabb, axis_gap.max(axis=1))
        # Where the envelope crosses a finger, separate each tetrahedron from each
        # convex pad component in the finger's own frame. This also covers faces.
        hulls = link_hulls()[path.split('/')[-1]]
        for i in np.where(whole_gap < .001)[0]:
            cells = ((soft[i] - pose[i, :3]) @ matrix[i])[tetrahedra]
            lo, hi = cells.min(axis=1), cells.max(axis=1)
            piece_gaps = []
            for hull in hulls:
                sep = np.maximum(lo - hull.vertices.max(axis=0), hull.vertices.min(axis=0) - hi)
                piece_gaps.append(np.linalg.norm(np.maximum(sep, 0), axis=1).min())
            whole_gap[i] = max(whole_gap[i], min(piece_gaps))
        bounds.append(whole_gap)
    return float(np.min(bounds) * 1000)


def first_surface_z(xy, triangles):
    """Lowest surface hit by each vertical line, including triangle edges."""
    a, b, c = triangles.transpose(1, 0, 2)
    ab, ac = b - a, c - a
    det = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
    usable = np.abs(det) > 1e-10
    indices = np.flatnonzero(usable)
    heights = []
    for point in xy:
        v = point - a[indices, :2]
        u = (v[:, 0] * ac[indices, 1] - v[:, 1] * ac[indices, 0]) / det[indices]
        w = (ab[indices, 0] * v[:, 1] - ab[indices, 1] * v[:, 0]) / det[indices]
        hit = (u >= -1e-8) & (w >= -1e-8) & (u + w <= 1 + 1e-8)
        zs = a[indices, 2] + u * ab[indices, 2] + w * ac[indices, 2]
        heights.append(zs[hit].min(initial=np.inf))
    return np.array(heights)


def flange_gaps(poses, nodes, grid, tets):
    """Sample the actual lower flange skin above a circle on the bottle lip, in mm."""
    # Only the first two radial strips on the underside form the sealing land.
    # Intersection with another part of a folded teat must not stand in for this surface.
    faces = tets[:, [[0, 1, 2], [0, 2, 3], [0, 1, 3], [1, 2, 3]]].reshape(-1, 3)
    faces, counts = np.unique(np.sort(faces, axis=1), axis=0, return_counts=True)
    land = faces[(counts == 1) & np.isin(faces, grid[:, :3, 0]).all(axis=1)]
    if len(land) != 4 * len(grid):
        raise ValueError("Unexpected silicone sealing-land topology")
    angle = (np.arange(len(grid)) + .5) * (2*np.pi / len(grid))
    xy = 24.5 * np.column_stack((np.cos(angle), np.sin(angle)))
    rim_height = max(z for _, z in RIM)
    gaps = []
    for pose, points in zip(poses, nodes, strict=True):
        local = Rotation.from_quat(pose[3:]).inv().apply(points - pose[:3]) * 1000
        gaps.append(first_surface_z(xy, local[land]) - rim_height)
    return np.array(gaps)


def evaluate(window):
    """Evaluate the last second, independent of arm roles, control plan or lift history."""
    poses, nodes = window['poses'], window['soft']
    contacts, tips = window['contacts'], window['tips']
    if len(poses) != 21 or len(contacts) != 240:
        raise ValueError("Expected 21 observations and 240 intervening physics substeps")
    if not all(np.isfinite(x).all() for x in (poses, nodes, tips, contacts)):
        raise ValueError("Invalid physics record")
    collar = poses[:, window['names'].index('collar')]
    bottle = poses[:, window['names'].index('bottle')]
    gaps = flange_gaps(bottle, nodes, window['grid'], window['tets'])
    rim_counts = (np.abs(gaps) <= SEATING_GAP_MM).sum(axis=1)
    captured = metrics(nodes, collar, window['grid'])[1].sum(axis=1)
    bearings = [[round(lug['under_fraction'] * 15) for lug in inspect_pair(c, b)['lugs']]
                for c, b in zip(collar, bottle, strict=True)]
    flanks = np.min(bearings, axis=0)
    free = bool(not contacts.any() and pad_clearance(nodes, tips, window['paths'],
                np.arange(len(poses)), window['tets']) > 0)
    secured = bool((flanks >= 12).all())
    seated = bool((rim_counts == 48).all() and (captured == 48).all())
    return dict(score=int(free and secured and seated), released=free,
                sealing_points_min=int(rim_counts.min()), collar_sectors_min=int(captured.min()),
                thread_samples_min=flanks.tolist())


def read_window(run, end_frame=None):
    with np.load(run/'physics.npz') as d:
        poses = d['poses']
        end = len(poses)-1 if end_frame is None else end_frame
        assert 20 <= end < len(poses)
        frames = slice(end-20,end+1)
        substeps = slice((end-20)*12,end*12)
        names = [s.split('/')[-1] for s in d['objects']]
        return dict(poses=poses[frames],soft=d['nodes'][frames],tips=d['arm_poses' if 'arm_poses' in d else 'tips'][frames],
                    paths=d['arm_paths' if 'arm_paths' in d else 'tip_paths'],grid=d['grid'],tets=d['tets'],names=names,
                    contacts=d['robotcounts'][substeps])
