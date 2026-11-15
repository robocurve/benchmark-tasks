"""Opt-in fused thread/cloth predicate prototype; not enabled by the runner.

Double precision and the CPU audit's exact bounds/tolerances are retained.
Every invocation evaluates a native substep. A latched first failing step and
mask matrix allow the caller to preserve the corresponding state before the
next substep. This module does not drive physics or decide task scores.
"""
import numpy as np
import warp as wp

wp.set_module_options({'enable_backward':False,'fast_math':False,'fuse_fp':False})


@wp.func
def _hit(a:wp.vec3d,b:wp.vec3d,v0:wp.vec3d,v1:wp.vec3d,v2:wp.vec3d):
    e1=v1-v0;e2=v2-v0;d=b-a;s=a-v0
    h=wp.cross(d,e2);det=wp.dot(e1,h)
    result=False
    if wp.abs(det)>wp.float64(1.e-20):
        inv=wp.float64(1.)/det
        u=wp.dot(s,h)*inv;q=wp.cross(s,e1)
        v=wp.dot(d,q)*inv;t=wp.dot(e2,q)*inv
        eps=wp.float64(1.e-7)
        result=u>=-eps and v>=-eps and u+v<=wp.float64(1.)+eps and t>=-eps and t<=wp.float64(1.)+eps
    return result


@wp.kernel
def crossing_kernel(points:wp.array(dtype=wp.vec3d),old_points:wp.array(dtype=wp.vec3d),
                    faces:wp.array(dtype=wp.vec3i),
                    segments:wp.array2d(dtype=wp.vec3d),old_segments:wp.array2d(dtype=wp.vec3d),
                    has_previous:int,step:int,first_step:wp.array(dtype=wp.int32),
                    masks:wp.array2d(dtype=wp.int32)):
    segment,face=wp.tid()
    # Once invalid, preserve the complete first-failure matrix. A caller may
    # check the latch at the action boundary without losing the substep.
    if first_step[0]<step:
        return
    f=faces[face]
    p0=points[f[0]];p1=points[f[1]];p2=points[f[2]]
    o0=old_points[f[0]];o1=old_points[f[1]];o2=old_points[f[2]]
    a=segments[segment,0];b=segments[segment,1]
    oa=old_segments[segment,0];ob=old_segments[segment,1]
    candidate=True;finite=True
    for axis in range(3):
        if not (wp.isfinite(p0[axis]) and wp.isfinite(p1[axis]) and wp.isfinite(p2[axis]) and
                wp.isfinite(a[axis]) and wp.isfinite(b[axis])):
            finite=False
        low=wp.min(a[axis],b[axis]);high=wp.max(a[axis],b[axis])
        tl=wp.min(wp.min(p0[axis],p1[axis]),p2[axis]);th=wp.max(wp.max(p0[axis],p1[axis]),p2[axis])
        if has_previous!=0:
            low=wp.min(low,wp.min(oa[axis],ob[axis]));high=wp.max(high,wp.max(oa[axis],ob[axis]))
            tl=wp.min(tl,wp.min(wp.min(o0[axis],o1[axis]),o2[axis]))
            th=wp.max(th,wp.max(wp.max(o0[axis],o1[axis]),o2[axis]))
        if high<tl-wp.float64(1.e-10) or low>th+wp.float64(1.e-10):
            candidate=False
    mask=int(0)
    if not finite:
        mask=4
    elif candidate:
        if _hit(a,b,p0,p1,p2):mask=mask|1
        if has_previous!=0:
            displacement=p0-o0
            if (_hit(oa,a-displacement,o0,o1,o2) or _hit(oa+displacement,a,p0,p1,p2) or
                    _hit(ob,b-displacement,o0,o1,o2) or _hit(ob+displacement,b,p0,p1,p2)):
                mask=mask|2
    masks[segment,face]=mask
    if mask!=0:
        wp.atomic_min(first_step,0,step)


def crossing_matrix(points,faces,segments,*,previous_points=None,previous_segments=None,device='cpu'):
    """Standalone comparator/benchmark, with explicit host transfers.

    This convenience wrapper is deliberately not the native runner integration.
    The future runner must retain its buffers on the GPU and preserve the first
    failing pair of states before checking the latch once per control action.
    """
    points=np.asarray(points,np.float64);faces=np.asarray(faces,np.int32);segments=np.asarray(segments,np.float64)
    prior=previous_points is not None
    if prior!=(previous_segments is not None):raise ValueError('Both previous states are required')
    old_points=points if not prior else np.asarray(previous_points,np.float64)
    old_segments=segments if not prior else np.asarray(previous_segments,np.float64)
    if (points.ndim!=2 or points.shape[1:]!=(3,) or faces.ndim!=2 or faces.shape[1:]!=(3,) or
            segments.ndim!=3 or segments.shape[1:]!=(2,3) or old_points.shape!=points.shape or
            old_segments.shape!=segments.shape or not len(points) or not len(faces) or not len(segments)):
        raise ValueError('Invalid geometry shape')
    if not all(np.isfinite(x).all() for x in (points,old_points,segments,old_segments)):
        raise ValueError('Nonfinite geometry')
    if faces.min()<0 or faces.max()>=len(points):raise ValueError('Invalid triangle index')
    args=[wp.array(points,dtype=wp.vec3d,device=device),wp.array(old_points,dtype=wp.vec3d,device=device),
          wp.array(faces,dtype=wp.vec3i,device=device),wp.array(segments,dtype=wp.vec3d,device=device),
          wp.array(old_segments,dtype=wp.vec3d,device=device)]
    latch=wp.full(1,2**31-1,dtype=wp.int32,device=device)
    result=wp.zeros((len(segments),len(faces)),dtype=wp.int32,device=device)
    wp.launch(crossing_kernel,dim=result.shape,inputs=[*args,int(prior),1,latch,result],device=device)
    return result.numpy(),int(latch.numpy()[0])


@wp.kernel
def _remember_points(current:wp.array(dtype=wp.vec3d),previous:wp.array(dtype=wp.vec3d),
                     saved_current:wp.array(dtype=wp.vec3d),saved_previous:wp.array(dtype=wp.vec3d),
                     first_step:wp.array(dtype=wp.int32),step:int):
    i=wp.tid()
    if first_step[0]==step:
        saved_current[i]=current[i];saved_previous[i]=previous[i]
    previous[i]=current[i]


@wp.kernel
def _remember_segments(current:wp.array2d(dtype=wp.vec3d),previous:wp.array2d(dtype=wp.vec3d),
                       saved_current:wp.array2d(dtype=wp.vec3d),saved_previous:wp.array2d(dtype=wp.vec3d),
                       first_step:wp.array(dtype=wp.int32),step:int):
    i,j=wp.tid()
    if first_step[0]==step:
        saved_current[i,j]=current[i,j];saved_previous[i,j]=previous[i,j]
    previous[i,j]=current[i,j]


class PersistentCrossingAudit:
    """Evaluate every substep on persistent device buffers; poll at 20 Hz.

    The existing CPU material/blade audits still need measured host nodes, so
    this first integration uploads those exact measurements. It removes the
    costly Python triangle tests while preserving the same inputs. Host/device
    buffers persist throughout the trial. No first-failure state is overwritten.
    """
    def __init__(self,meshes,segment_count,dt,*,device='cuda:0'):
        device=wp.get_device(device)
        self.meshes=meshes;self.device=device;self.dt=float(dt);self.updates=0;self.last_step=-1
        self.upload_event=wp.Event(device) if device.is_cuda else None
        self.first_step=wp.full(1,2**31-1,dtype=wp.int32,device=device)
        self.layers=[];self._summaries=None;self.failure_pair=None
        def vector_buffers(shape):
            return [wp.zeros(shape,dtype=wp.vec3d,device=device) for _ in range(4)]
        self.segments,self.previous_segments,self.saved_segments,self.saved_previous_segments=vector_buffers((segment_count,2))
        self.host_segments=np.zeros((segment_count,2,3),np.float64)
        self.host_segments_wp=wp.array(self.host_segments,dtype=wp.vec3d,device='cpu',copy=False)
        for mesh in meshes:
            host=np.zeros_like(mesh.rest,dtype=np.float64)
            self.layers.append({'host':host,'host_wp':wp.array(host,dtype=wp.vec3d,device='cpu',copy=False),
                'buffers':vector_buffers(len(host)),
                'faces':wp.array(np.asarray(mesh.faces,np.int32),dtype=wp.vec3i,device=device),
                'masks':wp.zeros((segment_count,len(mesh.faces)),dtype=wp.int32,device=device)})

    def update(self,points,segments,native_step):
        step=int(native_step)
        if step!=self.last_step+1 or len(points)!=len(self.layers):raise ValueError('Native substep coverage gap')
        if np.shape(segments)!=self.host_segments.shape:raise ValueError('Incomplete thread measurement')
        # Reuse host memory only after its preceding uploads have completed.
        # The event precedes the expensive predicate kernels, so this does not
        # impose a per-substep device-result readback or wait for those kernels.
        if self.upload_event is not None and self.updates:wp.synchronize_event(self.upload_event)
        np.copyto(self.host_segments,segments)
        wp.copy(self.segments,self.host_segments_wp)
        for measured,layer in zip(points,self.layers,strict=True):
            if np.shape(measured)!=layer['host'].shape:raise ValueError('Incomplete cloth measurement')
            np.copyto(layer['host'],measured)
            current,previous,_,_=layer['buffers']
            wp.copy(current,layer['host_wp'])
        if self.upload_event is not None:wp.record_event(self.upload_event)
        for layer in self.layers:
            current,previous,_,_=layer['buffers']
            wp.launch(crossing_kernel,dim=layer['masks'].shape,inputs=[current,previous,layer['faces'],
                self.segments,self.previous_segments,int(step>0),step,self.first_step,layer['masks']],device=self.device)
        # These kernels execute after both layer predicates on the same stream.
        # Save first-failure states before advancing the previous-state buffers.
        for layer in self.layers:
            current,previous,saved,saved_previous=layer['buffers']
            wp.launch(_remember_points,dim=len(layer['host']),inputs=[current,previous,saved,saved_previous,
                       self.first_step,step],device=self.device)
        wp.launch(_remember_segments,dim=self.segments.shape,inputs=[self.segments,self.previous_segments,
            self.saved_segments,self.saved_previous_segments,self.first_step,step],device=self.device)
        self.last_step=step;self.updates+=1

    def summaries(self):
        """Synchronize the latch once per action; reconstruct exact diagnostics only on failure."""
        first=int(self.first_step.numpy()[0])
        if first==2**31-1:
            return [{'valid':True,'steps':self.updates,'first_failure':None} for _ in self.layers]
        if self._summaries is not None:return self._summaries
        from contact_audit import ThreadClothAudit
        now_segments=self.saved_segments.numpy();old_segments=self.saved_previous_segments.numpy()
        self.failure_pair={'current_thread_segments':now_segments,'previous_thread_segments':old_segments,
                           'time_s':first*self.dt,'dt':self.dt,'native_substep':first}
        self._summaries=[]
        for i,(mesh,layer) in enumerate(zip(self.meshes,self.layers,strict=True)):
            _,_,saved,saved_previous=layer['buffers']
            current,previous=saved.numpy(),saved_previous.numpy()
            side=('left','right')[i]
            self.failure_pair.update({f'current_{side}':current,f'previous_{side}':previous,f'{side}_faces':mesh.faces})
            mask=layer['masks'].numpy()
            if np.any(mask&4):
                self._summaries.append({'valid':False,'steps':first+1,'first_failure':
                    {'native_substep':first,'time_s':first*self.dt,'reason':'nonfinite native geometry'}})
                continue
            replay=ThreadClothAudit(mesh.faces)
            if first>0:
                replay.update(previous,old_segments,(first-1)*self.dt)
                if replay.first_failure is not None:
                    raise RuntimeError('GPU/CPU mismatch: first saved previous state is already invalid')
            replay.update(current,now_segments,first*self.dt)
            failure=replay.first_failure
            if bool(mask.any())!=(failure is not None):raise RuntimeError('GPU/CPU failure snapshot mismatch')
            if failure is not None:
                if (int(np.count_nonzero(mask&1))!=failure['static_intersections'] or
                        int(np.count_nonzero(mask&2))!=failure['swept_intersections']):
                    raise RuntimeError('GPU/CPU first-failure intersection count mismatch')
                failure=dict(failure,native_substep=first,step=first+1)
            self._summaries.append({'valid':failure is None,'steps':first+1,'first_failure':failure})
        return self._summaries

    def save_failure(self,path):
        self.summaries()
        if self.failure_pair is not None:np.savez_compressed(path,**self.failure_pair)
