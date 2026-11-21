"""Conservative contact-transfer material state, separate from policy inputs.

Native contacts and tangential sliding supply work. A local one-pitch smear
kernel distributes wax into neighboring root surfaces; it cannot coat remote
angles or axial regions. This provisional law needs physical calibration.
"""
import numpy as np
import spec
class Coating:
    def __init__(self):
        self.angles=(np.arange(spec.ANGLE_CELLS)+.5)*2*np.pi/spec.ANGLE_CELLS
        self.z=(np.arange(spec.AXIAL_CELLS)+.5)*spec.BOLT_LENGTH/spec.AXIAL_CELLS
        self.theta,self.height=np.meshgrid(self.angles,self.z,indexing='ij')
        f=np.vectorize(spec.thread_radius)
        self.radius=f(self.theta,self.height)
        eps=1e-6;dz=(f(self.theta,self.height+eps)-f(self.theta,self.height-eps))/(2*eps)
        da=(f(self.theta+eps,self.height)-f(self.theta-eps,self.height))/(2*eps)
        self.area=np.sqrt(self.radius**2*(1+dz**2)+da**2)*(2*np.pi/spec.ANGLE_CELLS)*(spec.BOLT_LENGTH/spec.AXIAL_CELLS)
        self.film=np.zeros_like(self.area)
        self.initial_volume=float(np.prod(spec.WAX_SIZE));self.remaining_volume=self.initial_volume
        self.head_contaminated=False;self.film_disturbed=False;self.disturbance=None;self.native_contacts=0;self.sliding_contacts=0
    def check_nonapplicator_contact(self,point,colliders=None):
        """Wet material must not be touched by fingers, supports or the carrier.

        This is an observable cleanliness constraint, not simulated dry adhesion.
        A disturbed coating requires a fresh reset rather than assumed recovery.
        """
        point=np.asarray(point,float)
        if not np.isfinite(point).all():raise ValueError('Nonfinite contact')
        if not spec.TARGET_BAND[0]<=point[2]<=spec.TARGET_BAND[1]:return
        a=int((np.arctan2(point[1],point[0])%(2*np.pi))/(2*np.pi)*spec.ANGLE_CELLS)%spec.ANGLE_CELLS
        k=min(spec.AXIAL_CELLS-1,int(point[2]/spec.BOLT_LENGTH*spec.AXIAL_CELLS))
        self.film_disturbed |= bool(self.film[a,k]>=spec.FILM_VISIBLE_M)
        if self.film_disturbed and self.disturbance is None:
            self.disturbance={'local_point_m':point.tolist(),'colliders':colliders}
    def deposit(self,point,normal_force_n,sliding_m,*,native_contact):
        point=np.asarray(point,float)
        if not np.isfinite(point).all() or not np.isfinite([normal_force_n,sliding_m]).all():raise ValueError('Nonfinite contact')
        if not native_contact or normal_force_n<=0 or sliding_m<=0:return 0.
        self.native_contacts+=1
        if point[2]>=spec.BOLT_LENGTH:
            self.head_contaminated=True;return 0.
        if point[2]<0 or np.linalg.norm(point[:2])<spec.MINOR_RADIUS-.001:return 0.
        a=np.arctan2(point[1],point[0]);distance=np.abs((self.theta-a+np.pi)%(2*np.pi)-np.pi)
        mask=(distance<=np.deg2rad(12))&(abs(self.height-point[2])<=spec.PITCH/2)
        room=np.maximum(spec.FILM_CAP_M-self.film,0)*self.area*mask
        capacity=room.sum()
        volume=min(spec.TRANSFER_COEFFICIENT*normal_force_n*sliding_m,self.remaining_volume,capacity)
        if volume<=0:return 0.
        self.film+=room*(volume/capacity)/self.area
        self.remaining_volume-=volume;self.sliding_contacts+=1
        return float(volume)
    def measurements(self):
        band=(self.height>=spec.TARGET_BAND[0])&(self.height<=spec.TARGET_BAND[1])
        root=band&(self.radius<=spec.MINOR_RADIUS+.2*(spec.MAJOR_RADIUS-spec.MINOR_RADIUS))
        blue=self.film>=spec.FILM_VISIBLE_M
        visible_heights=self.height[blue]
        ratio=lambda m:float((self.area*m*blue).sum()/(self.area*m).sum())
        return {'coverage':ratio(band),'root_coverage':ratio(root),'head_clean':not self.head_contaminated,'film_disturbed':self.film_disturbed,
                'first_disturbance':self.disturbance,
                'coated_axial_interval_mm':[float(visible_heights.min()*1000),float(visible_heights.max()*1000)] if visible_heights.size else None,
                'used_volume_m3':float((self.film*self.area).sum()),'remaining_volume_m3':self.remaining_volume,
                'native_contacts':self.native_contacts,'sliding_contacts':self.sliding_contacts}
    def color_at(self,theta,z):
        a=(np.asarray(theta)%(2*np.pi)/(2*np.pi)*spec.ANGLE_CELLS).astype(int)%spec.ANGLE_CELLS
        k=np.clip((np.asarray(z)/spec.BOLT_LENGTH*spec.AXIAL_CELLS).astype(int),0,spec.AXIAL_CELLS-1)
        amount=np.clip(self.film[a,k]/spec.FILM_VISIBLE_M,0,1)[...,None]
        return (1-amount)*np.array([.12,.14,.17])+amount*np.array([.015,.12,.85])
