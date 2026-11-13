"""Experimental, conservative icing model; this is not a robot demonstration.

A measured piston position compresses a finite reservoir. Pressure, yield stress
and nozzle resistance determine flow. Star-tip parcels fall under gravity and
transfer their entire volume into a free-surface grid on first contact. A
conservative, yield-limited surface flow allows the icing to spread/slump. No
target, shell count, rubric or planned nozzle path is visible to this model.

Limitations: a single-valued surface cannot represent overhangs, long suspended
threads or voids. The model is suitable for investigating low shell deposits,
not yet evidence of calibrated buttercream. Properties below are declared
engineering starting values, not measurements of a real recipe.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class IcingParameters:
    initial_volume_m3: float = 20e-6
    piston_radius_m: float = .0125
    bulk_modulus_pa: float = 160_000.
    yield_pressure_pa: float = 6_000.
    conductance_m3_per_pa_s: float = 3e-11
    nozzle_radius_m: float = .0025
    star_depth: float = .35
    star_lobes: int = 6
    density_kg_m3: float = 850.
    shear_yield_pa: float = 800.
    consistency_pa_sn: float = 4_000.
    flow_index: float = .5
    pressure_transfer_depth_m: float = .002
    maximum_substep_s: float = 1 / 480

    @property
    def piston_area_m2(self):
        return math.pi * self.piston_radius_m**2

    @property
    def stroke_m(self):
        return self.initial_volume_m3 / self.piston_area_m2

    def validate(self):
        values = vars(self)
        if not all(np.isfinite(v) and v > 0 for v in values.values()):
            raise ValueError("All material parameters must be finite and positive")
        if self.star_depth >= 1 or int(self.star_lobes) != self.star_lobes:
            raise ValueError("Invalid star nozzle")
        if self.flow_index > 1:
            raise ValueError("This icing model requires a shear-thinning or linear flow law")


def surface_flux(pressure_gradient_pa_m, depth_m, *, yield_pa, consistency_pa_sn, flow_index):
    """No-slip Herschel-Bulkley film flux per unit width, in m²/s.

    Integrate u(z) through a film with zero surface shear. The yielded depth is
    Y = max(h - tau_y / |grad(p)|, 0). The unyielded surface plug moves with
    the yielded lower layer. This is a shallow-film approximation; it cannot
    represent unsupported overhangs. No decoration geometry enters the law.
    """
    gradient=np.asarray(pressure_gradient_pa_m,float)
    h=np.asarray(depth_m,float)
    magnitude=np.abs(gradient)
    yielded=np.maximum(h-yield_pa/np.maximum(magnitude,1e-30),0.)
    power=1./flow_index
    flux=(magnitude/consistency_pa_sn)**power*(
        yielded**(2+power)/(2+power)
        +(h-yielded)*yielded**(1+power)/(1+power))
    return np.sign(gradient)*flux


class Icing:
    """State is SI units. X/Y grid coordinates are cell centres, in world space."""

    def __init__(self, params=None, *, dx=.0005, xlim=(.18, .42), ylim=(-.12, .12),
                 cake_bounds=(.21, .39, -.07, .07), cake_top=.8005,
                 table_top=.7475):
        self.p = params or IcingParameters()
        self.p.validate()
        if not np.isfinite(dx) or dx <= 0:
            raise ValueError("Cell spacing must be finite and positive")
        self.dx = float(dx)
        self.x = np.arange(xlim[0] + dx/2, xlim[1], dx)
        self.y = np.arange(ylim[0] + dx/2, ylim[1], dx)
        self.X, self.Y = np.meshgrid(self.x, self.y, indexing="ij")
        x0, x1, y0, y1 = cake_bounds
        self.on_cake = ((self.X >= x0) & (self.X <= x1) &
                        (self.Y >= y0) & (self.Y <= y1))
        self.base = np.where(self.on_cake, cake_top, table_top)
        self.cake_top = cake_top
        self.table_top = table_top
        self.reset()
        # Resolve the aperture at the material-grid scale. A fixed set of 72
        # parcel origins leaves artificial gaps when the grid is refined.
        # Polar annuli include the centre; their exact area weights preserve
        # flow volume even though each cell is represented by one centroid.
        maximum_radius=self.p.nozzle_radius_m*(1+self.p.star_depth)
        angles=max(48,24*math.ceil(2*math.pi*maximum_radius/self.dx/24))
        rings=max(3,math.ceil(maximum_radius/self.dx))
        theta = np.arange(angles) * 2*math.pi/angles
        outer = self.p.nozzle_radius_m*(1 + self.p.star_depth*np.cos(self.p.star_lobes*theta))
        edges=np.linspace(0.,1.,rings+1)
        radial_area=np.diff(edges**2)
        radial=(2./3.)*np.diff(edges**3)/radial_area
        self.nozzle_xy = np.array([[r*o*math.cos(t), r*o*math.sin(t)]
                                  for t, o in zip(theta, outer) for r in radial])
        self.weights = (outer[:,None]**2*radial_area[None,:]).ravel()
        self.weights /= self.weights.sum()
        self.aperture_area_m2 = float(math.pi*np.mean(outer**2))

    def reset(self):
        self.volume = np.zeros_like(self.base)
        self.air_pos = np.empty((0, 3))
        self.air_vel = np.empty((0, 3))
        self.air_volume = np.empty(0)
        self.emitted_m3 = 0.
        self.lost_m3 = 0.
        self.pressure_pa = 0.
        self.last_flow_m3_s = 0.
        self.cake_damage_depth_m = 0.
        self.time_s = 0.
        self.nozzle_position = None
        self.nozzle_rotation = None
        self.surface_transport_m3=0.
        self.implicit_surface_edges=0

    @property
    def height(self):
        return self.volume/self.dx**2

    def reservoir_pressure(self, piston_m):
        if not np.isfinite(piston_m) or not -1e-5 <= piston_m <= self.p.stroke_m+1e-5:
            raise ValueError("Measured piston position outside the physical stroke")
        compression = max(0., self.p.piston_area_m2*float(piston_m)-self.emitted_m3)
        return self.p.bulk_modulus_pa * compression / self.p.initial_volume_m3

    def step(self, dt, *, piston_m, nozzle_pos, nozzle_rotation, nozzle_velocity=(0.,0.,0.)):
        """Advance material using measured dispenser state, never policy commands.

        The nozzle's local +Z is the outward flow axis. Return pressure reaction
        in newtons for the piston and equal/opposite force for its barrel.
        """
        dt = float(dt)
        pos, rot, vel = (np.asarray(v, float) for v in
                         (nozzle_pos, nozzle_rotation, nozzle_velocity))
        if (not np.isfinite(dt) or dt <= 0 or pos.shape != (3,) or rot.shape != (3,3)
                or vel.shape != (3,) or not all(np.isfinite(a).all() for a in (pos,rot,vel))
                or not np.allclose(rot.T@rot, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rot), 1., atol=1e-6)):
            raise ValueError("Finite position, proper rotation and positive dt required")
        self.nozzle_position,self.nozzle_rotation=pos.copy(),rot.copy()
        n = max(1, math.ceil(dt/self.p.maximum_substep_s))
        h = dt/n
        total = 0.
        for _ in range(n):
            self.pressure_pa = self.reservoir_pressure(piston_m)
            drive = max(0., self.pressure_pa-self.p.yield_pressure_pa)
            remaining = max(0., self.p.initial_volume_m3-self.emitted_m3)
            v = min(remaining, self.p.conductance_m3_per_pa_s*drive*h)
            if v > 0:
                # Parcels originate inside the physical aperture; all flow is
                # governed by pressure and the remaining reservoir volume.
                local = np.column_stack((self.nozzle_xy, np.zeros(len(self.weights))))
                p = local@rot.T + pos
                exit_speed = v/h/self.aperture_area_m2
                u = np.repeat((vel+rot[:,2]*exit_speed)[None,:], len(p), axis=0)
                self.air_pos = np.vstack((self.air_pos, p))
                self.air_vel = np.vstack((self.air_vel, u))
                self.air_volume = np.r_[self.air_volume, v*self.weights]
                self.emitted_m3 += v
                total += v
            self._fall(h)
            self._relax(h)
            self.time_s += h
        self.last_flow_m3_s = total/dt
        return self.reservoir_pressure(piston_m)*self.p.piston_area_m2

    def _indices(self, xy):
        ij = np.floor((xy - np.array((self.x[0], self.y[0])) + self.dx/2)/self.dx).astype(int)
        valid = ((ij[:,0]>=0)&(ij[:,0]<len(self.x))&(ij[:,1]>=0)&(ij[:,1]<len(self.y)))
        return ij, valid

    def _fall(self, dt):
        if not len(self.air_volume):
            return
        old = self.air_pos.copy()
        self.air_vel[:,2] -= 9.81*dt
        self.air_pos += self.air_vel*dt
        ij, valid = self._indices(self.air_pos[:,:2])
        surface = np.full(len(ij), -np.inf)
        ix = np.flatnonzero(valid)
        surface[ix] = self.base[ij[ix,0],ij[ix,1]]+self.volume[ij[ix,0],ij[ix,1]]/self.dx**2
        landed = valid & (self.air_pos[:,2] <= surface)
        if landed.any():
            # Conservative cloud-in-cell deposition avoids snapping the entire
            # parcel onto a single cell as the nozzle crosses grid boundaries.
            # The kernel spans one cell; no desired shape enters this transfer.
            xy=(self.air_pos[landed,:2]-np.array((self.x[0],self.y[0])))/self.dx
            lower=np.floor(xy).astype(int);fraction=xy-lower
            volumes=self.air_volume[landed]
            for di,dj in ((0,0),(1,0),(0,1),(1,1)):
                i,j=(lower+np.array((di,dj))).T
                weight=(fraction[:,0] if di else 1-fraction[:,0])*(fraction[:,1] if dj else 1-fraction[:,1])
                inside=(i>=0)&(i<len(self.x))&(j>=0)&(j<len(self.y))
                np.add.at(self.volume,(i[inside],j[inside]),volumes[inside]*weight[inside])
                self.lost_m3+=float((volumes[~inside]*weight[~inside]).sum())
        escaped = (~valid & (self.air_pos[:,2]<self.table_top)) | (self.air_pos[:,2]<self.table_top-.2)
        self.lost_m3 += float(self.air_volume[escaped & ~landed].sum())
        keep = ~(landed|escaped)
        self.air_pos, self.air_vel, self.air_volume = (a[keep] for a in
                                                      (self.air_pos,self.air_vel,self.air_volume))

    def _pressure_head(self, region=(slice(None),slice(None))):
        """Pressure-driven spreading only beneath the measured star aperture.

        When deposited cream submerges the outlet, its upstream surface receives
        reservoir pressure over the declared 2 mm transfer depth. This is a
        local constitutive approximation, independent of targets or shapes.
        """
        if self.nozzle_position is None or self.pressure_pa<=0:return 0.
        displacement=np.stack((self.X[region]-self.nozzle_position[0],self.Y[region]-self.nozzle_position[1],
                               self.base[region]+self.volume[region]/self.dx**2-self.nozzle_position[2]),axis=-1)
        local=displacement@self.nozzle_rotation
        radius=np.hypot(local[:,:,0],local[:,:,1])
        angle=np.arctan2(local[:,:,1],local[:,:,0])
        aperture=self.p.nozzle_radius_m*(1+self.p.star_depth*np.cos(self.p.star_lobes*angle))
        submerged=np.clip(-local[:,:,2]/self.p.pressure_transfer_depth_m,0.,1.)*(radius<aperture)
        return self.pressure_pa*submerged/(self.p.density_kg_m3*9.81)

    def _relax(self, dt):
        """Implicit, conservative neighbouring-cell film transport.

        Solve each disjoint pair for its end-of-substep flux. This avoids the
        explicit donor limiter that made spreading depend strongly on cell size.
        Alternating axes reduce splitting bias. Pressure footprint is frozen
        within each pair solve; submergence varies with the candidate depth.
        """
        occupied_rows=np.flatnonzero(np.any(self.volume>0,axis=1))
        if not len(occupied_rows):return
        occupied_cols=np.flatnonzero(np.any(self.volume>0,axis=0))
        # Flux is nearest-neighbour and requires a positive-volume donor.
        # Two empty cells surround all donors, including the first-axis flux,
        # so cropping cannot suppress material transport or alter boundaries.
        region=(slice(max(0,int(occupied_rows[0])-2),min(len(self.x),int(occupied_rows[-1])+3)),
                slice(max(0,int(occupied_cols[0])-2),min(len(self.y),int(occupied_cols[-1])+3)))
        volume=self.volume[region]
        base=self.base[region]
        area=self.dx**2
        rho_g=self.p.density_kg_m3*9.81
        axes=(0,1) if round(self.time_s/dt)%2==0 else (1,0)
        for axis in axes:
            for parity in (0,1):
                h=volume/area
                scale=np.zeros_like(volume);intercept=np.zeros_like(volume);slope=0.
                # A downward outlet can compress a supported surface. Upward
                # ejection remains ballistic; this height field has no ceiling.
                if (self.nozzle_position is not None and self.pressure_pa>0
                        and self.nozzle_rotation[2,2]<0):
                    displacement=np.stack((self.X[region]-self.nozzle_position[0],
                        self.Y[region]-self.nozzle_position[1],
                        base+h-self.nozzle_position[2]),axis=-1)
                    local=displacement@self.nozzle_rotation
                    radius=np.hypot(local[:,:,0],local[:,:,1])
                    angle=np.arctan2(local[:,:,1],local[:,:,0])
                    aperture=self.p.nozzle_radius_m*(1+self.p.star_depth*np.cos(self.p.star_lobes*angle))
                    scale=self.pressure_pa/rho_g*(radius<aperture)
                    slope=-self.nozzle_rotation[2,2]/self.p.pressure_transfer_depth_m
                    intercept=-local[:,:,2]/self.p.pressure_transfer_depth_m-slope*h
                left=((slice(parity,-1,2),slice(None)) if axis==0
                      else (slice(None),slice(parity,-1,2)))
                right=((slice(parity+1,None,2),slice(None)) if axis==0
                       else (slice(None),slice(parity+1,None,2)))
                vl,vr=volume[left],volume[right]
                surface=base+h+scale*np.clip(intercept+slope*h,0,1)
                gradient=rho_g*(surface[left]-surface[right])/self.dx
                donor_depth=np.where(gradient>0,vl,vr)/area
                active=(np.abs(gradient)*donor_depth>self.p.shear_yield_pa)&(donor_depth>0)
                if not np.any(active):continue
                self.implicit_surface_edges+=int(active.sum())
                vl0,vr0=vl[active].copy(),vr[active].copy()
                bl,br,al,ar,sl,sr=(a[active] for a in
                    (base[left],base[right],intercept[left],intercept[right],scale[left],scale[right]))
                # These bounds enforce finite volume and positivity directly.
                low,high=-vr0,vl0
                for _ in range(26):
                    flux=(low+high)/2
                    hl,hr=(vl0-flux)/area,(vr0+flux)/area
                    g=rho_g*(bl+hl+sl*np.clip(al+slope*hl,0,1)
                        -br-hr-sr*np.clip(ar+slope*hr,0,1))/self.dx
                    predicted=surface_flux(g,np.where(g>0,hl,hr),
                        yield_pa=self.p.shear_yield_pa,
                        consistency_pa_sn=self.p.consistency_pa_sn,
                        flow_index=self.p.flow_index)*dt*self.dx
                    residual=flux-predicted
                    low=np.where(residual<0,flux,low)
                    high=np.where(residual>=0,flux,high)
                flux=(low+high)/2
                vl[active],vr[active]=vl0-flux,vr0+flux
                self.surface_transport_m3+=float(np.abs(flux).sum())
        if self.volume.min(initial=0.) < -1e-18:
            raise ArithmeticError("Material solver produced negative volume")

    def tool_contact(self, points, velocities, dt, *, stiffness_n_m=1500., damping_ns_m=1.):
        """Penalty contact with the measured nozzle surface; returns reaction N.

        Points are samples on the real metal tip, not its centre or TCP. Cake
        penetration over 1 mm is irreversible damage for judging. Cream is
        displaced conservatively away from an intruding tip. This is an
        experimental sampled contact approximation, not a full collision mesh.
        """
        p, v = np.asarray(points,float), np.asarray(velocities,float)
        if p.ndim!=2 or p.shape[1]!=3 or v.shape!=p.shape or not np.isfinite(p).all() or not np.isfinite(v).all():
            raise ValueError("Contact points/velocities must be finite N by 3 arrays")
        stiffness=np.broadcast_to(np.asarray(stiffness_n_m,float),(len(p),))
        damping=np.broadcast_to(np.asarray(damping_ns_m,float),(len(p),))
        if not np.isfinite(stiffness).all() or not np.isfinite(damping).all() or np.any(stiffness<0) or np.any(damping<0):
            raise ValueError('Contact coefficients must be finite and nonnegative')
        ij, valid = self._indices(p[:,:2])
        reactions = np.zeros_like(p)
        ks=np.flatnonzero(valid)
        if not len(ks):return reactions
        i,j=ij[ks].T
        top=self.base[i,j]+self.volume[i,j]/self.dx**2
        depth=top-p[ks,2]
        reactions[ks,2]=np.where(depth>0,np.maximum(0.,stiffness[ks]*depth-damping[ks]*v[ks,2]),0.)
        # Project each covered column once, using the lowest sampled metal
        # surface. Repeating displacement for duplicate samples creates an
        # artificial dependence on quadrature density and iteration order.
        occupied,inverse=np.unique(i*len(self.y)+j,return_inverse=True)
        lowest=np.full(len(occupied),np.inf)
        np.minimum.at(lowest,inverse,p[ks,2])
        i,j=np.divmod(occupied,len(self.y))
        depth=np.maximum(0.,self.base[i,j]+self.volume[i,j]/self.dx**2-lowest)
        on_cake=self.on_cake[i,j]
        self.cake_damage_depth_m=max(self.cake_damage_depth_m,
            float(np.max((self.base[i,j]-lowest)[on_cake],initial=0.)))
        moved=np.minimum(self.volume[i,j],depth*self.dx**2)
        self.volume[i,j]-=moved
        neighbour_count=(i>0).astype(int)+(i+1<len(self.x))+(j>0)+(j+1<len(self.y))
        for di,dj in ((-1,0),(1,0),(0,-1),(0,1)):
            a,b=i+di,j+dj
            inside=(a>=0)&(a<len(self.x))&(b>=0)&(b<len(self.y))
            np.add.at(self.volume,(a[inside],b[inside]),moved[inside]/neighbour_count[inside])
        return reactions

    def balance(self):
        retained=float(self.volume.sum())
        airborne=float(self.air_volume.sum())
        return {"initial_m3":self.p.initial_volume_m3,
                "reservoir_m3":self.p.initial_volume_m3-self.emitted_m3,
                "deposited_m3":retained,"airborne_m3":airborne,"lost_m3":self.lost_m3,
                "surface_transport_m3":self.surface_transport_m3,
                "implicit_surface_edges":self.implicit_surface_edges,
                "error_m3":self.emitted_m3-retained-airborne-self.lost_m3}

    def save(self, path):
        np.savez_compressed(path,x=self.x,y=self.y,base=self.base,volume=self.volume,
                            on_cake=self.on_cake,dx=self.dx,emitted_m3=self.emitted_m3,
                            lost_m3=self.lost_m3,airborne_m3=float(self.air_volume.sum()),
                            damage_depth_m=self.cake_damage_depth_m,time_s=self.time_s)
