"""Original equivalent, CPU material law; this is NOT a native task or OEM reproduction."""
from dataclasses import dataclass
from math import isfinite, pi, atan

PARAMS = dict(
    bore_diameter_m=.0127, anchor_length_m=.0396875, nominal_thread='3/8-16',
    top_ring_outer_radius_m=.0062, top_ring_z_min_m=.018,
    petal_count=4, petal_angle_rad=88*pi/180, petal_length_m=.018,
    root_length_m=.004, root_thickness_m=.002, root_effective_width_m=.0075,
    steel_density_kg_m3=7850., steel_modulus_Pa=200e9, steel_yield_Pa=250e6,
    plug_height_m=.008, plug_radius_bottom_m=.0037, plug_radius_top_m=.0045,
    plug_initial_bottom_z_m=.005, nominal_plug_travel_m=.0018,
    punch_shaft_diameter_m=.006, punch_shoulder_diameter_m=.012,
    punch_shoulder_to_tip_m=.0284875, punch_upper_grip_length_m=.100,
    steel_steel_static_friction=.25, steel_steel_dynamic_friction=.20,
    steel_concrete_static_friction=.50, steel_concrete_dynamic_friction=.40,
    hammer_head_size_m=(.060,.030,.030), hammer_handle_size_m=(.020,.014,.280),
    ash_density_kg_m3=700., diagnostic_strikes_cap=6, diagnostic_controls_cap=2000,
    manufacturer_setting_energy_J=None, delivered_native_energy_J=None)


def hinge_parameters():
    p=PARAMS
    inertia=p['root_effective_width_m']*p['root_thickness_m']**3/12
    return dict(stiffness_Nm_per_rad=p['steel_modulus_Pa']*inertia/p['root_length_m'],
                yield_moment_Nm=p['steel_yield_Pa']*p['root_effective_width_m']*p['root_thickness_m']**2/6)

@dataclass
class PassivePlasticRoot:
    stiffness: float
    yield_moment: float
    plastic_angle: float=0.
    dissipated_work: float=0.
    previous_t: float|None=None

    def observe(self,actual_angle:float,actual_t:float):
        if not all(isfinite(x)for x in [actual_angle,actual_t,self.stiffness,self.yield_moment]):
            raise ValueError('Nonfinite material input')
        if self.stiffness<=0 or self.yield_moment<=0:raise ValueError('Invalid constitutive parameters')
        if self.previous_t is not None and actual_t<=self.previous_t:
            raise ValueError('Material updates require a fresh increasing observation')
        trial=self.stiffness*(actual_angle-self.plastic_angle)
        moment=max(-self.yield_moment,min(self.yield_moment,trial))
        new_plastic=actual_angle-moment/self.stiffness
        delta=new_plastic-self.plastic_angle
        self.dissipated_work+=self.yield_moment*abs(delta)
        self.plastic_angle=new_plastic;self.previous_t=actual_t
        return dict(actual_angle=actual_angle,plastic_angle=new_plastic,
                    resisting_torque_Nm=-moment,
                    elastic_energy_J=moment**2/(2*self.stiffness),
                    plastic_dissipation_J=self.dissipated_work)


def analytic_preflight():
    p=PARAMS;hp=hinge_parameters();k=hp['stiffness_Nm_per_rad'];my=hp['yield_moment_Nm']
    x,y,z=p['hammer_head_size_m'];a,b,l=p['hammer_handle_size_m']
    head=(x*y*z-a*b*z)*p['steel_density_kg_m3']
    haft=a*b*l*p['ash_density_kg_m3']
    mass=head+haft
    return dict(hinge=hp,initial_yield_angle_rad=my/k,
        plug_half_angle_rad=atan((p['plug_radius_top_m']-p['plug_radius_bottom_m'])/p['plug_height_m']),
        steel_steel_selflock_geometric_necessary_condition=(p['plug_radius_top_m']-p['plug_radius_bottom_m'])/p['plug_height_m']<p['steel_steel_dynamic_friction'],
        contact_boss_center_z_m=.008, nominal_boss_clearance_m=.00003,
        plug_radial_increase_m=.0018*.1,
        nominal_required_petal_radial_motion_m=.0018*.1-.00003,
        available_radial_bore_gap_m=p['bore_diameter_m']/2-p['top_ring_outer_radius_m'],
        computed_hammer_head_mass_kg=head,computed_hammer_haft_mass_kg=haft,
        computed_hammer_total_mass_kg=mass,
        ideal100mm_gravity_strike_J=mass*9.81*.1,
        limits='Necessary analytic checks only; original complete manufactured volumes/clearance/robot path still require authoring. No native result or OEM curve.')
