"""Mechanism regressions, including negative controls. No robot trial claims."""
import unittest
import numpy as np
from material import Icing, surface_flux
from scoring import assess, _ridge_length
from native import Coupling
from contact_audit import intersects_columns

DOWN=np.diag((1.,-1.,-1.))


class MaterialTests(unittest.TestCase):
    def test_stationary_surface_ignores_unintegrated_solver_velocity(self):
        c=Coupling(self.model());pose=np.array([.2,0.,.9,0.,0.,0.,1.])
        c.prime_boundary(pose)
        _,tipvel,_,vel=c.boundary_motion(pose,np.ones(6)*10.,1/960)
        np.testing.assert_allclose(tipvel,0.,atol=1e-12)
        np.testing.assert_allclose(vel,0.,atol=1e-12)

    def test_translation_preserves_full_displacement_without_clipping(self):
        c=Coupling(self.model());pose=np.array([.2,0.,.9,0.,0.,0.,1.]);dt=1/960
        c.prime_boundary(pose);translation=np.array([.001,-.002,.003]);pose[:3]+=translation
        _,tipvel,_,vel=c.boundary_motion(pose,np.zeros(6),dt)
        np.testing.assert_allclose(tipvel,translation/dt,atol=1e-12)
        np.testing.assert_allclose(vel,np.broadcast_to(translation/dt,vel.shape),atol=1e-12)

    def test_rotation_uses_each_actual_boundary_point(self):
        c=Coupling(self.model());pose=np.array([.2,0.,.9,0.,0.,0.,1.]);dt=1/960
        c.prime_boundary(pose);theta=.02;pose[5:]=[np.sin(theta/2),np.cos(theta/2)]
        tip,tipvel,world,vel=c.boundary_motion(pose,np.zeros(6),dt)
        np.testing.assert_allclose(tipvel,(tip-(pose[:3]+[0.,.08,0.]))/dt,atol=1e-12)
        np.testing.assert_allclose(vel,(world-(c.contact_samples+pose[:3]))/dt,atol=1e-12)
        self.assertGreater(np.ptp(vel[:,1]),.01)

    def test_reset_clears_boundary_history(self):
        c=Coupling(self.model());c.prime_boundary([0.,0.,0.,0.,0.,0.,1.]);c.reset()
        self.assertIsNone(c.previous_boundary_pose)

    def test_ridges_must_continue_beyond_one_fortunate_cross_section(self):
        dx=.0005;x=np.arange(-.004,.0041,dx);y=np.arange(-.005,.0051,dx)
        profile=.002+.002*(np.exp(-((x-.002)/.0005)**2)+np.exp(-((x+.002)/.0005)**2))
        surface=np.full((len(x),len(y)),.002)
        head=int(np.argmin(abs(y)))
        surface[:,head]=profile
        self.assertLess(_ridge_length(surface,0,len(y)-1,dx),.002)
        surface[:,head-1:head+4]=profile[:,None]
        self.assertGreaterEqual(_ridge_length(surface,0,len(y)-1,dx),.002)

    def test_duplicate_contact_samples_do_not_displace_extra_volume(self):
        a,b=self.model(),self.model()
        for m in (a,b):m.volume[:]=.003*m.dx**2;m.emitted_m3=float(m.volume.sum())
        p=[.235,0.,a.cake_top+.002]
        force_a=a.tool_contact([p],[[0.,0.,0.]],1/240)
        force_b=b.tool_contact([p,p],[[0.,0.,0.]]*2,1/240,
            stiffness_n_m=np.array([750.,750.]),damping_ns_m=np.array([.5,.5]))
        np.testing.assert_array_equal(a.volume,b.volume)
        np.testing.assert_allclose(force_a.sum(axis=0),force_b.sum(axis=0))

    def test_film_flux_reduces_to_newtonian_solution(self):
        gradient=np.array([-1000.,0.,1000.]);h=.003;viscosity=20.
        flux=surface_flux(gradient,h,yield_pa=0.,consistency_pa_sn=viscosity,flow_index=1.)
        np.testing.assert_allclose(flux,gradient*h**3/(3*viscosity),atol=1e-20)

    def test_yield_depends_on_depth_and_stress_not_slope_alone(self):
        flux=surface_flux(np.array([100_000.,100_000.]),np.array([.002,.010]),
                          yield_pa=800.,consistency_pa_sn=4000.,flow_index=.5)
        self.assertEqual(flux[0],0.)
        self.assertGreater(flux[1],0.)

    def test_small_icing_ridge_does_not_slump_below_yield(self):
        m=Icing(dx=.0005,xlim=(.225,.245),ylim=(-.01,.01))
        m.volume[15:25,15:25]=.003*m.dx**2
        before=m.volume.copy()
        for _ in range(480):m._relax(1/480)
        np.testing.assert_array_equal(m.volume,before)

    def model(self):
        return Icing(dx=.002)

    def advance(self,m,q,steps=120,pos=(.235,0.,.8055)):
        for _ in range(steps):
            m.step(1/240,piston_m=q,nozzle_pos=pos,nozzle_rotation=DOWN)

    def test_motion_without_pressure_cannot_paint(self):
        m=self.model()
        for y in np.linspace(-.045,.045,60):
            self.advance(m,0,steps=1,pos=(.235,y,.8055))
        self.assertEqual(m.emitted_m3,0.)
        self.assertEqual(float(m.volume.sum()),0.)

    def test_pressure_produces_real_material_and_reaction(self):
        m=self.model()
        reaction=m.step(1/240,piston_m=.01,nozzle_pos=(.235,0.,.8055),nozzle_rotation=DOWN)
        self.assertGreater(reaction,0.)
        self.assertGreater(m.air_volume.sum(),0.)
        self.advance(m,.01)
        self.assertGreater(m.volume.sum(),0.)
        self.assertLess(abs(m.balance()["error_m3"]),1e-15)

    def test_retraction_does_not_refill(self):
        m=self.model()
        self.advance(m,.01)
        emitted=m.emitted_m3
        self.advance(m,0,steps=10)
        self.assertEqual(m.emitted_m3,emitted)
        self.assertEqual(m.last_flow_m3_s,0.)
        self.assertLessEqual(m.emitted_m3,m.p.initial_volume_m3)

    def test_off_cake_flow_is_not_credited_as_a_border(self):
        m=self.model()
        self.advance(m,.01,pos=(.4,.09,.8055))
        self.advance(m,0,steps=100,pos=(.4,.09,.8055))
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=m.emitted_m3,lost_m3=m.lost_m3)
        self.assertEqual(r["score"],0.)
        self.assertGreater(r["spill_area_mm2"],20.)

    def test_volume_conserved_through_slumping(self):
        m=self.model()
        m.volume[60,60]=.5e-6
        m.emitted_m3=.5e-6
        maximum=m.height.max()
        for _ in range(100):m._relax(1/480)
        self.assertLess(m.height.max(),maximum)
        self.assertLess(abs(m.balance()["error_m3"]),1e-18)
        self.assertTrue(np.all(m.volume>=0))

    def test_submerged_pressure_spreads_without_creating_material(self):
        models=[Icing(dx=.0005,xlim=(.225,.245),ylim=(-.01,.01)) for _ in range(3)]
        for m in models:
            m.volume[:]=.003*m.dx**2;m.emitted_m3=float(m.volume.sum())
            m.nozzle_position=np.array((.235,0.,.8025));m.nozzle_rotation=DOWN
        models[0].pressure_pa=12_000.
        models[2].nozzle_position=np.array((.235,0.,.85))
        for m in models:
            m._relax(1/480)
            self.assertLess(abs(m.balance()['error_m3']),1e-18)
            self.assertTrue(np.all(m.volume>=0))
        self.assertFalse(np.allclose(models[0].volume,models[1].volume,atol=1e-14))
        np.testing.assert_array_equal(models[1].volume,models[2].volume)

    def test_nozzle_contact_displacement_conserves_volume(self):
        m=self.model();m.volume[:]=.003*m.dx**2;m.emitted_m3=float(m.volume.sum())
        m.tool_contact([[.235,0.,m.cake_top+.002]],[[0.,0.,-.01]],1/240)
        self.assertLess(abs(m.balance()['error_m3']),1e-18)
        self.assertTrue(np.all(m.volume>=0))

    def test_cake_collision_is_irreversible_damage(self):
        m=self.model()
        reaction=m.tool_contact([[.235,0.,m.cake_top-.002]],[[0.,0.,-.01]],1/240)
        self.assertGreater(reaction[0,2],0.)
        self.assertAlmostEqual(m.cake_damage_depth_m,.002)
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=0,damage_depth_m=m.cake_damage_depth_m)
        self.assertEqual(r["score"],0.)

    def test_force_ceases_below_yield(self):
        m=self.model()
        self.advance(m,.0001,steps=10)
        self.assertEqual(m.emitted_m3,0.)

    def test_fixed_volume_slope_is_resolution_sensitive_and_reported(self):
        # A sanity bound only. This does not certify convergence or calibration.
        a,b=Icing(dx=.002),Icing(dx=.001)
        for m in (a,b):
            self.advance(m,.008,steps=60)
        self.assertAlmostEqual(a.emitted_m3,b.emitted_m3,places=15)
        self.assertLess(abs(a.balance()["error_m3"]),1e-15)
        self.assertLess(abs(b.balance()["error_m3"]),1e-15)

    def test_nan_telemetry_cannot_score(self):
        m=self.model()
        m.volume[0,0]=float("nan")
        self.assertEqual(assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=0)["score"],0.)

    def test_unaccounted_material_cannot_score(self):
        m=self.model();m.volume[50,50]=1e-6
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=0.,completed=True,tools_clear=True)
        self.assertEqual(r['score'],0.)
        self.assertIn('match measured extrusion',r['reason'])

    def test_native_coupling_actually_uses_metal_lip_contact(self):
        c=Coupling(self.model())
        pose=np.array((.235,-.080,c.material.cake_top+.001,0.,0.,0.,1.))
        piston=pose.copy();piston[1]-=.060
        barrel_force,piston_force=c.advance(pose,np.zeros(6),piston,1/240)
        self.assertGreater(barrel_force[2],0.)
        self.assertGreater(c.material.cake_damage_depth_m,.001)
        self.assertEqual(c.material.emitted_m3,0.)

    def test_smooth_line_is_not_a_shell_border(self):
        m=self.model();X,Y=m.X,m.Y
        m.volume[(abs(X-.235)<.005)&(abs(Y)<.045)]=.004*m.dx**2
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),completed=True,tools_clear=True)
        self.assertEqual(r['score'],0.)

    def test_round_mounds_do_not_become_shells_by_narrowing(self):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05))
        for centre in (-.035,-.0175,0.,.0175,.035):
            radius=np.hypot(m.X-.235,m.Y-centre)
            height=.004*np.maximum(0.,1-(radius/.005)**2)
            m.volume=np.maximum(m.volume,height*m.dx**2)
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),completed=True,tools_clear=True)
        self.assertEqual(r['valid_shells'],0)
        self.assertEqual(r['score'],0.)

    def test_fine_point_does_not_invalidate_a_qualifying_tail(self):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05))
        for start in (-.04,-.025,-.01,.005,.02,.035):
            s=m.Y-start
            width=np.where(s<.002,.008,np.maximum(.002,.008-.6*(s-.002)))
            height=np.where(s<.002,.004,np.maximum(.001,.004-.3*(s-.002)))
            shape=(s>=0)&(s<.013)&(abs(m.X-.235)<width/2)
            m.volume[shape]=height[shape]*m.dx**2
            tip=(s>=.013)&(s<.014)&(abs(m.X-.235)<.0003)
            m.volume[tip]=.0006*m.dx**2
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),completed=True,tools_clear=True)
        self.assertGreaterEqual(r['valid_shells'],5)

    def test_long_equal_width_tail_is_measured_to_its_end(self):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05))
        s=m.Y+.01
        head=(s>=0)&(s<.003)&(abs(m.X-.235)<.004)
        tail=(s>=.003)&(s<.014)&(abs(m.X-.235)<.0015)
        m.volume[head]=.004*m.dx**2
        m.volume[tail]=.0015*m.dx**2
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),completed=True,tools_clear=True)
        self.assertEqual(r['valid_shells'],1)
        self.assertGreater(r['shells'][0]['length_mm'],12.)

    def bumped_tail(self,bump_height=.002,bump_width=.006,disconnected=False):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05))
        s=m.Y+.01
        head=(s>=0)&(s<.003)&(abs(m.X-.235)<.004)
        tail=(s>=.003)&(s<.017)&(abs(m.X-.235)<.0015)
        m.volume[head]=.004*m.dx**2
        m.volume[tail]=.0015*m.dx**2
        bump=(s>=.008)&(s<.010)&(abs(m.X-.235)<bump_width/2)
        m.volume[bump]=bump_height*m.dx**2
        if disconnected:m.volume[(s>=.006)&(s<.007)]=0.
        return assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),
                      completed=True,tools_clear=True)

    def test_low_tail_shoulder_does_not_split_a_shell(self):
        r=self.bumped_tail()
        self.assertEqual(r['valid_shells'],1)
        self.assertGreater(r['shells'][0]['length_mm'],15.)
        self.assertFalse(r['shells'][0]['visible_ridges'])

    def test_overheight_mound_cannot_be_skipped_to_borrow_a_tail(self):
        r=self.bumped_tail(bump_height=.009)
        self.assertEqual(r['valid_shells'],0)

    def test_overwide_mound_cannot_be_skipped_to_borrow_a_tail(self):
        r=self.bumped_tail(bump_width=.018)
        self.assertEqual(r['valid_shells'],0)

    def test_disconnected_icing_cannot_extend_a_low_shoulder_tail(self):
        r=self.bumped_tail(disconnected=True)
        self.assertEqual(r['valid_shells'],0)

    def test_laterally_disconnected_heads_and_tails_do_not_count(self):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05))
        for start in (-.043,-.013,.017):
            s=m.Y-start
            head=(s>=0)&(s<.003)&(abs(m.X-.233)<.004)
            tail=(s>=.003)&(s<.026)&(abs(m.X-.242)<.0015)
            m.volume[head]=.004*m.dx**2
            m.volume[tail]=.0015*m.dx**2
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),
                 completed=True,tools_clear=True)
        self.assertEqual(r['valid_shells'],0)
        self.assertEqual(r['score'],0.)

    def test_detached_side_blob_cannot_supply_head_dimensions(self):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05));s=m.Y+.02
        # The genuine connected head is too narrow. A separate lobe would
        # incorrectly make the projected width pass the six-millimetre rule.
        head=(s>=0)&(s<.003)&(abs(m.X-.233)<.0025)
        tail=(s>=.003)&(s<.026)&(abs(m.X-.233)<.001)
        blob=(s>=0)&(s<.003)&(abs(m.X-.241)<.0015)
        m.volume[head|blob]=.004*m.dx**2;m.volume[tail]=.0015*m.dx**2
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),
                 completed=True,tools_clear=True)
        self.assertEqual(r['valid_shells'],0)

    def test_detached_side_speck_does_not_change_a_real_shell(self):
        m=Icing(dx=.0005,xlim=(.22,.25),ylim=(-.05,.05));s=m.Y+.02
        head=(s>=0)&(s<.003)&(abs(m.X-.233)<.004)
        tail=(s>=.003)&(s<.026)&(abs(m.X-.233)<.0015)
        m.volume[head]=.004*m.dx**2;m.volume[tail]=.0015*m.dx**2
        judge=lambda:assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()),
                            completed=True,tools_clear=True)
        before=judge()
        blob=(s>=0)&(s<.003)&(abs(m.X-.241)<.001)
        m.volume[blob]=.007*m.dx**2
        after=judge()
        self.assertEqual(before['valid_shells'],1)
        self.assertEqual(after['shells'],before['shells'])
        self.assertEqual(after['score'],before['score'])

    def test_perfect_looking_material_without_execution_proof_scores_zero(self):
        m=self.model()
        m.volume[abs(m.X-.235)<.004]=.006*m.dx**2
        r=assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=float(m.volume.sum()))
        self.assertEqual(r['score'],0.)

    def test_invalid_grid_and_judgment_types_fail_closed(self):
        m=self.model();base=dict(dx=m.dx,emitted_m3=0.,completed=True,tools_clear=True)
        for field,value in [('completed','false'),('tools_clear',1),('damage_depth_m',-.1),('dx',None)]:
            kw=dict(base);kw[field]=value
            self.assertEqual(assess(m.x,m.y,m.volume,**kw)['score'],0.)
        for x,y,v in [(m.x+1,m.y,m.volume),([],[],np.empty((0,0))),(m.x,m.y*2,m.volume)]:
            self.assertEqual(assess(x,y,v,**base)['score'],0.)

    def test_mirrored_material_gets_the_same_score(self):
        m=Icing(dx=.0005)
        # Synthetic asymmetric shapes exercise rubric direction invariance;
        # these samples are never presented as simulated icing or demo evidence.
        for start in (-.045,-.030,-.015,0.,.015,.030):
            s=(m.Y-start)/.015
            width=np.where(s<.2,.002+.03*np.maximum(s,0),.008-.008*(s-.2))
            height=np.where(s<.2,.001+.02*np.maximum(s,0),.005-.005*(s-.2))
            mask=(s>=0)&(s<=1)&(abs(m.X-.235)<width/2)
            m.volume[mask]=np.maximum(m.volume[mask],height[mask]*m.dx**2)
        kw=dict(dx=m.dx,emitted_m3=float(m.volume.sum()),completed=True,tools_clear=True)
        a=assess(m.x,m.y,m.volume,**kw)
        b=assess(m.x,-m.y[::-1],m.volume[:,::-1],**kw)
        self.assertEqual(a['score'],b['score'])
        self.assertGreater(a['valid_shells'],0)

    def test_unsupported_finger_overlap_is_detected_between_grid_centres(self):
        import itertools
        m=self.model();m.volume[abs(m.X-.235)<.005]=.004*m.dx**2
        def box(lo,hi):return np.asarray(list(itertools.product(*zip(lo,hi))))
        touching=box((.233,-.002,.802),(.237,.002,.806))
        clear=touching+[0,0,.020]
        self.assertTrue(intersects_columns(touching,m.x,m.y,m.base,m.height,cake_mask=m.on_cake))
        self.assertFalse(intersects_columns(clear,m.x,m.y,m.base,m.height,cake_mask=m.on_cake))

    def test_passive_seal_damping_opposes_both_directions(self):
        c=Coupling(self.model())
        pose=np.array((.235,-.08,.85,0.,0.,0.,1.))
        p=pose.copy();p[1]-=.060
        c.advance(pose,np.zeros(6),p,1/240)
        p[1]+=.0001;c.advance(pose,np.zeros(6),p,1/240)
        self.assertGreater(c.last_seal_force_n,0.)
        self.assertLessEqual(abs(c.last_seal_force_n),c.dispenser.seal_effective_mass_kg*abs(c.last_piston_velocity_m_s)*240+1e-12)
        p[1]-=.0001;c.advance(pose,np.zeros(6),p,1/240)
        self.assertLess(c.last_seal_force_n,0.)


if __name__=="__main__":unittest.main()
