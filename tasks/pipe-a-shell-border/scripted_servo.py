"""Disclosed state-assisted scripted demo, using ordinary robot actions only.

The feasibility controller measures barrel/plunger poses and robot joints.
It estimates the current physical grasp and tracks a fixed nozzle/piston recipe.
It never writes object poses, joints, material, forces, scores or motor settings.
Dream does not receive this controller or its privileged pose measurements.
"""
from __future__ import annotations
import numpy as np
from native import rotation


class ScriptedServo:
    def __init__(self):
        self.last={};self.piston_integral=0.;self.piston_bias=0.
        self.grip_position=None;self.grip_rotation=None
        self.pusher_translation=np.zeros(3)

    def command(self,frame,reference,observed,barrel_pose,plunger_pose):
        target=np.asarray(reference,float).copy()
        recipe=frame.get('scripted_tool_target')
        self.last={'active':recipe is not None}
        if recipe is None:return target
        b,p=np.asarray(barrel_pose,float),np.asarray(plunger_pose,float)
        if b.shape!=(7,) or p.shape!=(7,) or not np.isfinite(np.r_[b,p]).all():
            raise ValueError('Finite measured tool poses required')
        from yam_arena import ik,station
        actual_R=rotation(b[3:]);axis=actual_R[:,1]
        wanted_R=np.asarray(recipe['barrel_rotation'],float)
        wanted_b=np.asarray(recipe['barrel_position_m'],float)
        actual_hand,actual_hand_R=ik.tcp_pose(np.asarray(observed)[7:13],station.BASES['right'])
        grip_p=actual_R.T@(actual_hand-b[:3]);grip_R=actual_R.T@actual_hand_R
        if self.grip_position is None:
            self.grip_position=grip_p.copy();self.grip_rotation=grip_R.copy()
        else:
            # Estimate slowly: instant corrections to elastic finger movement
            # create a positive feedback loop between the two contacting arms.
            self.grip_position+=.1*(grip_p-self.grip_position)
            self.grip_rotation=ik.slerp_matrix(self.grip_rotation,grip_R,.1)
        hand_p=wanted_b+wanted_R@self.grip_position;hand_R=wanted_R@self.grip_rotation
        q,pe,re=ik.solve(hand_p,ik.matrix_to_quat_xyzw(hand_R),target[7:13],station.BASES['right'],iters=160,tol=1e-7)
        if pe>.0005 or re>.005:raise RuntimeError(f'Barrel pose servo unreachable: {pe}, {re}')
        if np.linalg.norm(grip_p)>.06:raise RuntimeError('Barrel escaped physical grasp')
        target[7:13]=q
        self.last.update(tool_position_error_m=float(np.linalg.norm(b[:3]-wanted_b)),
            measured_grasp_position_m=grip_p.tolist(),filtered_grasp_position_m=self.grip_position.tolist(),
            right_ik_error_m=float(pe))
        if 'piston_m' in recipe:
            actual_q=float(np.dot(p[:3]-b[:3],axis)+.060)
            desired_q=float(recipe['piston_m'])
            error=desired_q-actual_q
            self.piston_integral=float(np.clip(self.piston_integral+2.*error*.05,-.003,.003))
            wanted_bias=float(np.clip(1.5*error+self.piston_integral,-.004,.004))
            self.piston_bias+=float(np.clip(wanted_bias-self.piston_bias,-.0004,.0004))
            bias=self.piston_bias
            # Follow the measured translational tracking lag slowly. Without
            # this common displacement, the pusher runs ahead of the barrel
            # during a tail stroke and loses compression. Do not chase its
            # instantaneous rotation or elastic finger motion.
            displacement=np.clip(b[:3]-wanted_b,-.008,.008)
            self.pusher_translation+=.3*(displacement-self.pusher_translation)
            push_p,push_R=ik.tcp_pose(target[:6],station.BASES['left'])
            push_p+=self.pusher_translation+wanted_R[:,1]*bias
            q,pe,re=ik.solve(push_p,ik.matrix_to_quat_xyzw(push_R),target[:6],station.BASES['left'],iters=160,tol=1e-7)
            if pe>.0005 or re>.005:raise RuntimeError(f'Piston pose servo unreachable: {pe}, {re}')
            target[:6]=q
            self.last.update(piston_target_m=desired_q,piston_measured_m=actual_q,
                piston_bias_m=bias,pusher_translation_m=self.pusher_translation.tolist(),
                left_ik_error_m=float(pe))
        else:
            self.piston_integral=0.;self.piston_bias=0.;self.pusher_translation[:]=0.
        return target
