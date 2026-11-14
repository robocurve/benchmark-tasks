"""Lazy simulator hooks: reset-only gap setting, rigid solver tuning, pose diagnostics."""
import functools
import numpy as np

_CFG = None
_LOG = None


def _arr(x):
    x = getattr(x,"torch",x)
    return x.detach().cpu().numpy() if hasattr(x,"detach") else np.asarray(x)


def spawn_shim(path,cfg,*args,**kwargs):
    import yam_mesh
    from pxr import PhysxSchema
    body=yam_mesh._spawn_mesh_part(path,cfg,*args,**kwargs)
    rigid=PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    rigid.CreateSolverPositionIterationCountAttr().Set(32)
    rigid.CreateSolverVelocityIterationCountAttr().Set(4)
    rigid.CreateMaxDepenetrationVelocityAttr().Set(.5)
    return body


def set_gap(emb,cfg,seed):
    base=emb._env.unwrapped
    asset=base.scene["jamb"]
    current=getattr(asset.data.joint_pos,"torch",asset.data.joint_pos)
    q=current.clone() if hasattr(current,"clone") else current.copy()
    q[:,asset.joint_names.index("setter_joint")]=cfg["trial_gap"]()-cfg["G0"]
    q[:,asset.joint_names.index("jamb_joint")]=0.
    v=q*0
    torch=getattr(emb,"_torch",None)
    ids=np.arange(len(q)) if torch is None else torch.arange(len(q),device=base.device)
    # Lock the gap-setting slide at this trial's setting by closing its travel limits on it (the rig's screw stop,
    # locked): joint stiction alone let it creep under the jamb spring's ~1 N load (GPU probe). Written only here.
    k=asset.joint_names.index("setter_joint")
    s=float(cfg["trial_gap"]()-cfg["G0"])
    lim=np.array([[[s-1e-6,s+1e-6]]]*len(q),dtype=np.float32)
    if torch is not None: lim=torch.tensor(lim,device=base.device)
    else:
        try:
            import torch as _torch
            lim=_torch.tensor(lim,device=base.device)
        except ImportError: pass
    fn=getattr(asset,"write_joint_position_limit_to_sim_index",None) or getattr(asset,"write_joint_position_limit_to_sim")
    fn(limits=lim,joint_ids=[k],env_ids=ids,warn_limit_violation=False)
    for kind,value,arg in (("position",q,"position"),("velocity",v,"velocity")):
        fn=getattr(asset,f"write_joint_{kind}_to_sim_index",None)
        if fn is None: fn=getattr(asset,f"write_joint_{kind}_to_sim")
        fn(**{arg:value,"env_ids":ids})
    print(f"SHIM_GAP seed={seed} gap_mm={cfg['trial_gap']()*1000:.3f} "
          f"setter_q_mm={(cfg['trial_gap']()-cfg['G0'])*1000:.3f}",flush=True)
    back=_arr(asset.data.joint_pos)[0]
    print("SHIM_JOINT_READ "+" ".join(f"{name}={back[k]*1000:.3f}mm" for k,name in enumerate(asset.joint_names)),flush=True)
    try:
        lims=_arr(asset.data.joint_pos_limits)[0]
        print("SHIM_LIMITS "+" ".join(f"{name}=[{lims[k][0]*1000:.3f},{lims[k][1]*1000:.3f}]mm"
                                       for k,name in enumerate(asset.joint_names)),flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"SHIM_LIMITS unreadable {e!r}",flush=True)


def diagnostic(cfg,objects):
    global _LOG
    run=cfg["_RUN"]
    token=(run["episode"],run["step"])
    if run["t"] is None or token==_LOG or run["step"]%(2 if cfg["FORCE_DIAG"] else 10): return
    _LOG=token
    j=objects["jamb"]
    m=cfg["terms"](objects)
    detail=[]
    for name in ("shim_left","shim_right"):
        s=objects[name]
        et,el,_=cfg["shim_axes"](s)
        tilt=np.degrees(np.arccos(np.clip(min(abs(et[0]),abs(el[1])),-1,1)))
        detail.append(f"{name}(tip_y={cfg['tip'](s)[1]*1000:.2f},butt_y={cfg['butt'](s)[1]*1000:.2f},"
                      f"flat_x={s.origin[0]*1000:.2f},tilt_deg={tilt:.2f})")
    hands=cfg["grippers"]()
    tcp="None" if hands is None else ";".join(f"{a}={np.round(p*1000,2).tolist()}" for a,(p,_) in hands.items())
    print(f"SHIM t={run['t']:.2f} gap_mm={cfg['gap_now'](j)*1000:.3f} "
          f"q_setter_mm={j.joints['setter']*1000:.3f} q_jamb_mm={j.joints['jamb']*1000:.3f} "
          +" ".join(detail)+f" pair={m['pair']} overlap_mm={m['overlap']*1000:.3f} "
          f"clear0_mm={m['clear0']*1000:.3f} bow_mm={m['bow']*1000:.3f} tier={m['tier']} TCP_mm={tcp}",flush=True)


def install(module):
    global _CFG
    _CFG=module if isinstance(module,dict) else vars(module)
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:
        return
    if getattr(cls,"_shim_wrapped",False):
        cls._shim_cfg[0]=_CFG
        return
    holder=[_CFG]
    inner_reset,inner_step=cls.reset,cls._task_step
    @functools.wraps(inner_reset)
    def reset(self,*args,**kwargs):
        cfg=holder[0]
        seed=kwargs.get("seed")
        cfg["ACTIVE_GAP"]=cfg["gap_for_seed"](seed)
        result=inner_reset(self,*args,**kwargs)
        if self.task is not None and "jamb" in self.task.names:
            set_gap(self,cfg,seed)
            # Refresh the observation after the reset-only joint write.
            if hasattr(self,"_observation"): result=self._observation()
        return result
    @functools.wraps(inner_step)
    def task_step(self,*args,**kwargs):
        result=inner_step(self,*args,**kwargs)
        cfg=holder[0]
        if (self.task is not None and "jamb" in self.task.names
                and cfg["_RUN"]["step"] % (2 if cfg["FORCE_DIAG"] else 10) == 0):
            from yam_arena import scene
            diagnostic(cfg,scene.object_states(self._env,self.task.names))
        return result
    cls.reset,cls._task_step=reset,task_step
    cls._shim_wrapped=True
    cls._shim_cfg=holder
