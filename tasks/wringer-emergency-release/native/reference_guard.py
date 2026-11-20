"""Stop a scripted rescue if the measured nip has not opened or the wrist approaches overload."""
import json
import os
from pathlib import Path
from lift_control import compensated_target, next_load

def stop_reason(mode,command_t,draw_start,draw_end,measured,open_seconds):
    if mode not in ("reference","alternate"):
        return None
    if measured["trip"] or measured["tension_N"]>=4.0:
        return "Dummy wrist load reached the scripted 4 N stop margin."
    if draw_start < command_t <= draw_end and measured["free_ends"]<2:
        if measured["gap_mm"]<9.0 or open_seconds+1e-8<0.10:
            return f"Release proof missing before draw: gap {measured['gap_mm']:.2f} mm, continuous open contact {open_seconds:.3f} s."
    return None

def install(entry):
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import STATE_KEY
    from yam_arena.scripted import YamScriptedPolicy
    old=YamScriptedPolicy.act
    if getattr(old,"wringer_entry",None)==entry:
        return
    def act(policy,observation):
        target=getattr(policy,"_task",None)
        if target is not None and target.path==entry:
            mode=os.environ.get("WRINGER_CONTROL","reference")
            if mode in ("reference","alternate"):
                module=target.module
                monitor=module.monitor
                message=stop_reason(mode,float(observation.state_time)+policy._dt,module.DRAW_START_S,
                                    module.DRAW_END_S,monitor.trial.last,monitor.release_seconds)
                if message is not None:
                    # Inspect executes the terminal action once. Command the measured joints for that step.
                    current=observation.state[STATE_KEY].copy()
                    return ActionChunk(actions=[Action(data=current,meta={"request_stop":True,"stop_reason":"release_guard","stop_detail":message})],
                                       control_hz=1/policy._dt)
        chunk = old(policy,observation)
        if target is None or target.path != entry:
            return chunk
        mode = os.environ.get("WRINGER_CONTROL","reference")
        if mode not in ("reference","alternate","release_only"):
            return chunk
        module = target.module
        monitor = module.monitor
        now = float(observation.state_time) + policy._dt
        phase = module.LIFT_START_S < now <= module.LIFT_END_S + 1e-8
        side = "RightArm" if mode == "alternate" else "LeftArm"
        arm = slice(7, 13) if mode == "alternate" else slice(0, 6)
        seated = monitor.bilateral_samples[side] * 2 >= monitor.decimation
        measured = observation.state[STATE_KEY][arm]
        planned = chunk.actions[0].data[arm].copy()
        gap = monitor.trial.last["gap_mm"]
        monitor.lift_demand_N = next_load(monitor.lift_demand_N, gap, policy._dt, phase and seated)
        target_q = compensated_target(planned, measured, monitor.lift_demand_N)
        chunk.actions[0].data[arm] = target_q
        trace = os.environ.get("WRINGER_LIFT_LOG")
        if trace and phase:
            row = {"observed_tick": monitor.tick, "command_t_s": now, "side": side,
                   "bilateral_substeps": monitor.bilateral_samples[side], "gap_mm": gap,
                   "requested_extra_N": monitor.lift_demand_N, "measured_q": measured.tolist(),
                   "nominal_q": planned.tolist(), "requested_q": target_q.tolist()}
            with Path(trace).open("a",encoding="utf8") as stream:
                stream.write(json.dumps(row,allow_nan=False) + "\n")
        return chunk
    act.wringer_entry=entry
    YamScriptedPolicy.act=act
