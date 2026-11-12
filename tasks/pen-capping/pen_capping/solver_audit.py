"""Read authored runtime solver settings and detect iteration clamping."""
from pxr import UsdPhysics

BOUND_NAMES=('min_position_iteration_count','max_position_iteration_count',
             'min_velocity_iteration_count','max_velocity_iteration_count')
USD_BOUNDS=('physxScene:minPositionIterationCount','physxScene:maxPositionIterationCount',
            'physxScene:minVelocityIterationCount','physxScene:maxVelocityIterationCount')


def _value(prim,name):
    attribute=prim.GetAttribute(name)
    return attribute.Get() if attribute else None


def collect_solver_settings(stage,physics_config,names,position,velocity):
    config={key:getattr(physics_config,key,None) for key in (*BOUND_NAMES,'solver_type')}
    scenes=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scenes.append(dict(path=str(prim.GetPath()),attributes={a.GetName():a.Get() for a in prim.GetAttributes() if 'iteration' in a.GetName().lower() or a.GetName().endswith('solverType')}))
    bodies={}
    for name in names:
        prim=stage.GetPrimAtPath('/World/envs/env_0/'+name)
        bodies[name]=dict(position=_value(prim,'physxRigidBody:solverPositionIterationCount'),
                          velocity=_value(prim,'physxRigidBody:solverVelocityIterationCount'))
    issues=[]
    for name,counts in bodies.items():
        if counts!={'position':position,'velocity':velocity}:issues.append(f'{name}: applied body counts {counts} differ from requested {position}/{velocity}')
    for key,usd_key in zip(BOUND_NAMES,USD_BOUNDS):
        available=[(s['path'],s['attributes'][usd_key]) for s in scenes if s['attributes'].get(usd_key) is not None]
        if not available and config[key] is not None:available=[('SimulationCfg.physx',config[key])]
        requested=position if 'position' in key else velocity
        for source,bound in available:
            if ('min_' in key and requested<bound) or ('max_' in key and requested>bound):
                issues.append(f'{source}: {key}={bound} would clamp requested {requested}')
    return dict(requested=dict(position=position,velocity=velocity),bodies=bodies,
                scene_solver_settings=scenes,simulation_config=config,issues=issues,
                scope='Authored runtime body/scene settings and configured global bounds; no solver-force measurement.')
