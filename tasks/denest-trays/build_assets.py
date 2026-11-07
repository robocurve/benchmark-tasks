"""Write original compound-box USD trays; standard library only, no downloaded assets."""
import importlib.util
from pathlib import Path

def build(task):
    root=Path(__file__).parent/'assets'; root.mkdir(exist_ok=True)
    for name,parts in task.PARTS.items():
        lines=['#usda 1.0','(defaultPrim = "Tray"; metersPerUnit = 1; upAxis = "Z")',
               'def Xform "Tray" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]) {',
               f'float physics:mass = {task.MASS[name]}',
               'def Material "Surface" (prepend apiSchemas = ["PhysicsMaterialAPI"]) {',
               'float physics:staticFriction = 0.8','float physics:dynamicFriction = 0.6',
               'float physics:restitution = 0','}']
        for i,(pos,size) in enumerate(parts):
            lines += [f'def Cube "part_{i}" (prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]) {{',
                      'double size = 1',f'double3 xformOp:translate = {pos}',f'double3 xformOp:scale = {size}',
                      'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
                      f'color3f[] primvars:displayColor = [{task.COLORS[name]}]',
                      'rel material:binding:physics = </Tray/Surface>','}']
        lines+=['}']; (root/f'{name}.usda').write_text('\n'.join(lines)+'\n')

if __name__=='__main__':
    spec=importlib.util.spec_from_file_location('denest_task',Path(__file__).with_name('task.py'))
    task=importlib.util.module_from_spec(spec); spec.loader.exec_module(task); build(task)
