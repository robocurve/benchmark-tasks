"""Generate the dry bench STL masters (mm) from trainer.scad. Requires numpy,
trimesh and its manifold3d boolean engine. All fixed/moving geometry is in a
common assembly coordinate system except the three upright dummy cartridges.
"""
import ast
import math
from pathlib import Path
import re
import numpy as np
import trimesh as tm

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "trainer.scad"
CIRCLE = 64

def move(mesh, xyz):
    mesh.apply_translation(xyz)
    return mesh

def box(xyz, size):
    return move(tm.creation.box(extents=size), xyz)

def rotate(mesh, axis, deg, pivot=(0, 0, 0)):
    mesh.apply_transform(tm.transformations.rotation_matrix(math.radians(deg), axis, pivot))
    return mesh

def pin(xyz, diameter, length, axis):
    mesh = tm.creation.cylinder(diameter / 2, length, sections=CIRCLE)
    if axis == "y":
        rotate(mesh, [1, 0, 0], 90)
    if axis == "x":
        rotate(mesh, [0, 1, 0], 90)
    return move(mesh, xyz)

def union(*pieces):
    return tm.boolean.union(pieces, engine="manifold")

def cut(mesh, *holes):
    return tm.boolean.difference([mesh, *holes], engine="manifold")

def read_shapes(module):
    text = SOURCE.read_text().split(f"module {module}() {{", 1)[1].split("\n}", 1)[0]
    parts = {}
    for line in text.strip().splitlines():
        name = line.split("//")[-1].strip()
        def vector(keyword):
            match = re.search(keyword + r"\((\[[^)]*\])(?=,center|\))", line)
            return ast.literal_eval(match[1])
        if "cube(" in line:
            shape = box((0, 0, 0), vector("cube"))
        else:
            section = vector("polygon")
            if section[-1] != section[0]:
                section.append(section[0])
            shape = tm.creation.revolve(np.array(section), sections=48)
            shape.fix_normals()
        rotate(shape, [0, 0, 1], vector("rotate")[2])
        parts[name] = move(shape, vector("translate"))
    return parts

def frame():
    solids = read_shapes("frame")
    core = [v for k,v in solids.items() if k.startswith(("trainer_", "socket_", "bay_", "housing_", "lever_", "seat_index", "window_"))]
    for angle, name in ((64.5, "stop_a"), (244.5, "stop_b")):
        core.append(solids[name])
        core.append(rotate(box((13.2, 0, 195.5), (6.0, 1.8, 1.7)), [0,0,1], angle))
    base = box((5.5,39,3),(121,140,6))
    # Rear mounts for the bezel, guide and hinge. Keep clear of the two entry slots.
    bridges = [
        box((23,0,196.5),(4,30,5)), box((-1,-15,212),(15,3,4)),
        box((-13,-7.4,214),(3.4,2.3,3)), box((-11,-7.4,214),(4,2.3,3)),
        box((21,18,222),(6,12,6)), box((14,24,226),(19,6,6)),
        box((3,24,227),(11,10,8)), box((-23,75,201.8),(12,8,16)),
        box((-32,74,201),(20,5,6)), box((-35,71,201.5),(17,3,16)),
        box((-43,70.5,212),(3,4,34)), box((-39,67.5,227.5),(11,9,4))]
    fixed = union(base, *core, *bridges)
    # Spring eye and a curved stop slot for the M2 rocker follower pin.
    def arc_center(angle):
        a = math.radians(angle)
        return (-23-12*math.cos(a), 71, 204.5-12*math.sin(a))
    arc = [pin(arc_center(a),2.55,5,"y") for a in np.linspace(0,32,65)]
    fixed = cut(fixed, union(*arc), pin((-35,64.5,227.5),1.6,13,"y"),
                pin((-23,75,204.5),3.3,13,"y"),
                box((3,24,227),(6.6,5.6,12)), pin((3,24,227),2.2,14,"x"),
                box((3,16.5,227),(3.8,12,45)), box((11.56,-14,226),(2.5,7,42)),
                box((-34,63,198.5),(14,6.6,5.5)),
                *(rotate(box((30,0,192.5),(32,5.5,4.0)),[0,0,1],angle) for angle in (44,224)),
                box((-46,-19,3),(70,38,10)),
                *(pin((x,y,3),4.5,10,"z") for x,y in ((-48,8),(-48,101),(58,-24),(58,101))))
    return fixed

def lever():
    parts = [move(v,(-23,0,204.5)) for v in read_shapes("lever").values()]
    blank = union(*parts, pin((-23,63,204.5),8.2,10,"y"), pin((-23,37,212.4),11.8,36,"y"),
                  box((-29,63,204.5),(13,5,3.4)), pin((-35,63,204.5),4.8,5,"y"))
    return cut(blank, pin((-23,63,204.5),3.3,20,"y"), pin((-35,63,204.5),2.0,12,"y"),
               pin((-23,37,212.4),9.2,38,"y"))

def carriage():
    parts = [move(v,(0,0,202.1)) for k,v in read_shapes("follower").items() if k != "flag_carrier"]
    # Join the visual flag to the fork. Slide tongue lies behind the rear of the socket.
    blank = union(*parts, box((11.56,-15.6,208.8),(1.6,22.4,1.6)),
                  box((-12.85,-16,207),(0.6,8,6)), box((-.72,-26,208.8),(26.1,1.6,1.6)), box((-13,-23,208.8),(.9,7,1.6)),
                  box((3,17,208.9),(3,15.5,1.6)), box((3,24,222.1),(6,5,34)),
                  box((3,24,239.1),(17,8,2.8)))
    slot = union(pin((3,24,215),2.55,9,"x"), pin((3,24,227),2.55,9,"x"),
                 box((3,24,221),(9,2.55,12)))
    return cut(blank, slot, *(pin((x,24,239.1),3.3,6,"z") for x in (-2,8)))

def dummy(name):
    parts = read_shapes(name)
    # Root webs join the decorative adapter and O-ring profiles to the solid body.
    # An open ballast socket and its cross vent accept a bonded solid metal slug.
    blank = union(*parts.values(), pin((0,0,129),12,8,"z"), pin((0,0,135),10.8,1.8,"z"))
    return cut(blank, pin((0,0,45),8.4,94,"z"), pin((0,0,8),2.2,29,"x"))

def write(name, mesh):
    if len(mesh.split(only_watertight=False)) != 1 or not mesh.is_volume:
        raise ValueError(f"{name}: disconnected or open master")
    mesh.export(HERE / (name + ".stl"))

if __name__ == "__main__":
    write("receiver", frame())
    write("rocker", lever())
    write("shutter", carriage())
    for filename, module in (("original","installed"), ("intact","spare"), ("reject","reserve")):
        write(filename, dummy(module))
