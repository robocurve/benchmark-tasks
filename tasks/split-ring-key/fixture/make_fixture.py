"""Generate the shop assembly and individual solids in millimetres (CadQuery 2.6.1).

Bench X points from ring to bearing, Y to the splitter side, Z up from the bench.
Splitter parts are drawn in outward-radial, tangent, forward-axial coordinates.
Pilot holes for tapped fasteners show the drill diameter. Threads are cosmetic.
"""
import argparse
import math
from pathlib import Path

import cadquery as cq

RADIUS = 18.01
END_ANGLE = -20.0
GUIDE_ANGLE = END_ANGLE - 25.0
WINDING = 700.0
PARK_OFFSET = 140.0
TRAVEL = 140.5
LIFT = 8.0
BENCH_HOLES = [(-5, -85), (265, -85), (-5, 85), (215, 85), (215, 290), (255, 290)]
BLUE, STEEL, ORANGE, BRASS = (0.11,0.37,0.48), (0.70,0.73,0.75), (0.90,0.40,0.09), (0.77,0.55,0.19)
HERE = Path(__file__).resolve().parent


def block(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1-x0, y1-y0, z1-z0, (x0, y0, z0))


def pin(start, axis, diameter, length):
    return cq.Solid.makeCylinder(diameter/2, length, start, axis)


def sleeve(start, axis, od, bore, length):
    return pin(start, axis, od, length).cut(pin(start, axis, bore, length))


def radial_frame(angle):
    angle = math.radians(angle)
    return cq.Plane((0, 0, 240), (0, math.cos(angle), -math.sin(angle)), (-1, 0, 0)).location


def add_fixture(assembly, feed=0, lift=0):
    if not 0 <= feed <= TRAVEL or not 0 <= lift <= LIFT:
        raise ValueError("Feed must be 0–140.5 mm and lift 0–8 mm.")
    def add(name, shape, color=STEEL, pose=None):
        if not shape.isValid() or len(shape.Solids()) != 1:
            raise ValueError(f"Invalid or disconnected part: {name}")
        assembly.add(shape, name=name, loc=pose or cq.Location(), color=cq.Color(*color))

    # Fixed bench members. Six through-holes also serve as the drilling template.
    base = block(-20, 280, -100, 100, 0, 16).fuse(block(200, 270, 65, 305, 0, 16)).clean()
    angle = math.radians(GUIDE_ANGLE)
    side_y = (RADIUS + 280)*math.cos(angle)
    side_top = 240 - (RADIUS + 280)*math.sin(angle)
    for x, y in BENCH_HOLES:
        base = base.cut(pin((x,y,0), (0,0,1), 6.6, 16))
    for x,y in [(250,-12), (250,12), (245,side_y), (255,side_y)]:
        base = base.cut(pin((x,y,0), (0,0,1), 5.5, 16))
    add("01_bench_plate", base, BLUE)

    tower = block(237,263,-21,21,16,261)
    for x in (237,255):
        tower = tower.cut(pin((x,0,240), (1,0,0), 28, 8))
    tower = tower.cut(pin((237,0,240), (1,0,0), 24, 26))
    for y in (-12,12):
        tower = tower.cut(pin((250,y,16), (0,0,1), 4.2, 16))
    brake_tab = block(225,248,-5,5,257,262).cut(pin((229.5,0,257),(0,0,1),2.5,5))
    add("02_bearing_upright", tower.fuse(brake_tab).clean(), BLUE)
    add("02_drag_pad",pin((229.5,0,254),(0,0,1),3,3),BRASS)
    for name,x in (("front",237), ("back",255)):
        add("03_bearing_"+name, sleeve((x,0,240),(1,0,0),28,12,8))
    # Inner-race spacers restrict the collars' contact to the inner races.
    for name,x in (("front",235),("back",263)):
        add("04_spacer_"+name, sleeve((x,0,240),(1,0,0),16,12.1,2))
    for name,x in (("front",224),("back",265)):
        add("05_collar_"+name, sleeve((x,0,240),(1,0,0),28,12,11))
    add("06_spindle", pin((15,0,240),(1,0,0),12,262))

    # Four independently rotating grips. Plate finished as one cross and hub.
    wheel = pin((203,0,240),(1,0,0),28,16)
    for theta in (0,90,180,270):
        arm = block(210,218,0,85,234,246).fuse(pin((210,85,240),(1,0,0),12,8))
        wheel = wheel.fuse(arm.rotate((0,0,240),(1,0,240),theta))
    wheel = wheel.cut(pin((203,0,240),(1,0,0),12,16))
    wheel = wheel.cut(pin((206,15,240),(0,-1,0),3.3,10))
    for n,(y,z) in enumerate(((85,240),(0,325),(-85,240),(0,155))):
        wheel = wheel.cut(pin((210,y,z),(1,0,0),5.2,8))
        add(f"08_grip_pin_{n+1}",pin((85,y,z),(1,0,0),5,140))
        add(f"08_pin_spacer_{n+1}",sleeve((111,y,z),(1,0,0),7,5.2,99),STEEL)
        add(f"09_spinning_knob_{n+1}",sleeve((90,y,z),(1,0,0),12,5.2,20),ORANGE)
    add("07_handwheel",wheel.clean(),BLUE)

    # Clamp screws into the rear strand from outside the ring. The front strand can move.
    jaw_frame = radial_frame(WINDING*5/20)
    jaw = sleeve((0,0,-27),(0,0,1),20,12,14)
    jaw = jaw.fuse(block(7, RADIUS+7,-3,3,-17,-12),
                   block(RADIUS-4,RADIUS+7,-3,3,-13,-1.98),
                   block(RADIUS-3.8,RADIUS-.99,-2,2,-2.1,-.72).intersect(
                       pin((0,0,-2.1),(0,0,1),2*(RADIUS-.99),1.38)),
                   block(RADIUS+3.1,RADIUS+7,-3,3,-2.2,.5)).clean()
    jaw = jaw.cut(pin((RADIUS+7,0,-1.24),(-1,0,0),1.6,4.1))
    jaw = jaw.cut(pin((0,10,-22),(0,-1,0),3.3,6))
    add("10_rotary_jaw", jaw, BRASS, jaw_frame)
    add("11_jaw_screw_tip",pin((RADIUS+.99,0,-1.24),(1,0,0),2,7.5),STEEL,jaw_frame)

    # Ring is a reference solid at rest; moving the catch in CAD does not deform it.
    # Sweep a smooth centreline over the single transition sector.
    ring_frame = radial_frame(0)
    pts = []
    for i in range(281):
        theta = WINDING*i/280
        if theta <= WINDING-360:
            ax = -.99
        elif theta >= 360:
            ax = .99
        else:
            u = (theta - (WINDING-360))/(720-WINDING)
            ax = -.99 + 1.98*(3*u*u-2*u*u*u)
        rad = math.radians(theta)
        pts.append(cq.Vector(RADIUS*math.cos(rad),RADIUS*math.sin(rad),ax))
    # The two individual turn solids meet at their surfaces. Keep the reference out
    # of the bill of fabricated parts; it is bought under the supplier number.
    for name, indices in (("rear",range(0,137)),("crossover",range(136,145)),("front",range(144,281))):
        ps = [pts[i] for i in indices]
        path = cq.Edge.makeSpline(ps)
        wire = cq.Wire.assembleEdges([path])
        start = cq.Plane(ps[0], normal=path.tangentAt(0))
        shape = cq.Workplane(start).circle(.99).sweep(cq.Workplane(obj=wire)).val()
        add("ref_ring_"+name,shape,STEEL,ring_frame)

    # The side bridge carries a common backing plate; all radial-guide holes share its datum.
    sidepost = block(239,261,side_y-8,side_y+8,16,side_top-32)
    for x in (245,255):
        sidepost = sidepost.cut(pin((x,side_y,16),(0,0,1),4.2,16))
        sidepost = sidepost.cut(pin((x,side_y,side_top-32),(0,0,-1),4.2,16))
    add("12_guide_upright", sidepost, BLUE)
    bridge = block(40,262,side_y-8,side_y+8,side_top-32,side_top-20)
    for x in (245,255):
        bridge = bridge.cut(pin((x,side_y,side_top-32),(0,0,1),5.5,12))

    # Frame brackets can be machined together or brazed as a single part.
    guide = radial_frame(GUIDE_ANGLE)
    backing = block(RADIUS+96,RADIUS+378,-18,18,-45,-40)
    backing = backing.cut(block(RADIUS+115,RADIUS+281,-4,4,-45,-40))
    for s in (RADIUS+104,RADIUS+292):
        backing = backing.fuse(block(s-6,s+6,-18,18,-40,-15))
        for t in (-10,10):
            backing = backing.cut(pin((s-6,t,-24),(1,0,0),6,12))
            backing = backing.cut(pin((s,t,-15),(0,0,-1),2.5,8))
        backing = backing.cut(pin((s-6,0,-22),(1,0,0),2.5,12))
    backing = backing.fuse(block(RADIUS+362,RADIUS+377,15,29,-45,-40))
    # Park spring tail mounting holes at 2 mm pitch for force adjustment.
    for delta in (-4,0,4):
        backing = backing.cut(pin((RADIUS+368.8+delta,25,-45),(0,0,1),2.5,5))
    # A plain bolted pedestal fills the vertical offset between bridge and backing plate.
    pad = block(40,55,side_y-8,side_y+8,side_top-29,side_top+4)
    union = backing.clean().moved(guide).fuse(bridge,pad).clean()
    union = union.cut(block(RADIUS+115,RADIUS+281,-4,4,-47,-37).moved(guide)).clean()
    if len(union.Solids()) != 1:
        raise ValueError("Bridge does not reach the backing plate")
    add("13_bridge_and_radial_guide",union,BLUE)
    for t,label in ((-10,"a"),(10,"b")):
        add("14_guide_rod_"+label,pin((RADIUS+98,t,-24),(1,0,0),6,200),STEEL,guide)

    # All splitter dimensions below use p=0 at the vane midline in its nominal entry section.
    off = RADIUS + PARK_OFFSET - feed
    moving = guide * cq.Location(cq.Vector(off,0,0))
    blade = moving * cq.Location(cq.Vector(0,0,lift))
    car = block(116,140,-18,18,-32,-15)
    for t in (-10,10):
        car = car.cut(pin((116,t,-24),(1,0,0),12,19))
        car = car.cut(pin((135,t,-24),(1,0,0),9,5))
    for s in (120,136):
        car = car.cut(pin((s,0,-32),(0,0,1),3.05,17))
    # Leaf clamp and spring anchor sit outside the side of the 6 mm rails.
    car = car.fuse(block(62,128,18.5,27,-23,-20),block(116,128,17,20,-23,-20),
                   block(62,71,19,27,-21,-16),
                   block(135.5,141,15,28,-31,-26)).clean()
    car = car.cut(pin((138,25,-31),(0,0,1),2.5,5))
    for s in (64,69):
        car = car.cut(pin((s,23,-16),(0,0,-1),2,5))
    for s in (122,133):
        car = car.cut(pin((s,6.5,-15),(0,0,-1),1.6,3.5))
    add("15_radial_carriage",car,BLUE,moving)
    for t,label in ((-10,"a"),(10,"b")):
        add("16_linear_bush_"+label,sleeve((116,t,-24),(1,0,0),12,6,19),STEEL,moving)
    for s,label in ((120,"a"),(136,"b")):
        add("17_lift_pin_"+label,pin((s,0,-45),(0,0,1),3,44),STEEL,blade)
    # Axial stop collars are set with a feeler block to give 8 mm lift.
    for s,label in ((120,"a"),(136,"b")):
        add("18_home_spacer_"+label,sleeve((s,0,-15),(0,0,1),6,3.15,2),STEEL,moving)
        add("18_lift_stop_"+label,sleeve((s,0,-44.8),(0,0,1),5.8,3,4.8),STEEL,blade)

    shoe = block(-2,3,-.8,.8,-3.08,-2.08)
    shoe = shoe.fuse(block(3,5.5,-1,1,-6,-2.08),block(5.4,12,-2,2,-9,-3.5),block(10,106,-4,6,-10,-5),
                     block(103,111,-4,9,-10,-5),block(108,140,4.8,9,-15,-5)).clean()
    for s in (122,133):
        shoe = shoe.cut(pin((s,6.5,-15),(0,0,1),2.2,11))
    add("19_backing_shoe",shoe,STEEL,moving)

    # Working end is metal, ground from both faces. Width is tangential.
    blade_profile = [(-2,-.03),(3,-.70),(16,-.70),(16,.70),(3,.70),(-2,.03)]
    nose = cq.Workplane("XZ").polyline(blade_profile).close().extrude(.6,both=True).val()
    nose = nose.fuse(block(5,19,-3,3,-.70,.70)).clean()
    for s in (9,16):
        nose = nose.cut(pin((s,0,-.70),(0,0,1),2.2,1.4))
    add("20_ground_catch",nose,STEEL,blade)
    arm = block(6,21,-3,3,-3,-.70).fuse(block(19,145,-3,3,-3,1.5),
            block(113,143,-4,4,-13,-2),block(127,134,2,25,-3.5,1.5),
            block(128,133,19,27,-16.5,-2.5)).clean()
    # Radial leaf can slide sideways at the eye as its free end deflects.
    arm = arm.cut(block(127.5,134,19.8,26.2,-16.1,-15.55))
    for s in (120,136):
        arm = arm.cut(pin((s,0,-13),(0,0,1),3,12))
    for s in (9,16):
        arm = arm.cut(pin((s,0,-3),(0,0,1),1.6,2.3))
    arm = arm.fuse(pin((140,0,1),(0,0,1),6,26)).clean()
    arm = arm.fuse(pin((140,0,27),(0,0,1),5,8)).clean()
    add("21_blade_arm",arm,STEEL,blade)
    cube = block(132,148,-8,8,27,43).cut(pin((140,0,27),(0,0,1),4.2,12))
    add("22_splitter_handle",cube,ORANGE,blade)
    leaf = block(62,132,20,26,-16,-15.7)
    for s in (64,69):
        leaf = leaf.cut(pin((s,23,-16),(0,0,1),2.7,.30))
    add("23_axial_leaf_at_rest",leaf,STEEL,moving)
    # Spring endpoints and length at park are shown as a tubular envelope.
    # The centreline rises from the carriage lug to a stand-off on the tail.
    # Use eyes with both centres at w=-36; small hooks are omitted.
    add("24_radial_return_envelope",sleeve((off+138,25,-36),(1,0,0),6.35,5.63,
        (RADIUS+368.8)-(off+138)),STEEL,guide)

    # Separate key rest at first-frame position. Key profile dimensions follow build_key().
    stand = block(-77,-2,-170,-110,0,12).fuse(block(-46,-32,-146,-134,12,214),
                                             block(-47,-33,-150,-130,213,219)).clean()
    for x in (-62,-16):
        for y in (-158,-122):
            stand = stand.cut(pin((x,y,0),(0,0,1),4.5,12))
    add("25_key_stand",stand,BLUE)
    bow = cq.Workplane("XY").circle(5.5).circle(3.5).extrude(2).val()
    outline = [(4,-4),(8,-9),(16,-8),(18,-4),(18,4),(16,8),(8,9),(4,4)]
    shoulder = cq.Workplane("XY").polyline(outline).close().extrude(2).val()
    key = bow.fuse(shoulder,block(16,42,-2.5,3.5,0,2))
    for i,depth in enumerate((1.5,.5,2,1,1.8)):
        key = key.fuse(block(17.8+4.5*i,20.2+4.5*i,-2.5-depth,-2.5,0,2))
    key = key.clean().rotate((0,0,0),(0,0,1),180).translate((-25,-140,219))
    add("26_brass_key",key,BRASS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,default=HERE/"fixture.step")
    parser.add_argument("--parts",type=Path,help="Also export each named component as a STEP solid.")
    parser.add_argument("--feed",type=float,default=0,help="Inward slide motion from park, mm.")
    parser.add_argument("--lift",type=float,default=0,help="Catch lift, mm (ring reference stays at rest).")
    args = parser.parse_args()
    asm = cq.Assembly(name="split_ring_vise_and_splitter")
    add_fixture(asm,args.feed,args.lift)
    asm.export(str(args.out))
    # The STEP writer pads some wrapped records with trailing spaces.
    lines = args.out.read_text().splitlines()
    args.out.write_text("\n".join(line.rstrip(" \t") for line in lines) + "\n")
    if args.parts:
        args.parts.mkdir(parents=True,exist_ok=True)
        for part in asm.children:
            cq.exporters.export(part.obj,str(args.parts/(part.name+".step")))


if __name__ == "__main__":
    main()
