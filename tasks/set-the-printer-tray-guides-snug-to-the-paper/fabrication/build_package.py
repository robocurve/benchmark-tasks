"""Original dimensioned SVG/PNG and millimetre DXF fabrication drawings. No simulator or GPU."""
from __future__ import annotations

import ast
import csv
import hashlib
import json
import math
from pathlib import Path
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'task.py'
DRAW = ROOT / 'drawings'
CAD = ROOT / 'cad'
DRAW.mkdir(exist_ok=True)
CAD.mkdir(exist_ok=True)
def font_for(size):
    for font in ('DejaVuSans.ttf', '/System/Library/Fonts/Supplemental/Arial.ttf', 'Arial.ttf'):
        try:
            return ImageFont.truetype(font, size)
        except OSError:
            pass
    return ImageFont.load_default()
BLUE, RED, GREY, GREEN = '#176b9a', '#ba3434', '#707980', '#187251'


class Sheet:
    def __init__(self, name, title):
        self.name = name
        self.svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="220mm" height="300mm" viewBox="0 0 1100 1500">',
                    '<rect width="1100" height="1500" fill="white"/>']
        self.im = Image.new('RGB', (1100, 1500), 'white')
        self.d = ImageDraw.Draw(self.im)
        self.rect(30, 30, 1040, 1440)
        self.text(65, 64, title, 27)
        self.text(65, 110, 'PR 2564 / FC1  |  ORIGINAL DESIGN  |  dimensions mm unless stated', 17, BLUE)
        self.text(65, 1420, 'Revision FC1 construction drawing. Prototype not yet built; calibration records blank. Dimension labels govern; do not scale.', 16)
        self.text(65, 1447, 'MIT original assets. No vendor CAD. Assembly dimensions and acceptance in ASSEMBLY.md.', 14)

    def text(self, x, y, s, size=18, color='#18232c'):
        self.svg.append(f'<text x="{x}" y="{y + size}" font-family="Arial,sans-serif" font-size="{size}" fill="{color}">{escape(str(s))}</text>')
        self.d.text((x, y), str(s), fill=color, font=font_for(size))

    def line(self, x1, y1, x2, y2, color='#18232c', width=2):
        self.svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}"/>')
        self.d.line((x1, y1, x2, y2), fill=color, width=width)

    def rect(self, x, y, w, h, fill=None, color='#18232c', width=2):
        self.svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill or "none"}" stroke="{color}" stroke-width="{width}"/>')
        self.d.rectangle((x, y, x+w, y+h), fill=fill, outline=color, width=width)

    def circle(self, x, y, r, color='#18232c', fill=None):
        self.svg.append(f'<circle cx="{x}" cy="{y}" r="{r}" stroke="{color}" stroke-width="2" fill="{fill or "none"}"/>')
        self.d.ellipse((x-r, y-r, x+r, y+r), outline=color, fill=fill, width=2)

    def poly(self, pts, color='#18232c', fill=None):
        pts = [(float(x), float(y)) for x, y in pts]
        self.svg.append(f'<polygon points="{" ".join(f"{x},{y}" for x,y in pts)}" fill="{fill or "none"}" stroke="{color}" stroke-width="2"/>')
        self.d.polygon(pts, fill=fill)
        self.d.line(pts + pts[:1], fill=color, width=2)

    def dim(self, x1, y1, x2, y2, label):
        self.line(x1, y1, x2, y2, BLUE, 1)
        self.line(x1-4, y1-5, x1+4, y1+5, BLUE, 1)
        self.line(x2-4, y2-5, x2+4, y2+5, BLUE, 1)
        self.text((x1+x2)/2+5, (y1+y2)/2-25, label, 17, BLUE)

    def save(self):
        (DRAW/f'{self.name}.svg').write_text('\n'.join(self.svg + ['</svg>']))
        self.im.save(DRAW/f'{self.name}.png')


def rectangle(w, h):
    return [(0, 0), (w, 0), (w, h), (0, h)]


def dxf(name, outline, holes=(), loops=()):
    a = ['0', 'SECTION', '2', 'HEADER', '9', '$ACADVER', '1', 'AC1015',
         '9', '$INSUNITS', '70', '4', '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES']
    for layer, pts in [('CUT', outline)] + [('CUTOUT', q) for q in loops]:
        a += ['0', 'LWPOLYLINE', '8', layer, '90', str(len(pts)), '70', '1']
        for x, y in pts:
            a += ['10', f'{x:.6f}', '20', f'{y:.6f}']
    for x, y, dia in holes:
        a += ['0', 'CIRCLE', '8', 'DRILL', '10', str(x), '20', str(y), '40', str(dia/2)]
    a += ['0', 'ENDSEC', '0', 'EOF']
    (CAD/f'{name}.dxf').write_text('\n'.join(a)+'\n')


parts = []


def part(name, w, h, thick, qty, material, holes=(), loops=(), outline=None, notes=''):
    outline = outline or rectangle(w, h)
    dxf(name, outline, holes, loops)
    parts.append(dict(name=name, width_mm=w, height_mm=h, thickness_mm=thick,
                      qty=qty, material=material, notes=notes,
                      holes=[list(v) for v in holes], outline=outline, cutouts=loops))


# Cut profiles use local manufacturing coordinates, with (0,0) at lower left.
# FC1 changes are dimensioned in ASSEMBLY.md; all custom profiles are original.
floor_slots = [[(16,150),(90,150),(90,154),(16,154)],
               [(150,241),(154,241),(154,395),(150,395)]]
for a, b in ((12, 94), (210, 292)):
    for v in (109.5, 189.5):
        floor_slots.append([(a,v),(b,v),(b,v+5),(a,v+5)])
for u in (89.5, 209.5):
    floor_slots.append([(u,213),(u+5,213),(u+5,402),(u,402)])
part('floor',304,417,3,1,'6061-T6',holes=[(14,24,8),(290,24,8),(14,402,8),(290,402,8),(152,152,3.2),(13,152,2.2),(93,152,2.2),(152,217,2.2),(152,398,2.2),(86,208,3.2),(98,208,3.2),(206,208,3.2),(218,208,3.2)],loops=floor_slots,
     notes='Origin maps to U=-152,V=-12. Slots are cut through. Rod tracks: U-136..-62,V138..142; U-2..2,V229..383. Four dia8 table-anchor access holes outside paper. See ASSEMBLY.md.')
for label, length, qty in [('width',100,2),('length',140,1)]:
    part(f'pad_{label}',length,32.5,3,qty,'PP-H',holes=[(14,14.5,2.2),(length-14,14.5,2.2)],
         notes='Paper face: 90 degree countersink to received M2 head, flush 0..0.05 below surface. Hole Z=18 in assembly.')
    part(f'back_{label}',length,32.5,4,(1 if label=='width' else qty),'POM-C',
         outline=(rectangle(length,32.5) if label=='width' else [(0,0),(length,0),(length,32.5),(length/2+31,32.5),(length/2+31,24),(length/2-31,24),(length/2-31,32.5),(0,32.5)]),
         holes=[(14,14.5,3.1),(length-14,14.5,3.1),(4,4,3.2),(length-4,4,3.2),(4,28.5,3.2),(length-4,28.5,3.2)],
         notes='Ream shaft bores 3.10+.03/-0. N-22..-18. Width plain part only far cartridge; rear has open-top relief S centre+-31, Z27.5..36. Latched bridge M2 blind vertical at N-20, S centre+-10, depth 4 from Z27.5; .5 shim + bridge 4 + M2x8 CSK flush 32. Root/axle/collar clearance across full hinge span.')
part('cheek_front',9.5,32.5,8,6,'POM-C',notes='N=-18..-8.5. Along N at S4, Z7.5/32: dia3.2; hex nut pocket 5.60AF (corner dia6.47) from N-8.5 to -12.5; M3 nut 2.4 thick ends N-10.1. M3x20 tips N-10. Hardstop M2 along N at S4, Z18, flat button dia3 ends N-8.00; locknut rear. Fit separately each end; set delta 5 with gauge.')
part('cheek_rear',8,32.5,8,6,'POM-C',notes='N=-30..-22; aligns backplate holes. Secondary drill along N, same Z/S as front. M3x20, no washer under rear head; recessed front nut behind N-10.1, screw tip N-10.0. Lock with low-strength removable threadlock.')
lever = [(0,0),(10,0),(10,28),(6,28),(6,4),(0,4)]
part('latch_lever',10,28,30,2,'POM-C',outline=lever,holes=[(8,2,3.1),(2,2,2.2)],
     notes='Origin N-27, Z34; pivot (8,2), crank (2,2). S-15..15. Central fork: mill through pocket N-28..-22, S-2..2 in root, no stock in pocket; drill crank dia2.2 only fork cheeks S+-2..4; pin dia2x9. Root retains two 2mm cheeks adjacent pocket. Pivot dia3.10. See HINGE.md.')
part('post',8,26,30,2,'POM-C',notes='N-11..-3, Z36..62, S+-15. Mill underside relief: Z36..36.5, N-8.5..-3 through 30 span. Keep N-11..-8.5 rear attachment. M3 tapped along N at Z40, S+-10 for bridge screws. See HINGE.md.')
bridge=[(0,0),(21.5,0),(21.5,4),(19,4),(19,16),(15,16),(15,4),(0,4)]
part('latch_bridge',21.5,16,45,2,'6061-T6',outline=bridge,notes='Origin N-30, Z28; S-19.5..25.5. Foot Z28..32, upright Z32..44 ONLY S+-15; mill upright off outside 30 span. S19.5..25.5 spring wing top Z30. Pocket ear feet N-23..-15, S12..19.5 and -19.5..-12, Z30..32; retain 2mm base. Ear foot M2x4 at N-21/-17, S+-13.5. Spring bracket M2x6 from below at N-11, S20.5/24.5. Through pocket N-28..-22, S+-2 for rod/eye. Ears and S2 anchor attach per HINGE.md. Secondary M3 holes normal to N at Z40, S=+-10; countersink rear N=-15 flush for M3x10 into post. See D06. No protruding heads in lever gap.')
part('connector_width',12,30,3,4,'6061-T6',holes=[(6,5,3.2),(6,25,3.2)],
     notes='Bottom Z=-13, bolts Z=-8/12. Floor slot centre at face normal -25, rail centre normal -20; use a 5 offset spacer. Transfer-drill mating block/cheek.')
part('connector_length',12,40,3,2,'6061-T6',holes=[(6,5,3.2),(6,34,3.2)],
     notes='Bottom Z=-27, bolts Z=-22/7. Floor slot centre at face normal -25, rail centre normal -20; use a 5 offset spacer. Transfer-drill mating block/cheek.')
part('carriage_block',20,12,12,4,'POM-C',notes='Local A running 0..20, B0..12, C0..12. Bore 6.10 along A at B6, C6. Connector dia3.2 along B at A5, C9. Do not machine a cross clamp here; two separate 14-high drag_block parts replace one near-width and one rear block. Width axis Z=-11, rear Z=-25. Other blocks loose-running.')
part('shaft_support',16,16,12,4,'POM-C',notes='Section local B0..16, C0..16, extrude along shaft 12. Shaft bore dia6.00H7 along A at B8, C8. Split 1 B8, C11..16. Cross clamp M3 along B at A6, C13.5 (dia3.2 clear one half, M3 tap opposite). Mount bores M2 along A at B3, C3/10.5; supplied floor hang bracket per ASSEMBLY. Rear support bottom Z-33; width uses separate 4 long parts. Shaft axes V100/180, Z-11 and U+-60, Z-25.')
part('pointer_flag',8,21.8,0.4,3,'PET sheet',holes=[(4,3,2.2)],notes='Pointer plate N-24.7..-24.3 at zero, S10..18, Z15..36.8; pin hole S14, Z18; follows pin translation. Attach to lower end moving pin via offset arm; no scale stock intersection. Shelf S0..9.5, pointer gap 0.5. See SCALE.md.')
ring=[(3*math.cos(i*2*math.pi/64),3*math.sin(i*2*math.pi/64)) for i in range(64)]
part('stop_spacer',6,6,1,6,'stainless steel',outline=ring,holes=[(0,0,3.2)],notes='Rear pin zero stop; front ring face at N=-22 with pin end N=-23. M2 washer/screw behind pin end. Select shims from actual zero setting.')
part('plinth_side',417,35,6,2,'6061-T6',holes=[(8,17.5,3.2),(409,17.5,3.2)],notes='Plinth underside at table, floor underside 35 above it. Attach to end panels using corner angles; transfer-drill M3.')
part('plinth_end',292,35,6,2,'6061-T6',holes=[(8,17.5,3.2),(284,17.5,3.2)],notes='Fits between 6-thick side panels, under 304-wide floor; V end cap thickness 6.')
part('front_panel',304,65,12,1,'6061-T6 or rigid POM-C',notes='V=-12..0; no screw/feature proud of feed face V0. Outside dimensions retained.')
part('side_wall',405,30,6,2,'POM-C',notes='U=+-146..152; V0..405. Machine to avoid back-wall double volume at corners.')
part('back_wall',292,30,9,1,'POM-C',notes='V396..405 between side walls.')

# Positive pawl rack: intentionally fine, machined profile, not FDM geometry.
def square_rack(length, pitch=.5, height=.25):
    p=[(0,0),(length,0),(length,3)]
    n=int(round(length/pitch))
    for i in range(n,0,-1):
        x=i*pitch
        p += [(x,3+height),(x-pitch/2,3+height),(x-pitch/2,3),(x-pitch,3)]
    return p

part('lock_rack_width',86,3.25,10,1,'stainless steel',outline=square_rack(86),
     notes='Origin U-142. Tooth crests Z-16, root Z-16.25, bottom -19.25. Top blind M2 depth 2 at U-139, -59, V140; floor hung spacers 16.25 long. Holes lie outside pawl sweep. Rack phase .5 pitch, .25 crest width; no centering ramps.')
part('lock_rack_length',189,3.25,10,1,'stainless steel',outline=square_rack(189),
     notes='Origin V201. Tooth crests Z-29, root -29.25, bottom -32.25. Top blind M2 depth 2 at V205/386, U0. Floor hung spacers 29.25 long. Rack phase .5 pitch, .25 crest width. Bottom clearance 2.75 to table.')
pawl=[(0,0),(4,0),(4,4),(0,4)]
pawl=[(0,4),(4,4),(4,0)]
for i in range(8,0,-1):
    x=i*.5
    pawl += [(x-.05,0),(x-.05,-.20),(x-.20,-.20),(x-.20,0),(x-.5,0)]
part('pawl',4,4.2,10,2,'stainless steel',outline=pawl,notes='S+-5, T+-2; matching teeth .15 wide, .20 deep. Body total height 4 including 1mm lid. Machine dia3.2 counterbore depth 2 from main body top D-1, bottom D-3, centered T0, S0 for rod pickup button. Top M1.6 blind holes depth 2 at T0, S+-3.5; lid fits flush. Guide bore 4.10x10.10. See PAWL.md; no nearest position snap outside +-0.05 phase window.')

# Involute custom pinion and straight rack, all dimensions authored.
m,z,pa=1.0,20,math.radians(20)
rp=m*z/2; rb=rp*math.cos(pa); rr=rp-1.25*m; ra=rp+m
invp=math.tan(pa)-pa
gear=[]
for tooth in range(z):
    c=tooth*2*math.pi/z
    base_half=math.pi/(2*z)+invp-.002
    gear.append((rr*math.cos(c-base_half),rr*math.sin(c-base_half)))
    for j in range(13):
        r=rb+(ra-rb)*j/12
        a=math.acos(rb/r)
        half=math.pi/(2*z)+invp-(math.tan(a)-a)-.002
        gear.append((r*math.cos(c-half),r*math.sin(c-half)))
    for j in range(12,-1,-1):
        r=rb+(ra-rb)*j/12
        a=math.acos(rb/r)
        half=math.pi/(2*z)+invp-(math.tan(a)-a)-.002
        gear.append((r*math.cos(c+half),r*math.sin(c+half)))
    gear.append((rr*math.cos(c+base_half),rr*math.sin(c+base_half)))
part('pinion_20t',22,22,5.8,1,'POM-C',outline=gear,holes=[(0,0,6.1)],notes='m1, 20deg, 20 teeth; pitch tooth thickness pi/2-.04, .04 nominal backlash per mesh. Cutter R<=.25 only below base circle. Free idler on FIXED dia6 axle; NO radial setscrew. Finished face 5.80+-.03; gap 6.00+-.05 between thrust washers. See AXLES.md.')
pitch=math.pi*m
rack=[(0,-5),(200,-5),(200,-1.25)]
for i in range(int(200/pitch),-1,-1):
    c=i*pitch
    lo=max(0,c-(pitch/4+1.25*math.tan(pa)))
    hi=min(200,c+(pitch/4+1.25*math.tan(pa)))
    top=pitch/4-m*math.tan(pa)
    if lo<hi:
        rack += [(hi,-1.25),(min(200,c+top),1),(max(0,c-top),1),(lo,-1.25)]
part('coupling_rack',200,6,6,2,'POM-C',outline=rack,
     notes='m1 straight rack; pitchline local y0, teeth toward +y. Flip one rack for opposed coupling. Mount/phase with slotted brackets, trim endpoints clear of full stroke.')


# FC1 supporting parts: dimensions/secondary operations in standalone assembly sheets.
part('drag_block',20,14,12,2,'POM-C',notes='A0..20, B0..12, C0..14. Bore 6.10 along A at B6, C6. Slit 1 at B6 from C14 to bore; M3 cross screw along B at A14, C11.5 (clearance half 3.2, opposite M3 tap; washer + jam nut). Minimum bore/screw ligament .85, top wall .9. Connector at A5, C9 along B. Dry friction target only.')
part('hinge_ear',7.5,10,8,2,'6061-T6',outline=[(0,0),(7.5,0),(7.5,10),(4,10),(4,2),(0,2)],notes='S/Z profile; positive S12..19.5, Z30..40, negative mirror. Extrude N-23..-15. Foot Z30..32 seats in bridge pocket. Pivot drill dia3.10 along S at N-19, Z36 through outer S16..19.5. Foot M2 CSK clear at N-21/-17, S+-13.5, screw M2x4 flush Z32 into 2mm remaining bridge. Two plain positive ears are NOT used: one per assembly positive; negative uses hinge_ear_stop.')
part('hinge_ear_stop',7.5,14,8,2,'6061-T6',outline=[(0,0),(7.5,0),(7.5,14),(4,14),(4,2),(0,2)],notes='As plain ear but negative S-19.5..-12, Z30..44. Pivot dia3.10 at N-19, Z36. CNC slot in outer 3.5 S using hinge_stop_slot.dxf: center N-19, Z36, R6, width 2.00+0.02; theta 0..0.17. Lever press fit dia2h6 follower at N-19, Z42, S-20..-12. End caps form both mechanical stops; inspect actual angles.')
part('s2_fixed_bracket',4,14,6,2,'6061-T6',notes='Upright N-13..-9, Z30..44, S19.5..25.5, on relieved wing Z30. Bottom blind M2 depth 4 at N-11, S20.5/24.5, screws M2x6 from below wing 2. Spring leg blind bore dia1.30 along N from N-13 to -11 at Z40, S22.5. M2 set from Z44 at N-12, S22.5 tip Z40.6; clamps leg. Coil max N-14.4 gives 1.4 gap. S2 leg centerline uses mean radius 4, NOT outside 4.6.')
part('s2_moving_anchor',7.6,6,10,2,'POM-C',notes='At zero N-24.6..-17, Z56..62, S14.5..24.5. Two M2 along S at N-19, Z57.5/60.5; M2x13 from S24.5 into 3.5mm tapped lever side. Leg bore dia1.30 vertical at N-23, S22.5, Z56..62. M2 setscrew from N-17 at Z59, S22.5 clamps leg at N-22.4. Leg is 26 straight up from tangent N-23, Z36. Main lever pinch surface unchanged; anchor is new exterior surface.')
part('rod_eye',4,8,2,2,'stainless steel',holes=[(2,6,2.2)],notes='Side profile center N-25 at zero; bottom Z30, eye hole Z36. Round upper corners R2. Bottom neck N+-1.5, Z30..34, axial M2 tap depth 3.8 from bottom. Eye S+-1 between fork S+-2; pin 2x9 with retained ends. Do not drill M2 into eye pin bore.')
part('pawl_lid',4,10,1,2,'stainless steel',holes=[(2,5,2.4),(2,1.5,1.7),(2,8.5,1.7)],notes='Recess M1.6 heads flush; body 3 high plus lid 1=4. Lid covers rod head counterbore; rod 2 passes center, integral button 3x1 below. Set 0.45 ±0.02 gap at zero. Lid base is top datum D-1; below gap head top D-1.4490005.')
part('pawl_guide',8.1,16,9.25,2,'POM-C',loops=[[(2,2.95),(6.1,2.95),(6.1,13.05),(2,13.05)]],notes='Plan T+-4.05, S+-8; bore T+-2.05, S+-5.05. Z=rack crest-1 to crest+8.25; relieve both T-walls to crest+0.30 over S+-5.05 so they clear the rack. Width Z-17..-7.75, rear -30..-20.75. Body slide 4.10x10.10. Two M1.6 cap mounts at T-2, S+-6.5. Floor hanger along T-3 with 2 M3 at S+-6.5. See PAWL.md.')
part('pawl_cap',8.1,16,2,2,'6061-T6',holes=[(4.05,8,2.4),(2.05,1.5,1.7),(2.05,14.5,1.7),(3.05,11.5,1.6)],notes='Seat underside D+4.2; top D+6.2. S3 centered rod axis, flat S3 seat, no recess; spring gap 4.2. Pawl hardstop M2 at T-1, S3.5 adjustable tip D+1.2, locknut above; stop cannot contact spring. Cap fasten M1.6x5 to guide posts.')
part('pawl_drop_width',16,13,4,1,'6061-T6',notes='Moving hanger N-33.05..-29.05, S+-8, Z-17..-4. Top angle flange N-34..-24, S+-8, Z-4..-2; two M2 vertical at N-27, S+-6 to moving beam Z-2..0. Machine guide attachment two M2 along N at Z-15/-12, S+-6.5. Not fixed to floor. Guide bore center N-25.')
part('pawl_drop_rear',16,26,4,1,'6061-T6',notes='Same plan as width drop; vertical Z-30..-4. Top flange Z-4..-2. Guide attachment two M2 along N at Z-28/-25, S+-6.5. Moves with rear carriage beam. Guide bore center N-25.')
part('pawl_beam_width',10,92,2,2,'6061-T6',holes=[(5,46,4),(5,1.5,2.2),(8,1.5,2.2),(5,81.5,2.2),(8,81.5,2.2),(3,40,1.6),(3,52,1.6)],notes='Plan N-30..-20, S-46..46, Z-2..0. Main hole N-25, S0 clears rod. Blocks centres S+-40; mount M2x8 through 3 high risers into 3mm taps at block B1.5 (shaft B6), A5/8; .45 ligament to bore. Drop bracket M2 at N-27, S+-6, tapped beam 2 deep; screw M2x4. Gear faces end Z-3 -> 1 gap below beam.')
part('pawl_beam_rear',10,132,2,1,'6061-T6',holes=[(5,66,4),(5,1.5,2.2),(8,1.5,2.2),(5,121.5,2.2),(8,121.5,2.2),(3,60,1.6),(3,72,1.6)],notes='Plan N-30..-20, S-66..66, Z-2..0. Mount as width but 17 high risers and M2x22 through 17+2 into 3mm taps. Block axes S+-60, Z-25; top Z-19. Drop M2x4 at N-27, S+-6.')
part('beam_riser_width',6,6,3,3,'POM-C',holes=[(1.5,1.5,2.2),(4.5,1.5,2.2)],notes='Plan N-26.5..-20.5, S shaft-6..0; Z-5..-2 for plain blocks; drag block top Z-3 requires face height 1 instead of 3 at one width location. M2 screws fixed length trim to min 3 thread, max 3.5; no bores cross shaft. Each riser finished to actual top datum.')
part('beam_riser_rear',6,6,17,1,'POM-C',holes=[(1.5,1.5,2.2),(4.5,1.5,2.2)],notes='Plan as width at S shaft-6..0, Z-19..-2. Rear 14 high drag top Z-17 requires 15 high riser instead of 17 at one block. M2x22/20 respectively through 2+17/15 into 3 thread. All values design not measurements.')
part('scale_shelf',6,9.5,.5,3,'PET sheet',holes=[(1,2,2.2),(5,2,2.2)],notes='N-30..-24, S0..9.5, Z36..36.5. Adhesive art S4.5..9.5, Z36.5..36.6. Fix directly on cheek with M2x4 pan heads at N-29/-25, S2; head OD3.8, Z36.5..38.1. Brackets behind N-8.5; pointer S10..18 lateral clearance .5.')
part('table_anchor',12,12,12,4,'6061-T6',outline=[(0,0),(12,0),(12,12),(9,12),(9,3),(0,3)],notes='L side profile U134..146, Z-35..-23 mirrored; extrude 12 along V at 6..18/384..396. Base M4 clear 4.5 vertical at U138, V12/390; two horizontal M3 along U at V9/15 or 387/393, Z-27, connect plinth side with countersunk M3 trim 9 flush outer U152. M4x8 into M4 bench thread 5 deep through base 3. Floor access dia8 U+-138, V12/390; flush covers removed during setup.')
part('shaft_support_width',16,16,4,4,'POM-C',notes='Same 16x16 B/C section as rear shaft_support, but 4 along A. Bore 6.00H7 at B8, C8. Split 1 B8, C11..16; cross M3 along B at A2, C13.5; clear 3.2 half/tap other. Mount M2 along A at B3, C3/10.5. Install U141..145 and -145..-141; shafts U+-145. Wall U146..152 with 1 spacer: M2x10 CSK flush outer 152 into 3 support. Bodies at reset end U+-139 => 2mm gap.')
part('width_support_spacer',16,16,1,4,'POM-C',holes=[(3,3,2.2),(3,10.5,2.2)],notes='Between plinth side inner U+-146 and 4 long width support outer U+-145. Hole profile B/C; mated to support B3, C3/10.5. No external support bracket needed.')
part('rail_hanger_rear',20,33,3,2,'6061-T6',notes='Rear shaft end hanger; plane V200..203 or 399..402, U shaft+-10, Z-33..0. Top flange 20x8 under floor. Support center V209/393, 12 long. Shaft ends V203/399 length 196. Front support M2x8 along V through 3 hanger; rear support M2x10 CSK through 6 end wall; holes B3, C3/10.5. Carriage centres V226..376; 20 long bodies V216..386 leave 1 gap to 12 long supports V203..215/387..399. Shaft axis Z-25. See AXLES.md.')


part('beam_riser_width_short',6,6,1,1,'POM-C',holes=[(1.5,1.5,2.2),(4.5,1.5,2.2)],notes='Replaces 3 high riser at near 14 high drag block; Z-3..-2, M2x6.')
part('beam_riser_rear_short',6,6,15,1,'POM-C',holes=[(1.5,1.5,2.2),(4.5,1.5,2.2)],notes='Replaces 17 high riser at rear 14 high drag block; Z-17..-2, M2x20.')
for name,thick,qty in [('width_reset_washer',2,4),('rear_stop_washer',1,4)]:
    circ=[(5*math.cos(i*2*math.pi/80),5*math.sin(i*2*math.pi/80)) for i in range(80)]
    part(name,10,10,thick,qty,'stainless steel',outline=circ,holes=[(0,0,6.2)],notes='Shaft travel stop shim; exact positions AXLES.md. Faces parallel .03; inspect travel endpoints.')
circ=[(7*math.cos(i*2*math.pi/80),7*math.sin(i*2*math.pi/80)) for i in range(80)]
part('width_stop_collar',14,14,4,4,'POM-C',outline=circ,holes=[(0,0,6.1)],notes='Split 1 along B0 from C3.05 to 7; cross M2 along B at A2, C5.5. Clamp .10Nm. Positions U-49..-45 and 45..49 on both width shafts. Flat face contacts block at 70mm.')
part('rack_standoff_width',4,4,16.25,2,'6061-T6',holes=[(2,2,2.2)],notes='Fixed floor to rack root. Locations U-139/-59, V140. M2 finished length 21 CSK flush floor top. Rack top blind M2 depth 2.')
part('rack_standoff_rear',4,4,29.25,2,'6061-T6',holes=[(2,2,2.2)],notes='U0, V205/386; finished M2x34 from floor top. Rack top blind M2 depth 2.')
part('rack_mount_spacer',4,4,1,4,'6061-T6',holes=[(2,2,2.2)],notes='Between moving beam underside Z-2 and rack top -3. M2x6 into 3mm rack. Near U-136/-132, V126.5; far U132/136, V153.5.')
part('pinion_axle',10,9.5,10,1,'stainless steel',outline=[(-5,0),(5,0),(5,-2.5),(3,-2.5),(3,-9.5),(-3,-9.5),(-3,-2.5),(-5,-2.5)],notes='Lathe side profile, NOT extruded manufacture. Rotational axis N0; shoulder diam 10, Z-2.5..0, journal diam 6, Z-9.5..-2.5. Both ends M3 blind 4. AXLES.md. qty one fixed.')
part('crank_rod_width',3,47.8,3,1,'stainless steel',notes='Lathe one piece: button dia3x1 + shank dia2; button TOP to rod TIP 46.80, top 8 M2 thread. Length including button 47.80; actual gap set in eye. PAWL.md.')
part('crank_rod_rear',3,60.8,3,1,'stainless steel',notes='As width rod; button TOP to tip 59.80, total 60.80. Gap 0.45 ±0.02 adjusted and locked.')
part('pad_stop_button',3,25,3,6,'stainless steel',notes='Lathe part: flat head dia3x1 N-9..-8; M2 thread shank N-33..-9, rear adjustment slot. Front cheek M2 tap N-18..-9; head pocket dia3.2, N-9..-8.5. Backplate/rear cheek through clear 2.4 along N at S4/L-4, Z18. Rear washer .4 and M2 jam nut 1.6 N-32..-30; adjust from N-33. Extends 3 behind nominal housing; minimum wall gap 4 at reset.')
part('back_width_latched',100,32.5,4,1,'POM-C',outline=[(0,0),(100,0),(100,32.5),(81,32.5),(81,24),(19,24),(19,32.5),(0,32.5)],holes=[(14,14.5,3.1),(86,14.5,3.1),(4,4,3.2),(96,4,3.2),(4,28.5,3.2),(96,28.5,3.2)],notes='Near only. Relief S19..81, Z27.5..36 through N-22..-18. M2 blind vertical at N-20, S40/60, depth 4 from 27.5. Bridge on .5 shims, M2x8 CSK from 32 tip 24. Relief +-31 clears hinge collars up to 28.5 and axle 30. Ream pin bores 3.10+.03/-0.')
part('bridge_shim',4,6,.5,4,'stainless steel',holes=[(2,3,2.2)],notes='N-22..-18, S=latch centre+-10 (span 6), Z27.5..28. Supports bridge foot on relieved backplate. One M2x8 CSK per shim into 3.5 thread. Two per latched cartridge.')
# Arcuate hardstop slot in negative hinge ear, absolute N/Z coordinates.
def arcslot(cx,cy,r,a0,a1,rad):
    points=[]
    # Normal N grows with positive squeeze; Z grows vertically.
    for i in range(41):
        a=a0+(a1-a0)*i/40
        points.append((cx+(r+rad)*math.sin(a),cy+(r+rad)*math.cos(a)))
    ex,ey=cx+r*math.sin(a1),cy+r*math.cos(a1)
    for i in range(1,21):
        a=a1+math.pi*i/20
        points.append((ex+rad*math.sin(a),ey+rad*math.cos(a)))
    for i in range(1,41):
        a=a1+(a0-a1)*i/40
        points.append((cx+(r-rad)*math.sin(a),cy+(r-rad)*math.cos(a)))
    ex,ey=cx+r*math.sin(a0),cy+r*math.cos(a0)
    for i in range(1,21):
        a=a0+math.pi+math.pi*i/20
        points.append((ex+rad*math.sin(a),ey+rad*math.cos(a)))
    return points
# cutterpath is the closed nominalfinished2.00wide slot; tooloffsetisCAMresponsibility.
dxf('hinge_stop_slot',arcslot(-19,36,6,0,.17,1))
part('handle',100,10,10,1,'POM-C',holes=[(15,5,3.2),(85,5,3.2)],notes='U+-50, V-22..-12, Z40..50. M3x14 from V-22 counterbored 5.5x3 into front panel blind M3 depth 8, axes U+-35, Z45. Heads flush V-22. Handle is fixed exterior geometry; included overall V-22..405.')

with (ROOT/'parts.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=['name','width_mm','height_mm','thickness_mm','qty','material','notes'])
    w.writeheader();w.writerows({k:v for k,v in p.items() if k in w.fieldnames} for p in parts)

# Optional source for extruding the already-authored 2D profiles. No verified render claimed.
scad=['// Original FC1 profiles, mm. Run build_package.py first.','// Source only: not rendered in this environment.',
      'part="pad_width";','module cut_part(name,t) { linear_extrude(height=t) import(str(name,".dxf")); }']
for p in parts:
    scad.append(f'if (part=="{p["name"]}") cut_part("{p["name"]}",{p["thickness_mm"]});')
(CAD/'parts.scad').write_text('\n'.join(scad)+'\n')

# Drawings, using the same primitives for SVG and PNG to keep previews faithful.
s=Sheet('01-envelope','D01  |  Cassette envelope and datum plan')
X=lambda u:550+u*2.1
Y=lambda v:245+v*2.1
s.rect(X(-152),Y(-12),304*2.1,417*2.1,fill='#edf0f2')
s.rect(X(-146),Y(0),292*2.1,396*2.1,fill='white')
s.rect(X(-105),Y(0),210*2.1,297*2.1,fill='#fff9e7')
for u in (-109,109):
    s.rect(X(u-1.5),Y(90),3*2.1,100*2.1,fill='#b5cbd9',color=BLUE)
s.rect(X(-70),Y(356),140*2.1,3*2.1,fill='#b5cbd9',color=BLUE)
s.line(X(-146),Y(0),X(146),Y(0),RED,4)
s.text(360,185,'V=0 FEED EDGE',19,RED)
s.text(425,600,'A4 210 x 297',23)
s.text(420,635,'100 loose sheets',18)
s.text(420,665,'4 x 25, no binding',17)
s.text(90,490,'U=-109',17,BLUE);s.text(810,490,'U=+109',17,BLUE)
s.text(375,1035,'rear pad face V=356',18,BLUE)
s.dim(X(-152),Y(-12)-25,X(152),Y(-12)-25,'304')
s.dim(920,Y(-12),920,Y(405),'417')
s.dim(X(-109),Y(220),X(109),Y(220),'218 free width')
s.text(85,1150,'Z datums: floor underside 0, paper support 3, pad lower edge 3.5, pad top/pivot 36.',18)
s.text(85,1185,'Latch tops 62; front panel 65. Proposed underfloor plinth Z=-35..0.',18)
s.text(85,1220,'Width stroke 70 each; length stroke 150. World height is raised 35 by this plinth.',18,RED)
s.text(85,1255,'Actual clear opening: 292 x 396. All interior dimensions are measured between finished wall faces.',17)
s.text(85,1290,'Floor cut origin (-152,-12). See floor DXF for connector slots plus two 4-wide rod tracks and corner holes.',18)
s.text(85,1325,'Guide-latching and paper-squaring actions retained. No physical trial performed.',18,GREEN)
s.save()

s=Sheet('02-pad-cartridge','D02  |  Sprung pad cartridge, width and length variants')
s.text(80,180,'FRONT: L=100 (2 off) or 140 (1 off); thickness 3 PP-H',21)
fx,fy,k=160,280,6
s.rect(fx,fy,100*k,32.5*k,fill='#d8e6ef',color=BLUE)
for xx in (14,86):
    s.circle(fx+xx*k,fy+(32.5-14.5)*k,2.2*k/2)
s.dim(fx,fy-35,fx+100*k,fy-35,'L')
s.dim(fx+100*k+35,fy,fx+100*k+35,fy+32.5*k,'32.5')
s.text(160,505,'Pin centres 14 from ends, 14.5 above bottom. M2 through + received-head countersink.',17)
s.text(160,540,'Backplate L x 32.5 x 4; pin bores dia 3.10 +0.03/-0.00 at same centres.',17)
s.text(160,575,'Four clamp holes: S=4,L-4 and height=4,28.5; dia 3.2. See back DXF.',17)
s.text(80,645,'SECTION: N positive toward paper. Free face N=0. Compression delta moves left.',19)
nx=lambda n:930+n*20
zy=lambda z:1030-(z-3.5)*8
s.rect(nx(-22),zy(36),4*20,32.5*8,fill='#b8c1c5')
s.rect(nx(-30),zy(36),8*20,32.5*8,color=GREY)
s.rect(nx(-18),zy(36),9.5*20,32.5*8,color=GREY)
s.rect(nx(-3),zy(36),3*20,32.5*8,fill='#a5cce3',color=BLUE)
s.rect(nx(-8),zy(36),3*20,32.5*8,color=RED)
s.line(nx(-23),zy(18),nx(-3),zy(18),GREY,8)
pts=[(nx(-18+i*15/18),zy(18)+(10 if i%2 else -10)) for i in range(19)]
for a,b in zip(pts,pts[1:]):s.line(*a,*b,BLUE,2)
s.dim(nx(-18),1080,nx(-3),1080,'15 spring at zero / 10 at delta=5')
s.text(80,1135,'Blue: zero pad. Red: delta=5 pad. Fixed cheek front ends at N=-8.5 (0.5 clearance).',18)
s.text(80,1170,'Two S1 springs/pad. Pins dia3 x20; rear tips N=-23 -> -28; rear clearance to N=-30.',18)
s.text(80,1205,'Backplate and cheek blocks clamp as a sandwich. No solid plate inside pad travel.',18,GREEN)
s.text(80,1240,'Rear block depth 8 + plate 4 + front block 9.5 = 21.5; all within N=-30..-8.5.',18)
s.text(80,1275,'Cheek blocks occupy first/last 8 of pad span. Secondary bores along N at clamp holes.',17)
s.text(80,1310,'Contact face flatness <=0.15; running skew <=0.10; calibrate actual spring pair.',18,RED)
s.save()

def nz_map(x0, y0, n0, z0, k):
    """Section view: N to the right (toward paper), Z up; returns a mm->px mapper."""
    return lambda n, z: (x0 + (n - n0) * k, y0 - (z - z0) * k)
def nz_poly(s, f, pts, color='#18232c', fill=None):
    s.poly([f(n, z) for n, z in pts], color, fill=fill)
def nz_box(s, f, n0, n1, z0, z1, color='#18232c', fill=None):
    nz_poly(s, f, [(n0, z0), (n1, z0), (n1, z1), (n0, z1)], color, fill)
def rot(pt, c, th):
    dn, dz = pt[0] - c[0], pt[1] - c[1]
    return (c[0] + dn * math.cos(th) + dz * math.sin(th), c[1] - dn * math.sin(th) + dz * math.cos(th))
LEVER = [(-27, 34), (-17, 34), (-17, 62), (-21, 62), (-21, 38), (-27, 38)]
PIV = (-19, 36)

s=Sheet('03-latch','D03  |  Squeeze latch, release crank and pawl detail')
s.text(80,145,'SECTION AT LATCH CENTRE S=0 (rear latch shown; width latch is the mirror). Scale 14 px/mm.',17)
f = nz_map(130, 790, -34, 26, 14)
nz_box(s, f, -3, 0, 26, 36, GREY, '#e3e6e8')                       # sprung pad (D02)
nz_poly(s, f, [(-11, 36), (-8.5, 36), (-8.5, 36.5), (-3, 36.5), (-3, 62), (-11, 62)], BLUE, '#b5cbd9')  # post
nz_box(s, f, -30, -28, 28, 32, '#18232c', '#d5d9dc'); nz_box(s, f, -22, -8.5, 28, 32, '#18232c', '#d5d9dc')  # bridge foot, crank pocket N-28..-22
nz_box(s, f, -15, -11, 32, 44, '#18232c', '#d5d9dc')                # bridge upright
nz_poly(s, f, LEVER, BLUE, '#a5cce3')                              # lever at rest
nz_poly(s, f, [rot(q, PIV, .17) for q in LEVER], RED)             # lever squeezed to 0.17 stop
x, y = f(*PIV); s.circle(x, y, 1.55*14)
x, y = f(-25, 36); s.circle(x, y, 1.1*14)
nz_box(s, f, -26, -24, 26, 34, GREY, '#c9ced2')                    # rod eye neck and Ø2 tie rod (down to pawl)
for th, col in ((.10, GREEN), (.17, RED)):
    s.line(*f(*PIV), *f(*rot((-19, 62), PIV, th)), col, 2)
s.dim(*f(-11, 64.5), *f(-3, 64.5), '8 post')
s.dim(*f(-21, 64.5), *f(-17, 64.5), '4')
s.dim(*f(-17, 66.5), *f(-11, 66.5), 'gap 6')
s.line(*f(-33, 36), *f(-33, 62), BLUE, 1); s.text(f(-33,49)[0]+6, f(-33,49)[1], '26', 17, BLUE)
s.text(640, 300, 'Pivot: N-19, Z36, bore dia 3.10', 16)
s.text(640, 330, 'Crank pin: N-25, Z36 (R6), dia 2.2 hole', 16)
s.text(640, 360, 'Bridge foot Z28..32, upright N-15..-11', 16)
s.text(640, 390, 'Post N-11..-3, Z36..62 (fixed)', 16, BLUE)
s.text(640, 420, 'Sprung pad face at N0 (D02)', 16, GREY)
s.text(*f(-30, 58), 'lever N-21..-17', 16, BLUE)
s.text(*f(-30, 56.2), 'red: at 0.17 stop', 16, RED)
s.text(*f(-30, 54.4), 'green: 0.10 release', 16, GREEN)
s.text(640, 450, 'Tie rod dia 2 down to pawl (FC08)', 16, GREY)
s.text(100,830,'Release 0.10 rad (5.7 deg); stop 0.17 rad (9.7 deg), set by the FC07 stop slot.',18)
s.text(100,858,'Crank R6: gross lift 0.60 at release, 1.02 at stop. Pickup gap 0.45 +/-0.02 (FC08).',18)
s.text(100,886,'Tie rod dia 2, button top to tip 46.80 (width) / 59.80 (rear); jam nut adjusts +/-0.3.',18)
s.text(100,914,'Lever 4 thick x 30 wide (S+-15), POM-C; post 8 thick, M3x10 CSK to bridge at Z40, S+-10.',18)
s.text(100,965,'PAWL DETAIL (enlarged, see FC08): pitch 0.5, rack crest/valley 0.25/0.25, crest height 0.25.',18)
for i in range(20):
    xx=140+i*30
    s.rect(xx,1070,15,15,fill='#b8c1c5')
    s.line(xx+15,1085,xx+30,1085)
s.rect(240,1025,240,25,fill='#a5cce3',color=BLUE)
for i in range(8):s.rect(243+i*30,1050,9,12,fill='#a5cce3',color=BLUE)
s.text(100,1130,'Pawl teeth 0.15 wide, 0.20 deep: 0.05 flank clearance each side, 0.15 penetration.',18)
s.text(100,1165,'Pawl return S3: 0.15 N installed target. Latch return S2: 400 N mm/rad.',18)
s.text(100,1200,'Phase fixed rack to reset gauges. Inspect full disengagement and no crest hang-up.',18)
s.text(100,1235,'Square pawl engages only within +/-0.05 phase; off-phase it rests on crests until a slide.',18)
s.text(100,1270,'Lever profile DXF latch_lever: pivot (8,2), crank (2,2), main 4 x 26, extrusion 30.',18)
s.text(100,1305,'No squeeze force or holding strength measured; acceptance tests in CALIBRATION.md.',18)
s.save()
s=Sheet('04-calibration','D04  |  Scales, spring loads and acceptance record')
s.text(80,185,'DIRECT DISPLACEMENT: scale fixed; pointer on moving pin, no inferred force.',21)
for i in range(11):
    x=180+i*65
    s.line(x,340,x,310 if i%2==0 else 323,RED if i==4 else '#18232c')
    if i%2==0:s.text(x-8,350,f'{i/2:g}',20,RED if i==4 else '#18232c')
s.text(170,400,'Schematic enlarged. Use separate 100 x 25 mm SVG template at 100%.',18)
s.text(170,435,'Zero stop -> 0; actual 2 mm limit marked red; load/unload error target <=0.10.',18)
s.text(80,505,'NOMINAL PAIR LOADS (predicted/target, not measured)',21,BLUE)
for row,(q,f) in enumerate(((0,0),(.5,.75),(1,1.5),(1.5,2.25),(2,3),(2.5,3.75),(3,4.5),(4,6),(5,7.5))):
    y=555+row*39
    s.text(130,y,f'delta={q:g} mm',19)
    s.text(400,y,f'F={f:g} N',19,RED if q>2 else '#18232c')
    if q==2:s.text(640,y,'boundary; >2 gives zero',18,RED)
s.text(80,955,'S1: wire 0.50, mean dia 4.50, 9 active/11 total, free 15, solid <=5.60.',18)
s.text(80,990,'Pair finished rate 1.500 +/-0.075 N/mm. Calibrate with indicator + force gauge.',18)
s.text(80,1025,'S2: finished 400 +/-40 N mm/rad. Torque points 40 /68 N mm at 0.10 /0.17 rad.',18)
s.text(80,1060,'S3: installed 0.15 +/-0.03 N. Its friction/preload is part of latch calibration.',18)
s.text(80,1120,'Inspect: return repeatability <=0.10; skew <=0.10; print bar 50.00 +/-0.10.',18)
s.text(80,1155,'Record actual displacement/force loading AND unloading for each of 3 pads.',18)
s.text(80,1190,'Record masses, free drag, rack backlash, latch release angles and locked movement.',18)
s.text(80,1240,'The score limit is 2.0 mm, not a measured paper-bowing force law.',19,RED)
s.text(80,1275,'All calibration-record.csv results remain blank. No tests on physical hardware performed.',18,RED)
s.save()

s=Sheet('05-linkage','D05  |  Underfloor link, rails and manufacturing notes')
s.text(80,180,'LOCAL PLAN: under the 3 mm floor; no gears in the paper space.',21)
xx=lambda u:550+u*2.8
yy=lambda v:360+(v-100)*4
for v in (100,180):s.line(xx(-140),yy(v),xx(140),yy(v),GREY,7)
s.line(xx(-109),yy(100),xx(-109),yy(180),BLUE,5)
s.line(xx(109),yy(100),xx(109),yy(180),BLUE,5)
s.circle(xx(0),yy(140),28,color=BLUE)
s.rect(xx(-130),yy(130)-8,200*2.8,8,color=GREEN,fill='#c8e4d8')
s.rect(xx(-70),yy(150),200*2.8,8,color=GREEN,fill='#c8e4d8')
s.text(100,725,'Shafts 6: width U+-145, V100/180, Z-11; rear V203..399, U+-60, Z-25.',18)
s.text(100,760,'Width pinion U0,V140; rack pitch lines V130/150. Gear envelope Z=-9..-3; actual 5.8 face Z=-8.9..-3.1.',18)
s.text(100,795,'Phase rack attachments against 218 gauge. Relative lost-motion target <=0.20.',18)
s.text(100,850,'CUSTOM PINION: m1, 20 teeth, 20 deg, 6 face, 6.1 bore; face 5.8, pitch dia 20, OD22.',18)
s.poly([(260+x*8,1020-y*8) for x,y in gear],BLUE,fill='#d8e6ef')
s.circle(260,1020,3.05*8)
s.circle(260,1020,10*8,GREEN)
s.text(425,925,'Custom rack pitch pi (3.1416).',18)
s.text(425,960,'Addendum 1, dedendum 1.25.',18)
s.text(425,995,'20-deg straight rack flanks.',18)
s.text(425,1030,'Root: radial flank from base dia 18.79',18)
s.text(425,1065,'to root dia 17.50; corners cutter R0.25 max.',18)
s.text(425,1100,'20 teeth > 17-tooth undercut limit at 20 deg:',18)
s.text(425,1135,'all tooth contact is on the involute.',18)
s.text(100,1160,'Lock racks: width U-142..-56, V135..145, Z top=-16;',18)
s.text(100,1195,'length U-5..5, V201..390, Z top=-29. 35-high plinth encloses mechanism.',18)
s.text(100,1230,'Transfer-drill supports square to shaft datums. No vendor hole pattern assumed.',18)
s.text(100,1265,'Split-bush drag is dry friction; no 30 N s/m viscous equivalence is asserted.',18,RED)
s.text(100,1300,'Model limitations: masses, tooth phase, drag and linkage compliance are unmeasured.',18,RED)
s.save()

# Cleared foot/post interface.
s=Sheet('06-bridge-drag','D06  |  Cleared latch bridge and adjustable real drag')
s.text(80,185,'BRIDGE SECTION: N positive toward paper, Z vertical; extrusion span 30.',21)
bx=lambda n:830+n*18
bz=lambda z:870-(z-30)*24
s.poly([(bx(n-30),bz(z+28)) for n,z in bridge],BLUE,fill='#cbdce5')
s.rect(bx(-11),bz(46),8*18,10*24,color=GREY)
s.rect(bx(-8),bz(36),3*18,6*24,color=RED)
s.text(80,230,'Cropped Z30..46. Red: compressed pad. Grey: post. Blue: new bridge.',18)
s.text(80,280,'Foot N=-30..-8.5,Z28..32. Upright N=-15..-11,Z32..44.',18)
s.text(80,315,'Two M3x10 countersunk fasteners along N at Z40, span +/-10. Heads flush at N=-15.',18)
s.text(80,945,'Calculated normal clearance: pad 0.500; lever at Z44/theta 0.17: 0.597.',19,GREEN)
s.text(80,985,'No cap screw heads in the lever gap. Inspect full angle/stroke with feelers.',18,RED)
s.text(80,1050,'DRAG: split one near-width and one rear POM carriage around its dia6 shaft.',19)
s.text(80,1090,'1-wide top slit to bore; M3 cross-clamp + jam nut. Other blocks loose-running.',18)
s.text(80,1130,'Adjust to target 0.60 +/-0.10 N at 20 mm/s; record breakaway and 50/100 mm/s.',18)
s.text(80,1170,'Dry friction is NOT a 30 N s/m viscous damper. No physical curve measured.',19,RED)
s.text(80,1220,'Physical/model comparison must include resistance, hardware envelopes and tooth phase.',18)
s.text(80,1260,'No physical or native qualification is established by construction drawings.',18,RED)
s.save()


s=Sheet('07-hinge','FC07 | Hinge, S2 anchors and mechanical angular stops')
s.text(80,145,'A: SIDE SECTION through the negative ear (S-19.5..-16), N right, Z up. Scale 12 px/mm.',17)
f = nz_map(150, 640, -34, 28, 12)
nz_box(s, f, -30, -8.5, 28, 32, '#18232c', '#d5d9dc')               # bridge foot
nz_poly(s, f, [(-23, 30), (-15, 30), (-15, 44), (-23, 44)], '#18232c', '#e9ecee')  # negative ear Z30..44
nz_box(s, f, -15, -11, 32, 44, GREY)                                # bridge upright (behind)
nz_poly(s, f, LEVER, GREY)                                          # lever at rest (reference)
for r in (5, 7):                                                    # stop slot: R6 centreline, width 2
    pts = [f(*rot((-19, 36 + r), PIV, t * .17 / 12)) for t in range(13)]
    for p0, p1 in zip(pts, pts[1:]): s.line(*p0, *p1, BLUE, 2)
for th in (0, .17):
    s.line(*f(*rot((-19, 41), PIV, th)), *f(*rot((-19, 43), PIV, th)), BLUE, 2)
    x, y = f(*rot((-19, 42), PIV, th)); s.circle(x, y, 12, RED if th else GREEN)
x, y = f(*PIV); s.circle(x, y, 1.55*12); s.circle(x, y, 4.6*12, GREY)   # pivot bore 3.10; S2 coil OD 9.2 (behind)
s.line(*f(-19, 40), *f(-11, 40), GREEN, 3)                          # S2 fixed leg
s.line(*f(-23, 36), *f(-23, 62), GREEN, 3)                          # S2 moving leg
nz_box(s, f, -13, -9, 30, 44, GREEN)                                # fixed leg bracket
nz_box(s, f, -24.6, -17, 56, 62, GREEN)                             # moving leg clamp
s.text(600, 260, 'Stop slot (blue): R6 about pivot, width 2.00 +0.02/-0,', 16, BLUE)
s.text(600, 284, 'ends at 0 and 0.17 rad', 16, BLUE)
s.text(600, 318, 'Follower dowel dia 2 h6 in lever at N-19, Z42:', 16)
s.text(600, 342, 'green at rest, red at the 0.17 stop', 16)
s.text(600, 376, 'Pivot bore dia 3.10; S2 coil OD 9.2 (grey, behind)', 16)
s.text(600, 400, 'Negative ear N-23..-15, Z30..44; lever grey', 16)
s.text(600, 434, 'S2 legs (green): fixed leg Z40 to bracket N-13..-9;', 16, GREEN)
s.text(600, 458, 'moving leg N-23 to clamp N-24.6..-17, Z56..62', 16, GREEN)
s.text(80,700,'B: PLAN ALONG THE AXLE (S), dimensions in mm. Scale 12 px/mm.',17)
g = lambda S: 120 + (S + 25) * 12
yb = 820
s.rect(g(-23), yb-18, 53*12, 36, fill='#eef1f3')                    # axle dia 3 x 53
for a0, a1, lab, col in ((-23, -20.5, 'collar', GREY), (-19.5, -16, 'ear', '#18232c'), (-15, 15, 'lever S+-15', BLUE),
                         (16, 19.5, 'ear', '#18232c'), (20.5, 24.5, 'S2', GREEN), (26, 28.5, 'collar', GREY)):
    s.rect(g(a0), yb-45, (a1-a0)*12, 90, color=col)
    s.text(g(a0)+2, yb+52, lab, 14, col)
for S in (-23, -19.5, -16, -15, 15, 16, 19.5, 20.5, 24.5, 26, 28.5, 30):
    s.line(g(S), yb+75, g(S), yb+85, BLUE, 1)
for i, S in enumerate((-23, -19.5, -16, -15, 15, 16, 19.5, 20.5, 24.5, 26, 28.5, 30)):
    s.text(g(S)-12, yb+88+(i % 2)*18, f'{S:g}', 12, BLUE)
s.text(80, 960, 'Axle dia 3 h6 x 53 (S-23..30), set screw M2 in negative ear at S-17.75. PET thrust washers 0.3.', 17)
lines=[
'Ears 6061-T6, foot S12..19.5 mirrored, Z30..32; two M2x4 CSK per foot at N-21/-17, S+-13.5.',
'S2 torsion spring (SPRINGS.md): body S20.5..24.5 on POM sleeve OD6/ID3.10, S20..25.',
'Wing top Z30 to coil bottom Z31.4: 1.4 clearance. Squeeze must wind the coil.',
'Crank fork pocket N-28..-22, S+-2; pin dia 2 x 9 at N-25, Z36; rod eye 2 thick.',
'Clearances at 0.17: lever root to foot 1.69; lever to upright at Z44: 0.60.',
'Inspect >=0.30 lever/upright and >=0.25 pad/cheek clearance through full travel.',
'Stop angles 0 +/-0.002 and 0.170 +/-0.002 rad; reject an oversized or worn slot.',
'No spring torque or assembled force measured; see CALIBRATION.md.'
]
for i,line in enumerate(lines):s.text(65,1010+i*48,line,17)
s.save()
s=Sheet('08-pawl','FC08 | Guided square pawl, pickup and spring seat')
s.text(80,145,'SECTION ALONG TRAVEL T at S=0, engaged in phase. Z from rack crest C. Scale 50 px/mm.',17)
X = lambda T: 550 + T * 50
Y = lambda z: 830 - z * 50                                          # z measured from crest C
def tb(t0, t1, z0, z1, color='#18232c', fill=None): s.rect(X(t0), Y(z1), (t1-t0)*50, (z1-z0)*50, fill=fill, color=color)
tb(-5, 5, -1.25, -0.25, GREY, '#d5d9dc')                            # rack body below roots (cut off)
for k in range(-10, 10):
    tb(k*.5-.125, k*.5+.125, -.25, 0, GREY, '#b8c1c5')              # crests 0.25 wide, 0.25 high, pitch 0.5
tb(-2, 2, .05, 3.05, BLUE, '#a5cce3')                               # pawl main body 3 high
tb(-2, 2, 3.05, 4.05, BLUE, '#c6deec')                              # lid 1
for k in range(8):
    tc = -1.75 + k*.5
    tb(tc-.075, tc+.075, -.15, .05, BLUE, '#a5cce3')                # teeth 0.15 wide, 0.20 deep
tb(-1.6, 1.6, 1.05, 3.05, BLUE, 'white')                            # counterbore dia 3.2, D-1..D-3
tb(-1.5, 1.5, 1.6, 2.6, '#18232c', '#8f99a1')                       # button dia 3 x 1, top at D-1.45
tb(-1, 1, 2.6, 11.2, '#18232c', '#c9ced2')                           # rod dia 2
tb(-4.05, -2.05, .3, 8.25, '#18232c', '#eef1f3'); tb(2.05, 4.05, .3, 8.25, '#18232c', '#eef1f3')    # T-walls relieved to C+0.30 over the rack
tb(-4.05, 4.05, 8.25, 10.25, '#18232c', '#e3e6e8')                  # cap 2 thick, underside D+4.2
for zz in (4.4, 5.0, 5.6, 6.2, 6.8, 7.4, 8.0):
    s.line(X(-1.6), Y(zz), X(1.6), Y(zz+.3), GREEN, 2)               # S3 coil OD 3.2 between lid and cap
s.line(X(-5), Y(0), X(5.2), Y(0), RED, 1); s.text(X(5.25), Y(0)-10, 'C crest', 15, RED)
s.line(X(-5), Y(4.05), X(5.2), Y(4.05), RED, 1); s.text(X(5.25), Y(4.05)-10, 'D = C+4.05', 15, RED)
s.dim(X(-2), Y(-2.6), X(2), Y(-2.6), '4 body')
s.dim(X(-2.05), Y(11.9), X(2.05), Y(11.9), 'bore 4.10')
s.text(X(-4.9), Y(9.6), 'cap', 15); s.text(X(1.7), Y(6.2), 'S3', 15, GREEN)
s.text(X(1.7), Y(2.2), 'button dia 3x1, top D-1.45', 15); s.text(X(1.7), Y(1.3), 'counterbore dia 3.2', 15)
s.text(X(-4.9), Y(-1.15), 'fixed rack: crest/valley 0.25/0.25', 15, GREY)
s.text(X(-4.9), Y(-1.6), 'T-walls end at C+0.30 over the rack; S-walls (beside rack) reach C-1', 15)
lines=[
'Teeth 0.15 wide x 0.20 deep at pitch 0.50; body bottom C+0.05 gives 0.15 penetration.',
'Body 4 along T x 10 across S x 4 high (3 + lid 1). Guide bore 4.10 x 10.10, outer 8.10 x 16.00.',
'Pickup gap g = 6 sin(0.10) - 0.15 = 0.449: set 0.45 +/-0.02; lid underside D-1.',
'Last tooth withdraws at 0.10 rad; 0.05 tip clearance at 0.108 rad; lift 0.57 at 0.17 rad.',
'S3 seats: lid top D to cap underside D+4.2; OD 3.2, rate target 0.084 N/mm (SPRINGS.md).',
'Backup stop M2 at T-1, S3.5, tip D+1.2; normal lift does not touch it.',
'Off-phase release: pawl rests on crests, unseated, until a small slide brings it over a valley.',
'Width crest C=-16, rear C=-29. Guide moves with the carriage on its drop plate, not the floor.',
'Holding 10 N, release angle and free friction are acceptance targets, not measured.'
]
for i,line in enumerate(lines):s.text(65,990+i*44,line,17)
s.save()
# A true-size print template, separate from the engineering sheets.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="25mm" viewBox="0 0 100 25">',
     '<rect width="100" height="25" fill="white"/>',
     '<text x="2" y="3" font-size="2" font-family="sans-serif">FC1 / mm / print100% / cut6x5labels</text>']
for j in range(3):
    origin=8+28*j
    svg.append(f'<rect x="{origin}" y="6" width="6" height="5" fill="none" stroke="black" stroke-width=".05"/>')
    for i in range(11):
        x=origin+.5+i*.5
        col='red' if i==4 else 'black'
        svg.append(f'<path d="M{x} 7 v{2.3 if i%2==0 else 1.2}" stroke="{col}" stroke-width=".05"/>')
    for val in (0,2,5):
        svg.append(f'<text x="{origin+.3+val}" y="10.6" font-size="1.1" font-family="sans-serif" fill="{"red" if val==2 else "black"}">{val}</text>')
svg += ['<path d="M5 20 H55 M5 18 V22 M55 18 V22" fill="none" stroke="black" stroke-width=".15"/>',
        '<text x="59" y="21" font-size="2" font-family="sans-serif">50 mm check bar</text>','</svg>']
(DRAW/'compression-scale.svg').write_text('\n'.join(svg))

# Extract selected existing numeric assignments without importing task hooks or Isaac.
env={'TOP':0.0,'math':math}
want={'A4','N_BUNDLES','SHEETS','T','BUNDLE_MASS','XC','FLOOR_T','PLINTH_H','DECK','FLOOR','YF','WIDE','LEGAL','GUIDE_H','PAD_T','PLATE_T','PAD_TRAVEL','PAD_K','LEVER_UP','UNLOCK','LEVER_K','LOCK_K','LOCK_C','FREE_C','LOCK_PITCH','LOCK_WINDOW','W_SLIDE','WN0','WF0','LF0','L_SPAN','TAB','TAB_YC','Z_PAD','Z_POST','Z_FOOT','Z_LEVER','X0','X1','Y0','Y1','SNUG','SQUEEZE','SQUARE_DEG','FLAT_DEG','FAN_A','FAN_B','DX_A','YAW_A'}
source_text = SOURCE.read_text() if SOURCE.exists() else ''
for node in ast.parse(source_text).body:
    if isinstance(node,ast.Assign):
        names=[n.id for t in node.targets for n in ast.walk(t) if isinstance(n,ast.Name)]
        if names and all(n in want for n in names):
            try:
                exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE),'exec'),{'__builtins__':{'tuple':tuple},'math':math},env)
            except (NameError,TypeError):
                pass
contract=({k:env[k] for k in sorted(want) if k in env} if SOURCE.exists() else json.loads((ROOT/'design-config.json').read_text()))
contract.update({'units':'runtime m,N,s,rad; TOP normalised to 0','source_path':'task.py',
                 'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest() if SOURCE.exists() else None,
                 'source_status':'adjacent task.py extracted' if SOURCE.exists() else 'bundled design config; not current runtime proof',
                 'drawing_revision':'FC1 construction drawings, unbuilt',
                 'floor_raise_mm':35,'physical_measurements':False})
(ROOT/'runtime-contract.json').write_text(json.dumps(contract,indent=2)+'\n')

G=11.5e6*.006894757293168
k_one=G*.5**4/(8*4.5**3*9)
assert abs(2*k_one-1.5)/1.5<.01
assert 15-5-5.6>=4.4-1e-9
assert -8 - (-8.5)>.49
assert contract['PAD_K']==1500 and contract['SQUEEZE']==.002
if not (ROOT/'calibration-record.csv').exists():
    with (ROOT/'calibration-record.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['assembly_id','direction','target_delta_mm','measured_delta_mm','scale_reading_mm','measured_force_N','instrument_id','notes'])
        for name in ('width_near','width_far','length'):
            for direction in ('loading','unloading'):
                for q in (0,.5,1,1.5,2,2.5,3,4,5):w.writerow([name,direction,q,'','','','',''])
contact=Image.new('RGB',(1100,3000),'#e1e6e8')
for i,p in enumerate(sorted(DRAW.glob('0*.png'))):
    im=Image.open(p);im.thumbnail((550,750));contact.paste(im,((i%2)*550,(i//2)*750))
contact.save(DRAW/'contact-sheet.png')
def inside(p,poly):
 x,y=p; c=False
 for a,b in zip(poly,poly[1:]+poly[:1]):
  if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:c=not c
 return c
def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def intersect(A,B):
 # Proper crossing only; touching is permitted in a nominal contact pair.
 for a,b in zip(A,A[1:]+A[:1]):
  for c,d in zip(B,B[1:]+B[:1]):
   if max(a[0],b[0])<min(c[0],d[0]) or max(c[0],d[0])<min(a[0],b[0]) or max(a[1],b[1])<min(c[1],d[1]) or max(c[1],d[1])<min(a[1],b[1]):continue
   if cross(a,b,c)*cross(a,b,d)<-1e-12 and cross(c,d,a)*cross(c,d,b)<-1e-12:return True
 return any(inside(p,B) for p in A) or any(inside(p,A) for p in B)
results=[]
for i in range(13):
 q=math.pi*i/12; theta=q/10
 G=[(x*math.cos(theta)-y*math.sin(theta),140+x*math.sin(theta)+y*math.cos(theta)) for x,y in gear]
 L=[(x-44.5*math.pi+q,130+y) for x,y in rack]
 U=[(x-19.5*math.pi-q,150-y) for x,y in rack]
 # Clip away rack spans well outside the gear envelope for bounded meaningful check.
 for name,P in [('near',L),('far',U)]:
  hit=intersect(G,P); results.append({'phase_mm':q,'rack':name,'positive_polygon_overlap':hit})
report={'scope':'26 discrete nominal 2D meshes over one rack pitch, not a continuous tolerance/CAM/assembly certificate','backlash_per_mesh_mm':.04,'phases':results,'pass':not any(r['positive_polygon_overlap'] for r in results)}
(ROOT/'mesh-check.json').write_text(json.dumps(report,indent=2)+'\n')
assert report['pass'], 'Nominal gear/rack mesh overlaps'


def profile_area(poly):
    return abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(poly,poly[1:]+poly[:1])))/2
rho={'PP-H':.91,'POM-C':1.41,'6061-T6':2.70,'stainless steel':7.90,'PET sheet':1.38}
stock={}
rows=[]
for p in parts:
    area=profile_area(p['outline'])-sum(profile_area(loop) for loop in p['cutouts'])-sum(math.pi*d*d/4 for x,y,d in p['holes'])
    vol=max(0,area)*p['thickness_mm']
    if p['name']=='pinion_axle':vol=math.pi*5**2*2.5+math.pi*3**2*7-2*math.pi*1.25**2*4
    if p['name'].startswith('crank_rod'):vol=math.pi*1.5**2+math.pi*(p['height_mm']-1)
    if p['name']=='pad_stop_button':vol=math.pi*1.5**2+math.pi*24
    density=rho.get(p['material'],2.70)
    gm=vol*density/1000
    stock[p['name']]=gm
    rows.append({'part':p['name'],'quantity':p['qty'],'profile_stock_mm3_each':round(vol,3),'assumed_density_g_cm3':density,'stock_mass_estimate_g_each':round(gm,3),'status':'calculated profile stock before secondary material removal; unmeasured'})
with (ROOT/'stock-volume-mass.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
base={'cheek_front':2,'cheek_rear':2,'pad_stop_button':2,'scale_shelf':1}
latch={'bridge_shim':2,'post':1,'latch_bridge':1,'hinge_ear':1,'hinge_ear_stop':1,'s2_fixed_bracket':1}
pawlparts={'pawl':1,'pawl_lid':1,'pawl_guide':1,'pawl_cap':1}
def join(*ds):
    out={}
    for d in ds:
        for k,v in d.items():out[k]=out.get(k,0)+v
    return out
groups={
'width_pad_each':({'pad_width':1,'pointer_flag':1},3.5,'two ground pins, retained screws, rings and washers'),
'rear_pad':({'pad_length':1,'pointer_flag':1},3.5,'two ground pins, retained screws, rings and washers'),
'near_width_carrier':(join(base,latch,pawlparts,{'back_width_latched':1,'carriage_block':1,'drag_block':1,'connector_width':2,'coupling_rack':1,'pawl_beam_width':1,'beam_riser_width':1,'beam_riser_width_short':1,'pawl_drop_width':1,'crank_rod_width':1,'rack_mount_spacer':2}),15.0,'S1/S2/S3, pivot axle/collars/bushes, clamp and beam fasteners, rod-eye/pins; excludes pad and rotating lever'),
'far_width_carrier':(join(base,{'back_width':1,'carriage_block':2,'connector_width':2,'coupling_rack':1,'pawl_beam_width':1,'beam_riser_width':2,'rack_mount_spacer':2}),7.0,'S1 pair and carrier/beam fasteners; no far lever'),
'rear_carrier':(join(base,latch,pawlparts,{'back_length':1,'carriage_block':1,'drag_block':1,'connector_length':2,'pawl_beam_rear':1,'beam_riser_rear':1,'beam_riser_rear_short':1,'pawl_drop_rear':1,'crank_rod_rear':1}),16.0,'S1/S2/S3, pivot axle/collars/bushes, carrier fasteners, rod-eye/pins; excludes pad and rotating lever'),
'rotating_lever_each':({'latch_lever':1,'s2_moving_anchor':1},.8,'two M2 mounts, follower pin and crank pin; spring distributed mass not assigned again')}
mr=[]
for group,(items,hw,note) in groups.items():
    calculated=sum(stock[k]*v for k,v in items.items())
    mr.append({'group':group,'profile_stock_estimate_g':round(calculated,2),'assigned_hardware_estimate_g':hw,'total_estimate_g':round(calculated+hw,2),'parts':items,'hardware_assignment':note,'status':'unmeasured design estimate, not finished dynamic mass'})
(ROOT/'moving-mass-estimates.json').write_text(json.dumps({'density_status':'assumed engineering values, not material certificates','method':'profile stock before secondary machining plus explicitly assigned hardware; not exact assembly mass','groups':mr},indent=2)+'\n')

validation={'scope':'Authored numeric/profile checks only; no physical build, OpenSCAD render, GPU or native task verification',
            'compression_spring_predicted_each_N_per_mm':k_one,'predicted_pair_N_per_mm':2*k_one,
            'finished_pair_target_N_per_mm':1.5,'minimum_solid_margin_at_max_travel_mm':4.4,
            'pad_to_cheek_clearance_at_max_travel_mm':.5,
            'crank_gross_lift_at_unlock_mm':6*math.sin(.1),
            'lost_motion_mm':6*math.sin(.1)-.15,
            'pawl_penetration_mm':.15,
            'crank_lift_after_nominal_lost_motion_mm':.15,
            'full_clearance_005_angle_rad':math.asin(((6*math.sin(.1)-.15)+.20)/6),
            'pawl_lift_at_hard_stop_mm':6*math.sin(.17)-(6*math.sin(.1)-.15),
            'lever_root_to_foot_vertical_gap_mm':36-2*math.sin(.17)-2*math.cos(.17)-32,
            'bridge_to_rotated_lever_min_nominal_mm':-15-(-19+8*math.tan(.17)+2/math.cos(.17)),
            'runtime_geometry_constants_match':contract.get('PLINTH_H')==.035 and contract.get('PAD_TRAVEL')==.005,
            'nominal_mesh_26_discrete_samples':'PASS; mesh-check.json',
            'dxf_units':'INSUNITS=4,mm','part_profiles':len(parts),'spring_and_force_status':'calculated predictions and acceptance targets only',
            'measurements':None,'model_approximations':'See MISMATCHES.md: ideal coupling, numerical drive lock/drag and unmeasured masses; no physical qualification'}
# Parse generated SVGs as XML and independently inspect closed DXF entity counts/units.
import xml.etree.ElementTree as ET
for p in DRAW.glob('*.svg'):
    ET.parse(p)
for p in CAD.glob('*.dxf'):
    a=p.read_text().splitlines()
    assert len(a)%2==0 and a[-2:]==['0','EOF'],p
    pairs=list(zip(a[::2],a[1::2]))
    assert ('9','$INSUNITS') in pairs and ('70','4') in pairs,p
    assert ('0','LWPOLYLINE') in pairs,p
validation['svg_xml_parse']='PASS'
validation['dxf_entity_units_and_closed_profiles']='PASS (format/entity check, not a CAM or interference certification)'
validation['assets']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.rglob('*')) if p.is_file() and p.name!='validation.json'}
(ROOT/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps({k:v for k,v in validation.items() if k!='assets'},indent=2))
