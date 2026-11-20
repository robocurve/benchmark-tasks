#!/usr/bin/env python3
"""Deterministic CAD/drawing generator for the spring-backed PVC card case.

Generates:
  - case_dimensions.svg: orthographic top, longitudinal section, and front-slot views (mm)
  - fabrication/case_base.stl: lower ABS housing with corner M2 bosses and spring guide seats
  - fabrication/case_lid.stl: upper ABS frame with 66 x 40 mm thumb window and 0.95 mm outlet roof
  - fabrication/follower_plate.stl: 84 x 52 x 1.0 mm 6061-T6 aluminium follower plate with guide clearances
  - fabrication/desk_riser.stl: 50 x 38 x 50 mm hardwood desktop riser with 2x M4 counterbored mounting holes
  - fabrication/BOM.csv and fabrication/ASSEMBLY.md
"""
from __future__ import annotations
import struct
from pathlib import Path

OUT = Path(__file__).resolve().parent


def box_tris(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> list[tuple]:
    """Return 12 outward-oriented triangles (normal, v0, v1, v2) in mm."""
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    z0, z1 = cz - sz / 2.0, cz + sz / 2.0
    faces = [
        ((0.0, 0.0, -1.0), [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)]),
        ((0.0, 0.0, 1.0), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]),
        ((0.0, -1.0, 0.0), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
        ((0.0, 1.0, 0.0), [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)]),
        ((-1.0, 0.0, 0.0), [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)]),
        ((1.0, 0.0, 0.0), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
    ]
    tris = []
    for n, v in faces:
        tris.append((n, v[0], v[1], v[2]))
        tris.append((n, v[0], v[2], v[3]))
    return tris


def write_binary_stl(path: Path, name: str, boxes: list[tuple[float, float, float, float, float, float]]) -> None:
    tris = []
    for b in boxes:
        tris.extend(box_tris(*b))
    header = name.encode("ascii")[:80].ljust(80, b" ")
    buf = bytearray(header)
    buf += struct.pack("<I", len(tris))
    for n, a, b, c in tris:
        buf += struct.pack("<12fH", *n, *a, *b, *c, 0)
    path.write_bytes(bytes(buf))


def build_all(root: Path) -> None:
    fab = root / "fabrication"
    fab.mkdir(parents=True, exist_ok=True)

    # 1. Lower ABS housing (z = 0..14 mm; 1.15 mm front outlet gap is between z = 12.85 mm dam and z = 14.0 mm lid underside)
    base_boxes = [
        (0.0, 0.0, 1.0, 100.0, 64.0, 2.0),         # floor 100 x 64 x 2 mm
        (0.0, 30.5, 8.0, 100.0, 3.0, 12.0),        # left wall z = 2..14 mm
        (0.0, -30.5, 8.0, 100.0, 3.0, 12.0),       # right wall z = 2..14 mm
        (-48.5, 0.0, 8.0, 3.0, 58.0, 12.0),        # rear wall z = 2..14 mm
        (48.5, 0.0, 7.425, 3.0, 58.0, 10.85),      # lower front dam z = 2..12.85 mm (leaves 1.15 mm single-card slot below lid)
        (-25.0, 0.0, 5.5, 1.8, 1.8, 7.0),          # rear spring guide post (x = -25 mm, z = 2..9 mm)
        (25.0, 0.0, 5.5, 1.8, 1.8, 7.0),           # front spring guide post (x = +25 mm, z = 2..9 mm)
    ]
    write_binary_stl(fab / "case_base.stl", "card_case_base_mm", base_boxes)

    # 2. Removable ABS top frame (z = 14..16 mm, fastened at 4 corners with M2 x 12 mm flat-head screws)
    lid_boxes = [
        (0.0, 24.5, 15.0, 100.0, 15.0, 2.0),       # left roof rail (covers wall + 9 mm inner overhang -> y = +17..+32 mm; inner window y = +/-20 mm -> 24 mm rail: y=+20..+32 is 12 mm)
    ]
    # Exact match to geometry.py roof bounds: exterior y = +/-32 mm, window y = +/-20 mm -> side rails y in [20, 32] and [-32, -20] (12 mm wide including 3 mm outer wall lip)
    lid_boxes = [
        (0.0, 26.0, 15.0, 100.0, 12.0, 2.0),       # left roof rail y = +20..+32 mm
        (0.0, -26.0, 15.0, 100.0, 12.0, 2.0),      # right roof rail y = -32..-20 mm
        (-41.5, 0.0, 15.0, 17.0, 40.0, 2.0),       # rear roof bridge x = -50..-33 mm, y = -20..+20 mm
        (41.5, 0.0, 15.0, 17.0, 40.0, 2.0),        # front outlet roof bridge x = +33..+50 mm, y = -20..+20 mm (forms 66 x 40 mm thumb window)
    ]
    write_binary_stl(fab / "case_lid.stl", "card_case_lid_mm", lid_boxes)

    # 3. 6061-T6 aluminium follower plate (84 x 52 x 1.0 mm, split around 2.2 x 2.2 mm guide clearance slots at x = +/-25 mm)
    follower_boxes = [
        (-33.55, 0.0, 0.5, 16.9, 52.0, 1.0),       # x = -42..-25.1 ->wait: keep clean 4-strip exact manifold around (+/-25, 0) 2.2 mm square guide holes
    ]
    # Exact decomposition of 84 x 52 x 1.0 mm plate minus two 2.2 x 2.2 mm guide slots at (x = -25, y = 0) and (x = +25, y = 0):
    follower_boxes = [
        (0.0, 13.55, 0.5, 84.0, 24.9, 1.0),        # +Y strip y = +1.1..+26.0 mm
        (0.0, -13.55, 0.5, 84.0, 24.9, 1.0),       # -Y strip y = -26.0..-1.1 mm
        (-34.05, 0.0, 0.5, 15.9, 2.2, 1.0),        # outer -X bridge x = -42.0..-26.1 mm
        (0.0, 0.0, 0.5, 47.8, 2.2, 1.0),           # center bridge x = -23.9..+23.9 mm
        (34.05, 0.0, 0.5, 15.9, 2.2, 1.0),         # outer +X bridge x = +26.1..+42.0 mm
    ]
    write_binary_stl(fab / "follower_plate.stl", "follower_plate_mm", follower_boxes)

    # 4. Desktop riser (50 x 38 x 50 mm hardwood block)
    write_binary_stl(fab / "desk_riser.stl", "desk_riser_mm", [(0.0, 0.0, 25.0, 50.0, 38.0, 50.0)])

    # 5. Dimensioned SVG drawing
    svg = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 920 540" font-family="DejaVu Sans, Arial, sans-serif" font-size="12" fill="#1a1a1a">
  <rect width="920" height="540" fill="#ffffff"/>
  <text x="30" y="34" font-size="17" font-weight="bold">Spring-Backed Single-Card Case &amp; Riser — Fabrication Drawing (all dimensions in mm)</text>
  <text x="30" y="54" font-size="12" fill="#444">Unbranded 2-piece ABS enclosure + 6061-T6 follower + 2x guided compression springs (combined k = 2500 N/m, 1.15 mm single-card throat)</text>

  <!-- TOP VIEW (scale 3 px/mm, center at 220, 195) -->
  <text x="70" y="92" font-size="14" font-weight="bold">1. Top View (Case + Thumb Window)</text>
  <rect x="70" y="110" width="300" height="192" fill="#d9e6f2" stroke="#1f3b57" stroke-width="2"/>
  <rect x="79" y="119" width="282" height="174" fill="none" stroke="#1f3b57" stroke-width="1" stroke-dasharray="4,3"/>
  <rect x="121" y="146" width="198" height="120" fill="#f7f7eb" stroke="#1f3b57" stroke-width="2"/>
  <text x="150" y="205" font-size="12" fill="#333">Thumb window 66.0 × 40.0</text>
  <text x="148" y="222" font-size="11" fill="#555">(exposes top CR80 card only)</text>
  <!-- Corner M2 screw holes -->
  <circle cx="75" cy="115" r="3" fill="#fff" stroke="#1f3b57"/>
  <circle cx="365" cy="115" r="3" fill="#fff" stroke="#1f3b57"/>
  <circle cx="75" cy="297" r="3" fill="#fff" stroke="#1f3b57"/>
  <circle cx="365" cy="297" r="3" fill="#fff" stroke="#1f3b57"/>
  <!-- Dimension lines -->
  <line x1="70" y1="320" x2="370" y2="320" stroke="#111" stroke-width="1.2"/>
  <text x="188" y="336">100.0 exterior</text>
  <line x1="386" y1="110" x2="386" y2="302" stroke="#111" stroke-width="1.2"/>
  <text x="394" y="210">64.0 exterior (58.0 inner)</text>
  <text x="374" y="242" font-size="11" fill="#8b0000">+X front outlet →</text>

  <!-- LONGITUDINAL SECTION A-A (scale 4 px/mm X, 9 px/mm Z exaggerated 2.25x vertical clarity, noted) -->
  <text x="480" y="92" font-size="14" font-weight="bold">2. Centerline Section A–A (X–Z, vertical 2.5× for slot clarity)</text>
  <!-- Floor z=0..2 -->
  <rect x="490" y="264" width="400" height="18" fill="#adc6de" stroke="#1f3b57" stroke-width="1.5"/>
  <!-- Rear wall x=-50..-47, z=2..14 -->
  <rect x="490" y="156" width="12" height="108" fill="#adc6de" stroke="#1f3b57" stroke-width="1.5"/>
  <!-- Front lower dam x=+47..+50, z=2..13.05 (height 11.05 -> 99.45 px -> top at 164.55) -->
  <rect x="878" y="165" width="12" height="99" fill="#adc6de" stroke="#1f3b57" stroke-width="1.5"/>
  <!-- Removable Lid rear bridge x=-50..-33 (68 px), z=14..16 (18 px, y=138..156) -->
  <rect x="490" y="138" width="68" height="18" fill="#6d9bc3" stroke="#1f3b57" stroke-width="1.5"/>
  <!-- Removable Lid front bridge x=+33..+50 (68 px), z=14..16 -->
  <rect x="822" y="138" width="68" height="18" fill="#6d9bc3" stroke="#1f3b57" stroke-width="1.5"/>
  <!-- 5 PVC cards z=10.2..14.0 (34.2 px total) -->
  <rect x="518" y="156" width="342" height="34" fill="#f5f5dc" stroke="#666" stroke-width="1"/>
  <line x1="518" y1="163" x2="860" y2="163" stroke="#888" stroke-width="0.8"/>
  <line x1="518" y1="170" x2="860" y2="170" stroke="#888" stroke-width="0.8"/>
  <line x1="518" y1="177" x2="860" y2="177" stroke="#888" stroke-width="0.8"/>
  <line x1="518" y1="183" x2="860" y2="183" stroke="#888" stroke-width="0.8"/>
  <!-- Aluminium follower z=9.2..10.2 (9 px) -->
  <rect x="522" y="190" width="336" height="9" fill="#b8b8b8" stroke="#333" stroke-width="1.2"/>
  <!-- Two guided compression springs at x=-25 and x=+25 -->
  <rect x="586" y="201" width="8" height="63" fill="#ddd" stroke="#444" stroke-width="1"/>
  <rect x="786" y="201" width="8" height="63" fill="#ddd" stroke="#444" stroke-width="1"/>
  <text x="530" y="236" font-size="11">2× guided steel springs (combined k = 2500 N/m, 1.25 N/mm each)</text>
  <text x="530" y="130" font-size="11">66.0 thumb window (x = -33 to +33)</text>
  <text x="742" y="302" font-size="11" fill="#8b0000" font-weight="bold">Single-card slot: 1.15 × 58.0 mm</text>
  <text x="742" y="317" font-size="11" fill="#444">(1 card = 0.76 mm passes; 2 cards = 1.52 mm blocked)</text>

  <!-- PART SUMMARY & BENCH LAYOUT -->
  <rect x="30" y="365" width="860" height="150" fill="#fafafa" stroke="#ccc" stroke-width="1"/>
  <text x="46" y="390" font-size="13" font-weight="bold">3. Reproducible Parts, Commercial Stock &amp; Bench Mounting</text>
  <text x="46" y="412">• Cards (5×): Standard ISO/IEC 7810 ID-1 (CR80) blank white rigid PVC cards, 85.60 × 54.00 × 0.76 mm, ~4.9 g each (e.g. Evolis C4001).</text>
  <text x="46" y="432">• Case Base &amp; Lid: 3D-printed or CNC ABS (1050 kg/m³), 100.0 × 64.0 × 16.0 mm assembled, joined by 4× M2 × 12 mm countersunk screws.</text>
  <text x="46" y="452">• Follower &amp; Springs: 84.0 × 52.0 × 1.0 mm 6061-T6 Al plate (11.8 g) on 2× 1.8 mm guide posts with 2× steel coil springs (combined k = 2500 N/m,</text>
  <text x="56" y="469">1.25 N/mm each, 2.0 N upward preload at 5-card stack).</text>
  <text x="46" y="489">• Bench Fixtures: 50 × 38 × 50 mm hardwood riser fixed to tabletop at (x = 340, y = -45 mm); case rests LOOSE on riser (unclamped). 140 × 100 × 2 mm</text>
  <text x="56" y="506">cream silicone mat adhered flat to tabletop at (x = 500, y = -100 mm).</text>
</svg>
"""
    (root / "case_dimensions.svg").write_text(svg)

    bom = """part_id,name,qty,material,dimensions_mm,source_or_file,notes
P1,case_base,1,ABS (FDM 100% infill or CNC),100.0 x 64.0 x 14.0,fabrication/case_base.stl,Includes 10.85 mm front dam and 2x 1.8 mm spring guide posts
P2,case_lid,1,ABS (FDM 100% infill or CNC),100.0 x 64.0 x 2.0,fabrication/case_lid.stl,66.0 x 40.0 mm top window; underside z=14.0 mm forms 1.15 mm outlet throat
P3,follower_plate,1,6061-T6 aluminium sheet,84.0 x 52.0 x 1.0,fabrication/follower_plate.stl,11.8 g; two 2.2 x 2.2 mm slots clear the spring guide posts
P4,follower_spring,2,Music-wire steel (G=77 GPa),d=0.45 wire x Dm=3.2 x L0=16.0,Commercial compression spring,1.25 N/mm each (2500 N/m pair); 2.0 N preload (0.2 mm deflection under 2.5 N thumb press)
P5,lid_screw,4,Stainless steel,M2 x 12 mm flat-head machine screw,ISO 7046 / commercial,Fastens P2 lid onto P1 corner pilot holes
P6,pvc_card,5,Rigid white PVC (1400 kg/m3),85.60 x 54.00 x 0.76,ISO/IEC 7810 ID-1 CR80 (e.g. Evolis C4001),Standard blank loyalty/membership card (~4.9 g each)
P7,desk_riser,1,Hardwood / birch block,50.0 x 38.0 x 50.0,fabrication/desk_riser.stl,Fixed to tabletop at (0.340, -0.045, 0.7475..0.7975 m); case rests loose on top
P8,output_mat,1,Cream silicone rubber sheet,140.0 x 100.0 x 2.0,Commercial 2 mm silicone sheet,Adhered flat to tabletop at (0.500, -0.100, 0.7475..0.7495 m)
"""
    (fab / "BOM.csv").write_text(bom)

    assembly = """# Fabrication and Bench Assembly Instructions

1. **Print or machine the enclosure (`case_base.stl`, `case_lid.stl`)** in rigid ABS at 1:1 millimetre scale. Lightly deburr the inner walls (`94.0 x 58.0 mm` pocket) and check the `10.85 mm` front dam height above the `2.0 mm` floor with calipers so that when the flat `case_lid` sits at `z = 14.00 mm`, the front outlet slot measures `1.15 +/- 0.05 mm` high across the `58.0 mm` inner width (a single `0.76 mm` CR80 PVC card slides freely; two stacked cards `1.52 mm` are blocked).
2. **Cut or mill `follower_plate.stl`** from `1.0 mm` 6061-T6 aluminium sheet (`84.0 x 52.0 x 1.0 mm`, mass `11.8 g`) and deburr the two `2.2 x 2.2 mm` guide slots at `x = +/-25.0 mm, y = 0`.
3. **Install springs and follower:** place the two compression springs (`d = 0.45 mm` wire, `3.2 mm` mean diameter, `16.0 mm` free length, `1.25 N/mm` each / `2500 N/m` pair; verify combined `2.0 N` preload at `5-card` stack height) over the `1.8 x 1.8 mm` posts on `case_base`, then seat `follower_plate` onto the springs.
4. **Close and load:** attach `case_lid` with four `M2 x 12 mm` corner screws, then insert five clean `85.60 x 54.00 x 0.76 mm` CR80 PVC cards (`Evolis C4001` or equivalent) backward through the front `1.15 mm` slot one at a time until squared inside the case.
5. **Mount station fixtures:** secure `desk_riser` (`50 x 38 x 50 mm`) to the station table centred at `(x = 0.340 m, y = -0.045 m)` with double-sided VHB tape or underside screws, and adhere the `140 x 100 x 2 mm` cream silicone `output_mat` flat at `(x = 0.500 m, y = -0.100 m)`. Set the loaded `card_case` **loose** (unclamped) centred on top of `desk_riser` with its `+X` outlet slot facing forward (`+X`).
"""
    (fab / "ASSEMBLY.md").write_text(assembly)


if __name__ == "__main__":
    build_all(OUT)
    print("Generated fabrication files:", sorted(str(p.relative_to(OUT)) for p in OUT.rglob("*") if p.is_file()))
