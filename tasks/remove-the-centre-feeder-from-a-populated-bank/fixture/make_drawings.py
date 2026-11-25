"""Generate the fixture table, shell cut list and dimensioned plan directly from task.MACHINE.
Run: python fixture/make_drawings.py (task's Python environment, no GPU required).
"""
import csv
import html
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import task

THICKNESS = 6.35  # mm, shell material; measure actual sheet and recalculate joints
BED_THICKNESS = 3.0  # mm finished target, +/-0.5; select nominal 1/8 in hardboard by caliper


def ranges(xr, yr, zr):
    return ([1000 * task.X_L + v for v in xr],
            sorted(1000 * task.Y_R - (v - 10) for v in yr), [100 + v for v in zr])


def write_csv(name, rows):
    with (HERE / name).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    rows, cuts = [], []
    for name, xr, yr, zr, _ in task.MACHINE:
        x, y, z = ranges(xr, yr, zr)
        a, b, c = xr[1]-xr[0], yr[1]-yr[0], zr[1]-zr[0]
        how = 'closed MDF shell; external faces at code coordinates'
        if name == 'bed':
            how = '560 x 365 hardboard; finished thickness 3.0 +/-0.5 mm; top at 143; underside 140 +/-1'
            cuts.append(dict(part=name, face='bed', qty=1, a_mm=a, b_mm=b, thickness_mm=BED_THICKNESS))
        elif 'y guide' in name:
            how = 'solid wood strip 12 x 550 x 8; retain both ends and full height'
            cuts.append(dict(part=name, face='solid strip', qty=1, a_mm=a, b_mm=b, thickness_mm=c))
        else:
            for face, qty, ca, cb in [('top/bottom (xy)',2,a,b), ('front/back (xz)',2,a,c-2*THICKNESS),
                                     ('left/right (yz)',2,b-2*THICKNESS,c-2*THICKNESS)]:
                assert min(ca, cb) > 0
                cuts.append(dict(part=name, face=face, qty=qty, a_mm=round(ca,2), b_mm=round(cb,2), thickness_mm=THICKNESS))
        rows.append(dict(name=name, x_m=f'{xr[0]}..{xr[1]}', y_m=f'{yr[0]}..{yr[1]}',
                         z_m=f'{zr[0]}..{zr[1]}', robot_x=f'{x[0]:g}..{x[1]:g}', robot_y=f'{y[0]:g}..{y[1]:g}',
                         above_bench=f'{z[0]:g}..{z[1]:g}', size_mm=f'{a} x {b} x {c}', substitute='build', how=how))
    # Derive these from the separately authored RAIL, BLADE and world transform, rather than a second dimension table.
    for name, xr, yr, zr in [('front rail',(0,600),(0,20),(0,20)),
                             ('slot blades',(103,498),tuple(10+v*1000 for v in task.BLADE[0]),
                              tuple(10+v*1000 for v in task.BLADE[1]))]:
        x,y,z=ranges(xr,yr,zr)
        rows.append(dict(name=name,x_m=f'{xr[0]}..{xr[1]}',y_m=f'{yr[0]:g}..{yr[1]:g}',z_m=f'{zr[0]:g}..{zr[1]:g}',
                         robot_x=f'{x[0]:g}..{x[1]:g}',robot_y=f'{y[0]:g}..{y[1]:g}',above_bench=f'{z[0]:g}..{z[1]:g}',
                         size_mm=f'{xr[1]-xr[0]} x {yr[1]-yr[0]:g} x {zr[1]-zr[0]:g}',substitute='genuine interface',
                         how='V-slot and genuine maker blades; preserve genuine feeder hook and catch'))
    write_csv('machine-boxes.csv',rows)
    write_csv('cut-list.csv',cuts)
    svg(rows)


def svg(rows):
    out=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1180" viewBox="0 0 1200 1180" '
         'font-family="Arial,sans-serif" font-size="12"><title>Complete substitute frame, dimensions from task.MACHINE</title>',
         '<rect width="1200" height="1180" fill="white"/>',
         '<text x="24" y="28" font-size="19">Complete machine envelope: plan and face coordinates (mm)</text>',
         '<text x="24" y="48">x_m along rail; y_m inward; heights above bench = z_m + 100. Every MACHINE box is built.</text>',
         '<text x="24" y="66">Robot x = 150 + x_m; robot y = -60 - y_m. Bed top 143; rail centre 110.</text>']
    out.append('<g transform="translate(80 350) scale(.76)">')
    for i,(name,xr,yr,zr,col) in enumerate(task.MACHINE):
        fill = '#d8d4ea' if name == 'bed' else '#cbd6df'
        out.append(f'<rect x="{xr[0]}" y="{yr[0]}" width="{xr[1]-xr[0]}" height="{yr[1]-yr[0]}" '
                   f'fill="{fill}" fill-opacity=".3" stroke="#536474" stroke-width="1.5"/>')
        out.append(f'<text x="{xr[0]+2}" y="{yr[0]+12}" font-size="12">{i+1}</text>')
    out.append('<rect x="0" y="0" width="600" height="20" fill="#222"/>')
    out.append('<rect x="103" y="-28" width="395" height="28" fill="#edc64b"/>')
    for k in range(3):
        xc=task.SLOT_X[task.FIRST+k]
        out.append(f'<rect x="{xc-7.5}" y="-85" width="15" height="165" fill="#111"/>')
        out.append(f'<rect x="{xc-6.5}" y="-265" width="13" height="178" fill="#bbb" stroke="#777"/>')
    out.append('<path d="M0,-125 L600,-125 M0,-130 L0,-120 M600,-130 L600,-120" fill="none" stroke="#111"/>')
    out.append('<text x="250" y="-135" font-size="15">rail 600</text>')
    out.append('<path d="M625,-80 L625,520 M620,-80 L630,-80 M620,520 L630,520" fill="none" stroke="#111"/>')
    out.append('<text x="615" y="200" font-size="13" transform="rotate(90 615 200)">side frame 600</text>')
    out.append('<text x="170" y="-215" font-size="14">3 reels, 178 dia.; slots 1-3</text>')
    out.append('<text x="170" y="70" font-size="13">5 mm bed/front gap</text>')
    out.append('<path d="M160,80 L240,80 M160,85 L240,85" stroke="#b43c2f"/>')
    out.append('<text x="-60" y="610" font-size="14">All outlines retained; numbers match table. Heights in table.</text>')
    out.append('</g>')
    out.append('<text x="610" y="105">PART / x_m range / y_m range / height range</text>')
    for i,r in enumerate(rows):
        txt=f'{i+1}. {r["name"]}:  x {r["x_m"]}; y {r["y_m"]}; h {r["above_bench"]}'
        out.append(f'<text x="610" y="{128+i*23}" font-size="11">{html.escape(txt)}</text>')
    for j,txt in enumerate(['Rail section: genuine 20 x 20 V-slot; bottom 100, top 120 above bench.',
                            'Slot blade: h 75..120; front y_m -28; genuine assembly includes its mating peg/PCB.',
                            'Closed shells follow all six exterior faces; cut-list.csv gives each panel.',
                            'Measure actual sheets. Bed: 3.0 +/-0.5 mm hardboard, top 143; shells 6.35 mm.',
                            'Two solid guides: 12 x 550 x 8. All positions +/-1 mm before enrollment.',
                            'Plan overlaps are intentional at joints; trim internal panels, never exterior faces.',
                            'See BUILD.md for joints, supports, cost, version limits and enrollment checks.']):
        out.append(f'<text x="24" y="{950+j*25}">{html.escape(txt)}</text>')
    out.append('</svg>')
    (HERE/'frame-plan.svg').write_text('\n'.join(out)+'\n')


if __name__ == '__main__':
    main()
