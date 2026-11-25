"""battery-pills scorer (scoring.py): the card's scorer.py (revision r7) verbatim, plus SimScorer at the end, which
builds the scorer's state from simulator body poses and gripper meshes. The rule part is pure stdlib.

Original docstring follows.
battery-pills scorer. Pure stdlib. score(state) -> float implements the card's Scoring field; thresholds from params.json.

State (raw numbers only, mm / deg, card frame: x forward, y left, z above the table top):
  tray:    {"x", "y", "yaw_deg"}                         tray centre and turn now (it may be slid or turned)
  holder:  {"x", "y", "z_bottom"}                        axis x, y; z_bottom = height of the CENTRE of its base above
                                                         the table top (lying or leaning raises it); None = not found
  dish:    {"x", "y", "z", "disp_1s_mm"}                 centre (upright dish); None = not found
  cell:    {"x", "y", "z", "disp_1s_mm"}                 centre; None = not found
  tablets: list of 5 x {"x", "y", "z", "disp_1s_mm"} or None (not found), at the end (final photos); no tablet's
           identity or history is used (r7)
  spilled: bool, trial-wide (r7, review TK-8X2ZNN): at some time in the run a tablet lay outside the dish (centre more
           than left_dish_radius_mm from the dish axis) with every gripper part at least spill_gap_mm from it in the top
           view; missing = unknown = spilled
  gripper_min_dist_mm: [left, right]  closest distance from any gripper part to any task object (cell, tablet, dish,
                       holder, tray) at the read
  holder_disp_1s_mm: number
Run `python3 -I card_scorer.py` for the negative-control table and the blind-policy sweep; exits non-zero on any mismatch.
"""
import json, math, os, sys

P = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'params.json')))
S, T, H, D, L = P['scoring'], P['tray'], P['holder'], P['dish'], P['layout']
NT = P['tablets']['count']
# Revision r5 (blind review r2 note): the Served outline is the G1-measured tray inner floor (served_outline_mm; the
# listing-based estimate until G1 replaces it), not a fixed 340 x 150.
OUTLINE = T['served_outline_mm']
HALF_X = OUTLINE['x'] / 2.0
HALF_Y = OUTLINE['y'] / 2.0
RIM = T['outer_mm']['z']
H_R = H['stowed_radius_mm']           # Stowed radius: half the jar's inner bottom diameter (G1; estimate until measured)
H_RIM = H['depth_mm']
H_BOTTOM_MAX = S['holder_on_table_max_mm']   # jar (key 'holder') standing on the table top: bottom within this of it
D_IN_R = S['dish_inner_radius_mm']           # a served tablet lies inside the dish: within this of its axis
D_HALF_H = D['height_mm'] / 2.0              # dish rim = dish centre z + half its height
D_FLOOR_R = D['floor_radius_mm']             # inner floor radius (floor_inner_diameter / 2)
D_FLOOR_TOP = D['floor_top_mm_estimate']     # floor top above the dish bottom
D_H = D['height_mm']


def on_floor(o):
    return o is None or o['z'] < S['floor_z_mm']


def on_tray(o, tray):
    """Centre inside the tray's inner floor outline, in the tray's own frame (it may be turned), and below its rim."""
    a = math.radians(tray.get('yaw_deg', 0.0))
    dx, dy = o['x'] - tray['x'], o['y'] - tray['y']
    u, v = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)
    return abs(u) <= HALF_X and abs(v) <= HALF_Y and o['z'] < RIM


def inner_surface(r):
    """Height of the dish's inner surface above the dish bottom at radius r: flat floor, then the straight inner wall
    from (D_FLOOR_R, D_FLOOR_TOP) to (D_IN_R, D_H). None beyond the rim's inner radius."""
    if r <= D_FLOOR_R:
        return D_FLOOR_TOP
    if r <= D_IN_R:
        return D_FLOOR_TOP + (r - D_FLOOR_R) * (D_H - D_FLOOR_TOP) / (D_IN_R - D_FLOOR_R)
    return None


def in_bowl(t, dish):
    """Tablet centre inside the dish's bowl: above its inner surface at its radius and below the rim."""
    r = math.hypot(t['x'] - dish['x'], t['y'] - dish['y'])
    zb = t['z'] - (dish['z'] - D_HALF_H)
    s = inner_surface(r)
    return s is not None and s < zb < D_H


def spill_sample(r_dish_mm, gap_xy_mm):
    """Revision r7 (review TK-8X2ZNN, Scoring): one top-view reading of one tablet is a spill when its centre lies outside
    the dish (beyond the rim's inner radius, in the dish's own frame) and every gripper part is at least spill_gap_mm from
    it in the top view: clearly released, not carried or pushed. A tablet under or between the fingers is within the gap
    by definition, so no occluded tablet is ever tracked; the flag is trial-wide, so no tablet's identity is kept."""
    return r_dish_mm > S['left_dish_radius_mm'] and gap_xy_mm >= S['spill_gap_mm']


def served(t, dish, tray):
    """Tablet inside the dish's bowl at the end, and the dish on the tray (centre in the outline in the tray frame, below
    the tray rim). Counted in the final tray photo only (r7: no per-tablet history)."""
    return on_tray(dish, tray) and in_bowl(t, dish)


def in_holder(o, h):
    return math.hypot(o['x'] - h['x'], o['y'] - h['y']) <= H_R and h['z_bottom'] <= o['z'] < h['z_bottom'] + H_RIM


def conditions(st):
    tabs = st.get('tablets')
    objs = [st.get('cell'), st.get('dish'), st.get('holder')] + (list(tabs) if tabs is not None else [None])
    if tabs is not None and len(tabs) != NT:
        objs.append(None)
    c = {}
    c['floor'] = any(on_floor(o) if (o is None or 'z' in o) else (o.get('z_bottom', -99) < S['floor_z_mm']) for o in objs)
    g = st.get('gripper_min_dist_mm')
    c['released'] = g is not None and len(g) == 2 and min(g) >= S['release_clearance_mm']
    moving = [st['dish']] + list(tabs or []) if not c['floor'] else []   # r4 (red-team p3 L3.2): the cell's stillness is not read
    c['still'] = (not c['floor']) and all(o['disp_1s_mm'] < S['still_translation_mm'] for o in moving) and \
        st.get('holder_disp_1s_mm', 99) < S['still_translation_mm']
    h = st.get('holder')
    c['holder_ok'] = h is not None and h['z_bottom'] <= H_BOTTOM_MAX
    c['stowed'] = (not c['floor']) and c['holder_ok'] and in_holder(st['cell'], h) and \
        not any(in_holder(t, h) for t in tabs)
    c['n_on_tray'] = 0 if c['floor'] else sum(1 for t in tabs if served(t, st['dish'], st['tray']))
    c['spilled'] = st.get('spilled') is not False      # r7: trial-wide; missing reading = spilled
    return c


def score(st):
    c = conditions(st)
    if c['floor'] or not c['released'] or not c['still'] or not c['stowed']:
        return 0.0
    n = c['n_on_tray']
    if n == NT:
        s = S['tiers']['full']
    elif n >= 1:
        s = S['tiers']['partial_tray']
    else:
        s = S['tiers']['stowed_only']
    return min(s, S['spill_cap']) if c['spilled'] else s


# ---------------------------------------------------------------- negative controls and blind sweep
FT = D['floor_top_mm_estimate']
TCZ = P['tablets']['thickness_mm_estimate'] / 2
CCZ = P['cell']['height_mm'] / 2
TRAY_FT = T['floor_top_mm_estimate']


def slots(cx, cy, phi):
    s = [(cx, cy)]
    for k in range(6):
        a = math.radians(phi + 60 * k)
        s.append((cx + L['ring_radius_mm'] * math.cos(a), cy + L['ring_radius_mm'] * math.sin(a)))
    return s


def start_state(draw):
    """draw: dish (dx,dy), tray (dx,dy), holder (dx,dy), phi, cell slot, empty slot."""
    dcx, dcy = D['start_centre_mm'][0] + draw['dish'][0], D['start_centre_mm'][1] + draw['dish'][1]
    tcx, tcy = T['start_centre_mm'][0] + draw['tray'][0], T['start_centre_mm'][1] + draw['tray'][1]
    hcx, hcy = H['start_centre_mm'][0] + draw['holder'][0], H['start_centre_mm'][1] + draw['holder'][1]
    sl = slots(dcx, dcy, draw['phi'])
    tabs = [{"x": sl[j][0], "y": sl[j][1], "z": FT + TCZ, "disp_1s_mm": 0.0}
            for j in range(7) if j not in (draw['cell'], draw['empty'])]
    return {"tray": {"x": tcx, "y": tcy, "yaw_deg": T['start_yaw_deg']},
            "holder": {"x": hcx, "y": hcy, "z_bottom": 0.0},
            "dish": {"x": dcx, "y": dcy, "z": D['height_mm'] / 2, "disp_1s_mm": 0.0},
            "cell": {"x": sl[draw['cell']][0], "y": sl[draw['cell']][1], "z": FT + CCZ, "disp_1s_mm": 0.0},
            "tablets": tabs, "gripper_min_dist_mm": [200.0, 200.0], "holder_disp_1s_mm": 0.0, "spilled": False,
            "_slots": sl}


def move_dish_to_tray(st, carry_cell=True):
    """Dish (with whatever is in it) set on the tray floor at (tray cx - 15, tray cy)."""
    dx = st['tray']['x'] + D['place_on_tray_offset_x_mm'] - st['dish']['x']
    dy = st['tray']['y'] - st['dish']['y']
    dcx, dcy = st['dish']['x'], st['dish']['y']

    def inside(o):
        return math.hypot(o['x'] - dcx, o['y'] - dcy) < D['top_diameter_mm'] / 2 and o['z'] < D['height_mm']
    for t in st['tablets']:
        if inside(t):
            t['x'] += dx; t['y'] += dy; t['z'] += TRAY_FT
    if carry_cell and inside(st['cell']):
        st['cell']['x'] += dx; st['cell']['y'] += dy; st['cell']['z'] += TRAY_FT
    st['dish']['x'] += dx; st['dish']['y'] += dy; st['dish']['z'] += TRAY_FT
    return st


def to_holder(o, st, dx=0.0, dy=0.0):
    o['x'] = st['holder']['x'] + dx; o['y'] = st['holder']['y'] + dy; o['z'] = 1.0 + 2.0


def deep(st):
    return json.loads(json.dumps(st))


NOM = {"dish": (0, 0), "tray": (0, 0), "holder": (0, 0), "phi": 0.0, "cell": 3, "empty": 6}


def full_state(draw=NOM):
    st = start_state(draw)
    to_holder(st['cell'], st)
    return move_dish_to_tray(st)


def set_outline(x_mm, y_mm):
    """Set the Served outline (the G1-measured tray inner floor) used by on_tray()."""
    global HALF_X, HALF_Y
    HALF_X, HALF_Y = x_mm / 2.0, y_mm / 2.0


def rows():
    R = []

    def add(name, st, exp, outline=None):
        R.append((name, st, exp, outline))
    s0 = start_state(NOM)
    add('start scene / do nothing (cell among the tablets)', deep(s0), 0.0)
    st = deep(s0); st['cell'].update(x=st['holder']['x'], y=st['holder']['y'], z=H_RIM + 10); st['gripper_min_dist_mm'] = [0.0, 200.0]
    add('cell held still 10 mm above the jar rim', st, 0.0)
    add('full: cell in jar, dish with 5 tablets on the tray', full_state(), 1.0)
    st = start_state(NOM); to_holder(st['cell'], st)
    for i, t in enumerate(st['tablets']):
        t.update(x=st['tray']['x'] - 40 + 20 * i, y=st['tray']['y'] + 100, z=TRAY_FT + TCZ)
    st['spilled'] = True
    add('tablets set loose on the tray one by one, dish left on the table (not in their dish, as the instruction asks)', st, 0.25)
    st = start_state(NOM)
    for i, t in enumerate(st['tablets']):
        t.update(x=st['tray']['x'] - 50 + 25 * i, y=st['tray']['y'] - 60 + 7 * i, z=TRAY_FT + TCZ)
    st['cell'].update(x=st['tray']['x'] + 30, y=st['tray']['y'] + 90, z=TRAY_FT + CCZ); to_holder(st['cell'], st)
    st['spilled'] = True
    add('pour route: dish tipped onto the tray, cell pinched off the bare tray floor into the jar, dish back on the table (pills not in their dish)', st, 0.25)
    st = start_state(NOM)
    for i, t in enumerate(st['tablets']):
        t.update(x=st['tray']['x'] - 50 + 25 * i, y=st['tray']['y'] - 100, z=TRAY_FT + TCZ)
    to_holder(st['cell'], st); st['spilled'] = True
    st['dish'].update(x=st['tray']['x'] - 15, y=st['tray']['y'] + 60, z=TRAY_FT + D['height_mm'] / 2)
    add('pour route, empty dish then set on the tray beside the loose tablets (pills not in their dish)', st, 0.25)
    st = start_state(NOM); move_dish_to_tray(st); to_holder(st['cell'], st)
    add('alternative order: dish served first, cell then lifted out into the jar', st, 1.0)
    st = full_state(); st['holder']['x'] += 50; to_holder(st['cell'], st)
    add('jar slid 50 mm along the table, still upright, cell inside', st, 1.0)
    st = full_state(); st['cell']['x'] += H_R - 0.1
    add(f'cell inside the jar at {H_R - 0.1:.1f} mm from its axis (Stowed radius, boundary in)', st, 1.0)
    st = full_state(); st['cell']['x'] += H_R + 0.1
    add(f'cell {H_R + 0.1:.1f} mm from the jar axis (beyond the Stowed radius)', st, 0.0)
    st = start_state(NOM); to_holder(st['cell'], st)
    add('cell stowed, dish (5 tablets) left on the table', st, 0.25)
    st = start_state(NOM); move_dish_to_tray(st)
    add('dish served with the cell still in it', st, 0.0)
    st = full_state(); st['cell'].update(x=st['tray']['x'] + 50, y=st['tray']['y'] + 100, z=TRAY_FT + CCZ)
    add('cell set on the tray beside the dish', st, 0.0)
    st = full_state(); st['cell'].update(x=200, y=150, z=CCZ)
    add('cell set on the table, not in the jar', st, 0.0)
    st = full_state(); to_holder(st['tablets'][0], st, 10, 0)
    add('cell in the jar with one tablet; 4 tablets on the tray', st, 0.0)
    st = start_state(NOM)
    for t in st['tablets']:
        to_holder(t, st, 5, 5)
    to_holder(st['cell'], st)
    add('whole dish tipped into the jar (cell and all tablets)', st, 0.0)
    st = full_state(); st['tablets'][0].update(x=200, y=100, z=TCZ); st['spilled'] = True
    add('stowed; 4 tablets on the tray, 1 spilt on the table', st, 0.5)
    st = full_state(); st['tablets'][0].update(x=200, y=100, z=TCZ); st['tablets'][1].update(x=210, y=-100, z=TCZ)
    st['spilled'] = True
    add('stowed; 3 tablets on the tray, 2 spilt on the table', st, 0.5)
    st = full_state(); st['tablets'][2].update(z=-700); st['spilled'] = True
    add('one tablet on the floor, rest full', st, 0.0)
    st = full_state(); st['cell'].update(x=500, y=700, z=-700)
    add('cell on the floor', st, 0.0)
    st = full_state(); st['tablets'][1] = None
    add('one tablet not found in the end photos', st, 0.0)
    st = full_state(); st['holder'].update(x=st['tray']['x'] + 30, y=st['tray']['y'] + 120, z_bottom=TRAY_FT); to_holder(st['cell'], st)
    st['cell']['z'] = TRAY_FT + 3
    add('jar with the cell set on the tray', st, 0.0)
    st = full_state(); st['holder']['z_bottom'] = H['top_diameter_mm'] / 2; st['cell'].update(x=st['holder']['x'] + 120, z=CCZ)
    add('jar knocked over (lying on its side, base centre 38 mm up), cell rolled out onto the table', st, 0.0)
    st = full_state(); st['holder']['z_bottom'] = H['top_diameter_mm'] / 2
    add('jar knocked over, cell still inside it', st, 0.0)
    st = full_state(); st['holder']['z_bottom'] = H_RIM; st['cell']['z'] = CCZ
    add('jar turned over (base on top) on top of the cell', st, 0.0)
    st = full_state(); st['holder']['z_bottom'] = round(H['top_diameter_mm'] / 2 * math.sin(math.radians(10)), 1)
    add('jar leaning 10 deg on one base edge (base centre 6.6 mm up), cell inside', st, 0.0)
    st = full_state(); st['cell']['z'] = H_RIM + CCZ
    add('cell resting on the jar rim', st, 0.0)
    st = full_state(); st['gripper_min_dist_mm'] = [30.0, 45.0]
    add('grippers 30.0 mm clear (boundary in)', st, 1.0)
    st = full_state(); st['gripper_min_dist_mm'] = [29.9, 45.0]
    add('a gripper 29.9 mm from the dish', st, 0.0)
    st = full_state(); st['dish']['disp_1s_mm'] = 1.9
    add('dish moved 1.9 mm in the last second', st, 1.0)
    st = full_state(); st['dish']['disp_1s_mm'] = 2.0
    add('dish moved 2.0 mm in the last second', st, 0.0)
    st = full_state(); st['cell']['disp_1s_mm'] = 2.5
    add('cell still rolling inside the jar (its stillness is not read, r4)', st, 1.0)
    st = full_state(); st['dish'].update(z=st['dish']['z'] + 60); st['gripper_min_dist_mm'] = [0.0, 0.0]
    for t in st['tablets']:
        t['z'] += 60
    add('dish held still 60 mm over the tray floor, cell stowed', st, 0.0)
    st = full_state(); st['gripper_min_dist_mm'] = [12.0, 200.0]
    add('cell stowed, one hand still holding the jar', st, 0.0)
    st = full_state(); t = st['tablets'][0]; t.update(x=st['dish']['x'] + D_IN_R - 0.1, y=st['dish']['y'],
                                                       z=st['dish']['z'] + D_HALF_H - 0.05)
    add('tablet 57.4 mm from the dish axis, 0.05 mm below the rim height (inside the bowl, boundary in)', st, 1.0)
    st = full_state(); t = st['tablets'][0]; t.update(x=st['dish']['x'] + D_IN_R + 0.1, y=st['dish']['y'],
                                                       z=st['dish']['z'] + D_HALF_H - 0.05)
    st['spilled'] = True
    add('tablet 57.6 mm from the dish axis, same height (beyond the rim\'s inner edge: not in the bowl)', st, 0.5)
    r45 = 45.0; s45 = inner_surface(r45)
    for dz, exp, lab in ((0.1, 1.0, '0.1 mm above'), (-0.1, 0.5, '0.1 mm below')):
        st = full_state(); t = st['tablets'][0]
        t.update(x=st['dish']['x'], y=st['dish']['y'] + r45, z=st['dish']['z'] - D_HALF_H + s45 + dz)
        add(f'tablet centre at r 45 mm, {lab} the dish\'s inner wall ({s45:.2f} mm over the dish bottom)', st, exp)
    # red-team p2 L2.1: tablets on the tray floor under the dish's flared wall
    st = full_state(); t = st['tablets'][0]
    t.update(x=st['dish']['x'] + 50.0, y=st['dish']['y'], z=TRAY_FT + TCZ)
    add('one tablet on the tray floor under the dish wall at r 50 mm, 4 in the dish (not served)', st, 0.5)
    st = start_state(NOM); to_holder(st['cell'], st)
    dcx, dcy = st['tray']['x'] + D['place_on_tray_offset_x_mm'], st['tray']['y']
    st['dish'].update(x=dcx, y=dcy, z=TRAY_FT + D_HALF_H)
    for i, t in enumerate(st['tablets']):
        a = math.radians(72 * i)
        t.update(x=dcx + 50 * math.cos(a), y=dcy + 50 * math.sin(a), z=TRAY_FT + TCZ)
    st['spilled'] = True
    add('pour route: dish tipped onto the tray, cell pinched off the tray floor into the jar, dish set upright over the '
        'tablets (all 5 at r 50 mm under its wall; pills not in their dish)', st, 0.25)
    st = full_state(); t = st['tablets'][0]
    t.update(x=st['dish']['x'] + 50.0, y=st['dish']['y'], z=TRAY_FT + P['tablets']['diameter_mm_estimate'] / 2)
    add('one tablet standing on edge on the tray floor under the dish wall at r 50 mm (not served)', st, 0.5)
    st = full_state(); st['tablets'][0]['z'] = st['dish']['z'] + D_HALF_H - 0.1
    add('tablet centre 0.1 mm below the dish rim height (boundary in)', st, 1.0)
    st = full_state(); st['tablets'][0]['z'] = st['dish']['z'] + D_HALF_H + 0.1
    add('tablet centre 0.1 mm above the dish rim height (not in the dish)', st, 0.5)
    for dxo, exp, lab in ((HALF_X - 0.1, 1.0, 'inside'), (HALF_X + 0.1, 0.25, 'outside')):
        st = full_state(); sh = st['tray']['x'] + dxo - st['dish']['x']
        for o in [st['dish']] + st['tablets']:
            o['x'] += sh
        add(f'dish centre 0.1 mm {lab} the tray floor outline (x), tablets in it', st, exp)
    for zc, exp, lab in ((RIM - 0.1, 1.0, '0.1 mm below'), (RIM + 0.1, 0.25, '0.1 mm above')):
        st = full_state(); sh = zc - st['dish']['z']
        for o in [st['dish']] + st['tablets']:
            o['z'] += sh
        add(f'dish centre {lab} the tray rim height, tablets in it', st, exp)
    st = full_state(); st['holder']['z_bottom'] = H_BOTTOM_MAX - 0.1; to_holder(st['cell'], st); st['cell']['z'] = H_BOTTOM_MAX + 3
    add('jar base centre 1.9 mm above the table top (boundary in)', st, 1.0)
    st = full_state(); st['holder']['z_bottom'] = H_BOTTOM_MAX + 0.1; to_holder(st['cell'], st); st['cell']['z'] = H_BOTTOM_MAX + 3
    add('jar base centre 2.1 mm above the table top (not standing on its base on the table)', st, 0.0)
    st = full_state(); st['cell']['z'] = H_RIM - 0.1
    add('cell centre 0.1 mm below the jar rim height, on its axis (boundary in)', st, 1.0)
    st = full_state(); st['tablets'][0].update(x=200, y=100, z=S['floor_z_mm'] + 0.1); st['spilled'] = True
    add('tablet centre 9.9 mm below the table top (not on the floor; off the tray)', st, 0.5)
    st = full_state(); st['tablets'][0].update(x=200, y=100, z=S['floor_z_mm'] - 0.1); st['spilled'] = True
    add('tablet centre 10.1 mm below the table top (on the floor)', st, 0.0)
    st = full_state(); st['tray']['x'] += 30
    for t in st['tablets']:
        t['x'] += 30
    st['dish']['x'] += 30
    add('tray pushed 30 mm with the dish on it (allowed)', st, 1.0)
    st = full_state(); st['dish'].update(z=-700)
    add('dish dropped to the floor', st, 0.0)
    # red-team p2 L3.1: tray slid and turned; the outline is tested in the tray's own frame
    yaw = S['tray_turn_row_deg']
    st = full_state(); st['tray']['yaw_deg'] = yaw
    ox, oy = 70.0, 165.0
    sx, sy = st['tray']['x'] + ox - st['dish']['x'], st['tray']['y'] + oy - st['dish']['y']
    for o in [st['dish']] + st['tablets']:
        o['x'] += sx; o['y'] += sy; o['z'] -= TRAY_FT
    add(f'tray turned {yaw} deg; dish with 5 tablets on the table at (70, 165) from the tray centre (tray frame '
        f'(159.7, 81.4): outside the floor outline)', st, 0.25)
    st = full_state(); a = math.radians(yaw); st['tray']['yaw_deg'] = yaw
    u, v = D['place_on_tray_offset_x_mm'], 60.0
    tx, ty = st['tray']['x'] + u * math.cos(a) - v * math.sin(a), st['tray']['y'] + u * math.sin(a) + v * math.cos(a)
    sx, sy = tx - st['dish']['x'], ty - st['dish']['y']
    for o in [st['dish']] + st['tablets']:
        o['x'] += sx; o['y'] += sy
    add(f'tray turned {yaw} deg with the dish on its floor at tray-frame (-15, 60) (served)', st, 1.0)
    # blind review r1: a cell resting on the jar floor against the wall of the (possibly wider) body is stowed
    st = full_state(); st['cell']['x'] += H_R - P['cell']['diameter_mm'] / 2
    add(f'cell flat on the jar floor against its wall ({H_R - P["cell"]["diameter_mm"] / 2:.2f} mm from the axis, Stowed radius '
        f'{H_R:.0f} from G1)', st, 1.0)
    # red-team p3 L2.1: refill route; all six discs tipped onto the tray, cell pinched off the bare tray floor into the jar,
    # empty dish set on the tray, the 5 tablets pinched back into it
    st = start_state(NOM)
    for i, t in enumerate(st['tablets']):
        t.update(x=st['tray']['x'] - 50 + 25 * i, y=st['tray']['y'] - 100, z=TRAY_FT + TCZ)
    st['spilled'] = True       # the tablets lay loose on the tray, clear of both grippers
    to_holder(st['cell'], st)
    dcx, dcy = st['tray']['x'] + D['place_on_tray_offset_x_mm'], st['tray']['y']
    st['dish'].update(x=dcx, y=dcy, z=TRAY_FT + D_HALF_H)
    for i, t in enumerate(st['tablets']):
        a = math.radians(72 * i)
        t.update(x=dcx + 12 * math.cos(a), y=dcy + 12 * math.sin(a), z=TRAY_FT + FT + TCZ)
    add('refill route: all six discs tipped onto the tray, cell pinched off the bare tray into the jar, empty dish set on the '
        'tray, 5 tablets pinched back into it at r 12 mm (spill: capped at 0.5; r7)', st, 0.5)
    # r7 (review TK-8X2ZNN): boundaries of one top-view spill reading (tablet centre's distance from the dish axis, gap to
    # the nearest gripper part in the top view); the tablet is back in the dish at the end in every row
    RL, GAP = S['left_dish_radius_mm'], S['spill_gap_mm']
    for r, g, exp, lab in ((RL - 0.1, 200.0, 1.0, 'inside the rim\'s inner edge: no spill'),
                           (RL + 0.1, 200.0, 0.5, 'outside the dish, clear of the grippers: spill'),
                           (70.0, GAP - 0.1, 1.0, 'a gripper part within the gap: carried or pushed, no spill'),
                           (70.0, GAP, 0.5, 'gripper parts at the gap: spill')):
        st = full_state(); st['spilled'] = spill_sample(r, g)
        add(f'one tablet seen at {r:.1f} mm from the dish axis with the nearest gripper part {g:.1f} mm from it in the top '
            f'view, back in the dish at the end ({lab})', st, exp)
    # red-team p3 L2.2: tablets swept to the dish floor's edge (allowed: moving a tablet within the dish), cell from the centre
    st = start_state(NOM)
    for i, t in enumerate(st['tablets']):
        a = math.radians(72 * i)
        t.update(x=st['dish']['x'] + 27 * math.cos(a), y=st['dish']['y'] + 27 * math.sin(a))
    st['spilled'] = spill_sample(27.0, 200.0)
    to_holder(st['cell'], st); move_dish_to_tray(st)
    add('tablets swept to r 27 mm on the dish floor (within the dish, allowed), cell pinched, dish served', st, 1.0)
    # red-team p3 L3.4: tray outline boundary along y (the 340 mm side)
    for dyo, exp, lab in ((HALF_Y - 0.1, 1.0, 'inside'), (HALF_Y + 0.1, 0.25, 'outside')):
        st = full_state(); sh = st['tray']['y'] + dyo - st['dish']['y']
        for o in [st['dish']] + st['tablets']:
            o['y'] += sh
        add(f'dish centre 0.1 mm {lab} the tray floor outline (y, {dyo:.1f} mm from the tray centre), tablets in it', st, exp)
    # red-team p3 L3.3: jar stood upright in the empty dish on the table (its bottom rests on the dish's inner wall)
    st = start_state(NOM)
    for i, t in enumerate(st['tablets']):
        t.update(x=st['tray']['x'] - 50 + 25 * i, y=st['tray']['y'] - 60, z=TRAY_FT + TCZ)
    zb = round(inner_surface(H_R), 2)
    st['holder'].update(x=st['dish']['x'], y=st['dish']['y'], z_bottom=zb)
    to_holder(st['cell'], st); st['cell']['z'] = zb + 3.0; st['spilled'] = True
    add(f'tablets poured loose on the tray, cell in the jar, jar stood in the empty dish on the table (base centre {zb} mm up)',
        st, 0.0)
    # r5 (blind review r2 note): G1 measures the tray floor at its pass minimum; that floor, not 340 x 150, sets the outline
    g1x, g1y = T['g1_pass_inner_floor_mm_min']['x'], T['g1_pass_inner_floor_mm_min']['y']
    for dyo, exp, lab in ((g1y / 2 - 0.1, 1.0, 'inside'), (g1y / 2 + 0.1, 0.25, 'outside'),
                          (OUTLINE['y'] / 2 - 0.1, 0.25, 'inside the estimate but outside')):
        st = full_state(); sh = st['tray']['y'] + dyo - st['dish']['y']
        for o in [st['dish']] + st['tablets']:
            o['y'] += sh
        add(f'G1 tray floor {g1x:.0f} x {g1y:.0f}: dish centre {dyo:.1f} mm along y from the tray centre ({lab} the measured '
            f'outline), tablets in it', st, exp, (g1x, g1y))
    # r6 (red-team p4 L2.1): the cell nudged up the sloped wall and over the rim onto the bare table, then pinched off the
    # table into the jar; the tablets never moved; dish served. The rule asks only that the cell leave the dish floor.
    st = start_state(NOM); st['cell'].update(x=st['dish']['x'] + 70.0, y=st['dish']['y'], z=CCZ)
    add('cell pushed over the dish rim onto the table 70 mm from the dish axis (not yet stowed)', deep(st), 0.0)
    to_holder(st['cell'], st); move_dish_to_tray(st)
    add('cell pushed over the dish rim onto the table, then pinched off the table into the jar; dish served', st, 1.0)
    # r7 (review TK-8X2ZNN): trial-wide spill flag from the top video; no tablet's identity or history is kept
    st = full_state(); st['spilled'] = any(spill_sample(r, g) for r, g in ((40.0, 0.0), (70.0, 0.0), (70.0, 3.0), (40.0, 0.0)))
    add('one tablet caught in the cell pinch, carried 70 mm from the dish axis between the fingers and dropped back on the '
        'dish floor (never 10 mm clear of a gripper part in the top view: no spill)', st, 1.0)
    st = full_state(); st['spilled'] = any(spill_sample(r, g) for r, g in ((70.0, 0.0), (70.0, 50.0), (70.0, 0.0)))
    add('one tablet set down on the table 70 mm from the dish axis, the gripper backed off 50 mm, then the tablet put back '
        '(spill: capped at 0.5)', st, 0.5)
    st = full_state(); st.pop('spilled')
    add('spill reading missing (top video unreadable): counts as a spill', st, 0.5)
    st = start_state(NOM); to_holder(st['cell'], st); st['spilled'] = True
    add('cell stowed, dish left on the table, a tablet spilt and put back (the cap does not raise 0.25)', st, 0.25)
    st = full_state(); st['spilled'] = True; st['gripper_min_dist_mm'] = [29.9, 200.0]
    add('spill, all 5 back in the dish on the tray, a gripper 29.9 mm from the dish (Released fails first)', st, 0.0)
    # r6 (red-team p4 L3.3): lower edge of the 0.5 tier, exactly 1 served
    st = full_state(); st['spilled'] = True
    for i in range(1, NT):
        st['tablets'][i].update(x=200 + 15 * i, y=100, z=TCZ)
    add('stowed; 1 tablet in the dish on the tray, 4 spilt on the table (boundary of the 0.5 tier)', st, 0.5)
    return R


def blind_draws():
    """14 draws: cell slot 0..6 twice, phi 0..60 end to end, offsets at the range ends in turn."""
    corners = [((15, 15), (10, 15), (10, 10)), ((-15, -15), (-10, -15), (-10, -10)), ((15, -15), (10, -15), (10, -10)),
               ((-15, 15), (-10, 15), (-10, 10)), ((0, 0), (0, 0), (0, 0))]
    out = []
    for i in range(14):
        cell = i % 7
        phi = round(60.0 * i / 13, 3) if i < 13 else 59.9
        empty = (cell + 3) % 7 if cell else 4
        dd, td, hd = corners[i % 5]
        out.append({"dish": dd, "tray": td, "holder": hd, "phi": phi, "cell": cell, "empty": empty})
    return out


def plan_end(plan, draw):
    st = start_state(draw)
    sl = st['_slots']

    def pick_at(px, py):
        """Fixed pinch at (px, py) on the dish floor; captures the object whose centre is within 3 mm; puts it in the holder."""
        objs = [st['cell']] + st['tablets']
        for o in objs:
            if math.hypot(o['x'] - px, o['y'] - py) <= 3.0 and o['z'] < D['height_mm']:
                to_holder(o, st)
                return
    dcx, dcy = D['start_centre_mm']
    if plan == 'A serve only':
        move_dish_to_tray(st)
    elif plan == 'B pinch at the nominal dish centre, then serve':
        pick_at(dcx, dcy); move_dish_to_tray(st)
    elif plan == 'C pinch at the ring slot at +x (phi 0), then serve':
        pick_at(dcx + L['ring_radius_mm'], dcy); move_dish_to_tray(st)
    elif plan == 'D pinch centre and +x ring slot, then serve':
        pick_at(dcx, dcy); pick_at(dcx + L['ring_radius_mm'], dcy); move_dish_to_tray(st)
    elif plan == 'F pinch at the dish centre as seen (dish located, cell not looked for), then serve':
        pick_at(st['dish']['x'], st['dish']['y']); move_dish_to_tray(st)
    elif plan == 'E tip the whole dish into the holder':
        for o in [st['cell']] + st['tablets']:
            to_holder(o, st, 3, 3)
    st.pop('_slots', None)
    return st


PLANS = ['A serve only', 'B pinch at the nominal dish centre, then serve', 'C pinch at the ring slot at +x (phi 0), then serve',
         'D pinch centre and +x ring slot, then serve', 'E tip the whole dish into the holder',
         'F pinch at the dish centre as seen (dish located, cell not looked for), then serve']



# ---------------------------------------------------------------- simulator adapter (build/battery-pills)
# Everything below reads simulator state (run.py passes it in); nothing here reaches a policy. NumPy only.
class SimScorer:
    """Builds the card scorer's state from body poses (card frame, mm) once per control step.

    update(poses, grip_dist, read_robot, grip_xy) with
      poses: {'tray'|'dish'|'jar'|'cell'|'tablet0'..'tablet4': (pos_world_m (3,), rot (3, 3))}; tray/dish/jar origins at
             their bottom centre on the axis, cell/tablet origins at their centre
      grip_dist: callable(name) -> (left_mm, right_mm), lower bounds of the gripper-part distance to that body
             (run.py: separating-plane bound over the collision hulls); called for every body when ``read_robot``
      grip_xy: callable(name) -> (left_mm, right_mm), lower bounds of the gap between that tablet and the gripper parts
             in the top view (horizontal plane; run.py: tablet footprint disc to each hull's convex footprint); called
             for a tablet whenever its centre lies beyond the rim's inner radius (r7 spill rule). Defaults to grip_dist.
    Returns dict(score, state, conditions).
    """

    TABLETS = [f'tablet{i}' for i in range(NT)]
    BODIES = ['tray', 'dish', 'jar', 'cell'] + TABLETS

    def __init__(self, control_hz=20, table_top_m=0.7475):
        self.hz, self.table = control_hz, table_top_m
        self.window = int(round(S['still_window_s'] * control_hz))
        self.hist = {n: [] for n in self.BODIES}
        self.spilled = False
        self.spills = []          # (step, tablet, r_dish_mm, gap_xy_mm) for the first readings that set the flag
        self.last_grip = [200.0, 200.0]
        self.steps = 0

    def _card(self, p):
        return [p[0] * 1000.0, p[1] * 1000.0, (p[2] - self.table) * 1000.0]

    def _disp(self, name):
        h = self.hist[name][-(self.window + 1):]
        if len(h) < self.window + 1:
            return 99.0           # not yet observed for a full window: unknown = moving
        cur = h[-1]
        return max(math.dist(cur, q) for q in h[:-1])

    def update(self, poses, grip_dist, read_robot=False, grip_xy=None):
        grip_xy = grip_xy or grip_dist
        self.steps += 1
        cen = {}
        for n in self.BODIES:
            p, r = poses[n]
            c = list(p)
            if n == 'dish':                       # scorer's dish 'z' is its centre: bottom + half the height
                c = [p[k] + r[k][2] * D_HALF_H / 1000.0 for k in range(3)]
            cen[n] = self._card(c)
            self.hist[n].append(cen[n])
            if len(self.hist[n]) > self.window + 2:
                self.hist[n].pop(0)
        dp, dr = poses['dish']
        for n in self.TABLETS:
            tp = poses[n][0]
            rel = [tp[k] - dp[k] for k in range(3)]
            local = [sum(dr[k][j] * rel[k] for k in range(3)) for j in range(3)]   # dish frame: R^T (p - o)
            r_d = math.hypot(local[0], local[1]) * 1000.0
            if r_d > S['left_dish_radius_mm']:
                g = min(grip_xy(n))
                if spill_sample(r_d, g):
                    if len(self.spills) < 20:
                        self.spills.append((self.steps, n, round(r_d, 2), round(g, 2)))
                    self.spilled = True
        if read_robot:
            mins = [200.0, 200.0]
            for n in self.BODIES:
                lr = grip_dist(n)
                mins = [min(mins[0], lr[0]), min(mins[1], lr[1])]
            self.last_grip = mins
        tr, trr = poses['tray']
        jp, jr = poses['jar']
        jb = self._card(jp)        # the jar's origin is the centre of its base
        st = {
            'tray': {'x': cen['tray'][0], 'y': cen['tray'][1],
                     'yaw_deg': math.degrees(math.atan2(trr[1][0], trr[0][0]))},
            'holder': {'x': jb[0], 'y': jb[1], 'z_bottom': jb[2]},
            'dish': {'x': cen['dish'][0], 'y': cen['dish'][1], 'z': cen['dish'][2], 'disp_1s_mm': self._disp('dish')},
            'cell': {'x': cen['cell'][0], 'y': cen['cell'][1], 'z': cen['cell'][2], 'disp_1s_mm': self._disp('cell')},
            'tablets': [{'x': cen[n][0], 'y': cen[n][1], 'z': cen[n][2], 'disp_1s_mm': self._disp(n)}
                        for n in self.TABLETS],
            'gripper_min_dist_mm': list(self.last_grip),
            'holder_disp_1s_mm': self._disp('jar'),
            'spilled': self.spilled,
        }
        c = conditions(st)
        s = score(st)
        summary = dict(st)
        summary['spill_readings'] = list(self.spills)
        return dict(score=s, state=summary, conditions=c)


def explain(c, s):
    if c['floor']:
        return 'something on the floor (or not found)'
    if not c['released']:
        return 'not released (a gripper within 30 mm)'
    if not c['still']:
        return 'not still over the last 1 s'
    if not c['stowed']:
        return 'cell not stowed (or a tablet in the jar, or the jar not on the table)'
    spill = '; a tablet spilt (capped at 0.5)' if c['spilled'] else ''
    return f"stowed; {c['n_on_tray']} of {NT} served{spill} -> {s}"


if __name__ == '__main__':
    bad = 0
    R = rows()
    print(f'Negative-control table ({len(R)} rows): expected vs computed')
    for i, (name, st, exp, outline) in enumerate(R, 1):
        st.pop('_slots', None)
        set_outline(*(outline or (OUTLINE['x'], OUTLINE['y'])))
        got = score(st)
        set_outline(OUTLINE['x'], OUTLINE['y'])
        ok = abs(got - exp) < 1e-9
        bad += not ok
        print(f'{i:2d}. {"PASS" if ok else "FAIL"}  expected {exp:.2f}  computed {got:.2f}  {name}')
    print()
    print('Blind-policy sweep (fixed open-loop plans; draws span every range end to end)')
    draws = blind_draws()
    for k, d in enumerate(draws, 1):
        print(f'  draw {k:2d}: cell slot {d["cell"]}, empty {d["empty"]}, phi {d["phi"]:.1f}, dish {d["dish"]}, tray {d["tray"]}, '
              f'holder {d["holder"]}')
    means = {}
    for p in PLANS:
        sc = [score(plan_end(p, d)) for d in draws]
        means[p] = sum(sc) / len(sc)
        print(f'  {p}: ' + ' '.join(f'{s:.2f}' for s in sc) + f'  mean {means[p]:.3f}')
    best = max(means.values())
    ok = best <= P['blind_sweep']['mean_limit']
    bad += not ok
    print(f'  best fixed-plan mean {best:.3f} (limit {P["blind_sweep"]["mean_limit"]}): {"PASS" if ok else "FAIL"}')
    # start scene scores 0 on every draw
    z = all(score(start_state(d) | {}) == 0.0 for d in draws)
    bad += not z
    print(f'  start scene scores 0 on all {len(draws)} draws: {"PASS" if z else "FAIL"}')
    f = all(score(full_state(d)) == 1.0 for d in draws)
    bad += not f
    print(f'  full end state scores 1.0 on all {len(draws)} draws: {"PASS" if f else "FAIL"}')
    print()
    print(f'{len(R)} rows + 3 sweep checks: {"ALL PASS" if not bad else str(bad) + " FAILED"}')
    sys.exit(1 if bad else 0)
