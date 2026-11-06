"""Permanent-ink attacks and all sixteen printed games. CPU only."""
from pathlib import Path
from dataclasses import replace
import sys
import types
import math
import numpy as np
import pytest
from yam_arena.task_file import load

MODULE = next(p for p in Path(__file__).parent.glob('*.py')
              if p.name not in ('ink.py','yam_parts.py','yam_joints.py') and not p.name.startswith('test_'))
T = load(MODULE).module


@pytest.fixture(autouse=True)
def isolated_episode():
    saved = dict(T._RUN)
    T._RUN.update(env=None, t=None, step=0, episode=T._RUN['episode']+1)
    T.TEST_BOARD, T.TEST_STROKES, T.TEST_GRIPPERS = None, None, None
    T.reset_path()
    yield
    T.TEST_BOARD, T.TEST_STROKES, T.TEST_GRIPPERS = None, None, None
    T._RUN.clear(); T._RUN.update(saved)
    T.reset_path()


def hung():
    return T.PEN.offline_state(np.asarray(T.PEN.pos)-[0,0,.001],0,{})


def score(strokes, board=0, pen=None):
    T.TEST_BOARD, T.TEST_STROKES = T.LAYOUTS[board], strokes
    return T.score({'pen':hung() if pen is None else pen})


@pytest.mark.parametrize('index', range(16))
def test_every_board_and_wrong_answer_transfer(index):
    strokes = T.nominal_strokes(index)
    symbol, cell = T.winning_move(T.LAYOUTS[index])
    assert T.classify_cell(strokes,cell)[0] == symbol
    assert score(strokes,index) == 1.
    for other in range(16):
        if T.winning_move(T.LAYOUTS[other]) != (symbol,cell):
            assert score(strokes,other) == 0., (index,other)


@pytest.mark.parametrize('index',range(len(T.cheat_cases())), ids=[x[0] for x in T.cheat_cases()])
def test_all_requested_cheats(index):
    _,board,strokes = T.cheat_cases()[index]
    assert score(strokes,board) == 0.


def test_layout_requirements():
    rows, coverage = T.layout_self_check()
    assert len(rows) == 16 and coverage == [2]*8


def test_end_state_tiers_release_and_motion():
    strokes = T.nominal_strokes(0)
    lying = T.PEN.offline_state((*T.BOARD_XY,T.TOP+.015),math.pi/2,{})
    assert score(strokes,pen=lying) == .6
    assert score(strokes,pen=replace(hung(),lin_vel=np.array([.05,0,0]))) == .8
    assert score(strokes,pen=replace(hung(),ang_vel=np.array([.06,0,0]))) == .8
    assert score(strokes,pen=replace(hung(),joint_vel={'nib':.06})) == .8
    T.TEST_GRIPPERS = {'right':(np.array([*T.STAND_XY,T.GRASP_TCP]),0.)}
    assert score(strokes) == .8
    T.TEST_GRIPPERS = {'right':(np.array([*T.STAND_XY,T.GRASP_TCP]),1.)}
    assert score(strokes) == .8  # Opening alone is not release.
    T.TEST_GRIPPERS = {'left':(np.array([*T.STAND_XY,T.GRASP_TCP]),0.),
                       'right':(np.array([.155,-.385,T.TOP+.170]),1.)}
    assert score(strokes) == .8
    T.TEST_GRIPPERS = None
    assert score(strokes) == 1.
    assert score(strokes,pen=lying) == .6  # Partial credit is not remembered.
    off = T.PEN.offline_state((.8,-.8,T.TOP-.1),0,{})
    assert score(strokes,pen=off) == .6  # The permanent correct mark remains.


@pytest.mark.parametrize('dy,expected', [
    (.008, 1.),
    (T.FORK_TIP_Y+T.COLLAR_RADIUS+.001, .6),
    (T.HANG_Y_MIN, 1.),
], ids=['8 mm further out', 'past prong tips unsupported', 'against rear plate'])
def test_requested_hanging_positions(dy, expected):
    origin = np.asarray(hung().origin)+[0,dy,0]
    pen = T.PEN.offline_state(origin,0,{})
    assert score(T.nominal_strokes(0),pen=pen) == expected


@pytest.mark.parametrize('dx', [0., -.0035, .0035])
def test_hanging_support_boundaries(dx):
    for dy,expected in [(T.HANG_Y_MIN,1.), (T.hang_y_max(dx),1.),
                        (T.HANG_Y_MIN-.0001,.6), (T.hang_y_max(dx)+.0001,.6)]:
        pen = T.PEN.offline_state(np.asarray(hung().origin)+[dx,dy,0],0,{})
        assert score(T.nominal_strokes(0),pen=pen) == expected
    pen = T.PEN.offline_state(np.asarray(hung().origin)+[.0036,0,0],0,{})
    assert score(T.nominal_strokes(0),pen=pen) == .6


def test_classify_diagnostics_simulator_only_once_per_40_steps(capsys):
    assert score(T.nominal_strokes(0)) == 1.
    assert capsys.readouterr().out == ''
    T.TEST_GRIPPERS = {}
    T._RUN.update(env=object(),t=2.,step=40)
    assert score(T.nominal_strokes(0)) == .8
    assert score(T.nominal_strokes(0)) == .8
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 1
    assert all(s in lines[0] for s in ('CLASSIFY', 'board=0', 'symbol=O', 'cell=(0, 0)',
                                      'glyph=O', 'details=', 'outside_mm=0.000'))
    T._RUN.update(t=2.05,step=41)
    assert score(T.nominal_strokes(0)) == .8
    assert capsys.readouterr().out == ''
    T._RUN.update(env=None,t=4.,step=80)
    assert score(T.nominal_strokes(0)) == .8
    assert capsys.readouterr().out == ''


def test_sticky_violate_restore_and_new_episode():
    T._RUN.update(t=0.,step=0)
    assert score(T.nominal_strokes(0)) == .8
    T._RUN.update(t=.05,step=1)
    assert score(T.cheat_cases()[6][2]) == 0.
    for step in range(2,30):
        T._RUN.update(t=step*.05,step=step)
        assert score(T.nominal_strokes(0)) == 0.
    T._RUN.update(t=None,episode=T._RUN['episode']+1)
    assert score(T.nominal_strokes(0)) == 1.


def test_timer_duplicate_calls_interruptions_and_skipped_steps():
    T._RUN.update(t=0.,step=0)
    for step in range(24):
        T._RUN.update(t=step*.05,step=step)
        v = score(T.nominal_strokes(0))
        assert v == (.8 if step < 20 else 1.)
        assert score(T.nominal_strokes(0)) == v
    moving = replace(hung(),lin_vel=np.array([.05,0,0]))
    T._RUN.update(t=1.2,step=24)
    assert score(T.nominal_strokes(0),pen=moving) == .8
    T._RUN.update(t=1.25,step=25)
    assert score(T.nominal_strokes(0)) == .8
    T._RUN.update(t=3.,step=60)
    assert score(T.nominal_strokes(0)) == .8


def test_pose_only_path_and_episode_reset():
    T._RUN.update(t=0.,step=0)
    symbol,cell = T.winning_move(T.LAYOUTS[0])
    xy = T.cell_center(cell)
    def physical(xy,q=.003):
        return T.PEN.offline_state((*xy,T.PAPER_TOP+.135-q),0,{'nib':q})
    state = physical(xy)
    assert T.writing(state)
    T.score({'pen':state}); T.score({'pen':state})
    assert len(T._STROKES) == 1 and len(T._STROKES[0]) == 1
    T._RUN.update(t=.05,step=1); T.record_path(physical(xy+[.001,0]))
    assert len(T._STROKES[0]) == 2
    T._RUN.update(t=.1,step=2); T.record_path(physical(xy,q=0))
    T._RUN.update(t=.15,step=3); T.record_path(physical(xy+[.005,0]))
    assert len(T._STROKES) == 2
    T._RUN['episode'] += 1; T.record_path(hung())
    assert not T._STROKES


@pytest.mark.parametrize('index',range(16))
def test_calibration_margin_and_retracing(index):
    symbol,cell = T.winning_move(T.LAYOUTS[index])
    paths = T.nominal_strokes(index)
    a = math.radians(5)
    rot = np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    c = T.cell_center(cell)
    shifted = [(p-c)@rot.T+c+[.003,-.002] for p in paths]
    assert score(shifted,index) == 1.
    if symbol == 'O':
        assert score(T.strokes_o(cell,.0147,378),index) == 1.
        assert score(T.strokes_o(cell,.006),index) == 0.
        assert score(T.strokes_o(cell,.023),index) == 1.  # Inside the zone.
        assert score(T.strokes_o(cell,.026),index) == 0.  # Ink outside the zone.
        assert score(T.strokes_o(cell,.015,250),index) == 0.
        # Retracing reads as one O in a photo, so it is accepted.
        assert score([np.vstack((paths[0],paths[0][::-1]))],index) == 1.
    else:
        assert score(paths[:1],index) == 0.
        assert score([np.vstack((p,p[::-1])) for p in paths],index) == 1.
        assert score(T.strokes_x(cell,.056),index) == 0.  # Ink outside the zone.
        plus = [np.array([c-ax*.015,c+ax*.015]) for ax in np.eye(2)]
        assert score(plus,index) == 0.


def test_deferred_plan_survives_reset_selection():
    T.TEST_BOARD = T.LAYOUTS[0]
    plan = T.demo()['right']
    durations = [w.duration for w in plan]
    for board in T.LAYOUTS:
        T.TEST_BOARD = board
        spec = T._drawing_spec(board)
        assert [w.duration for w in plan] == durations
        assert np.allclose([w.pos for w in plan[6:6+len(spec)]],[p for _,p,_ in spec])
    assert sum(durations)+T.SUCCESS_HOLD_S < .6*T.RUN_LIMIT_STEPS/20


def test_seed_reset_and_visuals_before_first_observation():
    """Recording USD adapter has no physics APIs; reset observes prints in-place."""
    import ink
    import yam_joints
    from types import SimpleNamespace as NS
    from contextlib import redirect_stdout
    from io import StringIO
    writes,prims = [],{}
    class Attr:
        def __init__(self,path,name): self.path,self.name=path,name
        def Set(self,value):
            prims[self.path].values[self.name]=value
            writes.append((self.path,self.name,value))
        def SetTargets(self,value): self.Set(value)
        def ConnectToSource(self,*value): self.Set(value)
    class Prim:
        def __init__(self,path,kind): self.path,self.kind,self.values=path,kind,{}
        def GetPrim(self): return self
        def GetPath(self): return self.path
        def GetStage(self): return stage
        def ConnectableAPI(self): return self
        def Bind(self,mat): pass
        def CreateInput(self,name,kind): return Attr(self.path,'input:'+name)
        def __getattr__(self,name):
            if name.startswith(('Create','Add')): return lambda *a: Attr(self.path,name)
            if name=='GetInvisibleIdsAttr': return lambda: Attr(self.path,'CreateInvisibleIdsAttr')
            if name=='GetVisibilityAttr': return lambda: Attr(self.path,'CreateVisibilityAttr')
            raise AttributeError(name)
    def schema(kind):
        return NS(Define=lambda stage,path: prims.setdefault(path,Prim(path,kind)))
    stage=NS(GetPrimAtPath=lambda path:prims[path])
    geom=NS(**{k:schema(k) for k in ('Xform','Cylinder','Cube','PointInstancer')},
            Imageable=lambda p:p,Tokens=NS(inherited='inherited',invisible='invisible'))
    shade=NS(Material=schema('Material'),Shader=schema('Shader'),MaterialBindingAPI=NS(Apply=lambda p:p))
    pxr=types.ModuleType('pxr')
    pxr.Gf=NS(Vec3f=lambda *a:tuple(a),Vec3d=lambda *a:tuple(a))
    pxr.Sdf=NS(ValueTypeNames=NS(Color3f='Color3f',Float='Float'))
    pxr.UsdGeom,pxr.UsdShade=geom,shade
    pxr.Vt=NS(IntArray=list,Int64Array=list,Vec3fArray=list)
    old_cfg=ink._CFG;old_roots=ink._ROOTS.copy();old_run=dict(T._RUN);old_board=T._CURRENT_BOARD
    old_test_board=T.TEST_BOARD
    def check_prints():
        record=ink._ROOTS['/sheet']
        board=T.current_board()
        for (cell,symbol),im in record['prints'].items():
            assert (im.values['CreateVisibilityAttr']=='inherited') == (board[cell[0]*3+cell[1]]==symbol)
    class Embodiment:
        def reset(self,scene=None,*,seed=None):
            if not ink._ROOTS:
                ink.spawn_visuals(stage,'/sheet')
            check_prints()  # The initial image is produced HERE, inside original reset.
            assert not T._STROKES and not ink._ROOTS['/sheet']['visible'].any()
            T._RUN['episode']+=1
            return 'first observation'
    fake=types.ModuleType('yam_arena.inspect_embodiment');fake.YamArenaEmbodiment=Embodiment
    mods={'pxr':pxr,'yam_arena.inspect_embodiment':fake}
    old={k:sys.modules.get(k) for k in mods}
    try:
        sys.modules.update(mods);ink._ROOTS.clear()
        T.TEST_BOARD=None
        ink.install(T)
        with redirect_stdout(StringIO()):
            for seed in (None,*range(32),-1):
                T._STROKES.append([np.zeros(2)])
                assert Embodiment().reset(seed=seed)=='first observation'
                assert T.current_board()==T.LAYOUTS[0 if seed is None else seed%16]
                assert not T._STROKES
        record=ink._ROOTS['/sheet']
        assert len(record['grid'])==3969 and len(record['prints'])==18
        assert sum(p.kind=='Cube' for p in prims.values())==598
        count=len(prims);writes.clear()
        symbol,cell=T.winning_move(T.current_board());a=T.cell_center(cell);b=a+[.005,0]
        def physical(xy,q=.003):
            return T.PEN.offline_state((*xy,T.PAPER_TOP+.135-q),0,{'nib':q})
        T._RUN.update(t=.05,step=1)
        with redirect_stdout(StringIO()):ink.update(physical(a))
        n=len(writes)
        ink.update(physical(a));assert len(writes)==n
        T._RUN['step']+=1;ink.update(physical(b))
        expected=ink.near_segment(record['grid'],a,b,T.DOT_R)
        assert np.array_equal(record['visible'],expected)
        assert len(prims)==count
        assert all(name in ('CreateInvisibleIdsAttr','CreateVisibilityAttr') for _,name,_ in writes)
        T._RUN['step']+=1;ink.update(physical(b,q=0))
        c=T.cell_center(T.wrong_cell(T.LAYOUTS.index(T.current_board())))
        T._RUN['step']+=1;ink.update(physical(c))
        assert np.array_equal(record['visible'],expected|ink.near_segment(record['grid'],c,c,T.DOT_R))
        with redirect_stdout(StringIO()):Embodiment().reset(seed=8)
        assert not record['visible'].any() and len(prims)==count
    finally:
        for k,v in old.items():
            if v is None:sys.modules.pop(k,None)
            else:sys.modules[k]=v
        ink._ROOTS.clear();ink._ROOTS.update(old_roots);ink._CFG=old_cfg
        T._RUN.clear();T._RUN.update(old_run)
        T._CURRENT_BOARD=old_board;T.TEST_BOARD=old_test_board;T.reset_path()


def test_tiny_stroke_scribble_cannot_hide_beside_an_x():
    index=8
    _,cell=T.winning_move(T.LAYOUTS[index]);c=T.cell_center(cell)
    dots=[np.array([c+[x,y],c+[x+.0008,y]])
          for x in np.linspace(-.018,.018,20) for y in np.linspace(-.018,.018,20)]
    assert score(T.nominal_strokes(index)+dots,index)==0.


def test_zone_segment_clipping_and_numeric_boundary():
    _,cell=T.winning_move(T.LAYOUTS[0]);c=T.cell_center(cell)
    h=T.CELL_SIZE/2+T.ZONE_EXPAND
    line=[np.array([c+[h-.002,0],c+[h+.002,0]])]
    # A 4 mm touch counts as 4.4 mm (one ink-disc diameter), shared pro rata.
    assert T.outside_length(line,cell)==pytest.approx(.0022)
    assert T.path_length(T.clip_paths(line,c-h,c+h)[0])==pytest.approx(.002)
    assert T.outside_length(T.dabs([c+[h+.003,0]]),cell)==pytest.approx(T.INK_DOT)
    assert T.outside_length([np.array([c+[h+.003,0]]*5)],cell)==pytest.approx(T.INK_DOT)
    m=T.nominal_strokes(0)
    edge=[np.array([c+[h-.004,0],c+[h+.001,0]])]  # 5 mm touch, exactly 1 mm outside
    assert T.outside_length(edge,cell)==pytest.approx(.001)
    assert score(m+edge)==1.  # Strictly more than 1 mm is forbidden.
    assert score(m+[edge[0]+[.0001,0]])==0.
    extra=[np.array([T.cell_center(T.wrong_cell(0)),T.cell_center(T.wrong_cell(0))+[.001,0]])]
    assert score(m+extra)==0.  # A 1 mm touch counts 4.4 mm.
