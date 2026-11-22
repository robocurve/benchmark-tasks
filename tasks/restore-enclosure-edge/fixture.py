"""Bench dimensions in metres. X points away from the operator; Z points up."""
WORLD = (.305, 0., .7475)
SIDES = ("left", "right")
SLUG = "restore-enclosure-edge"
TITLE = "Return a powered enclosure guard to service"
INSTRUCTION = "Repair the detached edge and leave the enclosure guard ready for operation."
MODES = ("reference", "idle", "repair_only", "empty_cycle", "displaced_empty_cycle", "unrepaired_probe", "leave_probe", "slow_reference", "operator_touch")
GRAY = (.55,.59,.63)
DARK = (.08,.11,.13)
RUBBER = (.12,.15,.18)
ORANGE = (.98,.37,.035)
GLOVE = (.97,.82,.12)
BLUE = (.15,.31,.55)
RED = (.89,.06,.045)
GREEN = (.12,.82,.33)
WHITE = (.93,.94,.90)
BODIES, SHAPES, PROPS = [], [], {}
def body(name,parent,xyz,joint=None,kind=None,axis=(0,0,1),bounds=(0,0),mass=.1,k=0.,d=0.,diag=(.0001,.0001,.0001)):
    BODIES.append((name,parent,xyz,joint,kind,axis,bounds,mass,k,d))
    PROPS[name]=((0.,0.,0.),diag)
def inertia(name): return PROPS[name]
def shape(name,owner,pos,size,color,kind="box"):
    SHAPES.append((name,owner,pos,size,color,kind))
def collision_allowed(a,b):
    return frozenset((a,b)) in ({"bulb","probe"},{"door","probe"},{"frame","probe"})
body("frame",None,(0,0,0),mass=5.)
body("door","frame",(0,0,.188),"height","slide",bounds=(-.122,0),mass=.55,k=13000,d=115,diag=(.004,.0004,.004))
body("sensor","door",(0,0,0),"switch_travel","slide",bounds=(0,.012),mass=.025,k=8650,d=14.)
body("bulb","sensor",(0,0,0),"foot_roll","hinge",(0,1,0),(-1.25,.02),.035,4.5,.018)
body("sled","frame",(-.104,-.073,0),"insertion","slide",(1,0,0),(0,.115),.18,0,5.)
body("probe","sled",(0,0,.095),"pointer","slide",bounds=(-.038,0),mass=.021,k=2100,d=8.)
body("button","frame",(-.133,.160,.076),"button_z","slide",bounds=(-.013,0),mass=.022,k=580,d=1.6)
body("flag","frame",(-.007,.221,.156),"trip_flag","slide",bounds=(0,.034),mass=.018,k=800,d=6.)
for y in (-.177,.177):
    s="l" if y>0 else "r"
    shape("foot_"+s,"frame",(.031,y,.012),(.13,.049,.024),DARK)
    shape("post_"+s,"frame",(.043,y,.125),(.0381,.0381,.226),GRAY)
    shape("slot_"+s,"frame",(.0235,y,.132),(.002,.006,.208),DARK,"guide_visual")
    shape("slide_"+s,"door",(.014,y,.008),(.034,.021,.036),GRAY)
shape("beam","door",(.008,0,.012),(.027,.300,.025),GRAY)
shape("exposed_lip","door",(.009,0,-.006),(.016,.300,.011),GRAY)
shape("channel_front","door",(-.007,0,-.001),(.004,.300,.008),DARK)
# Adjacent retained sections show the nominal cross section. Only the vulnerable span folds.
for name,y,length in (("north",.077,.143),("south",-.139,.020)):
    shape("retained_"+name,"door",(-.001,y,-.019),(.024,length,.038),RUBBER)
shape("rubber_crown","bulb",(-.029,-.061,-.010),(.018,.128,.018),RUBBER)
shape("clip_web","bulb",(-.0145,-.061,-.005),(.030,.128,.009),RUBBER)
shape("seat_marker","bulb",(-.034,.002,-.010),(.005,.005,.014),ORANGE,"guide_visual")
for i in range(11):
    shape("hazard_mark_"+str(i),"door",(-.0065,-.14+i*.027,.012),(.002,.015,.012),ORANGE,"guide_visual")
shape("track","frame",(-.048,-.076,.013),(.179,.080,.022),GRAY)
shape("base_plate","sled",(0,0,.028),(.069,.081,.011),DARK)
shape("handle_ear","sled",(0,-.185,.041),(.019,.037,.072),ORANGE)
shape("extension_bar","sled",(0,-.101,.028),(.019,.172,.012),GRAY)
# Spring-return instrument with a replaceable 50 mm OD contact jacket.
shape("probe_crown","probe",(0,0,0),(.050,.058,.050),ORANGE,"cylinder")
shape("plunger_stem","probe",(0,0,-.025),(.012,.012,.014),GRAY)
for y in (-.025,.025):
    shape("probe_band_"+("l" if y>0 else "r"),"probe",(0,y,0),(.0506,.003,.0506),WHITE,"band_visual")
shape("scale_face","sled",(-.045,-.072,.079),(.003,.041,.060),DARK,"guide_visual")
for i in range(8):
    shape("scale_"+str(i),"sled",(-.047,-.072,.095-i*.005),(.001,.033 if i%2==0 else .020,.0016),WHITE,"guide_visual")
shape("pointer_arm","probe",(-.023,-.031,0),(.046,.004,.003),RED,"guide_visual")
shape("pointer_rod","probe",(-.046,-.048,0),(.003,.036,.003),RED,"guide_visual")
shape("pointer_head","probe",(-.049,-.070,0),(.003,.033,.003),RED,"guide_visual")
shape("push_station","frame",(-.133,.160,.034),(.063,.061,.056),GRAY)
shape("cycle_button","button",(0,0,0),(.036,.036,.018),GREEN)
shape("indicator_box","frame",(-.005,.221,.136),(.041,.034,.049),DARK)
shape("trip_blade","flag",(0,0,-.013),(.023,.023,.032),RED)
shape("level_stripe","frame",(.021,.172,.193),(.003,.031,.005),WHITE,"guide_visual")

# Fixed operator stand-in on the loading shelf. No skin collider is on the carriage.
shape("loading_shelf","frame",(-.188,.065,.009),(.318,.093,.018),GRAY)
shape("operator_sleeve","frame",(-.276,.065,.050),(.183,.065,.064),BLUE,"ellipsoid")
shape("operator_sleeve_band","frame",(-.258,.065,.050),(.016,.068,.066),WHITE,"ellipsoid")
shape("operator_cuff","frame",(-.189,.065,.045),(.050,.061,.052),GLOVE,"ellipsoid")
shape("operator_palm","frame",(-.138,.065,.041),(.088,.074,.043),GLOVE,"ellipsoid")
for i,length in enumerate((.068,.078,.073,.057)):
    shape("operator_digit_"+str(i),"frame",(-.108+length/2,.090-i*.017,.032),
          (length,.015,.026),GLOVE,"ellipsoid")
shape("operator_thumb","frame",(-.123,.024,.036),(.057,.025,.027),GLOVE,"ellipsoid")
