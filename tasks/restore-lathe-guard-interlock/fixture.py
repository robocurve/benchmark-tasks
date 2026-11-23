"""Station coordinates in metres, referenced to the table. The spindle axis is Y."""
from math import radians, sin, cos, pi

WORLD = (.305, 0., .7475)
SIDES = ("left", "right")
TITLE = "Recommission the lathe chuck guard"
INSTRUCTION = "Maintenance is complete. Return the lathe chuck station to normal service."
MODES = ("reference","faulted_start","idle","remove_only","close_only","misplaced_insert","no_open_check",
         "unlatched","overpress","held","park_closed","operator_touch","slow_reference")
SHIELD_UP = radians(55)
KEY = (-.100,.089,.091)
TRAY = (-.113,.215,.068)
KEY_SIZE = (.023,.026,.030)
PAD_CENTER = (KEY[0], KEY[1], .069)
PAD_SIZE = (.040, .040, .012)
BUTTON = (.010,.145,.080)
PIVOT = (.058,0.,.112)
HANDLE = (-.131,-.068,.047)
CAM = (-.082,-.122,.163)
STEEL=(.46,.54,.60)
DARK=(.09,.13,.16)
BLUE=(.14,.34,.53)
WHITE=(.92,.91,.85)
YELLOW=(.97,.76,.06)
RED=(.85,.12,.08)
GREEN=(.14,.77,.32)
SKIN=(.66,.38,.23)
BODIES=[]; SHAPES=[]; PROPS={}
def body(name,parent,xyz,joint=None,kind=None,axis=(0,0,1),bounds=(0,0),mass=.1,k=0.,d=0.,com=(0.,0.,0.),diag=(.0001,.0001,.0001)):
    BODIES.append((name,parent,xyz,joint,kind,axis,bounds,mass,k,d))
    PROPS[name]=(com,diag)
def inertia(name): return PROPS[name]
def shape(name,owner,pos,size,color,kind="box"):
    SHAPES.append((name,owner,pos,size,color,kind))
def collision_allowed(a,b):
    return frozenset((a,b)) in ({"shield","cam"},{"shield","seatpad"})
def handle_at(a):
    x,y,z=HANDLE
    return (PIVOT[0]+x*cos(a)+z*sin(a),y,PIVOT[2]-x*sin(a)+z*cos(a))
body("frame",None,(0,0,0),mass=12)
body("shield","frame",PIVOT,"shield_angle","hinge",(0,1,0),(-.012,SHIELD_UP+.018),.32,0.,.095,com=(-.012,0,.022),diag=(.0012,.0025,.0022))
body("cam","frame",CAM,"cam_angle","hinge",(0,0,1),(-.05,1.64),.038,.055,.018,diag=(.000075,.000075,.000080))
body("rotor","frame",(-.011,0,.095),"shaft","hinge",(0,1,0),(-1e5,1e5),.24,0.,.008,diag=(.00021,.00032,.00021))
body("button","frame",BUTTON,"button_z","slide",bounds=(-.012,0),mass=.008,k=310,d=.7)
body("testpad","frame",PAD_CENTER,"pad_z","slide",bounds=(-.010,0),mass=.002,k=65,d=.38)
body("seatpad","frame",(-.104,-.044,.085),"seat_z","slide",bounds=(-.012,0),mass=.002,k=28,d=.3)
shape("tongue_bracket","shield",(-.147,-.044,.005),(.040,.012,.008),STEEL)
shape("tongue_stem","shield",(-.162,-.044,-.010),(.007,.012,.030),STEEL)
shape("guard_tongue","shield",(-.162,-.044,-.026),(.018,.018,.008),STEEL)
shape("seat_plunger","seatpad",(0,0,0),(.020,.027,.012),DARK)
shape("seat_indicator_green","seatpad",(0,.019,.012),(.014,.006,.006),GREEN,"guide_visual")
shape("seat_indicator_red","seatpad",(0,.019,.004),(.014,.006,.006),RED,"guide_visual")
shape("switch_index","frame",(-.119,-.025,.089),(.012,.009,.002),WHITE,"guide_visual")
shape("seat_switch_case","frame",(-.104,-.044,.043),(.032,.034,.049),DARK)
shape("lathe_bed","frame",(.012,.016,.020),(.169,.254,.037),BLUE)
shape("headstock","frame",(-.010,.065,.067),(.113,.051,.106),BLUE)
shape("apron","frame",(-.082,-.019,.043),(.035,.060,.048),BLUE)
shape("handwheel","frame",(-.110,-.018,.053),(.031,.013,.031),DARK,"cylinder")
for y in (-.064,.069):
    shape("hinge_cheek"+str(int(y*1000)).replace("-","n"),"frame",(.062,y,.079),(.029,.014,.068),STEEL)
shape("clear_canopy","shield",(-.064,0,.055),(.134,.123,.006),(.55,.77,.84),"clear")
shape("clear_front","shield",(-.128,0,.005),(.006,.123,.094),(.55,.77,.84),"clear")
for y,n in ((-.060,"r"),(.060,"l")):
    shape("roof_rail_"+n,"shield",(-.062,y,.056),(.140,.007,.010),STEEL)
    shape("face_rail_"+n,"shield",(-.130,y,.006),(.008,.007,.102),STEEL)
shape("front_sill","shield",(-.130,0,-.044),(.008,.126,.009),STEEL)
shape("guard_grip","shield",HANDLE,(.024,.031,.020),BLUE)
shape("keeper","shield",(-.137,-.083,.039),(.020,.034,.011),STEEL)
shape("counterweight","shield",(.043,0,.005),(.049,.082,.038),DARK)
shape("cam_pedestal","frame",(CAM[0],CAM[1],.072),(.031,.033,.144),BLUE)
shape("cam_blade","cam",(.014,0,0),(.037,.012,.006),STEEL)
shape("cam_spacer","cam",(0,0,.024),(.011,.011,.050),STEEL)
shape("cam_top","cam",(.016,0,.050),(.045,.010,.006),STEEL)
shape("cam_knob","cam",(.032,0,.061),(.016,.016,.019),RED)
shape("spindle_face","rotor",(0,0,0),(.101,.050,.101),STEEL,"cylinder")
for i in range(3):
    a=i*2*pi/3
    shape("jaw"+str(i),"rotor",(.032*cos(a),-.026,.032*sin(a)),(.017,.017,.015),DARK)
# Painted indices on the face and rim. Asymmetry distinguishes successive sectors.
MARK_ANGLES = {}
for label, angle, width, length, color in (
        ("red", 12., .017, .038, (.96, .045, .035)),
        ("white", 141., .010, .024, (.99, .99, .99))):
    a = radians(angle)
    for surface, radius, depth, height in (("face", .047-length/2, .001, length),
                                           ("rim", .0506, .048, .001)):
        name = "index_" + label + "_" + surface
        MARK_ANGLES[name] = angle
        shape(name, "rotor", (radius*sin(a), -.026 if surface=="face" else 0., radius*cos(a)),
              (width, depth, height), color, "guide_visual")
shape("switch_box","frame",(-.100,.089,.030),(.056,.055,.056),DARK)
shape("diagnostic_pad","testpad",(0,0,0),PAD_SIZE,RED)
# The insert sits freely in a shallow locating pocket on the moving pad.
# Four low walls limit planar creep while leaving the upper grasp faces exposed.
for offset, side in ((-1, "minus"), (1, "plus")):
    shape("pad_guide_x_" + side, "testpad", (offset*.0145,0,.008),
          (.002,.034,.004), STEEL)
    shape("pad_guide_y_" + side, "testpad", (0,offset*.016,.008),
          (.027,.002,.004), STEEL)

shape("green_start","button",(0,0,0),(.032,.032,.012),GREEN)
shape("start_station","frame",(BUTTON[0],BUTTON[1],.033),(.054,.050,.058),DARK)
shape("tool_tray","frame",(TRAY[0],TRAY[1],.032),(.085,.083,.041),BLUE)
for y,n in ((-.040,"r"),(.040,"l")):
    shape("tray_lip_"+n,"frame",(TRAY[0],TRAY[1]+y,.057),(.087,.005,.012),STEEL)
for x, end in ((-.040,"minus"),(.040,"plus")):
    shape("tray_end_"+end,"frame",(TRAY[0]+x,TRAY[1],.0585),(.005,.079,.015),STEEL)
# Translate the entire stationary operator in the table frame. The hand rests
# on a separate support outboard of the machine, with fingers visible and free.
OPERATOR_X = -.070
PALM_CENTER = (-.186 + OPERATOR_X, .018, .058)
def operator_part(name, center, size, color):
    x, y, z = center
    shape(name, "frame", (x+OPERATOR_X, y, z), size, color, "ellipsoid")

shape("palm_rest", "frame", (PALM_CENTER[0], PALM_CENTER[1], .01925),
      (.060, .084, .0385), (.34, .39, .35))
operator_part("operator_torso",(-.77,.055,.175),(.25,.39,.55),(.94,.54,.055))
operator_part("operator_head",(-.767,.055,.556),(.177,.151,.220),SKIN)
operator_part("operator_visor",(-.686,.054,.584),(.028,.144,.042),DARK)
operator_part("operator_upper_arm",(-.545,.041,.104),(.280,.085,.087),BLUE)
operator_part("operator_forearm",(-.338,.026,.063),(.221,.072,.071),BLUE)
operator_part("operator_cuff",(-.245,.025,.063),(.041,.064,.064),WHITE)
shape("operator_palm", "frame", PALM_CENTER, (.094,.076,.039), SKIN, "ellipsoid")
for i,l in enumerate((.067,.071,.066,.054)):
    operator_part("operator_digit_"+str(i),(-.156+l/2,.042-i*.017,.052),(l,.015,.020),SKIN)
operator_part("operator_thumb",(-.182,-.022,.051),(.054,.024,.024),SKIN)

# Lower body extends from the apron down to the floor at table height minus 0.75 m.
operator_part("operator_hips",(-.805,.055,-.088),(.22,.32,.23),DARK)
for y,side in ((-.036,"near"),(.146,"far")):
    operator_part("operator_leg_"+side,(-.810,y,-.400),(.159,.145,.64),DARK)
    operator_part("operator_boot_"+side,(-.770,y,-.692),(.239,.133,.103),(.13,.11,.10))
