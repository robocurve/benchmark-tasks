"""Joint-space YAM demonstrations synthesized from physical tool waypoints."""
from math import radians, cos, sin
from fixture import WORLD, KEY, TRAY, PIVOT, CAM, BUTTON, SHIELD_UP, handle_at, MODES, PALM_CENTER
from yam_arena.constants import FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED
from yam_arena.demo import Waypoint

def plan(mode="reference"):
    if mode not in MODES:raise ValueError(mode)
    left=[];right=[]
    if mode=="idle":return {}
    slow=1.17 if mode=="slow_reference" else 1.
    lp=(-.11,.277,.188);rp=(-.14,-.270,.177)
    def add(side,seconds,p,f,label,yaw=0):
        side.append(Waypoint(seconds*slow,tuple(a+b for a,b in zip(p,WORLD)),f,label,radians(yaw)))
    def move(side,seconds,x,y,z,f,label,yaw=0):add(side,seconds,(x,y,z),f,label,yaw)
    def button():
        x,y,z=BUTTON
        move(left,1.65,x,y,z+.060,CLOSED,"approach start")
        move(left,.70,x,y,z-.006,CLOSED,"press start")
        move(left,.25,x,y,z-.006,CLOSED,"hold start button")
        move(left,.80,x,y,z+.061,OPEN,"release start")
    if mode=="operator_touch":
        x, y, z = PALM_CENTER
        move(left,2.,x,y,z+.082,OPEN,"approach operator")
        move(left,2.,x,y,z-.009,CLOSED,"contact supervisor surrogate")
        return {"left":left}
    if mode=="faulted_start":
        button()
        return {"left":left}
    if mode!="close_only":
        x,y,z=KEY
        move(left,1.8,x,y,.156,OPEN,"approach yellow service insert")
        move(left,1.,x,y,.080,OPEN,"straddle insert")
        move(left,.7,x,y,.080,CLOSED,"pinch yellow insert")
        move(left,.95,x,y,.146,CLOSED,"lift from test socket")
        if mode=="misplaced_insert":
            move(left,1.,-.100,.173,.158,CLOSED,"clear raised guard")
            move(left,1.5,.052,.170,.151,CLOSED,"carry above front of machine")
            move(left,.8,.052,.113,.145,CLOSED,"carry insert above machine bed")
            move(left,1.,.052,.113,.079,CLOSED,"approach the bed")
            move(left,.6,.052,.113,.079,OPEN,"leave loose insert in machine")
            add(left,1.2,lp,OPEN,"withdraw")
            return {"left":left}
        move(left,1.35,TRAY[0],TRAY[1],.145,CLOSED,"transfer to tray")
        move(left,.85,TRAY[0],TRAY[1],.072,CLOSED,"lower to tray")
        move(left,.45,TRAY[0],TRAY[1],.072,OPEN,"release insert")
        move(left,.65,TRAY[0],TRAY[1],.144,OPEN,"clear tray")
    if mode=="remove_only":
        add(left,1.2,lp,OPEN,"park");return {"left":left}
    if mode not in ("close_only","no_open_check"):button()
    # The shield stays raised throughout the open-guard inhibit check.
    left_end=sum(p.duration for p in left)/slow
    add(right,max(11.75,left_end+.6),rp,OPEN,"wait outside guard")
    def h(a,dx,dz):x,y,z=handle_at(a);return (x+dx,y,z+dz)
    add(right,1.2,(.020,-.150,.300),CLOSED,"approach above hinge")
    add(right,1.35,h(SHIELD_UP,0,.040),CLOSED,"approach shield rim")
    for angle in ((55,46,35,23,11,0) if mode=="overpress" else (55,46,35,23,12,6)):
        add(right,.60,h(radians(angle),0,.006),CLOSED,"lower shield")
    if mode=="overpress":
        add(right,.6,(-.02,-.027,.210),CLOSED,"approach closed canopy")
        add(right,1.,(-.02,-.027,.117),CLOSED,"press beyond the rigid stop")
    add(right,.85,h(6*3.14159/180,0,.064),CLOSED,"withdraw from rim")
    add(right,.6,h(0,-.047,.077),OPEN,"release closed shield")
    if mode!="unlatched":
        cx,cy,cz=CAM
        move(right,1.20,cx+.033,cy-.040,cz+.101,CLOSED,"approach cam paddle")
        for angle in (0,22,44,66,88):
            a=radians(angle)
            move(right,.55,cx+.032*cos(a)+.014*sin(a),cy+.032*sin(a)-.014*cos(a),cz+.061,CLOSED,"turn local cam")
        move(right,.50,cx+.016,cy+.032,cz+.121,CLOSED,"lift clear of catch")
        move(right,.65,cx+.035,cy+.052,cz+.115,OPEN,"release cam")
        move(right,1.,.045,-.181,.285,OPEN,"clear the outboard catch")
    if mode=="held":add(right,1.,h(0,0,.082),OPEN,"remain over guarded chuck")
    else:add(right,1.2,rp,CLOSED if mode=="park_closed" else OPEN,"park right")
    right_end=sum(p.duration for p in right)/slow
    add(left,max(.1,right_end-left_end+.1),lp,OPEN,"wait for shield")
    button()
    add(left,1.3,lp,CLOSED if mode=="park_closed" else OPEN,"park left")
    return {"left":left,"right":right}
