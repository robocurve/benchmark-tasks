"""Eight-key terminal dimensions, in metres, x forward and y left."""
TABLE_TOP=0.7475
DECK_NAME,SCREEN_NAME,CARD_NAME='tm_deck','tm_screen','tm_card'
DECK_XY=(.220,.015)
DECK_SIZE=(.110,.300,.025)
CARD_XY=(.240,.280)
SCREEN_XY=(.365,-.020)
FIXTURE_MASS,FIXTURE_FRICTION=25.,1.2
CAP_SIZE=(.024,.024,.014)
CAP_CENTRE_Z,CAP_TOP_Z=.037,.044
TRAVEL,DOWN,UP=.005,.003,.0015
MOD_SPRING,MOD_MASS=8.,.0005
KEY_SPRING,KEY_MASS,DAMPING=100.,.002,5.
WINDOW=2
CODE='Ab7'
KEYS={'shift':(.195,.140),'ctrl':(.240,.140),'a':(.195,-.020),
      'b':(.195,-.065),'7':(.195,-.110),'enter':(.240,-.020),
      'l':(.240,-.065),'erase':(.240,-.110)}
MODIFIERS={'shift','ctrl'}
NONMODS=set(KEYS)-MODIFIERS
GREY,BLUE,GREEN,RED,CREAM,INK=(.74,.74,.72),(.16,.23,.38),(.1,.65,.22),(.82,.12,.10),(.94,.91,.83),(.04,.04,.04)
BODY=(.10,.11,.13)
SCREEN_BOXES=[((0,0,.015),(.060,.140,.030),BODY),
              ((0,0,.040),(.030,.030,.050),BODY),
              ((0,0,.110),(.020,.280,.150),BODY)]
CARD_SIZE=(.065,.150,.003)
RELEASE,STILL_STEPS,STILL_M,FIXTURE_LIMIT=.030,20,.001,.005
TCP_LINK_OFFSET=.138
RUN_STEPS=1200
# Five-by-five glyphs. Card letters differ by shape, with no case annotation.
GLYPHS={
 'A':('01110','10001','11111','10001','10001'),
 'a':('00000','01110','00001','01111','01111'),
 'b':('10000','10000','11110','10001','11110'),
 'B':('11110','10001','11110','10001','11110'),
 '7':('11111','00001','00010','00100','01000'),
 'l':('01100','00100','00100','00100','01110'),
 'L':('10000','10000','10000','10000','11111'),
 '&':('01100','10010','01100','10101','01011'),
 'shift':('00100','01110','11111','00100','00100'),
 'ctrl':('11110','10000','10000','10000','11110'),
 'enter':('00001','00001','00101','01111','00100'),
 'erase':('00000','00100','01111','11111','01111'),
}

def colour(key):
    return BLUE if key in MODIFIERS else GREEN if key=='enter' else RED if key=='erase' else GREY
