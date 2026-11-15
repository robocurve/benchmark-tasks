"""Ordinary stepped wooden peg rests, supported only by fixed scene geometry.

Each stepped wooden block supports a closed peg through contact alone. The
outer 20 mm of its handles overhang so a real finger can approach underneath.
The pegs have no constraint to either rest; both free jaws remain dynamic.
"""
from scene import TABLE


def peg_positions():
    return {('Left' if sign>0 else 'Right'):(.32,sign*.30,TABLE+.1325) for sign in (-1,1)}


def geometry():
    solids={}
    for side,pos in peg_positions().items():
        x,y,_=pos
        # Jaw underside is6.5 mm below the hinge, handle underside11 mm.
        solids[side+'Base']=dict(kind='box',size=(.09,.06,.008),
                               center=(x+.015,y,TABLE+.004))
        solids[side+'JawRest']=dict(kind='box',size=(.035,.016,.125),
                                  center=(x+.0175,y,TABLE+.125/2))
        solids[side+'HandleRest']=dict(kind='box',size=(.015,.016,.1205),
                                     center=(x-.0075,y,TABLE+.1205/2))
    return solids


def author(stage,path="/World/PegRests"):
    from native_cloth import author_box
    for name,solid in geometry().items():
        author_box(stage,path+'/'+name,solid['size'],solid['center'],(.59,.39,.22))
