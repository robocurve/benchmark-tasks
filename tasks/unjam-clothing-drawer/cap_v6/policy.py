"""Successful v4 compression task; A retains its handle grasp throughout."""
from drawer_layout_v6 import HANDLE

def run(api):
    api.move('left',(HANDLE[0],HANDLE[1],.9075),2,'A approach; B held')
    api.move('left',HANDLE,1.8,'A lower; B held')
    api.grip('left',0.,1)
    api.pull_until_jam()
    api.move('right',(.25,-.22,.9075),2,'B approach on its own side; A holds handle')
    api.grip('right',0.,.8)
    api.move('right',api.press_hover(),2,'B approach cloth; A holds handle')
    api.press_until_clear()
    api.pull_while_pressing()
    api.move('right',api.raised_right(),1.5,'B lift after obstruction clears')
    api.move('right',(.22,-.22,.9075),2,'B withdraw to its side')
    api.finish_pull()
    api.hold(1.5)
