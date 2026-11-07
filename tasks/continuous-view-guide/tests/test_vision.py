"""Image ablation and geometric distractors for the demonstration's view gate."""
import cv2
import numpy as np
from guide_task.vision import entrance_view,ViewGate

def image(width=70,guide=True):
    rgb=np.full((176,320,3),90,np.uint8)
    for x in (100,100+width):
        for y in (50,120):cv2.rectangle(rgb,(x-4,y-4),(x+4,y+4),(240,10,110),-1)
    if guide:cv2.rectangle(rgb,(100+width//2-3,60),(100+width//2+3,110),(10,190,210),-1)
    return rgb

def test_clear_and_large_markers_are_usable():
    assert entrance_view(image())['usable']

def test_three_observed_corners_bound_partial_occlusion():
    rgb=image();rgb[45:56,165:176]=90
    result=entrance_view(rgb)
    assert result['usable'] and result['corners']==3 and result['inferred_corner']

def test_markers_without_guide_and_oblique_views_are_not_usable():
    assert not entrance_view(image(guide=False))['usable']
    assert not entrance_view(image(width=15))['usable']
    assert not entrance_view(None)['usable']

def test_no_rgb_no_manipulation_and_fresh_views_resume():
    gate=ViewGate()
    for _ in range(30):assert not gate.update(np.zeros((176,320,3),np.uint8),True)[0]
    for _ in range(2):assert not gate.update(image(),True)[0]
    assert gate.update(image(),True)[0]
    for _ in range(3):advance,_=gate.update(None,True)
    assert not advance
    assert gate.update(None,False)[0]  # stationary-collar observer transit remains possible
