"""The physical motion after fastening depends on the actual tool state."""
from repair_plan import RepairSchedule


def test_cannot_pull_during_normal_lowering_window():
    rows=[{'label':'Unhook carrying pins','unhook':0.},
          {'label':'Pull handle from face','unhook':-.075},
          {'label':'Withdraw handle'}]
    s=RepairSchedule(rows)
    for _ in range(30):
        assert s.next_index({'key_released':False})==0
    assert s.next_index({'key_released':True})==0
    assert s.next_index({'key_released':True,'carrier_offset_x_mm':-45})==1
    assert s.next_index({'key_released':True,'carrier_offset_x_mm':-103})==1
    assert s.next_index({})==2


def test_clutch_allows_withdrawal_after_seating():
    drive={'front':'wall','wood':'strip','y':.114}
    rows=[{'label':'Approach site','repair_drive':drive},
          *[{'label':'Drive site','repair_drive':drive} for _ in range(10)],
          {'label':'Withdraw site','repair_drive':drive}]
    s=RepairSchedule(rows)
    assert s.next_index({'remaining':12})==0
    for count in [12,12,11,11,11]:s.next_index({'remaining':count})
    assert s.next_index({'remaining':11})==len(rows)-1



def test_keep_lifting_handle_on_panel_until_both_rows_attached():
    rows = [
        {'label':'Unload lower panel positive'},
        {'label':'Unhook carrying pins','unhook':0.},
        {'label':'Pull handle from face','unhook':-.075},
    ]
    schedule = RepairSchedule(rows)
    for row_count in (0, 1, 1, 0):
        assert schedule.next_index({'panel_rows': row_count}) == 0
    assert schedule.next_index({'panel_rows': 2}) == 0
    assert schedule.next_index({'panel_rows': 2}) == 1
