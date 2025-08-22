from typing import List
from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car.hyundai.hyundaican import init_can, create_scc_control, next_scc_counter

class CarController:
    def __init__(self, dbc_name: str):
        self.packer = CANPacker(dbc_name)
        init_can(self.packer)
        self._scc_cnt = None

    def update(self, enabled, a_req, jerk, v_set, gap, canbus):
        can_sends: List = []

        # maintain SCC counter
        self._scc_cnt = next_scc_counter(self._scc_cnt)
        scc_msg = create_scc_control(enabled, a_req, jerk, v_set, gap, cnt=self._scc_cnt)
        can_sends.append((canbus.scc_bus, scc_msg["name"], scc_msg["data"]))

        return can_sends
