from selfdrive.car.hyundai.hyundaicanfd import HyundaiCANFD

class CarInterface:
    def __init__(self, CP, canbus, log):
        self.CP = CP
        self.canbus = canbus
        self.log = log

        # Initialize CANFD with bounded ADAS disable logic
        self.cf = HyundaiCANFD(canbus, packer=None, log=log)
        self.cf.init()
        self.log.info("CarInterface: HyundaiCANFD initialized (no infinite ADAS disable loops)")
