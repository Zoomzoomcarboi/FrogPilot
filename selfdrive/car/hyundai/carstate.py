from cereal import car

class CarState:
    def __init__(self):
        self.main_enabled = False
        self.acc_req = 0
        self._op_long_enabled_latch = False

    def update(self, cp, op_long_active: bool):
        # Example: assuming cp gives signals
        self.main_enabled = cp.vEgoRaw > 0  # placeholder logic
        self.acc_req = cp.acc_req if hasattr(cp, 'acc_req') else 0

        if op_long_active:
            if self.acc_req == 1:
                self._op_long_enabled_latch = True
            if not self.main_enabled or self.acc_req == 0:
                self._op_long_enabled_latch = False
            enabled = self._op_long_enabled_latch
        else:
            enabled = (self.acc_req == 1)

        return enabled
