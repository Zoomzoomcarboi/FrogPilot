from cereal import car

class HyundaiFlags:
    CANFD = 1 << 0
    HDA2 = 1 << 1
    CANFD_NO_RADAR_DISABLE = 1 << 2  # keep this flag; disabling ADAS is optional via env var now

class CAR:
    IONIQ_6 = "HYUNDAI IONIQ 6"
    # ... other car definitions ...

def get_params(candidate: str) -> car.CarParams:
    CP = car.CarParams.new_message()
    if candidate == CAR.IONIQ_6:
        CP.experimentalLongitudinalAvailable = True
        # hyundaicanfd will only attempt ADAS disable if HYUNDAI_ADAS_DISABLE=1
    return CP
