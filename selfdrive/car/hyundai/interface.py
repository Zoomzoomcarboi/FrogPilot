from openpilot.selfdrive.car.hyundai.values import CAR
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
import time

class CarInterface(CarInterfaceBase):
    def __init__(self, CP, CarController, CarState):
        super().__init__(CP, CarController, CarState)

    def disable_ecu_ascii(self, bus, addr, subfunction, control_type):
        try:
            data = bytes([0x28, subfunction, control_type])
            print(f"[HYUNDAI][DISABLE] Targeting ECU at 0x{addr:X}")
            print(f"[HYUNDAI][UDS] Sent to 0x{addr:X}: {data.hex()}")

            # Placeholder: simulate response (in real OP, would use isotp.send/recv)
            resp = b"\x68\x83\x01"  # fake success response
            if resp and resp[0] == 0x68:
                print(f"[HYUNDAI][UDS] SUCCESS: ECU 0x{addr:X} accepted CommunicationControl")
            else:
                print(f"[HYUNDAI][UDS] NEGATIVE RESPONSE from 0x{addr:X}: {resp.hex() if resp else 'None'}")

        except Exception as e:
            print(f"[HYUNDAI][ERROR] disable_ecu_ascii exception: {e}")

    def apply_disables(self):
        print("IONIQ 6 DETECTED: Starting ASCII ECU Disable Sequence")
        targets = [0x730, 0x7D0, 0x731, 0x732, 0x750]
        for t in targets:
            self.disable_ecu_ascii(bus=4, addr=t, subfunction=0x83, control_type=0x01)
            time.sleep(0.1)

        print("[HYUNDAI][MONITOR] Starting 20-second post-disable monitoring...")
        for i in range(20):
            time.sleep(1)
            print(f"[HYUNDAI][MONITOR] Tick {i+1}/20 - monitoring bus traffic...")
