from cereal import car
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car import structs
import threading, time

# Minimal UDS test utility patched into CarInterface
from panda.python.uds import UdsClient, IsoTpMessage

class UDSTester:
    def __init__(self, bus):
        self.bus = bus

    def disable_ecu(self, addr, data):
        try:
            msg = IsoTpMessage(tx_addr=addr, rx_addr=addr+8, bus=self.bus)
            with UdsClient(msg) as client:
                # Ensure we always send bytes
                if isinstance(data, list):
                    payload = bytes(data)
                elif isinstance(data, bytes):
                    payload = data
                else:
                    payload = bytes([data])
                resp = client._isotp.send(payload)
                # Safe response logging
                if isinstance(resp, bytes):
                    resp_str = resp.hex()
                elif isinstance(resp, list):
                    resp_str = bytes(resp).hex()
                elif hasattr(resp, 'dat'):
                    resp_str = resp.dat.hex()
                else:
                    resp_str = str(resp)
                print(f"[HYUNDAI][UDS] Sent to 0x{addr:X}: {payload.hex()}")
                print(f"[HYUNDAI][UDS] Response from 0x{addr+8:X}: {resp_str}")
        except Exception as e:
            print(f"[HYUNDAI][UDS] Exception at 0x{addr:X}: {e}")

    def run(self):
        # target ECUs known from logs
        targets = [0x730, 0x7D0, 0x731, 0x732, 0x750]
        for addr in targets:
            # disable Tx+Rx
            self.disable_ecu(addr, [0x28, 0x83, 0x01])
            time.sleep(0.2)

class CarInterface(CarInterfaceBase):
    @staticmethod
    def get_params(candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles, params):
        ret = CarInterfaceBase.get_non_essential_params(candidate, fingerprint, car_fw)
        # run UDS disable thread once at boot
        threading.Thread(target=UDSTester(0).run, daemon=True).start()
        return ret
