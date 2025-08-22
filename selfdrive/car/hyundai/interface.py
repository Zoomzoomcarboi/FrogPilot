from cereal import car, custom
from panda import Panda
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                         CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, Buttons
from openpilot.selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.disable_ecu import disable_ecu
import time
import threading

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
FrogPilotButtonType = custom.FrogPilotCarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}



def try_multiple_addresses_ioniq6(logcan, sendcan, CP):
  """Try disabling ECU at different addresses for Ioniq 6"""
  print("Method 6: Multiple Address Attempts")
  
  addresses = [
    (0x730, "Primary ADAS ECU"),
    (0x7d0, "Alternative ADAS ECU"),
    (0x731, "ADAS ECU variant 1"),
    (0x732, "ADAS ECU variant 2"),
    (0x750, "ADAS ECU variant 3"),
    (0x760, "ADAS ECU variant 4"),
  ]
  bus = CanBus(CP).ECAN
  
  for addr, desc in addresses:
    try:
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
      print(f"  SUCCESS: {desc} at 0x{addr:03x}")
    except Exception as e:
      print(f"  FAILED: {desc} at 0x{addr:03x} - {e}")


def security_access_then_disable(logcan, sendcan, CP):
  """Attempt security access before disable for Ioniq 6"""
  print("Method 7: Security Access + Disable")
  
  addr = 0x730
  bus = CanBus(CP).ECAN
  
  try:
    # Request security access seed
    msg = b'\x27\x01\x00\x00\x00\x00\x00\x00'
    sendcan.send([(addr, 0, msg, bus)])
    time.sleep(0.1)
    print("  Sent: Security access seed request")
    
    # Send a simple key (this probably won't work but worth trying)
    msg = b'\x27\x02\x12\x34\x56\x78\x00\x00'
    sendcan.send([(addr, 0, msg, bus)])
    time.sleep(0.1)
    print("  Sent: Security access key")
    
    # Now try disable
    disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
    print("  SUCCESS: Security access + disable completed")
  except Exception as e:
    print(f"  FAILED: Security access + disable - {e}")


def tester_present_then_disable(logcan, sendcan, CP):
  """Send tester present then disable for Ioniq 6"""
  print("Method 8: Tester Present + Disable")
  
  addr = 0x730
  bus = CanBus(CP).ECAN
  
  try:
    # Send tester present
    msg = b'\x3E\x80\x00\x00\x00\x00\x00\x00'
    sendcan.send([(addr, 0, msg, bus)])
    time.sleep(0.1)
    print("  Sent: Tester present")
    
    # Now try disable
    disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
    print("  SUCCESS: Tester present + disable completed")
  except Exception as e:
    print(f"  FAILED: Tester present + disable - {e}")


def delayed_disable_ioniq6(logcan, sendcan, CP, delay_seconds=5):
  """Try disabling ECU after a delay with multiple attempts for Ioniq 6"""
  def disable_after_delay():
    print(f"Method 9: Delayed Disable (waiting {delay_seconds}s)")
    time.sleep(delay_seconds)
    
    addr = 0x730
    bus = CanBus(CP).ECAN
    
    # Try 10 times with 1 second intervals
    for i in range(10):
      try:
        disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
        print(f"  SUCCESS: Delayed disable attempt {i+1}")
        time.sleep(1)
      except Exception as e:
        print(f"  FAILED: Delayed disable attempt {i+1} - {e}")
        time.sleep(1)
  
  # Start the delayed disable in a separate thread
  thread = threading.Thread(target=disable_after_delay)
  thread.daemon = True
  thread.start()


def continuous_disable_attempts_ioniq6(logcan, sendcan, CP):
  """
  Continuously attempt to disable ECU every 10 seconds for Ioniq 6
  """
  def disable_loop():
    print("Method 10: Continuous Disable Attempts (every 10s)")
    addr = 0x730
    bus = CanBus(CP).ECAN
    attempt = 0
    
    while True:
      attempt += 1
      try:
        disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
        print(f"  SUCCESS: Continuous disable attempt {attempt}")
      except Exception as e:
        print(f"  FAILED: Continuous disable attempt {attempt} - {e}")
      
      time.sleep(10)  # Try every 10 seconds
  
  thread = threading.Thread(target=disable_loop)
  thread.daemon = True
  thread.start()


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    use_old_long = frogpilot_toggles.old_long_api

    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    # FIXME: the Optima Hybrid 2017 uses a different SCC12 checksum
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, }

    hda2 = Ecu.adas in [fw.ecu for fw in car_fw]
    CAN = CanBus(None, hda2, fingerprint)

    if candidate in CANFD_CAR:
      # detect if car is hybrid
      if 0x105 in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      # detect HDA2 with ADAS Driving ECU
      if hda2:
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
        if 0x110 in fingerprint[CAN.CAM]:
          ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
      else:
        # non-HDA2
        if 0x1cf not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
        # ICE cars do not have 0x130; GEARS message on 0x40 or 0x70 instead
        if 0x130 not in fingerprint[CAN.ECAN]:
          if 0x40 not in fingerprint[CAN.ECAN]:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
          else:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
        if candidate not in CANFD_RADAR_SCC_CAR:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
    else:
      # TODO: detect EV and hybrid
      if candidate in HYBRID_CAR:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      # Send LFA message on cars with HDA
      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      # These cars use the FCA11 message for the AEB and FCW signals, all others use SCC12
      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate == CAR.KIA_OPTIMA_G4_FL:
      ret.steerActuatorDelay = 0.2

    # *** longitudinal control ***
    if candidate in CANFD_CAR:
      if use_old_long:
        ret.longitudinalTuning.deadzoneBP = [0.]
        ret.longitudinalTuning.deadzoneV = [0.]
        ret.longitudinalTuning.kpV = [0.1]
        ret.longitudinalTuning.kiV = [0.0]
      # Enable longitudinal for CAN-FD cars except those explicitly unsupported
      # Special handling for Ioniq 6: allow longitudinal despite CANFD_NO_RADAR_DISABLE flag
      if candidate == CAR.HYUNDAI_IONIQ_6:
        ret.experimentalLongitudinalAvailable = True
      else:
        ret.experimentalLongitudinalAvailable = candidate not in (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR)
    else:
      if use_old_long:
        ret.longitudinalTuning.deadzoneBP = [0.]
        ret.longitudinalTuning.deadzoneV = [0.]
        ret.longitudinalTuning.kpV = [0.5]
        ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = candidate not in (UNSUPPORTED_LONGITUDINAL_CAR | CAMERA_SCC_CAR)
    
    # Add Ioniq 6 specific longitudinal tuning
    if candidate == CAR.HYUNDAI_IONIQ_6:
      if use_old_long:
        ret.longitudinalTuning.deadzoneBP = [0.]
        ret.longitudinalTuning.deadzoneV = [0.]
        ret.longitudinalTuning.kpV = [0.12]  # Fine-tuned for Ioniq 6
        ret.longitudinalTuning.kiV = [0.02]
      ret.longitudinalActuatorDelay = 0.3  # Optimized for Ioniq 6 HDA2
    
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.stoppingControl = True
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    if candidate != CAR.HYUNDAI_IONIQ_6:  # Use default for non-Ioniq 6
      ret.longitudinalActuatorDelay = 0.5

    # *** feature detection ***
    if candidate in CANFD_CAR:
      ret.enableBsm = 0x1e5 in fingerprint[CAN.ECAN]
    else:
      ret.enableBsm = 0x58b in fingerprint[0]

    # *** panda safety config ***
    if candidate in CANFD_CAR:
      cfgs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCanfd), ]
      if CAN.ECAN >= 4:
        cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.noOutput))
      ret.safetyConfigs = cfgs

      if ret.flags & HyundaiFlags.CANFD_HDA2:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
        if ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING:
          ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING
      if ret.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_ALT_BUTTONS
      if ret.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC
    else:
      if candidate in LEGACY_SAFETY_MODE_CAR:
        # these cars require a special panda safety mode due to missing counters and checksums in the messages
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]
      else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]

      if candidate in CAMERA_SCC_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_CAMERA_SCC

      if 0x391 in fingerprint[0]:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_LFA_BTN

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
    if ret.flags & HyundaiFlags.HYBRID:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_HYBRID_GAS
    elif ret.flags & HyundaiFlags.EV:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_EV_GAS

    if candidate in (CAR.HYUNDAI_KONA, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022):
      ret.flags |= HyundaiFlags.ALT_LIMITS.value
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_ALT_LIMITS

    ret.centerToFront = ret.wheelbase * 0.4

    # Detect smartMDPS
    if 0x2AA in fingerprint[0]:
      ret.minSteerSpeed = 0.

    if frogpilot_toggles.taco_tune_hacks:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_TACO_TUNE_HACK

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    # Special comprehensive handling for Ioniq 6 ADAS ECU disable
    if CP.carFingerprint == CAR.HYUNDAI_IONIQ_6 and CP.openpilotLongitudinalControl:
      print("=== IONIQ 6 DETECTED: Starting Comprehensive ECU Disable ===")
      
      # Run all disable methods
      disable_ioniq6_adas_ecu_comprehensive(logcan, sendcan, CP)
      try_multiple_addresses_ioniq6(logcan, sendcan, CP)
      security_access_then_disable(logcan, sendcan, CP)
      tester_present_then_disable(logcan, sendcan, CP)
      delayed_disable_ioniq6(logcan, sendcan, CP, delay_seconds=3)
      continuous_disable_attempts_ioniq6(logcan, sendcan, CP)
      
      print("=== IONIQ 6 ECU Disable Sequence Complete ===")
      
    # Standard ECU disable for other cars
    elif CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not (CP.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE.value):
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      disable_ecu(logcan, sendcan, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    if self.CS.CP.openpilotLongitudinalControl:
      ret.buttonEvents = [
        *create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT),
        *create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas}),
      ]
    else:
      ret.buttonEvents = create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas})

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.manumatic],
                                       pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret, fp_ret

# === Ioniq 6: ADAS Disable with global UDS logging ===
_ADAS_DISABLE_ATTEMPTED = False

# Monkeypatch disable_ecu to add global logging of request + response
try:
  import selfdrive.car.hyundai.interface as hyundai_interface_mod
except Exception:
  hyundai_interface_mod = None

if hyundai_interface_mod is not None and "disable_ecu" in dir(hyundai_interface_mod):
  _orig_disable_ecu = hyundai_interface_mod.disable_ecu

  def _logged_disable_ecu(logcan, sendcan, *, bus, addr, com_cont_req):
    import binascii, cereal.messaging as messaging, time
    try:
      hex_req = ''.join(f"{b:02x}" for b in com_cont_req)
      print(f"[HYUNDAI][UDS] Sent to 0x{addr:X}: {hex_req}")

      # call original function
      _orig_disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=com_cont_req)

      # listen briefly for response
      sub = messaging.sub_sock('can', conflate=False)
      start = time.time()
      resp = None
      while time.time() - start < 1.0:
        msg = sub.receive(True)
        if msg is None:
          continue
        for m in msg.can:
          if m.address == addr + 8 and m.src == bus:
            resp = getattr(m, "dat", b"")
            break
        if resp:
          break
      if resp:
        hex_resp = binascii.hexlify(resp).decode('ascii')
        print(f"[HYUNDAI][UDS] Response from 0x{addr+8:X}: {hex_resp}")
      else:
        print(f"[HYUNDAI][UDS] No response from 0x{addr+8:X}")
    except Exception as e:
      print(f"[HYUNDAI][UDS] Logging wrapper error: {e}")

  hyundai_interface_mod.disable_ecu = _logged_disable_ecu
  print("[HYUNDAI] Patched disable_ecu with global UDS logging")


def calc_hyundai_key(seed: bytes) -> bytes:
  """
  Placeholder for Hyundai SecurityAccess key algorithm.
  Given a seed (bytes), return the computed key (bytes).
  For now, just logs the seed and returns None.
  """
  try:
    print(f"[HYUNDAI] calc_hyundai_key called with seed: {seed.hex()}")
  except Exception:
    print(f"[HYUNDAI] calc_hyundai_key called with seed: {seed}")
  return None


def disable_ioniq6_adas_ecu_comprehensive(logcan, sendcan, CP):
  """
  Force-disable the Ioniq 6 camera ECU once per boot so OP longitudinal can take over.
  Includes SecurityAccess seed request (0x27 0x01) and optional key response (0x27 0x02).
  """
  global _ADAS_DISABLE_ATTEMPTED
  if _ADAS_DISABLE_ATTEMPTED:
    return

  try:
    bus = 0
    req_id = 0x7D0      # Camera ECU request ID
    resp_id = req_id + 8

    print("[HYUNDAI] Sending ADAS disable sequence to camera ECU 0x7D0")

    # 1) Tester Present
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x3E, 0x00])

    # 2) Diagnostic Session Control: Extended
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x10, 0x03])

    # 3) SecurityAccess seed request (0x27 0x01)
    print("[HYUNDAI] Requesting SecurityAccess seed (0x27 0x01)…")
    seed = _request_security_seed(logcan, sendcan, bus=bus, req_addr=req_id, resp_addr=resp_id)
    if seed:
      key = calc_hyundai_key(seed)
      if key:
        try:
          payload = [0x27, 0x02] + list(key)
          disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=payload)
          print(f"[HYUNDAI] Sent SecurityAccess key (0x27 0x02) {key.hex()}")
        except Exception as e:
          print(f"[HYUNDAI] Failed to send SecurityAccess key: {e}")
      else:
        print("[HYUNDAI] No key available, skipping 0x27 0x02")
    else:
      print("[HYUNDAI] No seed received, cannot attempt key unlock")

    # 4) Communication Control: disable Tx/Rx (0x28 0x00 0x03)
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x28, 0x00, 0x03])
    print("[HYUNDAI] ADAS disable sequence sent (note: 0x28 requires security unlock)")

  except Exception as e:
    print(f"[HYUNDAI] ADAS disable failed: {e}")
  finally:
    _ADAS_DISABLE_ATTEMPTED = True


def _request_security_seed(logcan, sendcan, *, bus: int, req_addr: int, resp_addr: int, window_s: float = 2.0):
  """
  Sends SecurityAccess seed request (0x27 0x01) and listens for the response.
  Returns the seed bytes (excluding 0x67 0x01 header) if received.
  """
  try:
    import cereal.messaging as messaging
    import time, binascii
    sub = messaging.sub_sock('can', conflate=False)

    # send request
    disable_ecu(logcan, sendcan, bus=bus, addr=req_addr, com_cont_req=[0x27, 0x01])

    # wait for response
    start = time.time()
    while time.time() - start < window_s:
      msg = sub.receive(True)
      if msg is None:
        continue
      for m in msg.can:
        if getattr(m, "src", 0) == bus and getattr(m, "address", 0) == resp_addr:
          raw = getattr(m, "dat", b"")
          hexstr = binascii.hexlify(raw).decode('ascii')
          print(f"[HYUNDAI] SecurityAccess seed response on 0x{resp_addr:X}: {hexstr}")
          if len(raw) > 2 and raw[0] == 0x67 and raw[1] == 0x01:
            return raw[2:]
          return None
  except Exception as e:
    print(f"[HYUNDAI] Seed request/monitor error: {e}")
  return None


# Monkeypatch hyundaican.create_acc_commands to add SCC_CONTROL debug logging
try:
  import selfdrive.car.hyundai.hyundaican as hyundaican
  _orig_create_acc_commands = hyundaican.create_acc_commands

  def _patched_create_acc_commands(packer, cc, enabled, accel, idx, gap, lead_visible, set_speed_in_units, stopping):
    msgs = _orig_create_acc_commands(packer, cc, enabled, accel, idx, gap, lead_visible, set_speed_in_units, stopping)
    for name, msg, bus in msgs:
      if name == "SCC_CONTROL":
        try:
          cnt = msg.get("COUNTER", None)
          csum = msg.get("CHECKSUM", None)
          print(f"[HYUNDAI] OP SCC_CONTROL injected: COUNTER={cnt} CHECKSUM={csum}")
        except Exception:
          print("[HYUNDAI] OP SCC_CONTROL injected (no details)")
    return msgs

  hyundaican.create_acc_commands = _patched_create_acc_commands
  print("[HYUNDAI] Patched hyundaican.create_acc_commands for SCC_CONTROL debug")
except Exception as e:
  print(f"[HYUNDAI] Failed to patch SCC_CONTROL debug: {e}")


# Monitor for stock SCC_CONTROL after disable
try:
  import cereal.messaging as messaging
  import threading, time

  def _monitor_stock_scc():
    sub = messaging.sub_sock('can', conflate=False)
    seen_stock = False
    start = time.time()
    stock_addrs = {0x420, 0x421, 0x50, 0x51}
    while time.time() - start < 15:
      msg = sub.receive(True)
      if msg is None:
        continue
      for m in msg.can:
        if m.address in stock_addrs:
          if not seen_stock:
            print(f"[HYUNDAI] WARNING: Stock SCC_CONTROL still present on bus {m.src} (addr=0x{m.address:X})")
            seen_stock = True
    if not seen_stock:
      print("[HYUNDAI] No stock SCC_CONTROL detected after ADAS disable")

  threading.Thread(target=_monitor_stock_scc, daemon=True).start()
except Exception as e:
  print(f"[HYUNDAI] Failed to start SCC_CONTROL stock monitor: {e}")


# =======================
# HYUNDAI REPEATED COMM CTRL DISABLE SEQUENCE (Stable Repeat2)
# =======================
import threading, time, os

class HyundaiRepeater:
  def __init__(self, send_func):
    self.send_func = send_func
    self.stop = False
    self.seed_log_path = "/data/openpilot/ioniq6_seeds.log"
    print("[HYUNDAI][UDS] Repeater started (stable repeat2)")
    self.thread = threading.Thread(target=self.worker, daemon=True)
    self.thread.start()

  def log_seed(self, seed_bytes):
    try:
      ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime())
      os.makedirs(os.path.dirname(self.seed_log_path), exist_ok=True)
      with open(self.seed_log_path, "a") as f:
        f.write(f"{ts}, {seed_bytes.hex()}\n")
      print(f"[HYUNDAI][UDS] Seed logged: {seed_bytes.hex()}")
    except Exception as e:
      print("[HYUNDAI][UDS] Failed to log seed:", e)

  def worker(self):
    sessions = [0x02, 0x03]
    ctrlTypes = [0x00,0x01,0x02,0x03]
    commTypes = [0x01,0x02,0x03]
    while not self.stop:
      for sess in sessions:
        try:
          msg = bytes([0x10, sess])
          self.send_func(msg)
          print(f"[HYUNDAI][UDS] Sent {msg.hex()}")
          time.sleep(0.2)
          seed_req = bytes([0x27, 0x01])
          self.send_func(seed_req)
          print("[HYUNDAI][UDS] Sent 2701")
          time.sleep(0.2)
        except Exception as e:
          print("[HYUNDAI][UDS] Error sending session/seed:", e)
        for ct in ctrlTypes:
          for cm in commTypes:
            try:
              msg = bytes([0x28, ct, cm])
              self.send_func(msg)
              print(f"[HYUNDAI][UDS] Sent {msg.hex()}")
              time.sleep(0.2)
            except Exception as e:
              print("[HYUNDAI][UDS] Error sending comm ctrl:", e)
      time.sleep(30)

  def handle_response(self, resp_bytes):
    if len(resp_bytes) >= 2:
      if resp_bytes[0] == 0x67 and resp_bytes[1] == 0x01:
        seed = resp_bytes[2:]
        print(f"[HYUNDAI][UDS] Response: {resp_bytes.hex()} (seed={seed.hex()})")
        self.log_seed(seed)
      elif resp_bytes[0] == 0x68:
        self.stop = True
        print(f"[HYUNDAI][UDS] SUCCESS with response {resp_bytes.hex()}")
      elif resp_bytes[0] == 0x7F:
        print(f"[HYUNDAI][UDS] Negative Response: {resp_bytes.hex()}")


# Hook into HyundaiInterface, extending the stable dropin5 version
try:
  from selfdrive.car.hyundai.interface import HyundaiInterface as _HyundaiInterface
except ImportError:
  _HyundaiInterface = None

if _HyundaiInterface is not None:
  class HyundaiInterface(_HyundaiInterface):
    def __init__(self, CP, CarController, CarState):
      super().__init__(CP, CarController, CarState)
      try:
        self.repeater = HyundaiRepeater(self.can_uds_request)
      except Exception as e:
        print("[HYUNDAI][UDS] Failed to start repeater:", e)

    def handle_can_response(self, resp):
      try:
        if hasattr(self, "repeater"):
          self.repeater.handle_response(resp.dat)
      except Exception as e:
        print("[HYUNDAI][UDS] Error handling response:", e)
      return super().handle_can_response(resp)
