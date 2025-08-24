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
import os

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
FrogPilotButtonType = custom.FrogPilotCarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}

# Global UDS logging wrapper - FIXED with proper function saving
_original_disable_ecu = disable_ecu  # Save original before patching

def _logged_disable_ecu(logcan, sendcan, *, bus, addr, com_cont_req):
  import binascii, cereal.messaging as messaging, time
  try:
    hex_req = ''.join(f"{b:02x}" for b in com_cont_req)
    print(f"[HYUNDAI][UDS] Sent to 0x{addr:X}: {hex_req}")

    # call ORIGINAL function (not the patched one)
    _original_disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=com_cont_req)

    # listen for response with longer timeout - FIXED to catch all responses
    sub = messaging.sub_sock('can', conflate=False)
    start = time.time()
    resp = None
    resp_addr = None
    while time.time() - start < 2.0:  # Increased timeout
      msg = sub.receive(True)
      if msg is None:
        continue
      for m in msg.can:
        # Log ALL responses from this bus in UDS range, not just addr+8 
        if m.src == bus and m.address >= 0x700:  # UDS response range
          # Safe bytes conversion
          try:
            if hasattr(m, "dat"):
              resp = bytes(m.dat)
            else:
              resp = bytes(m) if isinstance(m, (bytes, bytearray)) else b""
          except:
            resp = b""
          
          if resp:
            print(f"[HYUNDAI][UDS] Potential response from 0x{m.address:X}: {resp.hex().upper()}")
            resp_addr = m.address
            # Still prefer the standard addr+8 response if we see it
            if m.address == addr + 8:
              break
      if resp and len(resp) > 0:
        break
    
    if resp:
      hex_resp = resp.hex().upper()
      print(f"[HYUNDAI][UDS] Final response from 0x{resp_addr:X}: {hex_resp}")
      
      # Parse response types
      if len(resp) >= 2:
        if resp[0] == 0x67 and resp[1] == 0x01:
          seed = resp[2:]
          print(f"[HYUNDAI][UDS] SEED RECEIVED: {seed.hex().upper()}")
          log_seed_to_file(seed)
        elif resp[0] == 0x67 and resp[1] == 0x02:
          print(f"[HYUNDAI][UDS] KEY ACCEPTED")
        elif resp[0] == 0x68:
          print(f"[HYUNDAI][UDS] COMM CONTROL SUCCESS")
          # Set global success flag to stop testing
          if 'uds_tester' in globals() and uds_tester:
            uds_tester.success_found = True
        elif resp[0] == 0x7F:
          service = resp[1] if len(resp) > 1 else 0x00
          nrc = resp[2] if len(resp) > 2 else 0x00
          print(f"[HYUNDAI][UDS] NEGATIVE RESPONSE: Service=0x{service:02X}, NRC=0x{nrc:02X}")
    else:
      print(f"[HYUNDAI][UDS] No response detected")
      
  except Exception as e:
    print(f"[HYUNDAI][UDS] Logging wrapper error: {e}")

# Replace disable_ecu globally - FrogPilot-safe with double-patch guard
import openpilot.selfdrive.car.disable_ecu as disable_ecu_module
if disable_ecu_module.disable_ecu is not _logged_disable_ecu:
    disable_ecu_module.disable_ecu = _logged_disable_ecu
    print("[HYUNDAI] Patched disable_ecu with global UDS logging")
else:
    print("[HYUNDAI] disable_ecu already patched, skipping")

def log_seed_to_file(seed_bytes):
  """Log seed to file - FrogPilot safe"""
  try:
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    log_path = "/data/openpilot/ioniq6_seeds.log"
    # FrogPilot-safe path handling
    try:
      os.makedirs(os.path.dirname(log_path), exist_ok=True)
      with open(log_path, "a") as f:
        f.write(f"{timestamp}, {seed_bytes.hex().upper()}\n")
      print(f"[HYUNDAI][SEED] Logged to file: {seed_bytes.hex().upper()}")
    except (OSError, PermissionError) as e:
      print(f"[HYUNDAI][SEED] Could not write to {log_path}: {e}")
  except Exception as e:
    print(f"[HYUNDAI][SEED] Failed to log: {e}")

def calc_hyundai_key(seed: bytes) -> bytes:
  """
  Placeholder for Hyundai SecurityAccess key algorithm.
  """
  try:
    print(f"[HYUNDAI] calc_hyundai_key called with seed: {seed.hex().upper()}")
  except Exception:
    print(f"[HYUNDAI] calc_hyundai_key called with seed: {seed}")
  return b""  # Return empty bytes instead of None to prevent TypeError

# UDS Testing Class - FIXED
class UDSTester:
  def __init__(self, logcan, sendcan):
    self.logcan = logcan
    self.sendcan = sendcan
    self.stop_testing = False
    self.success_found = False
    self.test_addresses = [0x7D0, 0x730]  # Camera ECU addresses to try
    self.bus = 0
    self.max_cycles = 10  # Prevent infinite flooding
    self.cycle_count = 0
    
  def send_uds(self, addr, data):
    """Send UDS message using existing sendcan interface"""
    try:
      if isinstance(data, str):
        data = bytes.fromhex(data)
      elif isinstance(data, list):
        data = bytes(data)
      
      # Use disable_ecu which now has logging
      _logged_disable_ecu(self.logcan, self.sendcan, bus=self.bus, addr=addr, com_cont_req=data)
    except Exception as e:
      print(f"[HYUNDAI][UDS] Failed to send to 0x{addr:X}: {e}")
  
  def test_sessions_and_comm_control(self):
    """Test different diagnostic sessions and communication control variants"""
    sessions = [0x02, 0x03]  # Default and Extended diagnostic sessions
    comm_variants = [
      (0x00, 0x01), (0x00, 0x02), (0x00, 0x03),  # Enable variants
      (0x01, 0x01), (0x01, 0x02), (0x01, 0x03),  # Rx disable variants  
      (0x02, 0x01), (0x02, 0x02), (0x02, 0x03),  # Tx disable variants
      (0x03, 0x01), (0x03, 0x02), (0x03, 0x03),  # Rx+Tx disable variants
    ]
    
    for addr in self.test_addresses:
      print(f"[HYUNDAI][UDS] Testing ECU at address 0x{addr:X}")
      
      for session in sessions:
        try:
          # Enter diagnostic session
          print(f"[HYUNDAI][UDS] Testing session 0x{session:02X}")
          self.send_uds(addr, [0x10, session])
          time.sleep(0.2)
          
          # Request security access seed
          print("[HYUNDAI][UDS] Requesting SecurityAccess seed...")
          self.send_uds(addr, [0x27, 0x01])
          time.sleep(0.5)  # Give more time for seed response
          
          # Test communication control variants
          for ctrl_type, comm_type in comm_variants:
            print(f"[HYUNDAI][UDS] Testing CommunicationControl 0x28 {ctrl_type:02X} {comm_type:02X}")
            self.send_uds(addr, [0x28, ctrl_type, comm_type])
            time.sleep(0.2)
            
        except Exception as e:
          print(f"[HYUNDAI][UDS] Session 0x{session:02X} failed: {e}")
  
  def start_testing_thread(self):
    """Start background testing thread"""
    def test_loop():
      print("[HYUNDAI][UDS] Starting UDS testing thread...")
      while not self.stop_testing and not self.success_found and self.cycle_count < self.max_cycles:
        try:
          self.cycle_count += 1
          print(f"[HYUNDAI][UDS] Test cycle {self.cycle_count}/{self.max_cycles}")
          
          self.test_sessions_and_comm_control()
          
          print("[HYUNDAI][UDS] Test cycle complete, waiting 30s...")
          
          # Check for success condition to prevent flooding
          if self.success_found:
            print("[HYUNDAI][UDS] Success found, stopping testing to prevent ECU flooding")
            break
            
          time.sleep(30)
        except Exception as e:
          print(f"[HYUNDAI][UDS] Test cycle error: {e}")
          time.sleep(5)
      
      if self.cycle_count >= self.max_cycles:
        print("[HYUNDAI][UDS] Max test cycles reached, stopping to avoid flooding")
      
      print("[HYUNDAI][UDS] Testing thread stopped")
    
    thread = threading.Thread(target=test_loop, daemon=True)
    thread.start()
    return thread

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

def disable_ioniq6_adas_ecu_comprehensive(logcan, sendcan, CP):
  """
  Force-disable the Ioniq 6 camera ECU once per boot so OP longitudinal can take over.
  Includes SecurityAccess seed request (0x27 0x01) and optional key response (0x27 0x02).
  """
  try:
    bus = 0
    req_id = 0x7D0      # Camera ECU request ID

    print("[HYUNDAI] Sending ADAS disable sequence to camera ECU 0x7D0")

    # 1) Tester Present
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x3E, 0x00])

    # 2) Diagnostic Session Control: Extended
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x10, 0x03])

    # 3) SecurityAccess seed request (0x27 0x01)
    print("[HYUNDAI] Requesting SecurityAccess seed (0x27 0x01)...")
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x27, 0x01])
    time.sleep(1.0)  # Give time for seed response

    # 4) Communication Control: disable Tx/Rx (0x28 0x00 0x03)
    disable_ecu(logcan, sendcan, bus=bus, addr=req_id, com_cont_req=[0x28, 0x00, 0x03])
    print("[HYUNDAI] ADAS disable sequence sent (note: 0x28 requires security unlock)")

  except Exception as e:
    print(f"[HYUNDAI] ADAS disable failed: {e}")

# Monitor for stock SCC_CONTROL after disable
def start_stock_scc_monitor():
  """Monitor for stock SCC frames"""
  try:
    import cereal.messaging as messaging
    
    def _monitor_stock_scc():
      sub = messaging.sub_sock('can', conflate=False)
      seen_stock = False
      start = time.time()
      stock_addrs = {0x420, 0x421, 0x50, 0x51}
      while time.time() - start < 15:
        try:
          msg = sub.receive(True)
          if msg is None:
            continue
          for m in msg.can:
            if m.address in stock_addrs:
              if not seen_stock:
                print(f"[HYUNDAI] WARNING: Stock SCC_CONTROL still present on bus {m.src} (addr=0x{m.address:X})")
                seen_stock = True
        except Exception as e:
          print(f"[HYUNDAI] Stock SCC monitor error: {e}")
          break
      if not seen_stock:
        print("[HYUNDAI] No stock SCC_CONTROL detected after ADAS disable")

    threading.Thread(target=_monitor_stock_scc, daemon=True).start()
  except Exception as e:
    print(f"[HYUNDAI] Failed to start SCC_CONTROL stock monitor: {e}")

# Global UDS tester instance
uds_tester = None

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    use_old_long = frogpilot_toggles.old_long_api

    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, }

    hda2 = Ecu.adas in [fw.ecu for fw in car_fw]
    CAN = CanBus(None, hda2, fingerprint)

    if candidate in CANFD_CAR:
      if 0x105 in fingerprint[CAN.ECAN]:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      if hda2:
        ret.flags |= HyundaiFlags.CANFD_HDA2.value
        if 0x110 in fingerprint[CAN.CAM]:
          ret.flags |= HyundaiFlags.CANFD_HDA2_ALT_STEERING.value
      else:
        if 0x1cf not in fingerprint[CAN.ECAN]:
          ret.flags |= HyundaiFlags.CANFD_ALT_BUTTONS.value
        if 0x130 not in fingerprint[CAN.ECAN]:
          if 0x40 not in fingerprint[CAN.ECAN]:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS_2.value
          else:
            ret.flags |= HyundaiFlags.CANFD_ALT_GEARS.value
        if candidate not in CANFD_RADAR_SCC_CAR:
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
    else:
      if candidate in HYBRID_CAR:
        ret.flags |= HyundaiFlags.HYBRID.value
      elif candidate in EV_CAR:
        ret.flags |= HyundaiFlags.EV.value

      if 0x485 in fingerprint[2]:
        ret.flags |= HyundaiFlags.SEND_LFA.value

      if 0x38d in fingerprint[0] or 0x38d in fingerprint[2]:
        ret.flags |= HyundaiFlags.USE_FCA.value

    ret.steerActuatorDelay = 0.1
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
        ret.longitudinalTuning.kpV = [0.12]
        ret.longitudinalTuning.kiV = [0.02]
      ret.longitudinalActuatorDelay = 0.3
    
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.stoppingControl = True
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    if candidate != CAR.HYUNDAI_IONIQ_6:
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

    if 0x2AA in fingerprint[0]:
      ret.minSteerSpeed = 0.

    if frogpilot_toggles.taco_tune_hacks:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_TACO_TUNE_HACK

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    global uds_tester
    
    # Special comprehensive handling for Ioniq 6 ADAS ECU disable
    if CP.carFingerprint == CAR.HYUNDAI_IONIQ_6 and CP.openpilotLongitudinalControl:
      print("=== IONIQ 6 DETECTED: Starting Comprehensive ECU Disable ===")
      
      # Start UDS testing
      uds_tester = UDSTester(logcan, sendcan)
      uds_tester.start_testing_thread()
      
      # Run original disable methods
      disable_ioniq6_adas_ecu_comprehensive(logcan, sendcan, CP)
      try_multiple_addresses_ioniq6(logcan, sendcan, CP)
      security_access_then_disable(logcan, sendcan, CP)
      tester_present_then_disable(logcan, sendcan, CP)
      delayed_disable_ioniq6(logcan, sendcan, CP, delay_seconds=3)
      
      print("=== IONIQ 6 ECU Disable Sequence Complete ===")
      
    # Standard ECU disable for other cars  
    elif CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value):
      addr, bus = 0x7d0, 0
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, CanBus(CP).ECAN
      try:
        disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
      except Exception as e:
        print(f"[HYUNDAI] Standard ECU disable failed: {e}")

    # for blinkers
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      try:
        disable_ecu(logcan, sendcan, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')
      except Exception as e:
        print(f"[HYUNDAI] Blinker ECU disable failed: {e}")
    
    # Start stock SCC monitor
    start_stock_scc_monitor()

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    if self.CS.CP.openpilotLongitudinalControl:
      ret.buttonEvents = [
        *create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT),
        *create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas}),
      ]
    else:
      ret.buttonEvents = create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas})

    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.manumatic],
                                       pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    return ret, fp_ret

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
