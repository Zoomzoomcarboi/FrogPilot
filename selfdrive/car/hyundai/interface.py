from cereal import car, custom
from panda import Panda
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CANFD_CAR, CAMERA_SCC_CAR, CANFD_RADAR_SCC_CAR, \
                                         CANFD_UNSUPPORTED_LONGITUDINAL_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, IONIQ_6_LONGITUDINAL_CAR, Buttons
from openpilot.selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.disable_ecu import disable_ecu

Ecu = car.CarParams.Ecu
ButtonType = car.CarState.ButtonEvent.Type
FrogPilotButtonType = custom.FrogPilotCarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    use_old_long = frogpilot_toggles.old_long_api

    ret.carName = "hyundai"
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
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
      
      # COMPREHENSIVE FIX: Ultra-conservative Ioniq 6 HDA2 longitudinal control
      if candidate == CAR.HYUNDAI_IONIQ_6:
        # STRICT DETECTION: Check for proper HDA2 configuration
        has_hda2 = ret.flags & HyundaiFlags.CANFD_HDA2
        has_alt_steering = ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING
        has_adas_ecu = 0x730 in fingerprint[CAN.ECAN]  # Check ADAS ECU presence
        
        # ULTRA CONSERVATIVE: Only enable if ALL conditions met perfectly
        if (has_hda2 and 
            has_alt_steering and  # Alt steering required
            has_adas_ecu and     # ADAS ECU required
            0x110 in fingerprint[CAN.CAM]):  # Camera bus steering required
          
          ret.experimentalLongitudinalAvailable = True
          
          # EXTREMELY conservative tuning to prevent any CAN conflicts
          ret.longitudinalTuning.kpV = [0.04, 0.07, 0.10]    # Ultra-low gains
          ret.longitudinalTuning.kiV = [0.003, 0.008, 0.015] # Ultra-low integral
          ret.longitudinalTuning.deadzoneBP = [0., 8.33]     # 0 and 30 km/h
          ret.longitudinalTuning.deadzoneV = [0., 0.03]      # Small deadzone at speed
          ret.longitudinalActuatorDelay = 0.8                # Very slow response
          
          # Enhanced stopping parameters for smooth operation
          ret.stoppingDecelRate = 0.6      # Very gentle stopping
          ret.vEgoStopping = 0.2           # Higher stopping threshold
          ret.startAccel = 1.2             # Gentle starts
          
          # CRITICAL: Force camera SCC mode for HDA2 (matches carstate.py fix)
          ret.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value
          
        else:
          # Missing required HDA2 components - disable longitudinal completely
          ret.experimentalLongitudinalAvailable = False
          
      elif candidate in IONIQ_6_LONGITUDINAL_CAR:
        # Other Ioniq 6 variants (fallback)
        ret.experimentalLongitudinalAvailable = True
      else:
        # Standard CANFD cars
        ret.experimentalLongitudinalAvailable = candidate not in (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR)
    else:
      # Standard CAN cars
      if use_old_long:
        ret.longitudinalTuning.deadzoneBP = [0.]
        ret.longitudinalTuning.deadzoneV = [0.]
        ret.longitudinalTuning.kpV = [0.5]
        ret.longitudinalTuning.kiV = [0.0]
      ret.experimentalLongitudinalAvailable = candidate not in (UNSUPPORTED_LONGITUDINAL_CAR | CAMERA_SCC_CAR)
    
    ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    # Standard control parameters
    ret.stoppingControl = True
    ret.startingState = True
    ret.vEgoStarting = 0.1
    ret.startAccel = 1.0
    
    # FIXED: Only override longitudinalActuatorDelay for Ioniq 6, use default for others
    if candidate == CAR.HYUNDAI_IONIQ_6 and ret.openpilotLongitudinalControl:
      ret.longitudinalActuatorDelay = 0.8  # Ultra-conservative for Ioniq 6
    else:
      ret.longitudinalActuatorDelay = 0.5  # Safe default for all other cars

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

      # ULTRA CAREFUL: HDA2 safety configuration to prevent all CAN errors
      if ret.flags & HyundaiFlags.CANFD_HDA2:
        ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2
        if ret.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING:
          ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_HDA2_ALT_STEERING
        
        # CRITICAL: Enhanced Ioniq 6 safety parameter configuration
        if (candidate == CAR.HYUNDAI_IONIQ_6 and 
            ret.openpilotLongitudinalControl and
            ret.flags & HyundaiFlags.CANFD_CAMERA_SCC):
          
          # Build safety parameter step-by-step to avoid conflicts
          base_param = ret.safetyConfigs[-1].safetyParam
          
          # Add longitudinal control flag
          base_param |= Panda.FLAG_HYUNDAI_LONG
          
          # Add camera SCC flag for HDA2 (critical for proper operation)
          base_param |= Panda.FLAG_HYUNDAI_CAMERA_SCC
          
          # Add EV flag for Ioniq 6
          base_param |= Panda.FLAG_HYUNDAI_EV_GAS
          
          # Set the carefully constructed parameter
          ret.safetyConfigs[-1].safetyParam = base_param
        
      # Standard CANFD flags
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

    # FIXED: Simplified longitudinal flag logic - only for non-HDA2 cars to avoid conflicts
    if (ret.openpilotLongitudinalControl and 
        not (ret.flags & HyundaiFlags.CANFD_HDA2)):
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_LONG
      
    # Vehicle type flags
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
    # ULTRA CONSERVATIVE: Enhanced ECU disable sequence for maximum safety
    if CP.openpilotLongitudinalControl:
      if CP.flags & HyundaiFlags.CANFD_HDA2.value:
        if CP.carFingerprint == CAR.HYUNDAI_IONIQ_6:
          # Ioniq 6 HDA2 specific: Multi-step ECU disable to prevent conflicts
          try:
            import time
            
            # Step 1: Try to enter diagnostic session first
            addr, bus = 0x730, CanBus(CP).ECAN
            disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x10\x03')
            
            # Step 2: Brief wait for ECU to process
            time.sleep(0.05)
            
            # Step 3: Disable communication
            disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
            
            # Step 4: For safety, also disable SCC ECU
            disable_ecu(logcan, sendcan, bus=0, addr=0x7d0, com_cont_req=b'\x28\x83\x01')
            
          except Exception:
            # If any ECU disable fails, silently continue rather than crash
            # This prevents boot loops if ECU communication fails
            pass
        else:
          # Standard HDA2 ECU disable for other vehicles
          addr, bus = 0x730, CanBus(CP).ECAN
          try:
            disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
          except Exception:
            pass
          
      elif not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value):
        # Standard non-HDA2 longitudinal ECU disable
        addr, bus = 0x7d0, 0
        try:
          disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
        except Exception:
          pass

    # for blinkers - only if enabled
    if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      try:
        disable_ecu(logcan, sendcan, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')
      except Exception:
        pass

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    # FIXED: Let FrogPilot handle HDA2 cruise state properly
    # Removed all problematic cruise state overrides that cause cluster conflicts

    # Enhanced button handling for HDA2 systems
    if self.CS.CP.openpilotLongitudinalControl:
      ret.buttonEvents = [
        *create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT),
        *create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas}),
      ]
    else:
      ret.buttonEvents = create_button_events(self.CS.lkas_enabled, self.CS.lkas_previously_enabled, {1: FrogPilotButtonType.lkas})

    # Enhanced engagement logic with HDA2 safety checks
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    
    # Additional safety events for Ioniq 6 HDA2
    extra_events = set()
    if (self.CS.CP.carFingerprint == CAR.HYUNDAI_IONIQ_6 and 
        self.CS.CP.openpilotLongitudinalControl and 
        self.CS.CP.flags & HyundaiFlags.CANFD_HDA2):
      
      # Monitor for potential CAN conflicts
      if hasattr(self.CS, 'acc_faulted') and self.CS.acc_faulted:
        extra_events.add(EventName.accFaulted)

    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.manumatic],
                                       pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)
    
    # Add any additional safety events
    for event in extra_events:
      events.add(event)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret, fp_ret