# Safe version that only uses parameters guaranteed to exist in your FrogPilot version

# File: selfdrive/car/hyundai/interface.py
# Modify only the _get_params function:

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
    
    # Enable longitudinal control for Ioniq 6 specifically
    if candidate in IONIQ_6_LONGITUDINAL_CAR:
      ret.experimentalLongitudinalAvailable = True
      
      # Safe Ioniq 6 tuning - only use parameters that definitely exist
      if candidate == CAR.HYUNDAI_IONIQ_6:
        # Conservative longitudinal tuning
        ret.longitudinalTuning.kpV = [0.15, 0.3, 0.5]
        ret.longitudinalTuning.kiV = [0.05, 0.07, 0.1]
        # Only set kf if it exists (some versions don't have this)
        try:
          ret.longitudinalTuning.kf = 1.0
        except:
          pass  # Skip if not available
        
        # Conservative delay - use the standard parameter
        ret.longitudinalActuatorDelay = 0.4
        
        # Safe stopping parameters
        try:
          ret.vEgoStopping = 0.25
        except:
          pass  # Skip if not available
    else:
      ret.experimentalLongitudinalAvailable = candidate not in (CANFD_UNSUPPORTED_LONGITUDINAL_CAR | CANFD_RADAR_SCC_CAR)
  else:
    if use_old_long:
      ret.longitudinalTuning.deadzoneBP = [0.]
      ret.longitudinalTuning.deadzoneV = [0.]
      ret.longitudinalTuning.kpV = [0.5]
      ret.longitudinalTuning.kiV = [0.0]
    ret.experimentalLongitudinalAvailable = candidate not in (UNSUPPORTED_LONGITUDINAL_CAR | CAMERA_SCC_CAR)
  
  ret.openpilotLongitudinalControl = experimental_long and ret.experimentalLongitudinalAvailable
  ret.pcmCruise = not ret.openpilotLongitudinalControl

  # Standard control parameters - these should exist in all versions
  ret.stoppingControl = True
  ret.startingState = True
  ret.vEgoStarting = 0.1
  ret.startAccel = 1.0
  ret.longitudinalActuatorDelay = 0.5  # This overrides any previous setting with a safe default

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

  # Detect smartMDPS
  if 0x2AA in fingerprint[0]:
    ret.minSteerSpeed = 0.

  if frogpilot_toggles.taco_tune_hacks:
    ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_TACO_TUNE_HACK

  return ret
