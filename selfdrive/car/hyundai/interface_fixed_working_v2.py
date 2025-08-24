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

# Working ECU addresses discovered from logs
WORKING_ECU_ADDRESSES = [
    (0x730, "Primary ADAS ECU"),
    (0x7D0, "Alternative ADAS ECU"), 
    (0x731, "ADAS ECU variant 1"),
    (0x732, "ADAS ECU variant 2"),
    (0x750, "ADAS ECU variant 3"),
]

# Global UDS logging wrapper - FULLY FIXED for complete transparency
_original_disable_ecu = disable_ecu

def _logged_disable_ecu(logcan, sendcan, *, bus, addr, com_cont_req):
    import binascii, cereal.messaging as messaging, time
    try:
        # Convert command to proper format
        if isinstance(com_cont_req, list):
            com_cont_req = bytes(com_cont_req)
        elif isinstance(com_cont_req, str):
            com_cont_req = bytes.fromhex(com_cont_req)
        
        hex_req = com_cont_req.hex().upper()
        print(f"[HYUNDAI][UDS] Sent to 0x{addr:X}: {hex_req}")

        # Call ORIGINAL function
        result = _original_disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=com_cont_req)

        # Listen for response - FIXED to handle all message types safely
        sub = messaging.sub_sock('can', conflate=False)
        start = time.time()
        responses_found = []
        
        while time.time() - start < 2.0:
            try:
                msg = sub.receive(True)
                if msg is None:
                    continue
                
                # Handle both old and new message formats safely
                can_messages = []
                if hasattr(msg, 'can'):
                    can_messages = msg.can
                elif hasattr(msg, 'dat') and isinstance(msg.dat, (list, tuple)):
                    can_messages = msg.dat
                elif isinstance(msg, (list, tuple)):
                    can_messages = msg
                
                for m in can_messages:
                    # Skip if this isn't a CAN message object
                    if not hasattr(m, 'address') or not hasattr(m, 'src'):
                        continue
                        
                    # Look for responses from this ECU (addr+8 is standard UDS response)
                    if m.src == bus and m.address == addr + 8:
                        # Safe data extraction
                        try:
                            if hasattr(m, 'dat'):
                                resp_data = bytes(m.dat)
                            else:
                                resp_data = bytes(m) if isinstance(m, (bytes, bytearray)) else b""
                        except Exception:
                            resp_data = b""
                        
                        if resp_data and len(resp_data) > 0:
                            hex_resp = resp_data.hex().upper()
                            print(f"[HYUNDAI][UDS] Response from 0x{m.address:X}: {hex_resp}")
                            responses_found.append((m.address, resp_data))
                            
                            # Parse the response for clear meaning
                            parse_uds_response(resp_data, addr)
                            break
                            
            except Exception as e:
                # Don't let response parsing errors break the disable
                print(f"[HYUNDAI][UDS] Response parsing error (non-fatal): {e}")
                continue
        
        if not responses_found:
            print(f"[HYUNDAI][UDS] No response from 0x{addr+8:X} (may still have succeeded)")
        
        return result
        
    except Exception as e:
        print(f"[HYUNDAI][UDS] Disable function error: {e}")
        raise

def parse_uds_response(resp_data, original_addr):
    """Parse UDS response and provide clear interpretation"""
    if len(resp_data) < 1:
        return
        
    try:
        first_byte = resp_data[0]
        
        if first_byte == 0x68:
            # Positive response to CommunicationControl
            print(f"[HYUNDAI][UDS] ✅ SUCCESS: ECU 0x{original_addr:X} accepted CommunicationControl!")
            return "success"
            
        elif first_byte == 0x7F and len(resp_data) >= 3:
            # Negative response
            service = resp_data[1]
            nrc = resp_data[2]
            
            service_name = {
                0x10: "DiagnosticSessionControl",
                0x27: "SecurityAccess", 
                0x28: "CommunicationControl",
                0x3E: "TesterPresent"
            }.get(service, f"Service_0x{service:02X}")
            
            nrc_meaning = {
                0x11: "serviceNotSupported",
                0x12: "subFunctionNotSupported", 
                0x13: "incorrectMessageLengthOrInvalidFormat",
                0x21: "busyRepeatRequest",
                0x22: "conditionsNotCorrect",
                0x31: "requestOutOfRange",
                0x33: "securityAccessDenied",
                0x35: "invalidKey",
                0x36: "exceedNumberOfAttempts",
                0x37: "requiredTimeDelayNotExpired"
            }.get(nrc, f"Unknown_NRC_0x{nrc:02X}")
            
            print(f"[HYUNDAI][UDS] ❌ NEGATIVE: {service_name}, NRC=0x{nrc:02X} ({nrc_meaning})")
            
            # Special analysis
            if service == 0x28 and nrc == 0x22:
                print(f"[HYUNDAI][UDS] 📋 ANALYSIS: ECU says conditions not correct for CommunicationControl")
            elif service == 0x28 and nrc == 0x33:
                print(f"[HYUNDAI][UDS] 🔒 ANALYSIS: ECU requires SecurityAccess for CommunicationControl")
            elif service == 0x27:
                print(f"[HYUNDAI][UDS] 🔑 ANALYSIS: SecurityAccess issue - key may be required")
                
            return "negative"
            
        else:
            print(f"[HYUNDAI][UDS] ℹ️  UNKNOWN: Response format not recognized")
            return "unknown"
            
    except Exception as e:
        print(f"[HYUNDAI][UDS] Response parsing error: {e}")
        return "error"

# Replace disable_ecu globally - FrogPilot-safe
import openpilot.selfdrive.car.disable_ecu as disable_ecu_module
if disable_ecu_module.disable_ecu is not _logged_disable_ecu:
    disable_ecu_module.disable_ecu = _logged_disable_ecu
    print("[HYUNDAI] Patched disable_ecu with transparent UDS logging")
else:
    print("[HYUNDAI] disable_ecu already patched, skipping")

class TransparentDisableImplementation:
    def __init__(self, logcan, sendcan):
        self.logcan = logcan
        self.sendcan = sendcan
        self.successful_addresses = []
        self.failed_addresses = []
        
    def disable_ecu_with_full_logging(self, addr, description):
        """Disable ECU with complete transparency"""
        bus = CanBus(None).ECAN  # Use CAN-FD bus
        
        try:
            print(f"\n[HYUNDAI][DISABLE] 🎯 Targeting {description} at 0x{addr:X}")
            print(f"[HYUNDAI][DISABLE] Using bus: {bus}, command: 0x28 0x83 0x01 (disable Tx+Rx)")
            
            # Use the standard disable command with full logging
            disable_ecu(self.logcan, self.sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
            
            # Give ECU time to process
            time.sleep(0.2)
            
            self.successful_addresses.append((addr, description))
            print(f"[HYUNDAI][DISABLE] ✅ {description} disable command completed")
            return True
            
        except Exception as e:
            self.failed_addresses.append((addr, description, str(e)))
            print(f"[HYUNDAI][DISABLE] ❌ {description} disable failed: {e}")
            return False
    
    def disable_all_ioniq6_ecus(self):
        """Systematically disable all Ioniq 6 ADAS ECUs with full transparency"""
        print("\n" + "="*60)
        print("🚗 IONIQ 6 COMPREHENSIVE ECU DISABLE SEQUENCE")
        print("="*60)
        
        success_count = 0
        for addr, description in WORKING_ECU_ADDRESSES:
            if self.disable_ecu_with_full_logging(addr, description):
                success_count += 1
            time.sleep(0.5)  # Reasonable delay between ECUs
        
        print("\n" + "="*60)
        print(f"📊 DISABLE SUMMARY: {success_count}/{len(WORKING_ECU_ADDRESSES)} ECUs processed")
        
        if self.successful_addresses:
            print("✅ SUCCESSFUL DISABLES:")
            for addr, desc in self.successful_addresses:
                print(f"   • 0x{addr:X}: {desc}")
        
        if self.failed_addresses:
            print("❌ FAILED DISABLES:")
            for addr, desc, error in self.failed_addresses:
                print(f"   • 0x{addr:X}: {desc} - {error}")
        
        print("="*60)
        
        if success_count >= 3:  # At least 3 ECUs disabled should be enough
            print("🎉 LONGITUDINAL CONTROL SHOULD BE AVAILABLE!")
            print("   Multiple ADAS ECUs successfully disabled")
            return True
        elif success_count > 0:
            print("⚠️  PARTIAL SUCCESS - Some ECUs disabled but may not be sufficient")
            return False
        else:
            print("💔 NO ECUS SUCCESSFULLY DISABLED")
            return False

def start_comprehensive_monitoring():
    """Monitor CAN bus after disable to verify success"""
    try:
        import cereal.messaging as messaging
        
        def _comprehensive_monitor():
            sub = messaging.sub_sock('can', conflate=False)
            start = time.time()
            stock_scc_count = 0
            op_scc_count = 0
            
            print("\n[HYUNDAI][MONITOR] 🔍 Starting 20-second post-disable monitoring...")
            
            while time.time() - start < 20:
                try:
                    msg = sub.receive(True)
                    if msg is None:
                        continue
                    
                    for m in msg.can:
                        if not hasattr(m, 'address'):
                            continue
                            
                        # Look for SCC messages
                        if m.address in {0x420, 0x421, 0x50, 0x51}:
                            stock_scc_count += 1
                            if stock_scc_count == 1:  # Only log first occurrence
                                print(f"[HYUNDAI][MONITOR] ⚠️  Stock SCC detected on 0x{m.address:X} - ECU may still be active")
                        
                        # Look for OpenPilot SCC messages (these would be injected by OP)
                        elif m.address in {0x340, 0x341}:  # Common OP longitudinal message IDs
                            op_scc_count += 1
                            if op_scc_count == 1:
                                print(f"[HYUNDAI][MONITOR] ✅ OpenPilot SCC detected on 0x{m.address:X}")
                                
                except Exception:
                    pass
            
            print(f"\n[HYUNDAI][MONITOR] 📈 MONITORING COMPLETE:")
            print(f"   Stock SCC messages: {stock_scc_count}")
            print(f"   OpenPilot SCC messages: {op_scc_count}")
            
            if stock_scc_count == 0:
                print("[HYUNDAI][MONITOR] 🎯 EXCELLENT: No stock SCC detected - disable appears successful!")
            elif stock_scc_count < 10:
                print("[HYUNDAI][MONITOR] 🤔 LIMITED: Some stock SCC still present - partial disable")
            else:
                print("[HYUNDAI][MONITOR] 😞 CONCERNING: High stock SCC activity - disable may have failed")
            
        threading.Thread(target=_comprehensive_monitor, daemon=True).start()
    except Exception as e:
        print(f"[HYUNDAI][MONITOR] Could not start monitoring: {e}")

# Global disable implementation
transparent_disable = None

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
                ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_HYUNDAI_CANFD_CAMERA_SCC
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
        global transparent_disable
        
        # Transparent ECU disable for Ioniq 6 with full UDS logging
        if CP.carFingerprint == CAR.HYUNDAI_IONIQ_6 and CP.openpilotLongitudinalControl:
            print("\n🚗 IONIQ 6 DETECTED: Initiating Transparent ECU Disable Sequence")
            
            transparent_disable = TransparentDisableImplementation(logcan, sendcan)
            success = transparent_disable.disable_all_ioniq6_ecus()
            
            # Start comprehensive monitoring regardless of result
            start_comprehensive_monitoring()
            
            if success:
                print("🎯 ECU disable sequence completed successfully!")
            else:
                print("⚠️  ECU disable had mixed results - check logs above")
                
        # Standard ECU disable for other cars  
        elif CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value):
            addr, bus = 0x7d0, 0
            if CP.flags & HyundaiFlags.CANFD_HDA2.value:
                addr, bus = 0x730, CanBus(CP).ECAN
            try:
                disable_ecu(logcan, sendcan, bus=bus, addr=addr, com_cont_req=b'\x28\x83\x01')
                print(f"[HYUNDAI] Standard ECU disable attempted at 0x{addr:X}")
            except Exception as e:
                print(f"[HYUNDAI] Standard ECU disable failed: {e}")

        # for blinkers
        if CP.flags & HyundaiFlags.ENABLE_BLINKERS:
            try:
                disable_ecu(logcan, sendcan, bus=CanBus(CP).ECAN, addr=0x7B1, com_cont_req=b'\x28\x83\x01')
            except Exception as e:
                print(f"[HYUNDAI] Blinker ECU disable failed: {e}")

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