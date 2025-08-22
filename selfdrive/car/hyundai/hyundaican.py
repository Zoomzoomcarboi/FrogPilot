import crcmod
from typing import Optional, Dict, Any
from openpilot.selfdrive.car.hyundai.values import CAR, HyundaiFlags
from opendbc.can.packer import CANPacker

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

# --------------------
# SCC_CONTROL support
# --------------------
_packer: Optional[CANPacker] = None

def init_can(p: CANPacker):
  global _packer
  _packer = p

def _next_cnt(prev_cnt: int) -> int:
  if prev_cnt is None:
    return 0
  return (int(prev_cnt) + 1) & 0xF

def next_scc_counter(prev_cnt: Optional[int]) -> int:
  return _next_cnt(prev_cnt if prev_cnt is not None else -1)

def create_scc_control(enabled: bool,
                       a_req: float,
                       jerk: float,
                       v_set: float,
                       gap_level: int,
                       cnt: int = 0,
                       extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
  if _packer is None:
    raise RuntimeError("hyundaican._packer is not initialized")

  # Conservative defaults
  obj_valid = 1
  obj_status = 0
  if not enabled:
    a_req = 0.0
    jerk = 0.0
  gap_level = max(1, min(int(gap_level), 4))
  counter_val = int(cnt) & 0xF

  values = {
    "ACCMode": 1 if enabled else 0,
    "ACC_ObjStatus": obj_status,
    "ACC_ObjValid": obj_valid,
    "ACC_aReq": float(a_req),
    "ACC_JerkLowerLimit": float(jerk),
    "ACC_JerkUpperLimit": float(jerk) if jerk >= 0 else float(-jerk),
    "ACC_VSet": float(v_set),
    "ACC_GapLevel": gap_level,
    "COUNTER": counter_val,
  }
  if extra:
    values.update(extra)

  msg = _packer.make_can_msg("SCC_CONTROL", 0, values)
  return {"name": "SCC_CONTROL", "bus": 0, "data": msg[2]}

# --------------------
# existing LKAS/ACC functions unchanged below
# --------------------

def create_lkas11(packer, frame, CP, apply_steer, steer_req,
                  torque_fault, lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart):
  values = {s: lkas11[s] for s in [
    "CF_Lkas_LdwsActivemode",
    "CF_Lkas_LdwsSysState",
    "CF_Lkas_SysWarning",
    "CF_Lkas_LdwsLHWarning",
    "CF_Lkas_LdwsRHWarning",
    "CF_Lkas_HbaLamp",
    "CF_Lkas_FcwBasReq",
    "CF_Lkas_HbaSysState",
    "CF_Lkas_FcwOpt",
    "CF_Lkas_HbaOpt",
    "CF_Lkas_FcwSysState",
    "CF_Lkas_FcwCollisionWarning",
    "CF_Lkas_FusionState",
    "CF_Lkas_FcwOpt_USM",
    "CF_Lkas_LdwsOpt_USM",
  ]}
  values["CF_Lkas_LdwsSysState"] = sys_state
  values["CF_Lkas_SysWarning"] = 3 if sys_warning else 0
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_steer
  values["CF_Lkas_ActToi"] = steer_req
  values["CF_Lkas_ToiFlt"] = torque_fault
  values["CF_Lkas_MsgCount"] = frame % 0x10

  if CP.carFingerprint in (CAR.HYUNDAI_SONATA, CAR.HYUNDAI_PALISADE, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021, CAR.HYUNDAI_SANTA_FE,
                           CAR.HYUNDAI_IONIQ_EV_2020, CAR.HYUNDAI_IONIQ_PHEV, CAR.KIA_SELTOS, CAR.HYUNDAI_ELANTRA_2021, CAR.GENESIS_G70_2020,
                           CAR.HYUNDAI_ELANTRA_HEV_2021, CAR.HYUNDAI_SONATA_HYBRID, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022,
                           CAR.HYUNDAI_SANTA_FE_2022, CAR.KIA_K5_2021, CAR.HYUNDAI_IONIQ_HEV_2022, CAR.HYUNDAI_SANTA_FE_HEV_2022,
                           CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.KIA_STINGER_2022, CAR.KIA_K5_HEV_2020, CAR.KIA_CEED,
                           CAR.HYUNDAI_AZERA_6TH_GEN, CAR.HYUNDAI_AZERA_HEV_6TH_GEN, CAR.HYUNDAI_CUSTIN_1ST_GEN):
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

  elif CP.carFingerprint in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL):
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0
    values["CF_Lkas_LdwsSysState"] = 3 if enabled else 1
    values["CF_Lkas_LdwsOpt_USM"] = 2
    values["CF_Lkas_LdwsActivemode"] = 0
    values["CF_Lkas_FcwOpt_USM"] = 0

  elif CP.carFingerprint == CAR.HYUNDAI_GENESIS:
    values["CF_Lkas_LdwsActivemode"] = 2

  dat = packer.make_can_msg("LKAS11", 0, values)[2]

  if CP.flags & HyundaiFlags.CHECKSUM_CRC8:
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif CP.flags & HyundaiFlags.CHECKSUM_6B:
    checksum = sum(dat[:6]) % 256
  else:
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", 0, values)

# ... rest of your original functions remain unchanged ...
