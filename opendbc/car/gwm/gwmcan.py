import numpy as np
from opendbc.car import CanBusBase


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def create_steer_command_angle(packer, CAN: CanBus, camera_stock_values, apply_angle: float, lat_active: bool):
  """MK4: STEER_CMD with 14-bit STEER_REQUEST angle and two independent CRCs.

  Encoding (from DBC: factor=0.1, offset=-779.6):
    raw14 = round((angle_deg + 779.6) / 0.1)   range 0-16383 (zero-point raw14=7796)
  CRCs (both poly=0x1D):
    byte8  (CRC_X9B)    = crc8(bytes[9:16],  init=0xD1)  — covers counter
    byte16 (BYPASSME_3) = crc8(bytes[17:24], init=0x05)  — covers angle+counter2

  EPS_LKAS_ANGLE_ENABLE/CFG gate whether the EPS acts on STEER_REQUEST. They MUST
  mirror the camera's idle (passive) values while openpilot is not steering — sending
  the active pair (0x3f/0xe5) when disengaged drives the EPS into angle-control mode
  unrequested, which the camera flags as a fault and makes OEM AEB/ACC unavailable.
"""
  # STEER_REQUEST is a PHYSICAL value: the DBC signal (factor 0.1, offset -779.6) applies the
  # scaling on pack. Pre-scaling here double-applies factor/offset -> a garbage angle the EPS
  # rejects (it never grants authority, A_RX_STEER_REQUESTED stays != 1, wheel never moves).
  # Just clip to the representable physical range and let the packer build the 14-bit raw.
  steer_request = float(np.clip(apply_angle, -779.6, 16383 * 0.1 - 779.6))
  new_counter = (int(camera_stock_values["COUNTER_1"]) + 1) % 16

  if lat_active:
    angle_enable = 0x3F  # EPS executes the angle command
    angle_cfg = 0xE5     # angle-control config paired with 0x3f
  else:
    angle_enable = int(camera_stock_values.get("EPS_LKAS_ANGLE_ENABLE", 0x1F))
    angle_cfg = int(camera_stock_values.get("EPS_LKAS_ANGLE_CFG", 0x81))

  values = {
    "BYPASSME_1":             0,
    "BYPASSME_2":             camera_stock_values["BYPASSME_2"],
    "COUNTER_1":              new_counter,
    "STEER_REQUEST":          steer_request,
    "BYPASSME_4_HEAD":        int(camera_stock_values.get("BYPASSME_4_HEAD", 0x037007)),
    "EPS_LKAS_ANGLE_ENABLE":  angle_enable,
    "EPS_LKAS_ANGLE_CFG":     angle_cfg,
    "BYPASSME_4_TAIL":        int(camera_stock_values.get("BYPASSME_4_TAIL", 0xF)),
    "COUNTER_2":              new_counter,
    "BYPASSME_5":    camera_stock_values.get("BYPASSME_5", 0),
    "COUNTER_3":     camera_stock_values.get("COUNTER_3", 0),
    "COUNTER_4":     camera_stock_values.get("COUNTER_4", 0),
    "BYPASSME_6":    camera_stock_values.get("BYPASSME_6", 0),
  }

  # CRC_COUNTER (byte8) covers bytes 9-15; CRC_STEER (byte16) covers bytes 17-23.
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["CRC_STEER"]   = crc8_init(dat[17:24], 0x05)
  values["CRC_COUNTER"] = crc8_init(dat[9:16],  0xD1)

  return packer.make_can_msg("STEER_CMD", CAN.main, values)


def create_steer_command(packer, CAN: CanBus, camera_stock_values, steer: float, steer_req: bool):
  steer = int(steer)
  values = {
    "STEER_REQUEST": 1 if steer_req else 0,
    "SET_ME_X01": 1,
    "TORQUE_CMD": steer,
    "TORQUE_REFLECTED": -steer,
    "INVERT_DIRECTION": 1 if (steer > 0 and steer_req) else 0,
    "COUNTER": (camera_stock_values["COUNTER"] + 1) % 16,
    "BYPASS_ME": camera_stock_values["BYPASS_ME"],
  }

  # calculate and insert basic checksum
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["BASIC_CHECKSUM"] = gwm_basic_chksum_for_0x12B(dat)
  # calculate and insert CRC
  dat = packer.make_can_msg("STEER_CMD", 0, values)[1]
  values["CRC_X9B"] = checksum(dat[9:16], 0x9B)

  return packer.make_can_msg("STEER_CMD", CAN.main, values)


def create_longitudinal_command(packer, CAN, longitudinal_stock_values, accel, active, standstill, is_mk4: bool, regen: bool = False, braking: bool | None = None):
  # the powertrain brake-vs-gas state fields were renamed in the MK4 DBC; the MK3 DBC still calls
  # them BYPASSME_1 / BYPASS_ACC2 — reading the MK4 names unconditionally raised KeyError on MK3
  # (same crash class create_buttons_command already guards against, in the other direction)
  passthrough = ["SPEED_REAL", "COUNTER_BRAKE", "BYPASSME_2", "BYPASS_ACC1", "COUNTER_ACC"]
  passthrough += ["BRAKE_GAS_STATE_2", "BRAKE_GAS_STATE"] if is_mk4 else ["BYPASSME_1", "BYPASS_ACC2"]
  values = {s: longitudinal_stock_values[s] for s in passthrough}

  # `braking` is the gas<->brake decision. MK4 computes it in carcontroller with a lift-off threshold + hysteresis
  # (regen-lead); MK3 and any legacy caller falls back to the raw sign of accel (unchanged behavior).
  if braking is None:
    braking = accel < 0

  brake_or_gas = longitudinal_stock_values["BRAKE_OR_GAS_REQ"]
  brake_cmd = 0
  accel_cmd = 0
  if braking and active:
    brake_or_gas = 13
    brake_cmd = (accel * (107 - 41)) - 41
  elif active:
    brake_or_gas = 12
    # `accel` here is already speed-normalized in carcontroller (actuators.accel / ACCEL_MAX=2). The [0.25,1]
    # domain therefore put a ~0.5 m/s^2 throttle DEAD ZONE on the low end: GAS_CMD stayed 0 until
    # actuators.accel >= ~0.5, so a normal stop->go creep (planner accel ~0.3-0.4) commanded zero throttle
    # and the car barely launched from a stop (drive de: 66781/97435 positive-accel frames sent GAS_CMD=0).
    # MK4: map the full positive range so any positive accel demand produces throttle. MK3 unchanged.
    if is_mk4:
      accel_cmd = np.interp(accel, [0.0, 1.0], [0, 4577])
      # MK4: BRAKE_CMD's brake-OFF baseline is -41 (OEM sends -41 the whole time it's in gas mode, verified
      # route_c0 median -41 over 14k frames). Leaving it at 0 in gas mode sends a non-off brake value, so the
      # car braked while we were also commanding gas ("braking and accelerating at the same time", drive f6).
      brake_cmd = -41
    else:
      accel_cmd = np.interp(accel, [0.25, 1], [0, 4577])
  values |= {"BRAKE_OR_GAS_REQ": brake_or_gas, "BRAKE_CMD": brake_cmd, "GAS_CMD": accel_cmd}

  if is_mk4:
    # PHEV regen enable: the powertrain reads the brake-vs-gas STATE from the passthrough fields
    # BRAKE_GAS_STATE / BRAKE_GAS_STATE_2 (formerly BYPASS_ACC2 / BYPASSME_1), NOT from BRAKE_OR_GAS_REQ. The
    # builder otherwise copies the camera's stale value, so an openpilot brake request (REQ=13) on top of a
    # "gas"-state copy is a combo the OEM never sends -> the e-motor stays in drive and never flips to
    # generation (no regen). While actively commanding, drive these to the OEM's exact brake/gas-state values
    # so regen activates while braking; the regen *amount* stays the powertrain's call (the driver's
    # Low/Normal/Heavy setting). OEM-verified clean-binary across conditions: low-speed dd AND highway decel
    # route_c0/route_55 -> brake = 0 / 131068 (regen on), gas/coast = 524288 / 131070. standstill lives in
    # BYPASS_ACC1 (untouched). CRC-safe: BRAKE_GAS_STATE_2 in CRC_BRAKE_0xEF(data[9:16]), BRAKE_GAS_STATE in
    # CRC_ACC_0x87(data[25:32]); both recomputed below.
    # `regen` is gated by carcontroller's hysteresis (real braking only) so the bit does NOT flip on the
    # planner's near-zero cruise accel -> avoids the regen pulsing that made longitudinal jerky (drive e6).
    if active:
      if regen:
        values["BRAKE_GAS_STATE"] = 0
        values["BRAKE_GAS_STATE_2"] = 131068
      else:
        values["BRAKE_GAS_STATE"] = 524288
        values["BRAKE_GAS_STATE_2"] = 131070

    # MK4: standstill encoding rides in the passthrough BYPASS_ACC1 field (left untouched); only
    # STANDSTILL stays explicit. Same encoding as MK3's STANDSTILL_1 for the single signal.
    standstill1 = longitudinal_stock_values["STANDSTILL"]
    if braking and active:
      standstill1 = 1 if standstill else 0
    elif active:
      standstill1 = 0
    values["STANDSTILL"] = standstill1
  else:
    standstill1 = longitudinal_stock_values["STANDSTILL_1"]
    standstill2 = longitudinal_stock_values["STANDSTILL_2"]
    standstill3 = longitudinal_stock_values["STANDSTILL_3"]
    if braking and active:
      standstill1 = 1 if standstill else 0
      standstill2 = 3 if standstill else 4  # 3 "active" 4 "inactive"
      standstill3 = 0 if standstill else 1  # 0 "active" 1 "inactive"
    elif active:
      standstill1 = 0
      standstill2 = 4
      standstill3 = 1
    values |= {
      "STANDSTILL_1": standstill1,
      "STANDSTILL_2": standstill2,
      "STANDSTILL_3": standstill3,
    }

  data = packer.make_can_msg("ACC_CMD", 0, values)[1]
  values["CRC_BRAKE_0xEF"] = checksum(data[9:16], 0xEF)
  values["CRC_ACC_0x87"] = checksum(data[25:32], 0x87)

  return packer.make_can_msg("ACC_CMD", CAN.main, values)


def create_wheel_touch(packer, CAN: CanBus, eps_stock_values, ea_simulated_torque: float):
  values = {s: eps_stock_values[s] for s in [
    "A_CRC_X61",
    "A_BYPASSME_2",
    "A_RX_STEER_REQUESTED",
    "A_BYPASSME_1",
    "A_COUNTER",
    "B_CRC_X61",
    "B_RX_DRIVER_TORQUE",
    "B_BYPASSME_1",
    "B_BYPASSME_2",
    "B_RX_EPS_TORQUE",
    "B_COUNTER",
    "B_BYPASSME_3",
  ]}

  values.update({
    "B_RX_DRIVER_TORQUE": ea_simulated_torque,
  })

  # calculate checksum
  dat = packer.make_can_msg("RX_STEER_RELATED", 0, values)[1]
  values["B_CRC_X61"] = checksum(dat[9:16], 0x61)

  return packer.make_can_msg("RX_STEER_RELATED", CAN.camera, values)


def create_wheel_touch_mk4(CAN: CanBus, eps_stock_raw: bytes, spoof_torque: int):
  """MK4 hands-on keepalive. Re-transmit the EPS RX_STEER_RELATED (0x147) frame to the CAMERA bus with a
  spoofed driver torque, so the stock ADAS never enters its hands-off "hold the wheel" warning + safe-stop
  escalation (msg 683 byte5) and never limps the EPS mid-drive. comma's driver-monitoring camera is the real
  attention monitor (standard openpilot practice). The MK3 torque path does the equivalent via
  create_wheel_touch; MK4 uses the angle path and the MK4 DBC models only part of the 64-byte 0x147 frame, so
  we patch the RAW bytes and leave the unmodeled sub-blocks intact:
    B_RX_DRIVER_TORQUE = ((byte9 & 0x7F) << 4) | (byte10 >> 4), 11-bit signed  (verified vs CANParser)
    B_CRC_X61 (byte8)  = crc over bytes[9:16]  (same as create_wheel_touch)
  Only bytes 8/9/10 change. Sent to the camera bus only -- openpilot still reads the REAL driver torque from the
  main bus (0x147 there is untouched), so its override / steeringPressed logic is unaffected."""
  b = bytearray(eps_stock_raw)
  r = int(spoof_torque) & 0x7FF                       # 11-bit two's complement
  b[9] = (b[9] & 0x80) | ((r >> 4) & 0x7F)            # preserve bit79 (B_BYPASSME_1)
  b[10] = (b[10] & 0x0F) | ((r & 0x0F) << 4)          # preserve low nibble
  b[8] = checksum(bytes(b[9:16]), 0x61)               # recompute B-half CRC over the patched bytes
  return 0x147, bytes(b), CAN.camera


def create_buttons_command(packer, CAN: CanBus, counter, stock_msg, cancel_command=False):
  # AP_DECREASE_SPEED_COMMAND / AP_INCREASE_SPEED_COMMAND only exist on the MK3
  # STEER_AND_AP_STALK DBC, not MK4. Reading them unconditionally raised KeyError on MK4
  # and CRASHED the card daemon the first time openpilot sent a cancel (red screen, could
  # not re-engage). Guard with `if s in stock_msg` so MK4 skips the missing signals while
  # MK3 (which has all of them) is unchanged.
  values = {s: stock_msg[s] for s in [
    "STEERING_ANGLE",
    "STEERING_DIRECTION",
    "STEERING_RATE",
    "RATE_DIRECTION",
    "AP_ENABLE_COMMAND",
    "AP_DECREASE_SPEED_COMMAND",
    "AP_INCREASE_SPEED_COMMAND",
    "AP_REDUCE_DISTANCE_COMMAND",
    "AP_INCREASE_DISTANCE_COMMAND",
  ] if s in stock_msg}

  values |= {
    "AP_CANCEL_COMMAND":  stock_msg["AP_CANCEL_COMMAND"] or cancel_command,
    "COUNTER": counter,
  }

  data = packer.make_can_msg("STEER_AND_AP_STALK", 0, values)[1]
  values["CRC_X2D"] = checksum(data[1:8], 0x2D)

  return packer.make_can_msg('STEER_AND_AP_STALK', CAN.camera, values)


def create_hud_command(packer, CAN: CanBus, hud_stock_values, steer_required, is_mk4: bool):
  values = {s: hud_stock_values[s] for s in [
    "BYPASSME_1",
    "BYPASSME_2",
    "BY_PASSME",
    "COUNTER",
    "BYPASSME_3",
    "BYPASSME_4",
    "BYPASSME_5",
    "BYPASSME_6",
  ]}
  # CRUISE_STATE (MK3) / BYPASSME_CRUISE (MK4) is a camera-owned field we don't control;
  # forward it untouched so the rebuilt frame matches the OEM idle frame byte-for-byte.
  if is_mk4:
    values["BYPASSME_CRUISE"] = hud_stock_values["BYPASSME_CRUISE"]
  else:
    values["CRUISE_STATE"] = hud_stock_values["CRUISE_STATE"]

  values |= {
    "LKAS_STATE": 5 if steer_required else hud_stock_values["LKAS_STATE"],
  }

  data = packer.make_can_msg("LATERAL_STATE", 0, values)[1]
  values["CRC_X66"] = checksum(data[17:24], 0x66)

  return packer.make_can_msg("LATERAL_STATE", CAN.main, values)


def create_acc_cluster_mk4(CAN: CanBus, acc_stock_raw: bytes, set_speed_kph: float | None = None,
                          follow_dashes: int | None = None):
  """Re-TX camera ACC (0x2AB) onto the main bus with openpilot's set speed for the OEM cluster.

  Under OP_CRUISE the stock camera freezes ACC_SPEED_SELECTION, so the Haval dash never tracks
  openpilot's VCruiseHelper. We block the camera's 0x2AB from being forwarded (panda check_relay)
  and re-transmit the latest camera frame with:
    byte22 = ACC_SPEED_SELECTION (kph, factor 1)
    optional byte21 low 3 bits = CAR_DISTANCE_SELECTION (1..4 follow dashes)
    byte16 = CRC1 = crc8(bytes[17:24], xor=0x40)  — verified 600/600 frames, route 00000008

  When set_speed_kph is None the frame is forwarded byte-identical (still required while we own
  the relay slot). Follow-dashes None leaves the stock distance field untouched.
  """
  b = bytearray(acc_stock_raw)
  if set_speed_kph is not None:
    b[22] = int(round(float(np.clip(set_speed_kph, 0.0, 255.0)))) & 0xFF
  if follow_dashes is not None:
    dashes = int(np.clip(follow_dashes, 0, 7))
    b[21] = (b[21] & ~0x07) | (dashes & 0x07)
  b[16] = checksum(bytes(b[17:24]), 0x40)
  return 0x2AB, bytes(b), CAN.main


def checksum(data, xor_output):
  crc = 0
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ xor_output


def crc8_init(data, init):
  """CRC-8 poly=0x1D with non-zero init, xor_out=0. Used by MK4 STEER_CMD CRCs."""
  crc = init
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc


def gwm_basic_chksum_for_0x12B(d: bytearray) -> int:
  invert_direction = d[12] >> 7 & 0x1
  counter = d[15] & 0xF
  steer_requested = d[15] >> 5 & 0x1
  return (28 - (steer_requested * 8) - counter - invert_direction) & 0x1F
