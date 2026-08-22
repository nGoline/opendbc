#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.can.dbc import DBC
from opendbc.car.gwm.values import CAR, GwmSafetyFlags
from opendbc.car.gwm.interface import CarInterface
from opendbc.car.lateral import AngleSteeringLimits, get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety, away_round
from opendbc.car.gwm.gwmcan import checksum as _checksum


def round_angle_mk4(apply_angle, can_offset=0):
  # STEER_REQUEST: factor 0.1, offset -779.6 (mirrors test_tesla.round_angle)
  apply_angle_can = (apply_angle + 779.6) / 0.1 + can_offset
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 779.6


class SafetyAngleParams:
  """Mirror of the panda-side vm angle limits (opendbc/safety/lateral.h steer_angle_cmd_checks_vm):
  ISO lateral accel/jerk plus the average-road-roll allowance. The car-side CarControllerParams is
  intentionally tighter (3.0 / 2.5), so the safety boundary must be tested with these values."""
  STEER_STEP = 2  # 50 Hz command rate at the 100 Hz control step
  ANGLE_LIMITS = AngleSteeringLimits(360, ([], []), ([], []),
                                     MAX_LATERAL_ACCEL=3.0 + 9.81 * 0.06,
                                     MAX_LATERAL_JERK=3.0 + 9.81 * 0.06)


opendbc = "gwm_haval_h6_mk3_generated"
dbc = DBC(opendbc)


def get_signal_range(signal):
  if signal.is_signed:
    # For signed: -(2^(bits-1)) a (2^(bits-1)-1)
    max_val = (2 ** (signal.size - 1)) - 1
    min_val = -(2 ** (signal.size - 1))
  else:
    # For unsigned: 0 a (2^bits - 1)
    max_val = (2 ** signal.size) - 1
    min_val = 0

  # Apply factor and offset
  physical_max = max_val * signal.factor + signal.offset
  physical_min = min_val * signal.factor + signal.offset

  return physical_min, physical_max


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == 0xA1: # STEER_AND_AP_STALK
    ret[0] = _checksum(ret[1:], 0x2D)
  elif addr == 0x13B: # WHEEL_SPEEDS
    ret[0] = _checksum(ret[1:8], 0x7F)
  elif addr == 0x147: # RX_STEER_RELATED, block B
    ret[8] = _checksum(ret[9:16], 0x61)
  elif addr == 0x60: # CAR_OVERALL_SIGNALS2 (gas), block B
    ret[8] = _checksum(ret[9:16], 0x95)

  return addr, ret, bus


class TestGwmSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest, common.LongitudinalGasBrakeSafetyTest):
  TX_MSGS = [[0x12B, 0], [0x143, 0], [0x147, 2], [0xA1, 2], [0x23D, 0], [0x2AB, 0]]  # steer, long, wheel touch, cancel, HUD, ACC cluster
  RELAY_MALFUNCTION_ADDRS = {0: (0x12B, 0x143, 0x23D, 0x2AB), 2: (0x147,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12B, 0x143, 0x23D, 0x2AB]}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [253]
  MAX_RT_DELTA = 100
  MAX_TORQUE_ERROR = 80
  TORQUE_MEAS_TOLERANCE = 1

  MIN_GAS = -10
  MAX_GAS = 4577
  MIN_POSSIBLE_GAS = -11
  MAX_POSSIBLE_GAS = 4600  # reasonably excessive limits, not signal max
  INACTIVE_GAS = 0

  MAX_BRAKE = 107
  MAX_POSSIBLE_BRAKE = 180

  def setUp(self):
    self.packer = CANPackerSafety(opendbc)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, GwmSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"GAS_POSITION": gas}
    return self.packer.make_can_msg_safety("CAR_OVERALL_SIGNALS2", 0, values, fix_checksum=checksum)

  def _user_brake_msg(self, brake):
    values = {"PEDAL_BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("BRAKE2", 0, values)

  def _speed_msg(self, speed):
    values = {f"{pos}_WHEEL_SPEED": speed * 1.0 for pos in ["FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"AP_ENABLE_COMMAND": enable, "AP_CANCEL_COMMAND": not enable}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, values, fix_checksum=checksum)

  def test_main_cancel_button(self):
    self.safety.set_controls_allowed(True)
    self._rx(self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, {"AP_CANCEL_COMMAND": 1}, fix_checksum=checksum))
    self.assertFalse(self.safety.get_controls_allowed())

  def _torque_meas_msg(self, torque):
    # 11-bit signed signal clip to not produce errors on test
    torque_signal = dbc.name_to_msg["RX_STEER_RELATED"].sigs["B_RX_EPS_TORQUE"]
    min_torque, max_torque = get_signal_range(torque_signal)
    torque = max(min(torque, max_torque), min_torque)

    values = {"B_RX_EPS_TORQUE": torque}
    return self.packer.make_can_msg_safety("RX_STEER_RELATED", 0, values, fix_checksum=checksum)

  def _torque_cmd_msg(self, torque, steer_req=1):
    # 10-bit signed signal clip to not produce errors on test
    torque_signal = dbc.name_to_msg["STEER_CMD"].sigs["TORQUE_CMD"]
    min_torque, max_torque = get_signal_range(torque_signal)
    torque = max(min(torque, max_torque), min_torque)

    values = {"STEER_REQUEST": steer_req, "TORQUE_CMD": torque}
    return self.packer.make_can_msg_safety("STEER_CMD", 0, values)

  def _send_brake_msg(self, brake):
    values = {"BRAKE_CMD": -brake, "GAS_CMD": 0}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def _send_gas_msg(self, gas):
    values = {"GAS_CMD": gas, "BRAKE_CMD": 0}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def test_rx_hook(self):
    # speed
    self.assertTrue(self._rx(self._speed_msg(0)))
    # invalidate checksum
    msg = self._speed_msg(0)
    msg[0].data[0] = 0xFF
    self.assertFalse(self._rx(msg))

    # cruise
    self.assertTrue(self._rx(self._pcm_status_msg(0)))
    # invalidate checksum
    msg = self._pcm_status_msg(0)
    msg[0].data[0] = 0xFF
    self.assertFalse(self._rx(msg))

    # eps torque feedback (0x147 block B checksum)
    self.assertTrue(self._rx(self._torque_meas_msg(0)))
    msg = self._torque_meas_msg(0)
    msg[0].data[8] ^= 0xFF
    self.assertFalse(self._rx(msg))

    # gas (0x60 block B checksum)
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    msg = self._user_gas_msg(0)
    msg[0].data[8] ^= 0xFF
    self.assertFalse(self._rx(msg))


class TestGwmOpCruiseSafety(unittest.TestCase):
  """MK4 owns its own cruise loop (pcmCruise=False): the panda arms controls on the gentle-or-further
  DOWN stalk gesture (msg 0xC7 GEAR_STALK bit STALK_DOWN), not the FURTHER_DOWN-only msg 161 bit47 that
  the MK3 path uses. Cancel (msg 161) and brake still disarm. Uses the MK4 DBC + the OP_CRUISE flag."""

  mk4 = "gwm_haval_h6_mk4_generated"
  TX_MSGS = None  # rx-only arm test; excludes this class from the cross-mode TX scan in common.py

  def setUp(self):
    self.packer = CANPackerSafety(self.mk4)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm,
                                 GwmSafetyFlags.LONG_CONTROL | GwmSafetyFlags.OP_CRUISE)
    self.safety.init_tests()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _gear_stalk_msg(self, down):
    return self.packer.make_can_msg_safety("GEAR_STALK", 0, {"STALK_DOWN": 1 if down else 0})

  def _stalk_msg(self, enable=0, cancel=0):
    values = {"AP_ENABLE_COMMAND": enable, "AP_CANCEL_COMMAND": cancel}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, values, fix_checksum=checksum)

  def test_gentle_down_engages(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._gear_stalk_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._gear_stalk_msg(True))  # rising edge of STALK_DOWN
    self.assertTrue(self.safety.get_controls_allowed())

  def test_further_down_161_does_not_engage(self):
    # under op-cruise the legacy msg 161 bit47 (FURTHER_DOWN / AP_ENABLE) arm is disabled
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._stalk_msg(enable=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cancel_disengages(self):
    self._rx(self._gear_stalk_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._stalk_msg(cancel=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_no_engage_without_rising_edge(self):
    # a held STALK_DOWN (no rest in between) must not re-arm after a cancel
    self._rx(self._gear_stalk_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._stalk_msg(cancel=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._gear_stalk_msg(True))  # still high, no rising edge -> stays disarmed
    self.assertFalse(self.safety.get_controls_allowed())


class TestGwmMk4AngleSafety(common.AngleSteeringSafetyTest):
  """MK4 (angle control): the panda validates the 14-bit STEER_CMD angle with
  steer_angle_cmd_checks_vm (lateral accel/jerk limits via the vehicle model, Tesla-style)."""

  TX_MSGS = None  # angle-focused test class; excluded from the cross-mode TX scan in common.py

  mk4 = "gwm_haval_h6_mk4_generated"

  STEER_ANGLE_MAX = 360  # deg, matches gwm.h max_angle 3600 (0.1 deg units)
  DEG_TO_CAN = 10

  # vehicle-model path (like Tesla): v1 rate lookups unused
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  LATERAL_FREQUENCY = 50  # Hz

  cnt_angle_cmd = 0

  def setUp(self):
    self.packer = CANPackerSafety(self.mk4)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm,
                                 GwmSafetyFlags.LONG_CONTROL | GwmSafetyFlags.OP_CRUISE | GwmSafetyFlags.ANGLE_CONTROL)
    self.safety.init_tests()
    self.VM = VehicleModel(CarInterface.get_non_essential_params(CAR.GWM_HAVAL_H6_MK4))
    # every 0xA1 rx runs the cruise state machine (pcm_cruise_check), and the angle measurement
    # lives on 0xA1 — prime acc_main_on via the GEAR_STALK engage gesture so measurement frames
    # don't disengage controls mid-test (re-engagement still needs a fresh rising edge)
    self._rx(self.packer.make_can_msg_safety("GEAR_STALK", 0, {"STALK_DOWN": 0}))
    self._rx(self.packer.make_can_msg_safety("GEAR_STALK", 0, {"STALK_DOWN": 1}))

  def _speed_msg(self, speed):
    # wheel speed signal is kph (factor 0.05924739); tests pass m/s
    values = {f"{pos}_WHEEL_SPEED": speed * 3.6 for pos in ["FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, values, fix_checksum=checksum)

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {"STEER_REQUEST": angle, "EPS_LKAS_ANGLE_ENABLE": 0x3F if enabled else 0x1F}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("STEER_CMD", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEERING_ANGLE": abs(angle), "STEERING_DIRECTION": 1 if angle < 0 else 0}
    return self.packer.make_can_msg_safety("STEER_AND_AP_STALK", 0, values, fix_checksum=checksum)

  def test_angle_cmd_when_enabled(self):
    # lateral accel and jerk are properly tested below (vm limits, like Tesla).
    # NOTE: steer_angle_cmd_checks_vm does NOT enforce limits.max_angle in the active path
    # (upstream semantics, same as Tesla): the ISO lateral-accel limit is the effective cap,
    # and at the 1 m/s fudged-speed floor it sits above 360 deg (parking needs near-full lock).
    pass

  def test_lateral_accel_limit(self):
    # commanded angle must stay under the vm lateral-accel limit for the measured speed
    for speed in np.linspace(1., 40., 20):
      speed = float(speed)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # the panda fudges measured speed down by 1

        # comfortably at the limit -> allowed (2 CAN units of margin for speed quantization)
        max_angle = round_angle_mk4(get_max_angle_vm(speed, self.VM, SafetyAngleParams), -2) * sign
        max_angle = float(np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX))
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # above the limit -> blocked (unless the vm limit exceeds the 360 deg hard cap at low speed)
        above_raw = round_angle_mk4(get_max_angle_vm(speed, self.VM, SafetyAngleParams), 4) * sign
        above = float(np.clip(above_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX))
        should_tx = abs(above_raw) >= self.STEER_ANGLE_MAX
        self.safety.set_desired_angle_last(round(above * self.DEG_TO_CAN))
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(above, True)))

  def test_lateral_jerk_limit(self):
    # per-frame angle delta must stay under the vm lateral-jerk limit for the measured speed
    for speed in np.linspace(5., 40., 15):
      speed = float(speed)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        # within the per-frame delta -> allowed
        max_delta = round_angle_mk4(get_max_angle_delta_vm(speed, self.VM, SafetyAngleParams), -2) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # above the per-frame delta -> blocked
        over_delta = round_angle_mk4(get_max_angle_delta_vm(speed, self.VM, SafetyAngleParams), 4) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(over_delta, True)))
        # recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_no_angle_control_when_disallowed(self):
    # the EPS angle-enable pair must never be commanded active while controls are not allowed
    self.safety.set_controls_allowed(False)
    self._reset_angle_measurement(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
    # passthrough idle frame (enable low, angle at measured wheel) is fine
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))


class TestGwmMk4TxSafety(common.SafetyTest):
  """TX whitelist / forwarding / relay-malfunction checks under the full MK4 flag set
  (LONG_CONTROL | OP_CRUISE | ANGLE_CONTROL). The whitelist and fwd config are flag-independent
  in gwm_init, but only this class proves the flags don't widen them; the cruise/steering
  semantics that DO change per flag are covered by the focused classes above."""

  TX_MSGS = [[0x12B, 0], [0x143, 0], [0x147, 2], [0xA1, 2], [0x23D, 0], [0x2AB, 0]]  # steer, long, wheel touch, cancel, HUD, ACC cluster
  RELAY_MALFUNCTION_ADDRS = {0: (0x12B, 0x143, 0x23D, 0x2AB), 2: (0x147,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x147], 2: [0x12B, 0x143, 0x23D, 0x2AB]}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm,
                                 GwmSafetyFlags.LONG_CONTROL | GwmSafetyFlags.OP_CRUISE | GwmSafetyFlags.ANGLE_CONTROL)
    self.safety.init_tests()

  # attr-only test from CarSafetyTest (this class skips the rest of that suite, which needs
  # the per-platform message builders and flag-dependent cruise semantics)
  test_relay_malfunction = common.CarSafetyTest.test_relay_malfunction


if __name__ == "__main__":
  unittest.main()
