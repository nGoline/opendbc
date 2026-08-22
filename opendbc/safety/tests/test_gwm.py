#!/usr/bin/env python3
import unittest

import numpy as np

from opendbc.car.gwm.values import CarControllerParams
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed

STEER_CMD = 0x12b


class PandaLateralLimits:
  """The lateral accel/jerk the SAFETY C uses. It hardcodes them rather than taking
  them from CarControllerParams, so the panda stays deliberately looser than
  openpilot - our car-side values are tighter on purpose."""
  class ANGLE_LIMITS:
    MAX_LATERAL_ACCEL = 3.0 + (9.81 * 0.06)   # ISO_LATERAL_ACCEL + EARTH_G * AVERAGE_ROAD_ROLL
    MAX_LATERAL_JERK = 3.0 + (9.81 * 0.06)
  STEER_STEP = 2


class TestGwmSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (STEER_CMD,)}
  FWD_BLACKLISTED_ADDRS = {2: [STEER_CMD]}
  TX_MSGS = [[STEER_CMD, 0]]

  MAIN_BUS = 0
  CAM_BUS = 2

  STEER_ANGLE_MAX = 300
  DEG_TO_CAN = 10

  # Vehicle-model limits (v2), as Tesla does. The breakpoint-table test in the base
  # class does not apply; lateral accel and jerk are tested directly instead.
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  LATERAL_FREQUENCY = 50  # Hz
  cnt_angle_cmd = 0

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, PandaLateralLimits)

  def setUp(self):
    from opendbc.car.gwm.interface import CarInterface
    self.VM = VehicleModel(CarInterface.get_non_essential_params("GWM_HAVAL_H6_PHEV19_MK4"))
    self.packer = CANPackerSafety("gwm_haval_h6_phev_mk4")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {"STEER_REQUEST": angle, "EPS_LKAS_ANGLE_ENABLE": 0x3F if enabled else 0x1F}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("STEER_CMD", self.MAIN_BUS, values)

  def _torque_driver_msg(self, torque: int):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    # magnitude plus a direction bit, 1 = right = negative in openpilot's convention
    values = {"STEERING_ANGLE": abs(angle), "STEERING_DIRECTION": 1 if angle < 0 else 0}
    return self.packer.make_can_msg_safety("STEER_ANGLE", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ENGAGED": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC", self.CAM_BUS, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_FL": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": 1 if brake else 0}
    return self.packer.make_can_msg_safety("BRAKE", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_USER": gas}
    return self.packer.make_can_msg_safety("GAS", self.MAIN_BUS, values)

  # --- vehicle-model lateral limits -------------------------------------------
  # Unlike Tesla's signal, STEER_REQUEST (0.1, -779.6) represents 0 exactly, so the
  # grid is symmetric about zero and needs no sign-dependent unit offset.

  WHEEL_SPEED_SCALE = 0.05924739  # km/h per count, matches the DBC

  def _speed_pair(self, v):
    """(speed to send, speed the panda will use). The panda fudges 1 m/s off, and
    1 m/s is not a whole number of counts on our grid, so derive it explicitly
    rather than quantising before adding as the Tesla tests can."""
    sent = round_speed(away_round((v + 1.0) * 3.6 / self.WHEEL_SPEED_SCALE)
                       * self.WHEEL_SPEED_SCALE / 3.6)
    return sent, max(sent - 1.0, 1.0)

  def _grid(self, deg, units=0):
    return (away_round(deg / 0.1 + 1e-5) + units) * 0.1

  def test_angle_cmd_when_enabled(self):
    pass  # lateral accel and jerk are tested directly below

  def test_lateral_accel_limit(self):
    for v in np.linspace(1, 40, 40):
      sent, speed = self._speed_pair(v)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(sent)
        raw = get_max_angle_vm(speed, self.VM, PandaLateralLimits)

        at_limit = float(np.clip(self._grid(raw) * sign, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX))
        self.safety.set_desired_angle_last(round(at_limit * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(at_limit, True)),
                        f"rejected at the accel limit: {speed=:.2f} {sign=} {at_limit=:.2f}")

        over_raw = self._grid(raw, 2)
        over = float(np.clip(over_raw * sign, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX))
        self.safety.set_desired_angle_last(round(over * self.DEG_TO_CAN))
        self._tx(self._angle_cmd_msg(over, True))
        should_tx = over_raw >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(over, True)),
                         f"accel limit not enforced: {speed=:.2f} {sign=} {over=:.2f}")

  def test_lateral_jerk_limit(self):
    for v in np.linspace(1, 40, 40):
      sent, speed = self._speed_pair(v)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(sent)
        raw = get_max_angle_delta_vm(speed, self.VM, PandaLateralLimits)
        if self._grid(raw) >= get_max_angle_vm(speed, self.VM, PandaLateralLimits):
          continue

        self.safety.set_desired_angle_last(0)
        step = self._grid(raw) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(step, True)),
                        f"rejected at the jerk limit: {speed=:.2f} {sign=} {step=:.2f}")

        self.safety.set_desired_angle_last(0)
        too_fast = self._grid(raw, 2) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(too_fast, True)),
                         f"jerk limit not enforced: {speed=:.2f} {sign=} {too_fast=:.2f}")

  def test_firm_grab_disengages(self):
    # a firm driver grab drops controls on the rising edge, independently of openpilot
    # 150-400 hands lateral control back but must NOT disengage; only a hard grab does
    for tq, should_disengage in ((100, False), (250, False), (400, False), (500, True)):
      self.safety.set_controls_allowed(True)
      for _ in range(6):
        self._rx(self._torque_driver_msg(tq))
      self.assertEqual(not should_disengage, self.safety.get_controls_allowed(),
                       f"driver torque {tq}: controls_allowed wrong")

  def test_cruise_engaged_is_read_from_the_camera_bus(self):
    # the forward camera transmits 0x2ab; once the relay opens it is not on bus 0
    # at all, so a bus-0 read would leave controls_allowed stuck off forever
    self.safety.set_controls_allowed(False)
    for _ in range(10):
      self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

    self.safety.set_controls_allowed(False)
    msg = self._pcm_status_msg(True)
    msg[0].bus = self.MAIN_BUS
    for _ in range(10):
      self._rx(msg)
    self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
