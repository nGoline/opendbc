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


class TestGwmSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (STEER_CMD,)}
  FWD_BLACKLISTED_ADDRS = {2: [STEER_CMD]}
  TX_MSGS = [[STEER_CMD, 0]]

  MAIN_BUS = 0
  CAM_BUS = 2

  cnt_angle_cmd = 0

  STEER_ANGLE_MAX = 300
  DEG_TO_CAN = 10

  # Vehicle-model limits (v2), as Tesla does - get_max_angle_vm / get_max_angle_delta_vm
  # supply the envelope instead of breakpoint tables.
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

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

  # --- vehicle-model lateral limits (v2), mirroring the Tesla tests -----------
  # The breakpoint-table test in the base class does not apply; lateral accel and
  # jerk are tested directly against the vehicle model instead.
  #
  # Unlike Tesla's signal, STEER_REQUEST (0.1, -779.6) represents 0 exactly, at
  # raw 7796, so the grid is symmetric about zero and needs no sign-dependent
  # unit offset. Round the magnitude onto the grid, then apply the sign.

  WHEEL_SPEED_SCALE = 0.05924739  # km/h per count, matches the DBC

  def _speed_pair(self, v):
    """(speed to send, speed the panda will actually use for its limits).

    The panda fudges the speed down by 1 m/s, so the tests send v+1. Tesla can
    quantise before adding because 1 m/s is exactly 45 counts on its 0.08 km/h
    grid; on our 0.05924739 km/h grid it is 60.76 counts, so quantising first
    leaves the panda computing at a slightly different speed than the test does.
    Limits go as 1/v^2, which is enough to miss the at-the-limit assertions.
    """
    sent = round_speed(away_round((v + 1.0) * 3.6 / self.WHEEL_SPEED_SCALE)
                       * self.WHEEL_SPEED_SCALE / 3.6)
    return sent, max(sent - 1.0, 1.0)

  def _grid(self, deg, units=0):
    """A magnitude in degrees, on the STEER_REQUEST grid, plus `units` CAN counts."""
    return (away_round(deg / 0.1 + 1e-5) + units) * 0.1

  def test_angle_cmd_when_enabled(self):
    # lateral acceleration and jerk are tested directly below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(1, 40, 40):
      sent, speed = self._speed_pair(speed)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(sent)
        raw = get_max_angle_vm(speed, self.VM, CarControllerParams)

        at_limit = float(np.clip(self._grid(raw) * sign, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX))
        self.safety.set_desired_angle_last(round(at_limit * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(at_limit, True)),
                        f"rejected at the accel limit: {speed=:.2f} {sign=} {at_limit=:.2f}")

        # 2 CAN counts over, to clear the panda's 1-count tolerance
        over_raw = self._grid(raw, 2)
        over = float(np.clip(over_raw * sign, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX))
        self.safety.set_desired_angle_last(round(over * self.DEG_TO_CAN))
        self._tx(self._angle_cmd_msg(over, True))
        # below ~7 m/s the model's max angle exceeds STEER_ANGLE_MAX, so the clip
        # swallows the excess and the command is legal again
        should_tx = over_raw >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(over, True)),
                         f"accel limit not enforced: {speed=:.2f} {sign=} {over=:.2f}")

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(1, 40, 40):
      sent, speed = self._speed_pair(speed)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(sent)
        raw = get_max_angle_delta_vm(speed, self.VM, CarControllerParams)
        # a step this large is only meaningful inside the accel envelope
        if self._grid(raw) >= get_max_angle_vm(speed, self.VM, CarControllerParams):
          continue

        self.safety.set_desired_angle_last(0)
        step = self._grid(raw) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(step, True)),
                        f"rejected at the jerk limit: {speed=:.2f} {sign=} {step=:.2f}")

        # 2 CAN counts faster than allowed, from the same starting angle
        self.safety.set_desired_angle_last(0)
        too_fast = self._grid(raw, 2) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(too_fast, True)),
                         f"jerk limit not enforced: {speed=:.2f} {sign=} {too_fast=:.2f}")

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
