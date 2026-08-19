#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

STEER_CMD = 0x12b


class TestGwmSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (STEER_CMD,)}
  FWD_BLACKLISTED_ADDRS = {2: [STEER_CMD]}
  TX_MSGS = [[STEER_CMD, 0]]

  MAIN_BUS = 0
  CAM_BUS = 2

  STEER_ANGLE_MAX = 300
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2., 1., .3]
  ANGLE_RATE_DOWN = [3., 1.6, .5]

  def setUp(self):
    self.packer = CANPackerSafety("gwm_haval_h6_phev_mk4")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gwm, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"STEER_REQUEST": angle, "EPS_LKAS_ANGLE_ENABLE": 0x3F if enabled else 0x1F}
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
