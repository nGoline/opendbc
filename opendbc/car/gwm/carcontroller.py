import numpy as np

from opendbc.car import CanBusBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm.gwmcan import create_steer_command
from opendbc.car.gwm.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # Build the vehicle model from the platform's static params, not from the CP we
  # were handed, so the limits here are identical to the ones the panda computes.
  from opendbc.car.gwm.interface import CarInterface
  return CarInterface.get_non_essential_params("GWM_HAVAL_H6_PHEV19_MK4")

STEER_CMD_ADDR = 0x12b


class CarController(CarControllerBase):
  """Angle-based lateral control for the Haval H6 PHEV19 MK4.

  Longitudinal is not commanded - the stock ACC keeps running and openpilot only
  steers. The command is the camera's own STEER_CMD frame, patched with our angle,
  a stepped counter and two recomputed CRC-8s (see gwmcan.create_steer_command).
  """

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    # the EPS is on the main car bus; the camera's copy of STEER_CMD is blocked
    # from being forwarded there by the panda (CanMsg check_relay), so ours is the
    # only 0x12b the EPS sees while openpilot is running.
    self.main_bus = CanBusBase(CP, None).offset
    self.apply_angle_last = 0.0

    # Vehicle model used for lateral limiting, as Tesla does
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    can_sends = []

    if self.frame % CarControllerParams.STEER_STEP == 0:
      # Clamp the command to within MAX_ANGLE_ERROR of the measured wheel angle
      # BEFORE rate limiting, the way Ford does it. The EPS torque grows with
      # commanded-minus-measured, so this is what bounds how hard it can fight the
      # driver: push the wheel, it moves, the command follows, and no error - and
      # therefore no resistance, and nothing to snap back to - can accumulate.
      #
      # The camera does exactly this; it never exceeded 4.6 deg of error in 2.6M
      # measured frames. See CarControllerParams.MAX_ANGLE_ERROR.
      desired_angle = float(np.clip(CC.actuators.steeringAngleDeg,
                                    CS.out.steeringAngleDeg - CarControllerParams.MAX_ANGLE_ERROR,
                                    CS.out.steeringAngleDeg + CarControllerParams.MAX_ANGLE_ERROR))

      # Rate-limit via the vehicle model - constant lateral accel and jerk across
      # all speeds. When lat is not active this returns the measured wheel angle,
      # which is what the panda requires of an inactive angle command.
      apply_angle = apply_steer_angle_limits_vm(
        desired_angle, self.apply_angle_last, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM,
      )

      # only transmit once we have heard the camera's frame to patch. Without it
      # there is nothing to copy the bypassed bytes from, and no counter to step.
      if CS.stock_steer_cmd is not None:
        frame = create_steer_command(CS.stock_steer_cmd, apply_angle, CC.latActive)
        can_sends.append([STEER_CMD_ADDR, frame, self.main_bus])

      self.apply_angle_last = apply_angle

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
