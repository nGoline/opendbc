import numpy as np

from opendbc.car import CanBusBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm.gwmcan import create_steer_command
from opendbc.car.gwm.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # Build the vehicle model from the platform's static params so the limits here
  # are the same ones the panda computes.
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
    self.VM = VehicleModel(get_safety_CP())
    self.override_active = False
    self.override_counter = 0
    self.override_hold = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # Driver override: STOP COMMANDING, the way Tesla does. Debounced so brief
    # spikes do not trigger it, and held for ~1 s afterwards - without the hold the
    # latch clears the instant torque dips, openpilot grabs the wheel straight back
    # and the two of you oscillate.
    tq = abs(CS.out.steeringTorque)
    if tq > CarControllerParams.OVERRIDE_TORQUE:
      self.override_counter = min(CarControllerParams.OVERRIDE_FRAMES, self.override_counter + 1)
    else:
      self.override_counter = max(0, self.override_counter - 1)

    if self.override_active:
      self.override_hold = max(0, self.override_hold - 1)
      if self.override_counter <= 0 and self.override_hold <= 0:
        self.override_active = False
    elif self.override_counter >= CarControllerParams.OVERRIDE_FRAMES:
      self.override_active = True
      self.override_hold = CarControllerParams.OVERRIDE_HOLD_FRAMES

    lat_active = CC.latActive and not self.override_active

    if self.frame % CarControllerParams.STEER_STEP == 0:
      # Rate-limit through the vehicle model. When lat is not active this returns
      # the measured wheel angle, which is both what the panda requires of an
      # inactive command and what makes the hand-back bumpless: the command sits
      # exactly where the wheel already is.
      apply_angle = apply_steer_angle_limits_vm(
        CC.actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, lat_active, CarControllerParams, self.VM,
      )

      # Clamp to the measured wheel LAST. Clamping the rate limiter's input bounds
      # nothing: when the driver moves the wheel faster than the command may slew,
      # the command cannot follow and the error runs away regardless of the input.
      # This has to be the final operation on the value that is transmitted.
      if lat_active:
        apply_angle = float(np.clip(apply_angle,
                                    CS.out.steeringAngleDeg - CarControllerParams.MAX_ANGLE_ERROR,
                                    CS.out.steeringAngleDeg + CarControllerParams.MAX_ANGLE_ERROR))

      # only transmit once we have heard the camera's frame to patch. Without it
      # there is nothing to copy the bypassed bytes from, and no counter to step.
      if CS.stock_steer_cmd is not None:
        frame = create_steer_command(CS.stock_steer_cmd, apply_angle, lat_active)
        can_sends.append([STEER_CMD_ADDR, frame, self.main_bus])

      self.apply_angle_last = apply_angle

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
