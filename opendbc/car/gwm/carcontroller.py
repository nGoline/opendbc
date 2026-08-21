from opendbc.car import CanBusBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.gwm.gwmcan import create_steer_command
from opendbc.car.gwm.values import CarControllerParams

STEER_CMD_ADDR = 0x12b


class CarController(CarControllerBase):
  """Angle-based lateral control for the Haval H6 PHEV19 MK4.

  Longitudinal is not commanded - the stock ACC keeps running and openpilot only
  steers. The command is the camera's own STEER_CMD frame, patched with our angle,
  an effort level, a stepped counter and two recomputed CRC-8s (see gwmcan).
  """

  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    # the EPS is on the main car bus; the camera's copy of STEER_CMD is blocked
    # from being forwarded there by the panda (CanMsg check_relay), so ours is the
    # only 0x12b the EPS sees while openpilot is running.
    self.main_bus = CanBusBase(CP, None).offset
    self.apply_angle_last = 0.0
    self.lkas_effort = 0          # EPS authority, percent, ramped - never stepped
    self.driver_override = False

  def update(self, CC, CS, now_nanos):
    can_sends = []

    if self.frame % CarControllerParams.STEER_STEP == 0:
      # Is the driver fighting the wheel? Hysteresis either side so a hand resting
      # near the threshold doesn't chatter the effort ramp up and down.
      driver_torque = abs(CS.out.steeringTorque)
      if driver_torque > CarControllerParams.STEER_OVERRIDE_TORQUE:
        self.driver_override = True
      elif driver_torque < CarControllerParams.STEER_OVERRIDE_RELEASE:
        self.driver_override = False

      # Effort target. Winding down while the driver pushes is what makes the wheel
      # yield instead of fighting back; the camera itself ramps this byte rather
      # than stepping it, so we do too.
      #
      # Cruise dropping is a hard stop with no ramp available: the panda rejects any
      # frame that asserts the enable bit while controls are not allowed
      # (safety/lateral.h, "No angle control allowed when controls are not allowed"),
      # so a ramp-out there would simply not be transmitted.
      if not CS.out.cruiseState.enabled:
        self.lkas_effort = 0
      else:
        if not CC.latActive:
          target = 0
        elif self.driver_override:
          target = CarControllerParams.LKAS_EFFORT_OVERRIDE
        else:
          target = CarControllerParams.LKAS_EFFORT_MAX

        if target > self.lkas_effort:
          self.lkas_effort = min(target, self.lkas_effort + CarControllerParams.LKAS_EFFORT_UP)
        else:
          self.lkas_effort = max(target, self.lkas_effort - CarControllerParams.LKAS_EFFORT_DOWN)

      # Hold the enable bit through the ramp out, so authority is withdrawn before
      # the EPS is told to let go - the order the camera uses.
      enable = CC.latActive or self.lkas_effort > 0
      ramping_out = enable and not CC.latActive

      # Rate-limit the commanded angle to the car's own envelope.
      #
      # Winding effort down does not stop us commanding - it commands the same angle
      # with less authority. Only the ramp out is a handback, and there openpilot has
      # already set actuators.steeringAngleDeg to the measured angle for us
      # (latcontrol_angle: `if not active: angle_steers_des = CS.steeringAngleDeg`),
      # so the command follows the wheel while the last of the effort fades.
      #
      # Passing lat_active through the ramp out keeps that a rate-limited approach
      # rather than a snap: the panda applies its rate limits to every frame that
      # asserts the enable bit, and a snap to a fast-moving wheel would trip them.
      apply_angle = apply_std_steer_angle_limits(
        CC.actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, CC.latActive or ramping_out, CarControllerParams.ANGLE_LIMITS,
      )

      # only transmit once we have heard the camera's frame to patch. Without it
      # there is nothing to copy the bypassed bytes from, and no counter to step.
      if CS.stock_steer_cmd is not None:
        frame = create_steer_command(CS.stock_steer_cmd, apply_angle, enable, self.lkas_effort)
        can_sends.append([STEER_CMD_ADDR, frame, self.main_bus])

      self.apply_angle_last = apply_angle

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
