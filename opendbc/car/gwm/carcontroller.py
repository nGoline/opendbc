import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams, CAR

SteerControlType = structs.CarParams.SteerControlType

LongCtrlState = structs.CarControl.Actuators.LongControlState

MAX_USER_TORQUE = 100  # 1.0 Nm

# MK4 cluster set-speed display experiment: a deliberately distinct value so one glance at the dash tells us
# whether the forged msg 683 drives the cluster, independent of whether the real set speed is plumbed right.
MK4_ACC_DISPLAY_TEST_SPEED = 88  # km/h

# MK4 override thresholds, grounded in the stock LKAS: while steering the OEM tolerates driver torque
# up to ~67 routinely (p90) and only hands off around ~102 (median), ignoring brief spikes to ~156 (p99).
# Our old fixed >50 instant release was HALF that, so resting-hand contact flapped lateral on/off and
# reset the angle slew every few frames, so openpilot never built up a turn.
# Debounce a ~90 threshold with a latch + instant release on a firm grab.
OVERRIDE_TORQUE = 120          # sustained |driver torque| to hand off.
OVERRIDE_INSTANT_TORQUE = 250  # firm deliberate grab (> p99 ~224) -> release within one frame for a clean takeover
OVERRIDE_FRAMES = 7            # ~70 ms sustained @100 Hz before engaging override; tolerates brief spikes
OVERRIDE_HOLD_FRAMES = 100     # once overridden, stay handed-off ~1.0 s before re-asserting, like the OEM LKAS.
                               # The old latch re-asserted the instant torque dipped, so openpilot grabbed back
                               # "several times a second". The OEM waits ~1 s before trying again; this hold matches that and stops the flapping.


# MK4 regen brake-state hysteresis, per driver regen LEVEL (msg 726 REGEN_LEVEL, read in carstate).
# The powertrain enables regen off the brake-vs-gas state bit; tying it to every accel<0 made it flicker around
# the planner's near-zero cruise accel -> regen pulsed on/off -> jerky longitudinal (drive e6: state flipped
# 207x, 99% at |accel|<0.3). So only flip into regen for REAL braking, with hysteresis vs cruise noise (e6
# cruise accel ~ +0.13 +/-0.16). The ENTER threshold tracks the OEM's per-level entry-point measured on drive
# ec: Low~-0.5, Normal~-0.3, Heavy~0. Heavy uses a flicker-safe -0.15 (NOT full one-pedal; the car's separate
# one-pedal config is off) since true ~0 entry would re-flicker on OP's oscillating cruise accel. (level -> (enter, release) m/s^2)
REGEN_THRESHOLDS = {
  16: (-0.5, -0.2),    # Low (e.g. highway): regen only on real braking
  8:  (-0.3, -0.1),    # Normal
  24: (-0.15, 0.05),   # Heavy: strongest of the 3, flicker-safe approximation of one-pedal
}
REGEN_THRESHOLDS_DEFAULT = (-0.5, -0.2)  # unknown level -> Low (most conservative)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.CAN = gwmcan.CanBus(CP)
    self.accel = 0.0
    self.is_mk4 = CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4
    self.VM = VehicleModel(CP)
    self.override_active = False   # MK4: debounced driver-override latch
    self.override_counter = 0
    self.override_hold = 0         # MK4: frames remaining in the post-override OEM-style hold-off
    self.regen_brake = False       # MK4: debounced "real braking" latch for the regen brake-state bit
    self.acc_display_counter = 0   # MK4: our own running counter for the rebuilt ACC display (msg 683)

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    if self.is_mk4:
      tq = abs(CS.out.steeringTorque)
      if tq > OVERRIDE_INSTANT_TORQUE:
        self.override_counter = OVERRIDE_FRAMES
      elif tq > OVERRIDE_TORQUE:
        self.override_counter = min(OVERRIDE_FRAMES, self.override_counter + 1)
      else:
        self.override_counter = max(0, self.override_counter - 1)
      if self.override_active:
        # OEM-style hold: keep control handed back for OVERRIDE_HOLD_FRAMES, then release only once the
        # driver torque has actually decayed.
        if self.override_hold > 0:
          self.override_hold -= 1
        if self.override_counter <= 0 and self.override_hold <= 0:
          self.override_active = False
      elif self.override_counter >= OVERRIDE_FRAMES:
        self.override_active = True
        self.override_hold = OVERRIDE_HOLD_FRAMES
      lat_active = CC.latActive and not self.override_active
    else:
      lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if CC.cruiseControl.cancel:
      counter = (CS.steer_and_ap_stalk_msg['COUNTER'] + 1) % 16
      can_sends.append(gwmcan.create_buttons_command(
        self.packer,
        self.CAN,
        counter,
        CS.steer_and_ap_stalk_msg,
        cancel_command=True,
      ))

    if self.frame % 2 == 0: # 50 Hz
      if self.CP.steerControlType == SteerControlType.angle:
        # MK4: 14-bit angle command (STEER_REQUEST) in STEER_CMD. Command the controller's wheel angle straight
        # through, then limit lateral jerk + accel via the vehicle model (Tesla-style). The helper holds the
        # current wheel angle when not lat_active, giving a bumpless hand-back on driver override.
        target_angle = actuators.steeringAngleDeg
        apply_angle = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last,
                                                  CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active,
                                                  CarControllerParams, self.VM)
        # MK4: stop the command from winding far past the measured wheel during EPS under-execution
        # (see MK4_ANGLE_ERROR_MAX). MK3 (torque path) is unaffected.
        if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4 and lat_active:
          apply_angle = float(np.clip(apply_angle,
                                      CS.out.steeringAngleDeg - CarControllerParams.MK4_ANGLE_ERROR_MAX,
                                      CS.out.steeringAngleDeg + CarControllerParams.MK4_ANGLE_ERROR_MAX))
        can_sends.append(gwmcan.create_steer_command_angle(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          apply_angle=apply_angle,
          lat_active=lat_active,
        ))
        self.apply_angle_last = apply_angle
      else:
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
        # Prevent sending the same 'apply_torque = 1' torque repeatedly, as it can cause EPS faults.
        if abs(apply_torque) == 1:
          apply_torque = apply_torque * 2
        if not lat_active:
          apply_torque = 0
        can_sends.append(gwmcan.create_steer_command(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          steer=apply_torque,
          steer_req=lat_active,
        ))
        self.apply_torque_last = apply_torque

        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.params.STEER_MAX, self.params.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(gwmcan.create_wheel_touch(
          self.packer,
          self.CAN,
          eps_stock_values=CS.eps_stock_values,
          ea_simulated_torque=ea_simulated_torque,
        ))

      # Longitudinal control
      if self.CP.openpilotLongitudinalControl:
        standstill = actuators.longControlState == LongCtrlState.stopping
        self.accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if self.accel < 0:
          accel = - abs(self.accel / CarControllerParams.ACCEL_MIN)
        else:
          accel = self.accel / CarControllerParams.ACCEL_MAX
        # MK4 regen brake-state hysteresis, level-aware (see REGEN_THRESHOLDS above): the ENTER threshold tracks
        # the driver's regen level (CS.regen_level from msg 726). Only request regen for real braking, not the
        # planner's near-zero coast during cruise (that flicker made longitudinal jerky on e6).
        regen_enter, regen_release = REGEN_THRESHOLDS.get(CS.regen_level, REGEN_THRESHOLDS_DEFAULT)
        if CC.longActive and self.accel < regen_enter:
          self.regen_brake = True
        elif (not CC.longActive) or self.accel > regen_release:
          self.regen_brake = False
        can_sends.append(gwmcan.create_longitudinal_command(
          self.packer,
          self.CAN,
          longitudinal_stock_values=CS.longitudinal_stock_values,
          accel=accel,
          active=CC.longActive,
          standstill=standstill,
          is_mk4=self.is_mk4,
          regen=self.regen_brake,
        ))

    if self.frame % 5 == 0: # 20 Hz
      # HUD updates
      can_sends.append(gwmcan.create_hud_command(
        self.packer,
        self.CAN,
        hud_stock_values=CS.hud_stock_values,
        steer_required=CC.latActive,
        is_mk4=self.is_mk4,
      ))

    # MK4: cluster set-speed display. Panda claims msg 683 (check_relay), so once that's flashed the camera's
    # copy is blocked from the cluster and we MUST send this every cycle (forward stock when not overriding),
    # matching the ~10 Hz camera rate so the passed-through counters stay continuous.
    # EXPERIMENT: force CRUISE_STATE_2=3 + a DISTINCT 88 km/h while engaged, to confirm the cluster actually
    # renders our forged 683. Once confirmed, swap MK4_ACC_DISPLAY_TEST_SPEED for CC.hudControl.setSpeed.
    if self.is_mk4 and self.frame % 10 == 0:  # 10 Hz, matches the camera's ACC rate
      self.acc_display_counter = (self.acc_display_counter + 1) % 16
      can_sends.append(gwmcan.create_acc_display(
        self.packer,
        self.CAN,
        acc_stock_values=CS.acc_stock_values,
        counter=self.acc_display_counter,
        override=CC.enabled,
        set_speed_kph=MK4_ACC_DISPLAY_TEST_SPEED,
      ))

    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.angle:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
