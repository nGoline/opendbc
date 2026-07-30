import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams, CAR

SteerControlType = structs.CarParams.SteerControlType

LongCtrlState = structs.CarControl.Actuators.LongControlState

MAX_USER_TORQUE = 100  # 1.0 Nm

# MK4 hands-on keepalive: driver torque we spoof to the camera/ADAS (via 0x147 on the camera bus) so it never
# runs its hands-off "hold the wheel" warning + safe-stop escalation and the EPS keeps steering hands-off.
# comma's DM camera is the attention monitor. The MK3 torque path does this dynamically (ea_simulated_torque =
# apply_torque*2). The first MK4 try used a FIXED 65 and the EPS still limped at highway speed (routes 110-112,
# verified transmitting @50Hz) -- because the OEM only recognizes "hands on" around |torque| > ~102, so 65 STILL
# read as hands-off. Fix: sit the FLOOR clearly above ~102 and scale up with the commanded angle like MK3 (more
# steering effort -> more apparent driver torque), capped below a real hard grab so a genuine override still
# passes through. Follows the commanded steer direction. Tunable on-road: raise the floor if it still limps,
# lower the cap if the ADAS flags implausible torque.
MK4_HANDS_ON_TORQUE = 120      # floor (was 65): above the OEM hands-on recognition point (~102)
MK4_HANDS_ON_TORQUE_MAX = 170  # cap: strong hands-on, still below the driver's real hard grabs (125-214)
MK4_HANDS_ON_ANGLE_GAIN = 8    # extra spoofed torque per deg of |apply_angle| (mimics MK3's dynamic scaling)

# MK4 carcontroller lat_active latch (NOT carstate.steeringPressed — that is the separate shared-code
# steerOverride event at torque>120). This path drops latActive for STEER_CMD only. Tuned for fully
# hands-off use: OVERRIDE_TORQUE=100 so a deliberate grab takes over cleanly (route 110 grabs 44–214
# failed when this sat at 120). Instant path above OEM hands-on (~102). Debounce absorbs spikes.
OVERRIDE_TORQUE = 100          # sustained |driver torque| to hand off (= MK3 MAX_USER_TORQUE)
OVERRIDE_INSTANT_TORQUE = 150  # firm grab -> release within one frame
OVERRIDE_FRAMES = 7            # ~70 ms @100 Hz
# Hold lat off ~1 s after override (OEM-style) so OP does not re-grab every torque dip.
OVERRIDE_HOLD_FRAMES = 100


# MK4 regen-lead: BRAKE_GAS_STATE (regen enable) uses a light accel hysteresis so cruise chatter around
# 0 m/s^2 does not flip regen↔coast every frame (drive e6 jerk). BRAKE_OR_GAS_REQ / BRAKE_CMD still
# follow sign(accel) every frame — do NOT hold brake mode or pin GAS_CMD=0 across a deadband: that
# FAULTED the OEM ACC (drives 13e-146, "Cruise Fault: Restart the Car"). Regen MAGNITUDE stays the
# powertrain's call (driver Low/Normal/Heavy on the stalk selector); we only gate the enable state.
#
# Enter regen when accel <= REGEN_ON; leave when accel >= REGEN_OFF or long inactive. Band is small
# (~0.13 m/s^2) — enough for planner noise, not a deep coast-hold.
MK4_REGEN_ON_MS2 = -0.08
MK4_REGEN_OFF_MS2 = 0.05


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
    self.regen_brake = False       # MK4: hysteretic REGEN state latch (BRAKE_GAS_STATE only; see MK4_REGEN_*)
    # OEM cluster (0x2AB): latch last good set-speed + follow dashes so the Vmax icon stays solid
    # while engaged (OEM: icon always on when ACC active). Avoid thrashing distance every frame.
    self.acc_cluster_set_kph: float | None = None
    self.acc_cluster_follow: int | None = None

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
        # (see MK4_ANGLE_ERROR_MAX). When the EPS is not granting angle authority (A_RX != 1), use a
        # tighter band so we don't keep pushing opposite-signed commands into a non-tracking EPS.
        # MK3 (torque path) is unaffected.
        if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4 and lat_active:
          eps_obeying = int(CS.eps_stock_values.get("A_RX_STEER_REQUESTED", 1)) == 1
          err_max = (CarControllerParams.MK4_ANGLE_ERROR_MAX if eps_obeying
                     else CarControllerParams.MK4_ANGLE_ERROR_MAX_NOT_OBEYING)
          apply_angle = float(np.clip(apply_angle,
                                      CS.out.steeringAngleDeg - err_max,
                                      CS.out.steeringAngleDeg + err_max))
        can_sends.append(gwmcan.create_steer_command_angle(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          apply_angle=apply_angle,
          lat_active=lat_active,
        ))
        self.apply_angle_last = apply_angle

        # MK4 hands-on keepalive: re-transmit the EPS 0x147 frame to the camera. The panda never forwards the
        # stock 0x147 main->camera while this safety mode is active (TX config has .check_relay=true), so we are
        # the camera's ONLY source of EPS feedback whenever onroad -- engaged OR not. Send every frame like the
        # MK3 create_wheel_touch does; while engaged patch in a dynamic hands-on torque (floor/cap/angle-gain
        # from highway limp analysis) so the ADAS never runs its hands-off warning/safe-stop escalation, and
        # let a real driver grab pass through. Disengaged, the real torque is re-encoded unchanged.
        if CS.eps_stock_raw is not None:
          spoof_torque = int(CS.out.steeringTorque)
          if CC.enabled:
            mag = min(MK4_HANDS_ON_TORQUE_MAX, MK4_HANDS_ON_TORQUE + MK4_HANDS_ON_ANGLE_GAIN * abs(apply_angle))
            if abs(spoof_torque) <= mag:
              spoof_torque = int(mag if apply_angle >= 0 else -mag)
          can_sends.append(gwmcan.create_wheel_touch_mk4(self.CAN, CS.eps_stock_raw, spoof_torque))
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
        # braking (BRAKE_OR_GAS_REQ / cmds): plain sign of accel every frame.
        # regen_brake (BRAKE_GAS_STATE only): light hysteresis — see MK4_REGEN_* above.
        braking = self.accel < 0
        if self.is_mk4:
          if not CC.longActive:
            self.regen_brake = False
          elif self.accel <= MK4_REGEN_ON_MS2:
            self.regen_brake = True
          elif self.accel >= MK4_REGEN_OFF_MS2:
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
          braking=braking,
          longitudinal_stock_raw=CS.acc_cmd_stock_raw if self.is_mk4 else None,
        ))

    if self.frame % 5 == 0:  # 20 Hz
      # HUD updates
      can_sends.append(gwmcan.create_hud_command(
        self.packer,
        self.CAN,
        hud_stock_values=CS.hud_stock_values,
        steer_required=CC.latActive,
        is_mk4=self.is_mk4,
        hud_stock_raw=CS.hud_stock_raw if self.is_mk4 else None,
      ))

    # MK4 OP_CRUISE: re-TX camera ACC (0x2AB) onto main with openpilot set speed so the Haval
    # cluster tracks VCruiseHelper. Match OEM camera rate (~10 Hz; panda RX check 10U).
    # Always re-TX while we hold the relay slot. While engaged: keep set-speed + CRUISE activated
    # latched (OEM keeps the Vmax icon solid whenever ACC is on — intermittent chrome was from
    # frozen camera state + thrashing distance on every personality tick).
    if self.frame % 10 == 0:  # 10 Hz
      if self.is_mk4 and self.CP.openpilotLongitudinalControl and CS.acc_stock_raw is not None:
        hud = CC.hudControl
        set_kph = None
        follow = None
        cruise_active = bool(CC.enabled)
        # In Drive with OP available but not enabled: soft ACC "available" (quiet cancel path).
        cruise_standby = (not cruise_active) and bool(CS.out.cruiseState.available)
        if CC.enabled:
          raw = float(hud.setSpeed) * CV.MS_TO_KPH if hud.speedVisible else 0.0
          if 0.0 < raw < 200.0:
            self.acc_cluster_set_kph = raw
          # Hold last good set while engaged even if one frame publishes 255/unset
          set_kph = self.acc_cluster_set_kph
          if set_kph is None and CS.out.vEgo > 0.5:
            # First-engage race before VCruiseHelper init: show vEgo floor 20 like cruise.py
            set_kph = float(np.clip(CS.out.vEgo * CV.MS_TO_KPH, 20.0, 145.0))
            self.acc_cluster_set_kph = set_kph
          # Follow dashes: map personality 1..3 -> OEM 1..4; only update latch on change.
          # Default to 3 if HUD has not published bars yet — OEM dist 0 = "disabled" (no ICC icon).
          if hud.leadDistanceBars > 0:
            new_follow = 4 if hud.leadDistanceBars >= 3 else int(hud.leadDistanceBars)
            if self.acc_cluster_follow is None or new_follow != self.acc_cluster_follow:
              self.acc_cluster_follow = new_follow
          if self.acc_cluster_follow is None:
            self.acc_cluster_follow = 3
          follow = self.acc_cluster_follow
        else:
          # Keep latched set/follow for soft standby demote (quiet OEM cancel); clear off Drive.
          if cruise_standby:
            set_kph = self.acc_cluster_set_kph
            follow = self.acc_cluster_follow
          else:
            self.acc_cluster_set_kph = None
            self.acc_cluster_follow = None
        can_sends.append(gwmcan.create_acc_cluster_mk4(
          self.CAN, CS.acc_stock_raw,
          set_speed_kph=set_kph,
          follow_dashes=follow,
          cruise_active=cruise_active,
          cruise_standby=cruise_standby,
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
