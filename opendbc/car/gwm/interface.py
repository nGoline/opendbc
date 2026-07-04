from opendbc.car import structs, get_safety_config, Bus, create_button_events
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import CAR, GwmSafetyFlags

SteerControlType = structs.CarParams.SteerControlType

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType

# MK4 (angle control) hands-off latch for the steerTempUnavailable counter. The angle-path EPS flips
# A_RX_STEER_REQUESTED to 2 ("driver override") at modest driver torque, so A_RX!=1 alone is NOT a
# fault — it's a normal hands-on override. GWM exposes no dedicated EPS-inhibited signal (cf. Tesla
# EAC_INHIBITED / Ford EPAS_Failure), so fault only when the EPS stops AND the driver isn't the cause:
# the wheel hasn't been grabbed recently. A genuine hands-off limp keeps driver torque ~0.
MK4_GRAB_TORQUE = 80          # |driver torque| marking a deliberate hands-on override
MK4_GRAB_HOLD_FRAMES = 150    # ~1.5 s: keep treating the wheel as hands-on after the last grab
# A_RX==2 is NOT sufficient for a fault: at highway speed the EPS flips it to 2 for ~1 s while still executing
# the command hands-off (route 103 seg10/11/12: A_RX=2 ~1 s, |actual-command| <= 0.6 deg, torque < 17 -> a
# spurious "TAKE CONTROL"). A genuine limp/override has the wheel DIVERGING from the command (seg20: 8.4 deg),
# so also require a real tracking error before counting a fault. Threshold clears the ~0.6 deg false-fire band
# (and the ~2.4 deg worst-case actuator-delay lag) with margin, well below a real divergence.
MK4_ANGLE_TRACK_ERR = 3.0     # deg; |steeringAngleDeg - commanded apply_angle| above this = wheel genuinely off-command


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  # MK3 personality sync: frames between synthetic gap-button pulses, so openpilot's personality
  # feedback (hudControl.leadDistanceBars) can round-trip before the next pulse
  GAC_SYNC_INTERVAL = 25

  def __init__(self, CP):
    super().__init__(CP)
    self.lat_active = False
    self.isEPSobeying = True
    self.steer_fault_temporary_counter = 0
    self.recent_grab = 0    # MK4: frames remaining in the hands-on hold-off (see MK4_GRAB_* above)
    self.last_commanded_angle = 0.0    # MK4: last apply_angle, for the tracking-error fault gate
    self.current_personality = 0
    self.pcm_follow_distance = 0
    self.press_gac_button = False    # MK3 only
    self.last_gac_press_frame = -self.GAC_SYNC_INTERVAL    # MK3 only (self.frame comes from CarInterfaceBase)

  def apply(self, CC, now_nanos):
    self.lat_active = CC.latActive
    self.last_commanded_angle = CC.actuators.steeringAngleDeg
    hud_control = CC.hudControl
    self.current_personality = hud_control.leadDistanceBars
    return super().apply(CC, now_nanos)

  def update(self, can_packets):
    # MK4: stash the raw EPS RX_STEER_RELATED (0x147) frame off the main bus so the carcontroller can re-transmit
    # it to the camera (hands-on keepalive). The DBC doesn't model the whole 64-byte frame, so we forward raw
    # bytes and patch only the driver-torque field there (see gwmcan.create_wheel_touch_mk4). can_packets is a
    # list of (nanos, [(address, dat, src), ...]) tuples (see can_capnp_to_list) -- index, don't attribute.
    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      for _, msgs in can_packets:
        for address, dat, src in msgs:
          if address == 0x147 and src == 0:
            self.CS.eps_stock_raw = bytes(dat)

    ret = super().update(can_packets)

    # read AFTER super().update() has parsed this cycle's packets, so these are same-epoch with ret
    self.isEPSobeying = self.can_parsers[Bus.main].vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1
    self.pcm_follow_distance = self.can_parsers[Bus.cam].vl["ACC"]["CAR_DISTANCE_SELECTION"]

    # steerTempUnavailable: count a fault only when the EPS stops obeying AND it isn't the driver's doing.
    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # (a) wheel wasn't grabbed recently (hands-off override hold-off — see MK4_GRAB_* note above), and
      # (b) the wheel is actually diverging from the command (a genuine limp/override drifts off; a spurious
      #     highway A_RX==2 stays glued to it — see MK4_ANGLE_TRACK_ERR note above). Both gates required.
      # ret.steeringTorque is B_RX_DRIVER_TORQUE (set in carstate), same epoch as the rest of ret
      self.recent_grab = MK4_GRAB_HOLD_FRAMES if abs(ret.steeringTorque) > MK4_GRAB_TORQUE else max(0, self.recent_grab - 1)
      hands_on = self.recent_grab > 0
      not_tracking = abs(ret.steeringAngleDeg - self.last_commanded_angle) > MK4_ANGLE_TRACK_ERR
      self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) \
                                            if (self.lat_active and not self.isEPSobeying and not hands_on and not_tracking) else 0
    else:
      self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (self.lat_active and not self.isEPSobeying) \
                                            else 0

    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100

    # Driving personality:
    #  - MK4: cycled by the wheel follow-distance buttons -> gapAdjustCruise buttonEvents built in carstate
    #    (the OEM CAR_DISTANCE_SELECTION dash goes stale once openpilot owns the car, so we no longer mirror it).
    #  - MK3: mirror the OEM follow-distance dash. The stock ACC cycles 4 distances, openpilot has 3
    #    personalities (distances 3 and 4 both map to the farthest). While they disagree, pulse gap-adjust
    #    press/release pairs (openpilot cycles on release), rate-limited so leadDistanceBars can catch up.
    if self.CP.carFingerprint != CAR.GWM_HAVAL_H6_MK4:
      prev_gac_button = self.press_gac_button
      if self.press_gac_button:
        self.press_gac_button = False    # always complete the press with a release
      else:
        target_personality = min(int(self.pcm_follow_distance), 3)
        out_of_sync = self.pcm_follow_distance > 0 and target_personality != self.current_personality
        if out_of_sync and (self.frame - self.last_gac_press_frame) >= self.GAC_SYNC_INTERVAL:
          self.press_gac_button = True
          self.last_gac_press_frame = self.frame
      ret.buttonEvents = create_button_events(int(self.press_gac_button), int(prev_gac_button), {1: ButtonType.gapAdjustCruise})
      self.frame += 1

    return ret

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    if candidate == CAR.GWM_HAVAL_H6_MK4:
      # Angle-based steering: do NOT call configure_torque_tune — controlsd would
      # call update_live_torque_params on LatControlAngle which doesn't implement it.
      ret.steerControlType = SteerControlType.angle
      ret.steerActuatorDelay = 0.08
      # MK4 owns its own cruise loop: the OEM ACC freezes its set speed once openpilot drives, so don't
      # follow it. openpilot manages engagement + set-speed from the wheel buttons (carstate buttonEvents).
      ret.pcmCruise = False
      # Tell the panda to arm controls off the gentle-DOWN stalk gesture (0xC7 GEAR_STALK) instead of the
      # FURTHER_DOWN-only msg 161 bit47 — matches the engage source in carstate so the two gates stay in sync.
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.OP_CRUISE.value
      # Validate the 14-bit angle command in the panda (lateral accel/jerk limits via the vehicle model)
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.ANGLE_CONTROL.value
    else:
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      ret.longitudinalActuatorDelay = 0.25
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stopAccel = -0.75
      ret.stoppingDecelRate = 0.75
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.4]

    return ret
