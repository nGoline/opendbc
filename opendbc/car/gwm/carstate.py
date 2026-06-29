from opendbc.car import Bus, CanBusBase, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC, CAR
import copy

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.steer_and_ap_stalk_msg = {}
    self.eps_stock_values = {}
    self.camera_stock_values = {}
    self.longitudinal_stock_values = {}
    self.hud_stock_values = {}
    self.steer_fault_temporary_counter = 0

    self.is_activation_lever_pulled = False
    self.prev_activation_lever_pulled = False
    self.main_on = False
    self.regen_level = 16  # MK4 driver regen-level selector (msg 726 REGEN_LEVEL): 8=Normal, 16=Low, 24=Heavy. Default Low.

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    self.steer_and_ap_stalk_msg = copy.copy(cp.vl["STEER_AND_AP_STALK"])
    self.eps_stock_values = copy.copy(cp.vl["RX_STEER_RELATED"])
    self.camera_stock_values = copy.copy(cp_cam.vl["STEER_CMD"])
    self.longitudinal_stock_values = copy.copy(cp_cam.vl["ACC_CMD"])
    self.hud_stock_values = copy.copy(cp_cam.vl["LATERAL_STATE"])

    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # Driver's regen-level setting; carcontroller uses it to set the regen brake-state entry threshold.
      self.regen_level = int(cp.vl["REGEN_CONFIG"]["REGEN_LEVEL"])

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FRONT_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["FRONT_RIGHT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_LEFT_WHEEL_SPEED"],
      cp.vl["WHEEL_SPEEDS"]["REAR_RIGHT_WHEEL_SPEED"]
    )

    # CRUISE_STATE_2: 0=fault, 1=inactive, 2=available, 3=active (confirmed MK4).
    # Only 0 is a real fault. 1 (inactive) is normal whenever the stock ACC isn't engaged
    # (e.g. parked) -- treating <2 as a fault fired a permanent "Cruise Fault" while parked
    # and blocked engagement. 0 only ever appears as a 1-frame boot transient in healthy logs.
    ret.accFaulted = bool(cp_cam.vl["ACC"]["CRUISE_STATE_2"] == 0)
    ret.cruiseState.speed = cp_cam.vl["ACC"]["ACC_SPEED_SELECTION"] * CV.KPH_TO_MS
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.speed = -1

    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # ACC_CMD.STANDSTILL = camera's standstill request (1 only when actually stopped).
      # Secondary source: covers the ~1 km/h rounding floor in the speed signal.
      # The old ACC(0x2ab).STANDSTILL_1 was NOT standstill — it tracks driver lateral
      # override (mirrors A_RX_STEER_REQUESTED==2), so it was 1 whenever hands were on the
      # wheel. Using it here wrongly forced standstill while driving.
      acc_standstill = bool(cp_cam.vl["ACC_CMD"]["STANDSTILL"])
      ret.standstill = abs(ret.vEgoRaw) < 1e-3 or acc_standstill
    else:
      ret.standstill = abs(ret.vEgoRaw) < 1e-3

    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # GAS_PEDAL_CMD stays ~3.5% at idle (engine idle control) — use GAS_PEDAL_USER
      # which tracks driver foot position and goes to 0 when foot is off.
      ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0
    else:
      ret.gasPressed = cp.vl["CAR_OVERALL_SIGNALS2"]["GAS_POSITION"] > 0
    ret.brakePressed = cp.vl["BRAKE2"]["PEDAL_BRAKE_PRESSED"] != 0
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"] if not ret.brakePressed else 0

    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      drive_mode = int(cp.vl["DRIVE_GEAR"]["DRIVE_MODE_GEAR_REAL"])
    else:
      drive_mode = int(cp.vl["CAR_OVERALL_SIGNALS"]["DRIVE_MODE"])

    ret.parkingBrake = drive_mode == 0
    ret.gearShifter = GearShifter.drive if drive_mode == 1 else \
                      GearShifter.neutral if drive_mode == 2 else \
                      GearShifter.reverse if drive_mode == 3 else \
                      GearShifter.park

    # STEERING_ANGLE is an unsigned magnitude; STEERING_DIRECTION / RATE_DIRECTION give the side.
    # openpilot convention: steeringAngleDeg POSITIVE = LEFT turn.
    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # Sign corrected 2026-06-22: the previous `(1 if DIRECTION else -1)` form was INVERTED — it read
      # POSITIVE on RIGHT turns (proven via GPS heading, corr +0.82). openpilot's canonical steering angle is
      # OPPOSITE-signed to desiredCurvature: the shared-code command (actuators.steeringAngleDeg =
      # get_steer_from_curvature(-desiredCurvature)) correlates -0.9 with desiredCurvature and the physical wheel
      # tracks it, yet the old readback correlated -0.9 with that command (= same sign as desiredCurvature) =>
      # flipped. A flipped readback fed paramsd a sign-wrong angle↔yaw relationship (likely the steerRatio/
      # angleOffset runaway) and rendered the UI steering wheel backwards. Matches the MK3 branch and Tesla
      # (which negates its raw SAS for the same reason).
      ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
      ret.steeringRateDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_RATE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["RATE_DIRECTION"] > 0 else 1)
    else:
      ret.steeringAngleDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_ANGLE"] * (-1 if cp.vl["STEER_AND_AP_STALK"]["STEERING_DIRECTION"] else 1)
      ret.steeringRateDeg = cp.vl["STEER_AND_AP_STALK"]["STEERING_RATE"] * (-1 if (cp.vl["STEER_AND_AP_STALK"]["RATE_DIRECTION"] > 0) else 1)

    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # EPS_FAULT_PERMANENT (bit125) cycles 0/1 during normal MK4 operation → spurious
      # steerTempUnavailable. Real safety is the panda driver; disable counter-based detection.
      ret.steerFaultTemporary = False
    else:
      self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (cp.vl["RX_STEER_RELATED"]["EPS_FAULT_PERMANENT"] == 1) else 0
      ret.steerFaultTemporary = self.steer_fault_temporary_counter > 100
    ret.steerFaultPermanent = False

    ret.steeringTorque = cp.vl["RX_STEER_RELATED"]["B_RX_DRIVER_TORQUE"]
    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      # B_RX_EPS_TORQUE bytes are static on MK4 (always reads 0); use driver torque as proxy
      ret.steeringTorqueEps = ret.steeringTorque
    else:
      ret.steeringTorqueEps = cp.vl["RX_STEER_RELATED"]["B_RX_EPS_TORQUE"]
    # steeringPressed gates the REAL lateral override: car_specific.py fires steerOverride (-> OVERRIDE_LATERAL,
    # openpilot stops steering) whenever this is True. The OEM LKAS keeps steering through driver torque up to ~100 (median handoff) and
    # tolerates spikes to ~156. Raise the MK4 gate toward that so the driver can rest a hand; MK3 keeps
    # the stock 50. (carcontroller's debounced override is a secondary, finer gate for a sustained grab.)
    if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4:
      ret.steeringPressed = abs(ret.steeringTorque) > 120
    else:
      ret.steeringPressed = abs(ret.steeringTorque) > 50

    ret.doorOpen = any([cp.vl["DOOR_DRIVER"]["DOOR_REAR_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_FRONT_RIGHT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_REAR_LEFT_OPEN"],
                        cp.vl["DOOR_DRIVER"]["DOOR_DRIVER_OPEN"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT"]["SEAT_BELT_DRIVER_STATE"])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"],
                                                                      cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])
    ret.leftBlindspot = bool(cp.vl["RADAR_BEHIND"]["BSM_LEFT"] > 0)
    ret.rightBlindspot = bool(cp.vl["RADAR_BEHIND"]["BSM_RIGHT"] > 0)

    if cp.vl["STEER_AND_AP_STALK"]["AP_CANCEL_COMMAND"] or ret.brakePressed:
      self.main_on = False
    self.is_activation_lever_pulled = bool(cp.vl["STEER_AND_AP_STALK"]["AP_ENABLE_COMMAND"])
    # MK4: the FURTHER_DOWN stalk gesture activates ACC but is the same physical motion as
    # shifting gears (N→D, R→D). Require the car to already be moving so gear shifts
    # from standstill don't accidentally engage openpilot.
    if not self.is_activation_lever_pulled and self.prev_activation_lever_pulled and not self.main_on:
      if self.CP.carFingerprint != CAR.GWM_HAVAL_H6_MK4 or abs(ret.vEgoRaw) > 0.5:
        self.main_on = True
    self.prev_activation_lever_pulled = self.is_activation_lever_pulled

    ret.cruiseState.available = self.main_on
    ret.cruiseState.enabled = self.main_on

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Compute bus offset from number of safetyConfigs so multipanda setups
    # (internal + external pandas) map DBCs to the correct physical bus.
    can_base = CanBusBase(CP, None)
    main_bus = can_base.offset
    adas_bus = can_base.offset + 1
    cam_bus = can_base.offset + 2

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], main_bus),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], adas_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], cam_bus),
    }
