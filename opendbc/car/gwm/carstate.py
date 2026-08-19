from opendbc.can.parser import CANParser
from opendbc.car import Bus, CanBusBase, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC, CarControllerParams

GearShifter = structs.CarState.GearShifter

GEAR_MAP = {
  0: GearShifter.park,
  1: GearShifter.drive,
  2: GearShifter.neutral,
  3: GearShifter.reverse,
}

# Message rates, measured off this car (route_c0 segment 40, 60 s). The CAN parser
# marks a message invalid if it arrives slower than declared, so these have to be
# right or the whole port reports a CAN fault and refuses to engage.
MAIN_MESSAGES = [
  ('STEER_ANGLE', 100),
  ('GAS', 100),
  ('WHEEL_SPEEDS', 50),
  ('STEER_TORQUE', 50),
  ('BRAKE', 50),
  ('GEAR', 50),
  ('LIGHTS', 20),
  ('DOORS', 20),
  ('SEATBELT', 2),
]

# Transmitted by the forward camera: only present on the camera bus once the
# harness relay opens.
CAM_MESSAGES = [
  ('LATERAL_STATE', 20),
  ('ACC', 10),
]


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # the camera's last raw STEER_CMD (0x12b) frame, stashed by the interface so
    # the controller can patch it. None until the camera is first heard.
    self.stock_steer_cmd = None
    self.eps_lka_active = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # speed
    self.parse_wheel_speeds(ret,
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FR'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RR'],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    # steering. Angle and rate are magnitudes with separate direction bits
    # (1 = right/CW). openpilot's convention is positive-left, so left is positive.
    angle = cp.vl['STEER_ANGLE']['STEERING_ANGLE']
    ret.steeringAngleDeg = angle * (-1.0 if cp.vl['STEER_ANGLE']['STEERING_DIRECTION'] else 1.0)
    rate = cp.vl['STEER_ANGLE']['STEERING_RATE']
    ret.steeringRateDeg = rate * (-1.0 if cp.vl['STEER_ANGLE']['RATE_DIRECTION'] > 0 else 1.0)
    ret.steeringTorque = cp.vl['STEER_TORQUE']['DRIVER_TORQUE']
    ret.steeringTorqueEps = ret.steeringTorque
    ret.steeringPressed = self.update_steering_pressed(
      abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    # EPS_FAULT_PERMANENT cycles 0/1 during normal operation on this platform, so
    # it is not a fault source. The panda safety mode is the real guard.
    ret.steerFaultPermanent = False
    ret.steerFaultTemporary = False
    self.eps_lka_active = cp_cam.vl['LATERAL_STATE']['LKAS_STATE'] == 5

    # gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl['GEAR']['GEAR_REAL']), GearShifter.unknown)

    # pedals
    ret.gasPressed = cp.vl['GAS']['GAS_USER'] > 0.
    ret.brakePressed = bool(cp.vl['BRAKE']['BRAKE_PRESSED'])

    # cruise. CRUISE_ENGAGED (0x2ab byte47 bit4) is the engagement flag: it matched
    # all seven labelled ACC/ICC runs of the 2026-08-18 differential drive and was
    # low outside them. The panda safety mode reads the same bit, so openpilot's
    # engaged state and the panda's controls_allowed cannot drift apart.
    ret.cruiseState.enabled = bool(cp_cam.vl['ACC']['CRUISE_ENGAGED'])
    # this car has no plain cruise control - ACC is what the stalk engages - and no
    # separate main-on bit has been identified, so treat it as always available and
    # let cruiseState.enabled do the gating.
    ret.cruiseState.available = True
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    # openpilot is not driving longitudinal, so it has no set speed of its own.
    ret.cruiseState.speed = -1

    # blinkers, belt, door. The lamps blink, so debounce them into a steady signal.
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl['LIGHTS']['LEFT_TURN'], cp.vl['LIGHTS']['RIGHT_TURN'])
    ret.seatbeltUnlatched = bool(cp.vl['SEATBELT']['DRIVER_SEATBELT'])
    ret.doorOpen = bool(cp.vl['DOORS']['DRIVER_DOOR'])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    can_base = CanBusBase(CP, None)
    dbc = DBC[CP.carFingerprint][Bus.pt]
    return {
      Bus.pt: CANParser(dbc, MAIN_MESSAGES, can_base.offset),
      Bus.cam: CANParser(dbc, CAM_MESSAGES, can_base.offset + 2),
    }
