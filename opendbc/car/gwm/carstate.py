from opendbc.can.parser import CANParser
from opendbc.car import Bus, CanBusBase, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.gwm.values import DBC, CarControllerParams

GearShifter = structs.CarState.GearShifter

GEAR_MAP = {
  0: GearShifter.park,
  1: GearShifter.drive,
  2: GearShifter.neutral,
  3: GearShifter.reverse,
}

# Measured rates on this car, for reference (route_c0 seg 40, and unchanged across
# three stationary captures): STEER_ANGLE 100, GAS 100, WHEEL_SPEEDS 50, STEER_TORQUE
# 50, BRAKE 50, GEAR 50, LIGHTS 20, DOORS 20, SEATBELT 2 on the main bus;
# LATERAL_STATE 20, ACC 10 on the camera bus.
#
# They are deliberately NOT declared to the parser. Declaring a frequency asserts it,
# and an assertion that holds in one driving state can fail in another - the failure
# mode is a permanent canError ("Unknown Vehicle Variant") with every message
# physically present. Left unspecified, the parser learns each message's real rate
# from the data and sets its own timeout from that, so it still catches a message
# that stops, and bus_timeout still catches a dead bus. The panda rx checks in
# safety/modes/gwm.h assert the rates that actually matter for safety.

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # the camera's last raw STEER_CMD (0x12b) frame, stashed by the interface so
    # the controller can patch it. None until the camera is first heard.
    self.stock_steer_cmd = None
    self.eps_lka_active = False
    self.eps_fault_frames = 0
    self.disengage_frames = 0

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
    # EPS_FAULT_PERMANENT toggles 0/1 spuriously during normal operation on this
    # platform, so a single frame means nothing. Debounced over ~1 s it is a real
    # fault. steerFaultPermanent stays False - nothing observed warrants latching
    # until the car is restarted.
    self.eps_fault_frames = (self.eps_fault_frames + 1) if \
      cp.vl['STEER_TORQUE']['EPS_FAULT_PERMANENT'] else 0
    ret.steerFaultPermanent = False
    ret.steerFaultTemporary = self.eps_fault_frames > CarControllerParams.EPS_FAULT_FRAMES

    # A hard grab disengages outright: `steeringDisengage` drops controls_allowed on
    # the rising edge. Debounced so a single spike cannot do it. Ordinary force to
    # help a turn stays well under this and only hands lateral control back.
    self.disengage_frames = (self.disengage_frames + 1) if \
      abs(ret.steeringTorque) > CarControllerParams.DISENGAGE_TORQUE else 0
    ret.steeringDisengage = self.disengage_frames >= CarControllerParams.DISENGAGE_FRAMES

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
    # The stock ACC's set speed, which openpilot displays. Leaving this at -1 fed
    # openpilot a nonsense negative speed. ACC_SPEED_SELECTION is byte 22 of 0x2ab
    # in km/h - verified against the drive of 2026-08-23: set 80 holds an actual
    # p50 of 79.3 km/h and set 100 holds 90.7 rising to 100.8, with 85/90/95
    # appearing transiently as the 5 km/h scroll steps between them.
    ret.cruiseState.speed = cp_cam.vl['ACC']['ACC_SPEED_SELECTION'] * CV.KPH_TO_MS

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
      Bus.pt: CANParser(dbc, [], can_base.offset),
      Bus.cam: CANParser(dbc, [], can_base.offset + 2),
    }
