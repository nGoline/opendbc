from opendbc.car import CanBusBase, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import WHEEL_SPEED_FACTOR, GwmSafetyFlags

STEER_CMD_ADDR = 0x12b
ACC_CMD_ADDR = 0x143


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP):
    super().__init__(CP)
    # STEER_CMD is transmitted by the forward camera. Once the harness relay opens
    # it is only ever seen on the camera bus - measured across a full openpilot-
    # engaged drive, 0x12b appeared 2999 times on bus 2 and zero times on bus 0.
    # Watching bus 0 for it means never seeing it, and never steering.
    self.cam_bus = CanBusBase(CP, None).offset + 2

  def update(self, can_packets):
    # Grab the camera's raw STEER_CMD before parsing, so the controller can patch
    # that exact frame rather than compose one.
    #
    # Frames arrive as plain (address, dat, src) tuples - that is what
    # can_capnp_to_list builds and what CANParser itself unpacks. Reading .address
    # off them raises AttributeError on the first CAN packet and kills card, which
    # shows up as canError plus processNotRunning rather than as anything car-shaped.
    for _, msgs in can_packets:
      for addr, dat, src in msgs:
        if src != self.cam_bus:
          continue
        if addr == STEER_CMD_ADDR:
          self.CS.stock_steer_cmd = bytes(dat)
        elif addr == ACC_CMD_ADDR:
          self.CS.stock_acc_cmd = bytes(dat)
    return super().update(can_packets)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    # Lateral only. The stock ACC keeps handling speed; openpilot steers by angle.
    #
    # The steering command (STEER_CMD, 0x12b) is gated frame-to-frame by two plain
    # CRC-8s that we recompute, so it can be sent without any key. See gwmcan.
    # openpilot owns engagement instead of following the stock ACC, engaging on the
    # soft-down stalk gesture the stock system ignores. That keeps the stock ACC
    # from ever running, which is what stops it chiming on every override and
    # cancel. The safety mode must be told, or it would still gate controls on the
    # stock cruise bit and block every frame openpilot sends.
    ret.pcmCruise = False
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]
    ret.safetyConfigs[0].safetyParam |= GwmSafetyFlags.OP_CRUISE.value
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerAtStandstill = True

    # Measured on this car: the EPS angle readback (0x245) lags the commanded angle
    # by ~0.08 s. Left at the 0.0 default the angle controller has no lead at all
    # and hunts.
    ret.steerActuatorDelay = 0.08
    # 1.0, matching ford and nissan - the other angle ports that use the same crude
    # saturation heuristic. LatControlAngle flags saturation on
    # |desired - measured| > 2.5 deg sustained for this long, which on an angle car
    # fires on ordinary tracking lag entering a turn and nags with a repeating
    # chime. tesla and hyundai keep 0.4 only because they take the precise
    # steer_limited_by_safety signal instead of that heuristic; we cannot.
    ret.steerLimitTimer = 1.0

    ret.wheelSpeedFactor = WHEEL_SPEED_FACTOR
    ret.radarUnavailable = True

    # Longitudinal is opt-in behind the alpha toggle. With openpilot owning cruise
    # the stock ACC is not running, so if openpilot does not command speed then
    # nothing does and the driver is on the pedals.
    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    if alpha_long:
      ret.safetyConfigs[0].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      # 0.3 s, measured: the camera's brake command leads the resulting
      # deceleration by that much, peaking at r=0.79 across a lag sweep.
      ret.longitudinalActuatorDelay = 0.3

      # The feedforward is fitted from the car's own behaviour and lands within
      # ~0.19 m/s^2 RMS of what the camera commands, so the integrator only has to
      # trim rather than discover the pedal. Scheduled by speed as honda and gm do;
      # an earlier port used a single unscheduled 0.4 and hunted.
      ret.longitudinalTuning.kiBP = [0., 5., 35.]
      ret.longitudinalTuning.kiV = [1.0, 0.7, 0.5]

      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stopAccel = 0.0
      ret.stoppingDecelRate = 0.8

    return ret
