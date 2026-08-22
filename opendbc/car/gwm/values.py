from dataclasses import dataclass, field

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu

# ~3.4 degrees, 6% superelevation. Higher actual roll lowers lateral acceleration.
AVERAGE_ROAD_ROLL = 0.06


class CarControllerParams:
  # STEER_CMD (0x12b) is a 50 Hz frame. CarController runs at 100 Hz, so send on
  # every second frame.
  STEER_STEP = 2

  # Lateral limits use the VEHICLE MODEL (v2), as Tesla does, not the hand-tuned
  # speed-breakpoint tables (v1). The v1 tables here were guesses and they were
  # measurably too tight: on the drives of 2026-08-21 they cut more than 0.5 deg
  # off openpilot's request in 15-23% of engaged frames, up to 18.3 deg, which
  # under-steers curves and cuts the lane. At 25 m/s v1 allowed 7.5 deg/s while
  # the camera's own command routinely slews faster than that.
  #
  # v2 derives the limits from constant lateral accel and jerk instead, which is
  # speed-correct by construction - 22.4 deg/s at 25 m/s - and is what
  # opendbc/safety/lateral.h says should replace the breakpoint function.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    300,        # max commanded angle, deg
    ([], []),   # v1 tables unused - the vehicle model supplies the rate limits
    ([], []),

    # Extra tolerance for average road roll, since the panda has no roll estimate.
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),

    # Hard ceiling on slew, mostly for low speed where the model's limit goes
    # very large. MEASURED off the camera over 30267 command steps: p99.99 is
    # 1.0 deg/step and the largest step it ever took was 3.8, so the EPS
    # demonstrably accepts well beyond this.
    MAX_ANGLE_RATE=3.0,  # deg per 20 ms frame
  )

  # Driver-torque threshold (raw units of STEER_TORQUE.DRIVER_TORQUE) above which
  # the driver is considered to be holding the wheel. Provisional - confirm on
  # the road against a light hand.
  STEER_DRIVER_ALLOWANCE = 120

  # Never command an angle more than this far from where the wheel actually is.
  #
  # This is what makes the wheel yield. The EPS servos to the commanded angle, and
  # the torque it applies grows with the error, so bounding the error bounds how
  # hard it can push back. Push the wheel, it moves, the command follows it, and
  # the resistance never builds.
  #
  # MEASURED off the camera, which does exactly this. Across 2.6M frames where the
  # OEM was commanding - ICC lane centering and ACC lane-departure nudges, several
  # drives - |commanded - measured| never exceeded 4.6 deg, and the ceiling is flat
  # across every driver-torque bucket, so it is a hard clamp and not a torque-
  # dependent softening. Normal tracking sits at a median of 0.4 deg, so 4.0 leaves
  # the controller all the room it needs and only bites when the driver is winning.
  #
  # openpilot without this clamp reached 22-28 deg of error against a resisting
  # driver, which saturates the EPS. That is the whole of the "wheel is stiff and
  # fights back" complaint. See haval-port captures/2026-08-21-steering-effort.md.
  #
  # Same mechanism Ford uses (apply_ford_curvature_limits clips to current
  # curvature +- CURVATURE_ERROR) and the panda supports natively via
  # enforce_angle_error / max_angle_error, which is the follow-up to this.
  MAX_ANGLE_ERROR = 4.0


# Wheel speed is encoded at 0.05924739 km/h per count in the DBC (the value the
# working reference port settled on), so no extra scale factor is needed.
WHEEL_SPEED_FACTOR = 1.0

# Brake pedal position rests at this raw value with the pedal up, not at zero.
BRAKE_PEDAL_REST = 25


@dataclass
class GWMCarDocs(CarDocs):
  package: str = "Intelligent Cruise Control (ICC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class GWMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'gwm_haval_h6_phev_mk4',
  })


class CAR(Platforms):
  # MK4 is the 2024-present generation on the evolved LEMON platform - the one
  # with Coffee OS 3 and the redesigned cluster. It spans four powertrain lines,
  # which is why the platform is named for the line and not just the generation:
  #
  #   HEV2     1.6 kWh, self-charging, no plug
  #   PHEV19   19 kWh, FWD, ~110-115 km EV range   <- the one captured here
  #   PHEV35   35 kWh, AWD, ~175 km, DC charging
  #   GT       PHEV35 mechanicals in a fastback body
  #
  # PHEV35 and GT share a drivetrain and would likely share an entry; HEV2 is
  # the furthest from this one and least likely to share powertrain CAN. Nothing
  # but PHEV19 has been captured, so none of them are listed.
  #
  # mass, wheelbase and the axle split are from the owner's manual for this
  # exact car. Axle loads are 1110 kg front / 805 kg rear, so the centre of mass
  # sits 805/1915 of the wheelbase back from the front axle - centerToFront
  # scales with the *rear* fraction, not the front.
  #
  # steerRatio 18.0 is MEASURED on this car, from logged yaw rate against wheel
  # angle, and it is flat across angle (no variable rack). The 14.5 that falls out
  # of the manual's 11.9 m kerb-to-kerb turning circle is ~20% low - that figure
  # includes body overhang, so it overstates the road-wheel angle at lock.
  # steerRatio scales the angle command directly, so 14.5 would under-steer every
  # curve and leave paramsd fighting to make up the difference.
  GWM_HAVAL_H6_PHEV19_MK4 = GWMPlatformConfig(
    [GWMCarDocs("GWM Haval H6 PHEV19 2026")],
    CarSpecs(mass=1915., wheelbase=2.738, centerToFrontRatio=0.420, steerRatio=18.0),
  )


# The DIDs come back concatenated in a single response, so the request asks for
# them together rather than one at a time:
#   b'\xf1\x873612100XEB83000\xf1\x89S013A01XKN34003'
# The engine ECU answers F187 and F189 and ignores F182, so only two come back.
GWM_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)

GWM_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

# Some ECUs on this platform answer at request + 0x6a instead of the usual + 0x8.
GWM_RX_OFFSET = 0x6a

# The engine ECU is on the DIAGNOSTIC bus (1), not the powertrain bus. Querying
# bus 0 finds nothing at all and the car never fingerprints. Confirmed on this
# car: openpilot's query goes out on bus 1 and 0x7e0 answers on 0x7e8 there.
# The sweep below is deliberately a superset - both rx offsets, OBD multiplexing
# on and off, and bus 0 as a last resort - because it is the combination that has
# actually fingerprinted this car.
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus, obd_multiplexing in [(1, True), (1, False), (0, False)] for request in [
    Request(
      [GWM_VERSION_REQUEST],
      [GWM_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      rx_offset=GWM_RX_OFFSET,
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
    Request(
      [GWM_VERSION_REQUEST],
      [GWM_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=bus,
      obd_multiplexing=obd_multiplexing,
    ),
  ]],
)

DBC = CAR.create_dbc_map()
