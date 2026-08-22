from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class CarControllerParams:
  # STEER_CMD (0x12b) is a 50 Hz frame. CarController runs at 100 Hz, so send on
  # every second frame.
  STEER_STEP = 2

  # Angle-rate limits, in degrees of commanded wheel angle per send step (50 Hz).
  # The breakpoints are v_ego in m/s. These bound how fast openpilot may move the
  # command; the panda safety mode enforces the same envelope independently.
  #
  # Reference measured from the car's own stock lane-centering (STEER_CMD while
  # the EPS was executing it): the command stayed within +-8 deg and moved at a
  # p99 of ~5 deg/s, spiking to ~25 deg/s only at engagement transitions. These
  # limits sit a little above that for authority without approaching the rates
  # the EPS rejects. They are a starting point and want road tuning - if the
  # wheel lags in a curve, raise the high-speed value; if it feels twitchy or
  # faults, lower it.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    300,        # max commanded angle, deg
    ([], []),   # v1 rate tables unused - the vehicle model supplies the limits
    ([], []),

    # Vehicle-model limits, as Tesla does. The hand-tuned v1 breakpoints allowed
    # 7.5 deg/s at 25 m/s and were cutting into the request in up to 23% of
    # engaged frames, which under-steers curves.
    MAX_LATERAL_ACCEL=3.0,   # m/s^2, ~ISO 11270 comfort
    MAX_LATERAL_JERK=2.5,    # m/s^3, conservative
    MAX_ANGLE_RATE=2.0,      # deg per 20 ms frame (100 deg/s), low-speed backstop
  )

  # Never let the TRANSMITTED angle sit more than this far from the measured wheel.
  # Applied AFTER the rate limiter, which is the whole point: clamping the limiter's
  # INPUT bounds nothing, because when the driver moves the wheel faster than the
  # command may slew, the command cannot follow and the error runs away. Driven
  # 2026-08-22 with the clamp on the input: 19.1% of frames exceeded it, to 88.5 deg.
  MAX_ANGLE_ERROR = 4.0  # deg

  # --- driver override -------------------------------------------------------
  # openpilot STOPS COMMANDING when the driver takes the wheel, as Tesla does
  # (lat_active = CC.latActive and hands_on_level < 3). The limiter then returns
  # the measured angle, so the command sits exactly where the wheel is: no error,
  # nothing to fight, nothing to snap back to.
  #
  # Measured on the camera across 34382 frames where it was commanding: driver
  # torque p99 is 129 and p99.9 is 183, and the OEM stops commanding once the
  # driver reaches roughly 176-272. So a gate at 150 sits above ordinary steering
  # and below where the OEM itself gives up.
  OVERRIDE_TORQUE = 150      # sustained |driver torque| that hands control back
  OVERRIDE_FRAMES = 7        # ~70 ms at 100 Hz, so brief spikes do not trigger it

  # Once handed back, STAY handed back for ~1 s before trying again. Without this
  # the latch clears the instant torque dips and openpilot grabs the wheel back
  # several times a second - which is the oscillation seen on 2026-08-22.
  OVERRIDE_HOLD_FRAMES = 100

  # A fast override ALWAYS disengages outright, the way Tesla treats its EPS high
  # angle rate fault. Requires torque as well as rate so that a genuine fast curve
  # cannot trigger it. The camera never exceeded 68 deg/s while commanding.
  FAST_OVERRIDE_RATE = 100.0   # deg/s of measured wheel movement
  FAST_OVERRIDE_TORQUE = 150   # and at least this much driver torque

  # EPS_FAULT_PERMANENT toggles spuriously in normal operation, so it only counts
  # as a fault once it has been continuously set for ~1 s.
  EPS_FAULT_FRAMES = 100

  # Driver-torque threshold (raw units of STEER_TORQUE.DRIVER_TORQUE) above which
  # the driver is considered to be holding the wheel. Provisional - confirm on
  # the road against a light hand.
  STEER_DRIVER_ALLOWANCE = 120


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
