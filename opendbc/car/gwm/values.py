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
    300,                                    # max commanded angle, deg
    ([0., 5., 25.], [1.0, 0.5, 0.15]),      # up:   deg/step vs m/s
    ([0., 5., 25.], [1.5, 0.8, 0.25]),      # down: deg/step vs m/s
  )

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


# Both DIDs come back concatenated in a single response, so the request has to
# ask for them together rather than one at a time:
#   b'\xf1\x873612100XEB83000\xf1\x89S013A01XKN34003'
GWM_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER) + \
  p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER)

GWM_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [GWM_VERSION_REQUEST],
      [GWM_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
