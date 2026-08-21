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

  # EPS_LKAS_ANGLE_CFG (byte 22) is a steering EFFORT level, not a config constant.
  # It reads as LKAS_EFFORT_BASE + percent, 0-100.
  #
  # This was originally recorded as a constant 0xe5 active / 0x81 passive, because
  # every openpilot-engaged capture contained only the two values we send ourselves.
  # The OEM ICC drive (route 000001ef) shows the camera ramping it: +4 per 50 Hz
  # frame from 0x81 to 0xe5 over ~0.48 s on engagement, and back down over ~0.30 s
  # on release, at every one of the 20 transitions across 7 labelled ICC runs.
  # See haval-port captures/2026-08-21-steering-effort.md.
  LKAS_EFFORT_BASE = 0x81
  LKAS_EFFORT_MAX = 100
  LKAS_EFFORT_UP = 4       # matches the camera's ramp in, 0->100 in ~0.5 s
  LKAS_EFFORT_DOWN = 6     # matches the camera's ramp out, 100->0 in ~0.3 s

  # Effort held while the driver is fighting the wheel. Zero means the EPS stops
  # pushing entirely and the wheel goes light, which is the point.
  #
  # NOTE: zero effort with the enable bit still asserted is a state the OEM camera
  # was never observed in - it ramps down through the low values and then clears
  # the enable. If the EPS faults on it, raise this floor rather than reverting the
  # wind-down.
  LKAS_EFFORT_OVERRIDE = 0

  # Driver torque that counts as fighting the wheel, with hysteresis so a hand
  # resting near the threshold doesn't chatter the effort up and down. Sits above
  # STEER_DRIVER_ALLOWANCE for the same reason Toyota's LTA allowance does: some
  # resistance while changing lanes should not immediately cut authority.
  # Measured on the first steered drive: p50 driver torque 33, override peaks 250+.
  STEER_OVERRIDE_TORQUE = 150
  STEER_OVERRIDE_RELEASE = 100


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
