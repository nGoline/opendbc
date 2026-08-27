from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class GwmSafetyFlags(IntFlag):
  # openpilot owns engagement from the soft-down stalk gesture instead of following
  # the stock ACC. Must be set whenever the port runs pcmCruise=False, or openpilot
  # would believe it is engaged while the panda blocks every frame.
  OP_CRUISE = 1
  # openpilot drives longitudinal. Without it the panda will not transmit ACC_CMD.
  LONG_CONTROL = 2


# GEAR_STALK (0xC7) byte 1 is an enumerated position, always a multiple of 15.
# Measured across 59494 frames: 1 rest, 3 soft UP, 4 hard UP, 5 soft DOWN, 6 hard
# DOWN. Every stock-ACC engagement in our captures was preceded by a hard DOWN;
# soft DOWN appeared 9 times and never once with cruise engaged, so the stock ACC
# ignores it. That is what makes it usable as openpilot's own engage gesture.
STALK_REST = 1
STALK_SOFT_DOWN = 5
STALK_HARD_DOWN = 6


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
  # 8.0 deg. This was 4.0, matching the widest error the camera was ever seen to
  # run. That was too tight: entering a turn openpilot wants to lead the wheel by
  # about 7 deg, and capping the lead at 4 left the command 3.5 deg short of the
  # request, so the wheel arrived late. Measured 2026-08-23, |desired - measured|
  # runs to p99 3.78 and a max of 8.18 - that is the lead the controller actually
  # needs, and throttling it is what under-steers turns and trips steerSaturated.
  #
  # This is now a backstop rather than the override mechanism. Overrides are
  # handled by handing control back at OVERRIDE_TORQUE, which stops commanding
  # altogether; the clamp only bounds the EPS during the ~70 ms before that latch
  # engages, and for forces too light to trigger it.
  MAX_ANGLE_ERROR = 8.0  # deg

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

  # A hard grab disengages outright. Two tiers: OVERRIDE_TORQUE hands lateral
  # control back and stays engaged, this disengages.
  #
  # Torque, NOT wheel rate. A first attempt used rate > 100 deg/s and disengaged on
  # ordinary turns, because that was calibrated against the CAMERA commanding
  # hands-off (max 68 deg/s) rather than against a driver. Measured on the drives
  # of 2026-08-22: while openpilot is steering the wheel rate reaches p99 64 and a
  # max of 117 deg/s, but in manual driving it reaches p99 359 and a max of 3308.
  # There is no rate threshold that separates "override" from "turning the wheel".
  #
  # Driver torque does separate them. While openpilot steers: p99 120, p99.9 242,
  # max 344. 400 never fired across either drive, and leaves the 150-400 band to
  # hand control back without disengaging - which is what forcing a turn should do.
  DISENGAGE_TORQUE = 400
  DISENGAGE_FRAMES = 5   # ~50 ms sustained, so a single spike cannot disengage

  # --- longitudinal ------------------------------------------------------
  # Feedforward FITTED to the car's own behaviour: over 18116 frames where the
  # camera commanded gas and 4310 where it braked, regressing the pedal it chose
  # against the acceleration that resulted 0.3 s later gives
  #
  #   gas_raw   =  1177*accel + 5.8*v + 12892*sin(pitch) + 1110   (R2 0.79)
  #   brake_mag = -18.4*accel - 0.08*v +     0*sin(pitch) +  1.6  (R2 0.88)
  #
  # The grade term is the one that matters for smoothness, and it validates
  # itself: if it really is gravity it should equal GAS_PER_ACCEL * 9.81 = 11546,
  # and the fit independently found 12892. Feeding it forward means the
  # integrator starts near the right pedal on a hill instead of having to
  # discover it, which is what a driver does by feel.
  #
  # CAVEAT: the fitting drive was grade-biased (mostly downhill), so the constant
  # and speed terms are less trustworthy than the accel and grade gains. The
  # integrator absorbs the residual; expect to revisit these after a hilly drive.
  GAS_PER_ACCEL = 1177.0     # raw counts per m/s^2
  GAS_PER_SPEED = 5.8        # raw counts per m/s, drag and rolling resistance
  GAS_BASE = 1110.0          # raw counts at zero accel, zero speed, flat
  BRAKE_PER_ACCEL = 18.4     # magnitude counts per m/s^2 of deceleration
  BRAKE_BASE = 1.6
  # NOTE the brake fit's grade coefficient came out at ~0, so the brake magnitude
  # is set by the target deceleration ALONE. Feeding the grade in here as well
  # over-braked by 0.88 m/s^2 against the camera's own commands.

  # Gas and brake are mutually exclusive on this car - the camera never commands
  # both - so a deadband decides which. Hysteresis keeps a demand hovering near
  # zero from flapping between pedals.
  # MEASURED, not chosen. Sweeping the threshold against 26344 frames where the
  # camera was commanding, -0.98 on (accel + grade) reproduces its gas-vs-brake
  # choice 93.9% of the time; thresholding on accel alone tops out at 83%, so the
  # grade belongs in this decision even though it does not belong in the brake
  # magnitude. A first guess of -0.10 agreed only 34.7% of the time.
  LIFT_OFF_ACCEL = -1.00     # below this (grade included) we brake
  RESUME_GAS_ACCEL = -0.80   # above this we go back to gas

  ACCEL_MIN = -3.5           # m/s^2
  ACCEL_MAX = 2.0            # m/s^2

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
