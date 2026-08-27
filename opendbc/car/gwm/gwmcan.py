"""Message composition for the GWM Haval H6 PHEV19 MK4.

The steering command frame (STEER_CMD, 0x12b) is not built from scratch. It is a
*patch*: take the camera's own last STEER_CMD frame verbatim, overwrite the bytes
we own (the commanded angle, the two enable bytes, the counter in both of the
places it lives), recompute the two CRC-8s that cover them, and leave every other
byte exactly as the camera sent it.

What actually gates the frame is those two plain CRC-8s, both poly 0x1D:

  byte 8   CRC_COUNTER = crc8(bytes  9..15, init 0xD1)   covers COUNTER_1
  byte 16  CRC_STEER   = crc8(bytes 17..23, init 0x05)   covers the angle + COUNTER_2

Both were verified against captured frames - 3002/3002 on an OEM-engaged segment
(route_c0) and 2999/2999 on an openpilot-engaged segment (drive e2). There is no
key involved. The frame does carry a 6-byte trailer that looks like a MAC, but it
is copied through untouched and does not gate the LKAS path; an earlier reading of
this port claimed the trailer authenticated the command and was the source of
intermittent disengagements. It is not, and it isn't.

The counter is the part that is easy to get wrong. COUNTER_1 (byte 15 low nibble)
and COUNTER_2 (byte 23 low nibble) were EQUAL in every captured frame - 3002/3002
and 2999/2999. They are one logical counter written into two CRC domains. Writing
a free-running counter into only COUNTER_2 leaves the pair inconsistent on
essentially every frame, which is exactly the shape of "the EPS accepts commands
most of the time and intermittently doesn't".
"""


def crc8(data: bytes, poly: int, init: int) -> int:
  crc = init
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
  return crc


# STEER_CMD (0x12b) byte map, verified against captured frames:
#   byte 8       CRC_COUNTER = crc8(bytes 9..15, poly 0x1D, init 0xD1)
#   byte 15      low nibble = COUNTER_1
#   byte 16      CRC_STEER   = crc8(bytes 17..23, poly 0x1D, init 0x05)
#   bytes 17-18  STEER_REQUEST, 14-bit big-endian: (B17 << 6) | (B18 >> 2)
#   byte 21      EPS_LKAS_ANGLE_ENABLE  0x3f active / 0x1f passive
#   byte 22      EPS_LKAS_ANGLE_CFG     0xe5 active / 0x81 passive
#   byte 23      low nibble = COUNTER_2
STEER_CRC_POLY = 0x1D
STEER_CRC_INIT = 0x05
COUNTER_CRC_INIT = 0xD1
STEER_ANGLE_OFFSET = -779.6
STEER_ANGLE_SCALE = 0.1

LKAS_ENABLE_ACTIVE = 0x3F
LKAS_CFG_ACTIVE = 0xE5


def create_steer_command(stock_frame: bytes, apply_angle: float, active: bool) -> bytes:
  """Patch the camera's own STEER_CMD frame with our angle, counter and CRCs.

  stock_frame must be the most recent raw 0x12b payload seen on the CAMERA bus. If
  it is missing (camera not yet heard from), the caller must not transmit. When
  inactive the enable/config bytes are left at whatever the camera is sending, so
  a passive frame stays byte-identical to the camera's apart from the counter.
  """
  d = bytearray(stock_frame)

  raw = round((apply_angle - STEER_ANGLE_OFFSET) / STEER_ANGLE_SCALE)
  raw = max(0, min(0x3FFF, raw))                       # clamp to the 14-bit field
  d[17] = (raw >> 6) & 0xFF
  d[18] = (d[18] & 0x03) | ((raw & 0x3F) << 2)         # keep byte-18 low 2 bits

  if active:
    d[21] = LKAS_ENABLE_ACTIVE
    d[22] = LKAS_CFG_ACTIVE

  # one logical counter, two homes, one step past the camera's
  counter = ((d[15] & 0x0F) + 1) & 0x0F
  d[15] = (d[15] & 0xF0) | counter
  d[23] = (d[23] & 0xF0) | counter

  d[8] = crc8(bytes(d[9:16]), STEER_CRC_POLY, COUNTER_CRC_INIT)
  d[16] = crc8(bytes(d[17:24]), STEER_CRC_POLY, STEER_CRC_INIT)
  return bytes(d)


# ---------------------------------------------------------------------------
# ACC_CMD (0x143) - the camera's longitudinal command.
#
# Same shape as STEER_CMD: take the camera's most recent frame and patch it. It
# has FOUR independent CRC blocks rather than two, each covering the seven bytes
# after its check byte, and each block ends with a counter nibble:
#
#   byte 0  = crc8(bytes  1..7,  init 0xF9)    counter at byte 7  low nibble
#   byte 8  = crc8(bytes  9..15, init 0x73)    counter at byte 15 low nibble
#   byte 16 = crc8(bytes 17..23, init 0x3F)    counter at byte 23 low nibble
#   byte 24 = crc8(bytes 25..31, init 0x79)    counter at byte 31 low nibble
#
# All verified 4000/4000 per block against captured frames. Bytes 56-63 are a MAC
# and are copied through untouched.
#
# The powertrain reads the brake-vs-gas STATE from BRAKE_GAS_STATE and its
# duplicate, NOT from BRAKE_OR_GAS_REQ. Regeneration keys off the state, so a
# brake request left on top of a copied "gas" state is a combination the OEM never
# sends - the motor stays in drive and the car brakes on friction alone.
ACC_CRC_BLOCKS = ((0, 1, 8, 0xF9), (8, 9, 16, 0x73), (16, 17, 24, 0x3F), (24, 25, 32, 0x79))
ACC_COUNTERS = (7, 15, 23, 31)

# BRAKE_CMD's OFF baseline as it appears ON THE WIRE. The camera sends exactly
# this in every frame where it is inactive or commanding gas - 140 in all 36k such
# frames. The DBC offset of -181 makes that read as -41 when decoded, but this
# builder works in RAW field units throughout to avoid mixing the two: the control
# law is fitted against raw counts, so raw is what it hands us.
BRAKE_OFF_RAW = 140

REQ_GAS = 12
REQ_BRAKE = 13

# BRAKE_GAS_STATE, and its duplicate BRAKE_GAS_STATE_2, as the camera sends them.
# Top bit of byte 29 (and of byte 12) is the one that moves: set for gas, clear for
# brake. Measured across 44k frames, those two bits always agree.
def _set_brake_gas_state(d: bytearray, braking: bool) -> None:
  if braking:
    d[29] &= 0x7F
    d[12] &= 0x7F
  else:
    d[29] |= 0x80
    d[12] |= 0x80


def create_longitudinal_command(stock_frame: bytes, gas_raw: int, brake_mag: int,
                                braking: bool, active: bool) -> bytes:
  """Patch the camera's ACC_CMD with our gas/brake demand, counters and CRCs.

  UNITS. `gas_raw` is the GAS_CMD field exactly as it goes on the wire, and
  `brake_mag` is how far BELOW the off baseline to pull BRAKE_CMD, so zero means
  brake off. Both are what the fitted control law produces. Applying the DBC
  offsets here as well put brake-off on the wire as raw 181 instead of 140, which
  reads as a live brake demand while commanding gas - the panda rejected every
  frame, correctly.

  When inactive the camera's own demand is left alone and only the counters and
  CRCs are refreshed.
  """
  d = bytearray(stock_frame)

  if active:
    d[9] = (d[9] & 0xE0) | (REQ_BRAKE if braking else REQ_GAS)

    d[13] = max(0, min(0xFF, BRAKE_OFF_RAW - brake_mag))

    graw = max(0, min(0x1FFF, gas_raw))
    d[27] = (d[27] & 0xE0) | ((graw >> 8) & 0x1F)
    d[28] = graw & 0xFF

    _set_brake_gas_state(d, braking)

  # every block carries its own counter, stepped one past the camera's
  for i in ACC_COUNTERS:
    d[i] = (d[i] & 0xF0) | (((d[i] & 0x0F) + 1) & 0x0F)

  for cb, lo, hi, init in ACC_CRC_BLOCKS:
    d[cb] = crc8(bytes(d[lo:hi]), STEER_CRC_POLY, init)

  return bytes(d)
