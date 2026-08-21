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

from opendbc.car.gwm.values import CarControllerParams

LKAS_EFFORT_BASE = CarControllerParams.LKAS_EFFORT_BASE
LKAS_EFFORT_MAX = CarControllerParams.LKAS_EFFORT_MAX


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
#   byte 22      EPS_LKAS_ANGLE_CFG     effort, 0x81 + percent (0x81 idle, 0xe5 full)
#   byte 23      low nibble = COUNTER_2
STEER_CRC_POLY = 0x1D
STEER_CRC_INIT = 0x05
COUNTER_CRC_INIT = 0xD1
STEER_ANGLE_OFFSET = -779.6
STEER_ANGLE_SCALE = 0.1

LKAS_ENABLE_ACTIVE = 0x3F
LKAS_ENABLE_PASSIVE = 0x1F


def create_steer_command(stock_frame: bytes, apply_angle: float, enable: bool, effort: int) -> bytes:
  """Patch the camera's own STEER_CMD frame with our angle, effort, counter and CRCs.

  stock_frame must be the most recent raw 0x12b payload seen on the CAMERA bus. If
  it is missing (camera not yet heard from), the caller must not transmit.

  `effort` is the EPS authority in percent, 0-100, and the caller is responsible for
  ramping it - see CarController. The enable byte and the effort byte are always
  written rather than copied, so an OEM engagement on the camera bus can never be
  forwarded to the EPS through us. Idle values (0x1f / 0x81) are exactly what the
  camera sends at rest, so a passive frame still matches it byte for byte apart
  from the counter.
  """
  d = bytearray(stock_frame)

  raw = round((apply_angle - STEER_ANGLE_OFFSET) / STEER_ANGLE_SCALE)
  raw = max(0, min(0x3FFF, raw))                       # clamp to the 14-bit field
  d[17] = (raw >> 6) & 0xFF
  d[18] = (d[18] & 0x03) | ((raw & 0x3F) << 2)         # keep byte-18 low 2 bits

  d[21] = LKAS_ENABLE_ACTIVE if enable else LKAS_ENABLE_PASSIVE
  d[22] = LKAS_EFFORT_BASE + max(0, min(LKAS_EFFORT_MAX, effort))

  # one logical counter, two homes, one step past the camera's
  counter = ((d[15] & 0x0F) + 1) & 0x0F
  d[15] = (d[15] & 0xF0) | counter
  d[23] = (d[23] & 0xF0) | counter

  d[8] = crc8(bytes(d[9:16]), STEER_CRC_POLY, COUNTER_CRC_INIT)
  d[16] = crc8(bytes(d[17:24]), STEER_CRC_POLY, STEER_CRC_INIT)
  return bytes(d)
