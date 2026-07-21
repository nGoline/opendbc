""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.gwm.values import CAR

Ecu = CarParams.Ecu

# CAN fingerprint from Haval H6 PHEV 2026 (dongle c8b5c5dbab75f7b2, routes 00000002/00000003).
# OBD/FW query returns 0 ECUs on this harness (ecu_responses=[]), so FW-only matching falls
# through to MOCK. Exact CAN fingerprint identifies the car without OBD.
FINGERPRINTS = {
  CAR.GWM_HAVAL_H6_MK4: [{
    96: 64, 161: 8, 181: 64, 189: 16, 199: 8, 240: 8, 241: 8, 259: 64, 273: 8, 283: 64,
    288: 64, 295: 8, 296: 8, 299: 64, 302: 64, 311: 64, 315: 64, 323: 64, 327: 64, 347: 64,
    357: 8, 367: 64, 395: 64, 411: 8, 412: 16, 415: 64, 515: 64, 536: 8, 545: 8, 546: 64,
    551: 64, 570: 8, 573: 64, 576: 64, 579: 64, 581: 8, 589: 64, 596: 8, 623: 8, 628: 64,
    639: 16, 649: 64, 659: 8, 661: 8, 664: 64, 670: 64, 671: 8, 674: 48, 683: 64, 692: 64,
    696: 64, 699: 64, 707: 64, 714: 8, 717: 8, 719: 64, 726: 64, 768: 8, 783: 8, 793: 16,
    795: 8, 799: 8, 833: 16, 837: 16, 849: 16, 880: 8, 901: 8, 917: 16, 1001: 16, 1045: 8,
    1281: 8,
  }],
}

FW_VERSIONS = {
  CAR.GWM_HAVAL_H6: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x873612100XEC56000\xf1\x89S013A01XKN17002',  # Haval H6 HEV 2023
    ],
  },
  CAR.GWM_HAVAL_H6_MK4: {
    (Ecu.engine, 0x7e0, None): [
      b'\xf1\x873612100XEB83000\xf1\x89S013A01XKN34003',  # Haval H6 PHEV 2026
    ],
  },
}
