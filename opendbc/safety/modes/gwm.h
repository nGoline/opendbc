#pragma once

#include "opendbc/safety/declarations.h"

#define GWM_ADAS_ACTIVATION      0xA1U // RX from STEER_AND_AP_STALK
#define GWM_GAS                  0x60U // RX from CAR_OVERALL_SIGNALS
#define GWM_BRAKE               0x120U // RX from BRAKE2
#define GWM_SPEED               0x13BU // RX from WHEEL_SPEEDS
#define GWM_RX_STEER_RELATED    0x147U // RX from EPS to CAMERA
#define GWM_STEER_CMD           0x12BU // TX from OP to EPS
#define GWM_CRUISE              0x2ABU
#define GWM_LONG_CONTROL        0x143U // TX from OP to PCM
#define GWM_BLIND_SPOT          0x16FU
#define GWM_HUD                 0x23DU
#define GWM_GEAR_STALK          0xC7U // RX, MK4 op-cruise: gentle/further DOWN lever gesture (arms controls)

// CAN bus
#define GWM_MAIN_BUS 0U
#define GWM_CAMERA_BUS  2U

// MK4 owns its own cruise loop (FLAG_GWM_OP_CRUISE): arm controls on the gentle-DOWN stalk gesture
// (GWM_GEAR_STALK) instead of the FURTHER_DOWN-only msg 161 bit47. Set in gwm_init from the safety param.
static bool gwm_op_cruise = false;
static bool gear_stalk_down_prev = false;
// MK4 steers by angle (FLAG_GWM_ANGLE_CONTROL): STEER_CMD carries a 14-bit angle at bytes 17-18
// instead of the MK3 10-bit torque at bytes 12-13. Set in gwm_init from the safety param.
static bool gwm_angle_control = false;

static uint8_t gwm_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if ((msg->addr == GWM_SPEED) || (msg->addr == GWM_ADAS_ACTIVATION)) {
    cnt = msg->data[7] & 0xFU;
  } else {
  }
  return cnt;
}

// 64-byte messages carry an independent CRC8 (poly 0x1D) per 8-byte block. GWM_RX_STEER_RELATED and
// GWM_GAS validate block B (bytes 8-15), where the EPS torque / gas signals used below live.
static uint32_t gwm_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if ((msg->addr == GWM_SPEED) || (msg->addr == GWM_ADAS_ACTIVATION)) {
    chksum = msg->data[0] & 0xFFU;
  } else if ((msg->addr == GWM_RX_STEER_RELATED) || (msg->addr == GWM_GAS)) {
    chksum = msg->data[8] & 0xFFU;
  } else {
  }
  return chksum;
}

static uint32_t gwm_compute_checksum(const CANPacket_t *msg) {
  uint8_t crc = 0x00;
  const uint8_t poly = 0x1D;
  uint8_t xor_out = 0x00;
  int start = 1;

  // xor_out constants derived from logged frames (routes 075b.../00000163 MK3, 04219e.../000000c3 MK4);
  // 0x61 and 0x95 are constant across both platforms
  if (msg->addr == GWM_ADAS_ACTIVATION) {
    xor_out = 0x2DU;
  } else if (msg->addr == GWM_SPEED) {
    xor_out = 0x7FU;
  } else if (msg->addr == GWM_RX_STEER_RELATED) {
    // block B: CRC at byte 8 covers bytes 9-15
    xor_out = 0x61U;
    start = 9;
  } else if (msg->addr == GWM_GAS) {
    // block B: CRC at byte 8 covers bytes 9-15 (GAS_POSITION)
    xor_out = 0x95U;
    start = 9;
  } else {
  }

  for (int i = start; i < (start + 7); i++) {
    uint8_t byte = msg->data[i];
    crc ^= byte;
    for (int bit = 0; bit < 8; bit++) {
      if ((crc & 0x80U) != 0U) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
      crc &= 0xFFU;
    }
  }
  uint8_t chksum = crc ^ xor_out;
  return chksum;
}

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == GWM_MAIN_BUS) {
    // GAS_POSITION
    if (msg->addr == GWM_GAS) {
      gas_pressed = msg->data[9] > 0U;
    }

    if (msg->addr == GWM_SPEED) {
      uint32_t fl = ((msg->data[1] << 8) | msg->data[2]) & 0x1FFFU;
      uint32_t fr = ((msg->data[3] << 8) | msg->data[4]) & 0x1FFFU;
      uint32_t rl = ((msg->data[41] << 8) | msg->data[42]) & 0x1FFFU;
      uint32_t rr = ((msg->data[43] << 8) | msg->data[44]) & 0x1FFFU;
      float speed = (float)((fr + rr + rl + fl) / 4.0f * 0.05924739 * KPH_TO_MS);
      vehicle_moving = speed > 0.0f;
      UPDATE_VEHICLE_SPEED(speed);
    }

    if (msg->addr == GWM_BRAKE) {
      brake_pressed = GET_BIT(msg, 11U);
    }

    if (msg->addr == GWM_RX_STEER_RELATED) {
      int torque_meas_new = ((msg->data[13] & 0x7U) << 8) | (msg->data[14]);
      torque_meas_new = to_signed(torque_meas_new, 11) + 548;
      update_sample(&torque_meas, torque_meas_new);

      // increase torque_meas by 1 to be conservative on rounding
      torque_meas.min--;
      torque_meas.max++;
    }

    // state machine to enter and exit controls for button enabling
    if (msg->addr == GWM_ADAS_ACTIVATION) {
      if (gwm_angle_control) {
        // STEERING_ANGLE (13|13@0+, 0.1 deg) is an unsigned magnitude; STEERING_DIRECTION (bit 16) gives the side.
        // Stored in 0.1 deg units to match the STEER_CMD angle command scale.
        uint32_t angle_raw = (((uint32_t)msg->data[1] & 0x3FU) << 7) | ((uint32_t)msg->data[2] >> 1);
        int angle_meas_new = (int)angle_raw;
        if ((msg->data[2] & 0x01U) != 0U) {
          angle_meas_new = -angle_meas_new;
        }
        update_sample(&angle_meas, angle_meas_new);
      }

      bool cruise_button = GET_BIT(msg, 47U);
      // enter controls on the rising edge of the FURTHER_DOWN stalk gesture (MK3 / non-op-cruise only;
      // MK4 op-cruise arms off the gentle-DOWN gesture on GWM_GEAR_STALK below)
      if (!gwm_op_cruise && cruise_button && !cruise_button_prev) {
        acc_main_on = true;
      }
      // exit controls once cancel (UP / lateral button) or brake is pressed — applies to both paths
      bool cancel_button = GET_BIT(msg, 46U);
      if (cancel_button || brake_pressed) {
        acc_main_on = false;
      }
      pcm_cruise_check(acc_main_on);
      cruise_button_prev =  cruise_button ? 1 : 0;
    }

    // MK4 op-cruise: enter controls on the rising edge of the gentle-or-further DOWN stalk gesture.
    // Cancel/brake still disarm via GWM_ADAS_ACTIVATION above. carstate engages openpilot on the same bit.
    if (gwm_op_cruise && (msg->addr == GWM_GEAR_STALK)) {
      bool stalk_down = GET_BIT(msg, 14U);
      if (stalk_down && !gear_stalk_down_prev) {
        acc_main_on = true;
      }
      pcm_cruise_check(acc_main_on);
      gear_stalk_down_prev = stalk_down;
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits GWM_TORQUE_STEERING_LIMITS = {
    .max_torque = 253,
    .max_rate_up = 4,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .max_rt_delta = 100,
    .type = TorqueMotorLimited,
  };

  const LongitudinalLimits GWM_LONG_LIMITS = {
    .max_gas = 4577,
    .min_gas = -10,
    .inactive_gas = 0,
    .max_brake = 107,
  };

  const AngleSteeringLimits GWM_ANGLE_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, matches CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off GWM_HAVAL_H6_MK4 CarSpecs to match openpilot
  const AngleSteeringParams GWM_ANGLE_STEERING_PARAMS = {
    .slip_factor = -0.00061259397,  // calc_slip_factor(VM)
    .steer_ratio = 18.0,
    .wheelbase = 2.738,
  };

  bool tx = true;
  bool violation = false;

  if (msg->bus == GWM_MAIN_BUS) {
    if (msg->addr == GWM_STEER_CMD) {
      if (gwm_angle_control) {
        // MK4: 14-bit angle command (STEER_REQUEST 143|14@0+, factor 0.1, offset -779.6 deg).
        // Kept in 0.1 deg units; -7796 recenters the DBC offset so 0 = straight ahead.
        uint32_t desired_angle_raw = (((uint32_t)msg->data[17] << 6) | ((uint32_t)msg->data[18] >> 2)) & 0x3FFFU;
        int desired_angle = (int)desired_angle_raw - 7796;
        // EPS_LKAS_ANGLE_ENABLE (byte 21): 0x3F commands the EPS to execute the angle
        bool steer_control_enabled = msg->data[21] == 0x3FU;
        violation |= steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, GWM_ANGLE_STEERING_LIMITS, GWM_ANGLE_STEERING_PARAMS);
      } else {
        int desired_torque = (((msg->data[12] & 0x7FU) << 3) | ((msg->data[13] & 0xE0U) >> 5));
        desired_torque = to_signed(desired_torque, 10) + 1;
        bool steer_req = GET_BIT(msg, 125U);
        violation |= steer_torque_cmd_checks(desired_torque, steer_req, GWM_TORQUE_STEERING_LIMITS);
      }
    }

    if (msg->addr == GWM_LONG_CONTROL) {
      int brake_raw = msg->data[13];
      brake_raw = 181 - brake_raw;
      violation |= longitudinal_brake_checks(brake_raw, GWM_LONG_LIMITS);

      int gas_raw = ((msg->data[27] & 0x1FU) << 8) | (msg->data[28]);
      gas_raw = gas_raw - 192;
      violation |= longitudinal_gas_checks(gas_raw, GWM_LONG_LIMITS);
    }
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static safety_config gwm_init(uint16_t param) {
  static const CanMsg GWM_TX_MSGS[] = {
    {GWM_ADAS_ACTIVATION, GWM_CAMERA_BUS, 8, .check_relay = false}, // Cancel command
    {GWM_RX_STEER_RELATED, GWM_CAMERA_BUS, 64, .check_relay = true}, // EPS steering feedback to camera
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true}, // Steering command
    {GWM_HUD, GWM_MAIN_BUS, 64, .check_relay = true}, // HUD and dashboard
  };

  static const CanMsg GWM_LONG_TX_MSGS[] = {
    {GWM_ADAS_ACTIVATION, GWM_CAMERA_BUS, 8, .check_relay = false}, // Cancel command
    {GWM_RX_STEER_RELATED, GWM_CAMERA_BUS, 64, .check_relay = true}, // EPS steering feedback to camera
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true}, // Steering command
    {GWM_LONG_CONTROL, GWM_MAIN_BUS, 64, .check_relay = true}, // Longitudinal control message from camera
    {GWM_HUD, GWM_MAIN_BUS, 64, .check_relay = true}, // HUD and dashboard
  };

  static RxCheck gwm_rx_checks[] = {
    {.msg = {{GWM_ADAS_ACTIVATION, GWM_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, steer rate
    {.msg = {{GWM_SPEED, GWM_MAIN_BUS, 64, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // speed
    {.msg = {{GWM_GAS, GWM_MAIN_BUS, 64, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // gas pedal (block B checksum)
    // BRAKE2 checksum stays ignored: its block-A xor is 0xEE on MK3 but not constant on MK4, and this safety
    // mode is shared. Counters on all three ignored: logs show a systematic ~1/15 counter irregularity.
    {.msg = {{GWM_BRAKE, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // brake2
    {.msg = {{GWM_RX_STEER_RELATED, GWM_MAIN_BUS, 64, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // eps feedback to camera (block B checksum)
    {.msg = {{GWM_STEER_CMD, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // copy stock steering cmd
    {.msg = {{GWM_CRUISE, GWM_CAMERA_BUS, 64, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // CRUISE_STATE, ACC
    {.msg = {{GWM_LONG_CONTROL, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // Longitudinal control message from camera
    {.msg = {{GWM_BLIND_SPOT, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // Blind spot monitor
    {.msg = {{GWM_HUD, GWM_CAMERA_BUS, 64, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // HUD and dashboard
  };

  // MK4 op-cruise needs GWM_GEAR_STALK whitelisted so the rx hook is called for it (the arm lives there).
  // 20 Hz periodic on the main bus; checksum/counter algorithm not reverse-engineered, so ignore both.
  // Separate array so MK3 (which has no 0xC7) doesn't fault on a missing message.
  static RxCheck gwm_op_cruise_rx_checks[] = {
    {.msg = {{GWM_ADAS_ACTIVATION, GWM_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // cruise state, steering angle, steer rate
    {.msg = {{GWM_SPEED, GWM_MAIN_BUS, 64, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // speed
    {.msg = {{GWM_GAS, GWM_MAIN_BUS, 64, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // gas pedal (block B checksum)
    // BRAKE2 checksum stays ignored: its block-A xor is 0xEE on MK3 but not constant on MK4, and this safety
    // mode is shared. Counters on all three ignored: logs show a systematic ~1/15 counter irregularity.
    {.msg = {{GWM_BRAKE, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // brake2
    {.msg = {{GWM_RX_STEER_RELATED, GWM_MAIN_BUS, 64, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // eps feedback to camera (block B checksum)
    {.msg = {{GWM_STEER_CMD, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // copy stock steering cmd
    {.msg = {{GWM_CRUISE, GWM_CAMERA_BUS, 64, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // CRUISE_STATE, ACC
    {.msg = {{GWM_LONG_CONTROL, GWM_CAMERA_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // Longitudinal control message from camera
    {.msg = {{GWM_BLIND_SPOT, GWM_MAIN_BUS, 64, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // Blind spot monitor
    {.msg = {{GWM_HUD, GWM_CAMERA_BUS, 64, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // HUD and dashboard
    {.msg = {{GWM_GEAR_STALK, GWM_MAIN_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, // MK4: gentle-DOWN engage gesture
  };

  bool gwm_longitudinal = false;
  gwm_op_cruise = false;
  gear_stalk_down_prev = false;
  gwm_angle_control = false;
#ifdef ALLOW_DEBUG
  const int FLAG_GWM_LONG_CONTROL = 1;
  const int FLAG_GWM_OP_CRUISE = 2;
  const int FLAG_GWM_ANGLE_CONTROL = 4;
  gwm_longitudinal = GET_FLAG(param, FLAG_GWM_LONG_CONTROL);
  gwm_op_cruise = GET_FLAG(param, FLAG_GWM_OP_CRUISE);
  gwm_angle_control = GET_FLAG(param, FLAG_GWM_ANGLE_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  // FIXME: cppcheck thinks that gwm_longitudinal is always false. This is not true
  // if ALLOW_DEBUG is defined but cppcheck is run without ALLOW_DEBUG
  // cppcheck-suppress knownConditionTrueFalse
  safety_config ret = gwm_longitudinal ? BUILD_SAFETY_CFG(gwm_rx_checks, GWM_LONG_TX_MSGS) : \
                                          BUILD_SAFETY_CFG(gwm_rx_checks, GWM_TX_MSGS);
  // cppcheck-suppress knownConditionTrueFalse
  if (gwm_op_cruise) {
    SET_RX_CHECKS(gwm_op_cruise_rx_checks, ret);
  }
  return ret;
}

const safety_hooks gwm_hooks = {
  .init = gwm_init,
  .rx = gwm_rx_hook,
  .tx = gwm_tx_hook,
  .get_counter = gwm_get_counter,
  .get_checksum = gwm_get_checksum,
  .compute_checksum = gwm_compute_checksum,
};
