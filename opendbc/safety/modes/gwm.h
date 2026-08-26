#pragma once

#include "opendbc/safety/declarations.h"

// GWM Haval H6 PHEV19 MK4 - angle-based lateral, lateral-only (stock ACC keeps
// longitudinal). openpilot sends STEER_CMD (0x12b); everything else is RX.

#define GWM_STEER_CMD     0x12bU  // TX to EPS: STEER_REQUEST angle command
#define GWM_GEAR_STALK    0x0c7U  // RX: gear stalk position - engages openpilot
#define GWM_STEER_ANGLE   0x0a1U  // RX: measured wheel angle + direction
#define GWM_STEER_TORQUE  0x147U  // RX: driver torque
#define GWM_WHEEL_SPEEDS  0x13bU  // RX: wheel speeds
#define GWM_ACC           0x2abU  // RX: cruise state (camera bus)
#define GWM_GAS           0x0b5U  // RX: gas pedal
#define GWM_BRAKE         0x120U  // RX: brake pressed

// openpilot owns engagement instead of following the stock ACC. Set when the port
// runs pcmCruise=False. Without it this mode behaves exactly as before.
#define GWM_FLAG_OP_CRUISE 1

#define GWM_MAIN_BUS 0U
// The forward camera transmits STEER_CMD, LATERAL_STATE and ACC. Once the relay
// opens they are only on the camera bus - measured over a full engaged drive,
// 0x2ab appeared 600 times on bus 2 and zero times on bus 0. Watching bus 0 for
// the cruise state means never seeing it and never allowing controls.
#define GWM_CAM_BUS 2U

// STEER_REQUEST is 0.1 deg per count, zero at raw 7796. The measured angle
// (0xA1) is 0.1 deg per count with a direction bit. Both are kept in 0.1-deg
// units so the command and the measurement share one signed scale.
#define GWM_STEER_ZERO 7796

// Driver torque that disengages outright, as an independent backstop to the same
// rule in carstate. Set ABOVE the openpilot-side threshold of 400 so openpilot
// disengages first and this only catches the case where it did not. Measured over
// the 2026-08-22 drives, driver torque never exceeded 344 while openpilot was
// steering, and 449 at any point.
#define GWM_DISENGAGE_TORQUE 450

static bool gwm_op_cruise = false;
static bool gwm_engage_prev = false;

static void gwm_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == GWM_MAIN_BUS) {
    if (msg->addr == GWM_GEAR_STALK) {
      // GEAR_STALK position, byte 1: bit 6 = DOWN, bit 4 = FURTHER. A SOFT down
      // (down without further) is the gesture openpilot engages on - the stock ACC
      // ignores it entirely, which is the whole point: it never engages, so it
      // never chimes. A hard down is the stock ACC's own gesture and must NOT
      // engage us, or both systems would drive at once.
      //
      // Requiring the car to be moving means a stationary gear shift cannot arm
      // controls: DOWN is the same physical motion as R->N.
      const bool soft_down = ((msg->data[1] & 0x40U) != 0U) && ((msg->data[1] & 0x10U) == 0U);
      if (gwm_op_cruise) {
        if (soft_down && !gwm_engage_prev && vehicle_moving) {
          controls_allowed = true;
        }
      }
      gwm_engage_prev = soft_down;
    }

    if (msg->addr == GWM_STEER_ANGLE) {
      // AP_CANCEL_COMMAND, byte 5 bit 6. Always exits controls, in either mode.
      if ((msg->data[5] & 0x40U) != 0U) {
        controls_allowed = false;
      }

      int raw = (int)(((msg->data[1] & 0x3FU) << 7) | (msg->data[2] >> 1));  // STEERING_ANGLE, 0.1 deg
      int sign = ((msg->data[2] & 0x1U) != 0U) ? -1 : 1;                     // STEERING_DIRECTION, 1 = right
      update_sample(&angle_meas, raw * sign);
    }

    if (msg->addr == GWM_WHEEL_SPEEDS) {
      int fl = (int)(((msg->data[1] & 0x1FU) << 8) | msg->data[2]);          // WHEEL_SPEED_FL
      vehicle_moving = fl > 0;
      UPDATE_VEHICLE_SPEED(fl * 0.05924739 * KPH_TO_MS);
    }

    if (msg->addr == GWM_STEER_TORQUE) {
      // DRIVER_TORQUE: signed 11-bit, bit 78 len 11 big-endian
      int t = (int)(((msg->data[9] & 0x7FU) << 4) | (msg->data[10] >> 4));
      update_sample(&torque_driver, to_signed(t, 11));

      // A firm grab disengages outright, on the rising edge. openpilot detects the
      // fast-override case itself and reports it through steeringDisengage; this is
      // the independent backstop, and it is why the threshold sits well above the
      // 150 that merely hands lateral control back.
      const int max_tq = SAFETY_MAX(SAFETY_ABS(torque_driver.min), SAFETY_ABS(torque_driver.max));
      steering_disengage = max_tq > GWM_DISENGAGE_TORQUE;
    }

    if (msg->addr == GWM_GAS) {
      gas_pressed = msg->data[30] > 0U;                                      // GAS_USER
    }

    if (msg->addr == GWM_BRAKE) {
      brake_pressed = ((msg->data[1] >> 3) & 1U) != 0U;                      // BRAKE_PRESSED
    }
  }

  if ((msg->bus == GWM_CAM_BUS) && (msg->addr == GWM_ACC)) {
    // CRUISE_ENGAGED, byte 47 bit 4. Ground-truthed against seven labelled ACC and
    // ICC engagement runs. carstate reads this exact bit, so the two gates agree.
    if (!gwm_op_cruise) {
      pcm_cruise_check(((msg->data[47] >> 4) & 1U) != 0U);
    }
  }
}

static bool gwm_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Deliberately a little looser than openpilot's own limits in values.py
  // (CarControllerParams.ANGLE_LIMITS), so normal commanding never trips the
  // panda, but not so loose that the envelope stops meaning anything.
  static const AngleSteeringLimits GWM_STEERING_LIMITS = {
    .max_angle = 3000,          // 300 deg, in 0.1-deg CAN units
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // Must match VehicleModel(CarInterface.get_non_essential_params(...)) on the
  // openpilot side, or the two envelopes disagree.
  static const AngleSteeringParams GWM_STEERING_PARAMS = {
    .slip_factor = -0.000612593970148998,  // calc_slip_factor(VM)
    .steer_ratio = 18.,
    .wheelbase = 2.738,
  };

  if (msg->addr == GWM_STEER_CMD) {
    int raw = (int)((msg->data[17] << 6) | (msg->data[18] >> 2));            // STEER_REQUEST
    int desired_angle = raw - GWM_STEER_ZERO;                               // centered, 0.1 deg
    bool lka_active = msg->data[21] == 0x3FU;                               // EPS_LKAS_ANGLE_ENABLE

    if (steer_angle_cmd_checks_vm(desired_angle, lka_active, GWM_STEERING_LIMITS, GWM_STEERING_PARAMS)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config gwm_init(uint16_t param) {
  gwm_op_cruise = GET_FLAG(param, GWM_FLAG_OP_CRUISE);
  gwm_engage_prev = false;
  static const CanMsg GWM_TX_MSGS[] = {
    {GWM_STEER_CMD, GWM_MAIN_BUS, 64, .check_relay = true},
  };

  // Frequencies measured off this car (route_c0 segment 40, 60 s).
  static RxCheck gwm_rx_checks[] = {
    {.msg = {{GWM_STEER_ANGLE,  GWM_MAIN_BUS, 8,  100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_WHEEL_SPEEDS, GWM_MAIN_BUS, 64, 50U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_STEER_TORQUE, GWM_MAIN_BUS, 64, 50U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_GAS,          GWM_MAIN_BUS, 64, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_BRAKE,        GWM_MAIN_BUS, 64, 50U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_GEAR_STALK,   GWM_MAIN_BUS, 8,  20U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{GWM_ACC,          GWM_CAM_BUS,  64, 10U,  .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(gwm_rx_checks, GWM_TX_MSGS);
}

const safety_hooks gwm_hooks = {
  .init = gwm_init,
  .rx = gwm_rx_hook,
  .tx = gwm_tx_hook,
};
