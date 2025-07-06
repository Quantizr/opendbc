#pragma once

#include "opendbc/safety/safety_declarations.h"

#define CAN_ACTUATOR_TQ_FAC 0.125
#define CAN_ACTUATOR_CONTROL_STATUS_SOFTOFF_BIT 2

// enum {
//   ODYSSEY_BTN_NONE = 0,
//   // HONDA_BTN_MAIN = 1,
//   ODYSSEY_BTN_SET = 1,
//   ODYSSEY_BTN_RESUME = 2,
//   ODYSSEY_BTN_CANCEL = 3,
// };

static void odyssey_rx_hook(const CANPacket_t *to_push) {
  const bool pcm_cruise = true;

  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  if (addr == 0x0C8) { //0x0C8 = ENGINE_DATA
    // first 2 bytes are XMISSION_SPEED
    vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
  }

  // check ACC main state
  if (addr == 0xD4) { //0xD4 = CRUISE_CONTROL
    acc_main_on = GET_BIT(to_push, 5U);
    if (!acc_main_on) {
      controls_allowed = false;
    }
  }

  // enter controls when PCM enters cruise state
  if (pcm_cruise && (addr == 0x12C)) { //POWERTRAIN_DATA
    const bool cruise_engaged = GET_BIT(to_push, 51U);
    pcm_cruise_check(cruise_engaged);
  }

  // TODO: for future with brake/throttle actuator where cruise might not be engaged
  // state machine to enter and exit controls for button enabling
  // if (addr == 0xD4) { // CRUISE_CONTTROL
  //   int button = (GET_BYTE(to_push, 0) & 0xC0U) >> 6; // top two bits

  //   // enter controls on the falling edge of set or resume
  //   bool set = (button != ODYSSEY_BTN_SET) && (cruise_button_prev == ODYSSEY_BTN_SET);
  //   bool res = (button != ODYSSEY_BTN_RESUME) && (cruise_button_prev == ODYSSEY_BTN_RESUME);
  //   if (acc_main_on && !pcm_cruise && (set || res)) {
  //     controls_allowed = true;
  //   }

  //   // exit controls once main or cancel are pressed
  //   if ((button == ODYSSEY_BTN_CANCEL)) {
  //     controls_allowed = false;
  //   }
  //   cruise_button_prev = button;
  // }

  if (addr == 0x12C) { // POWERTRAIN_DATA
    brake_pressed = GET_BIT(to_push, 48U);
  }

  if (addr == 0xAA) { // DRIVER_THROTTLE_POSITION
    gas_pressed = GET_BYTE(to_push, 0) > 1U;
  }

  if ((addr == 0x22F)  && (bus == 1)) {
    int torque_meas_new = (int8_t)(GET_BYTE(to_push, 2)) * (int)(CAN_ACTUATOR_TQ_FAC * 1000); //Nm * 1000
    update_sample(&torque_meas, torque_meas_new);

    // increase torque_meas by 1 to be conservative on rounding
    torque_meas.min--;
    torque_meas.max++;

    if((((GET_BYTE(to_push, 1)>>4)>>CAN_ACTUATOR_CONTROL_STATUS_SOFTOFF_BIT) & 0x1) != 0x0) { //Soft off status means motor is shutting down due to error
      controls_allowed = false;
    }
  }
}

static bool odyssey_tx_hook(const CANPacket_t *to_send) {

  // StepperServoCan adjusts torque in increments of 0.125 Nm
  const TorqueSteeringLimits ODYSSEY_STEERING_LIMITS = { // multiplied by 1000 since max_torque, max_rate_up, etc. are ints
    .max_torque = 5000, // 2.5 Nm * 1000 *2
    .dynamic_max_torque = false,
    // .max_torque_lookup = {
    //   {9., 17., 17.},
    //   {350, 250, 250},
    // },
    .max_rate_up = 125, // real value should be 60 but torque is in 0.125 Nm increments
    .max_rate_down = 125, // real value should be 100 but torque is in 0.125 Nm increments
    .max_rt_delta = 2750, // max change per 250ms with 10% buffer, real max_rate_down * 100 * 0.25 * 1.1 * 2
    .max_torque_error = 1000,
    .type = TorqueMotorLimited,
  };

  UNUSED(to_send);
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);


  // STEER: safety check
  if ((addr == 0x22E) && (bus == 1)) {
    bool steer_req = ((GET_BYTE(to_send, 1) >> 4) & 0b11u) != 0x0;
    int desired_torque = (int8_t)(GET_BYTE(to_send, 4)) * (int)(CAN_ACTUATOR_TQ_FAC * 1000); // multiplied by 1000 since TorqueSteeringLimits are ints
    if (steer_torque_cmd_checks(desired_torque, steer_req, ODYSSEY_STEERING_LIMITS)) {
      tx = false;
    }
  }
  return tx;
}

static safety_config odyssey_init(uint16_t param) {

  // Note: even though this message can't ever spontaneously appear on destination bus, we need check_relay = true for
  // MADS safety tests since stock_ecu_check is called when check_relay = true and mads_state_update happens in stock_ecu_check
  static const CanMsg ODYSSEY_TX_MSGS[] = {{0x22E, 1, 5, .check_relay = true}}; //STEERING_COMMAND

  static RxCheck odyssey_rx_checks[] = {
    {.msg = {{0x405, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 3U}, { 0 }, { 0 }}}, //BODY
    {.msg = {{0x6A, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 143U}, { 0 }, { 0 }}}, //BRAKE_PRESSURE
    {.msg = {{0xD4, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, //CRUISE_CONTROL
    {.msg = {{0xAA, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, //DRIVER_THROTTLE_POSITION
    {.msg = {{0xC8, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, //ENGINE_DATA
    {.msg = {{0x188, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, //GEARBOX
    {.msg = {{0x1F4, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 25U}, { 0 }, { 0 }}}, //LIGHTS
    {.msg = {{0x12C, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, //POWERTRAIN_DATA
    {.msg = {{0x1C0, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 48U}, { 0 }, { 0 }}}, //WHEEL_SPEEDS
    {.msg = {{0x22F, 1, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, //STEERING_STATUS
  };

  UNUSED(param);
  safety_config ret = BUILD_SAFETY_CFG(odyssey_rx_checks, ODYSSEY_TX_MSGS);
  ret.disable_forwarding = true;
  return ret;
}

const safety_hooks honda_odyssey_hooks = {
  .init = odyssey_init,
  .rx = odyssey_rx_hook,
  .tx = odyssey_tx_hook,
  // .get_counter = honda_get_counter,
  // .get_checksum = honda_get_checksum,
  // .compute_checksum = honda_compute_checksum,
};

