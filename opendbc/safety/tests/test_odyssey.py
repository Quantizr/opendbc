#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerPanda, MAX_WRONG_COUNTERS
from opendbc.can.packer import CANPacker


class TestOdyssey(common.PandaCarSafetyTest):
  TX_MSGS = [[0x22E, 1]]
  RELAY_MALFUNCTION_ADDRS = {1: (0x22E)}
  FWD_BUS_LOOKUP = {}

  PT_BUS = 0
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerPanda("honda_odyssey_2005")
    self.packer_steer = CANPackerPanda("ocelot_controls")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaOdyssey, 0)
    self.safety.init_tests()

  def _powertrain_data_msg(self, cruise_on=None, brake_pressed=None):
    # preserve the state
    if cruise_on is None:
      # or'd with controls allowed since the tests use it to "enable" cruise
      cruise_on = self.safety.get_cruise_engaged_prev() or self.safety.get_controls_allowed()
    if brake_pressed is None:
      brake_pressed = self.safety.get_brake_pressed_prev()

    values = {
      "CRUISE_ENGAGED": cruise_on,
      "BRAKE_PRESSED": brake_pressed,
    }
    return self.packer.make_can_msg_panda("POWERTRAIN_DATA", self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    return self._powertrain_data_msg(cruise_on=enable)

  def _speed_msg(self, speed):
    values = {"XMISSION_SPEED": speed}
    return self.packer.make_can_msg_panda("ENGINE_DATA", self.PT_BUS, values)

  def _acc_state_msg(self, main_on):
    values = {"CRUISE_MAIN": main_on}
    return self.packer.make_can_msg_panda("CRUISE_CONTROL", self.PT_BUS, values)

  def _button_msg(self, buttons, main_on=False, bus=None):
    bus = self.PT_BUS if bus is None else bus
    values = {"CRUISE_BUTTONS": buttons}
    return self.packer.make_can_msg_panda("CRUISE_CONTROL", bus, values)

  def _user_brake_msg(self, brake):
    return self._powertrain_data_msg(brake_pressed=brake)

  def _user_gas_msg(self, gas):
    values = {"DRIVER_THROTTLE_POSITION": gas*2} # gas > 1 = gas_pressed
    return self.packer.make_can_msg_panda("DRIVER_THROTTLE_POSITION", self.PT_BUS, values)

  def _send_steer_msg(self, mode, steer):
    values = {
        "COUNTER": 1,
        "STEER_MODE": mode,
        "STEER_ANGLE": 0,
        "STEER_TORQUE": steer,
    }
    canpacker = CANPacker('ocelot_controls')
    msg = canpacker.make_can_msg("STEERING_COMMAND", 0, values)
    addr = msg[0]
    dat  = msg[1]
    values["CHECKSUM"] = self._calc_checksum_8bit(dat, addr)
    return self.packer_steer.make_can_msg_panda("STEERING_COMMAND", self.STEER_BUS, values)


  def _calc_checksum_8bit(self, work_data: bytearray, msg_id: int): # 0xb8 0x1a0 0x19e 0xaa 0xbf
    checksum = msg_id
    for byte in work_data: #checksum is stripped from the data
      checksum += byte     #add up all the bytes

    checksum = (checksum & 0xFF) + (checksum >> 8) #add upper and lower Bytes
    checksum &= 0xFF #throw away anything in upper Byte
    return checksum

  def test_ssc(self):
    self.assertFalse(self._tx(self._send_steer_msg(1, 1)))
    self.assertTrue(self._tx(self._send_steer_msg(0, 0)))
    self.safety.set_controls_allowed(True)
    # SSC increments by 0.125
    self.assertTrue(self._tx(self._send_steer_msg(1, 0.03))) # this sends a 0 for steer_torque
    self.assertTrue(self._tx(self._send_steer_msg(1, 0.063))) # this sends a 0.125 Nm for steer_torque
    self.assertTrue(self._tx(self._send_steer_msg(1, 0.175))) # this sends a 0.125 Nm for steer_torque
    self.assertFalse(self._tx(self._send_steer_msg(1, 0.34))) # this sends a 0.375 Nm for steer_torque, not allowed jump


if __name__ == "__main__":

  unittest.main()
