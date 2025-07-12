from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
# from opendbc.car.honda.values import HondaFlags, HONDA_BOSCH, HONDA_BOSCH_RADARLESS, HONDA_BOSCH_CANFD, CAR, CarControllerParams
from enum import Enum
from opendbc.can.packer import CANPacker

class CanBus:
  PT_CAN =    0
  SERVO_CAN = 1 # required for steering

class SteeringModes(Enum):
  Off = 0
  TorqueControl = 1
  AngleControl = 2
  SoftOff = 3

# *** StepperServoCAN ***
def create_steer_command(frame: int, mode: SteeringModes, steer_tq: float = 0, steer_delta: float = 0):
    """Creates a CAN message for the actuator STEERING_COMMAND"""
    packer = CANPacker('ocelot_controls')
    values = {
        "COUNTER": frame % 16,
        "STEER_MODE": mode.value,
        "STEER_ANGLE": steer_delta,
        "STEER_TORQUE": steer_tq,
    }
    msg = packer.make_can_msg("STEERING_COMMAND", 0, values)
    addr = msg[0]
    dat  = msg[1]
    values["CHECKSUM"] = calc_checksum_8bit(dat, addr)

    return packer.make_can_msg("STEERING_COMMAND", CanBus.SERVO_CAN, values)

def calc_checksum_8bit(work_data: bytearray, msg_id: int): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8) #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

def calc_checksum_cruise(work_data: bytearray):# 0x194 this checksum is special - initialized with 0
  return calc_checksum_8bit(work_data, 0)