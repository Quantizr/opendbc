from collections import deque
import numpy as np

from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, create_button_events, structs, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.odyssey.values import DBC , CruiseButtons
from opendbc.car.interfaces import CarStateBase
from opendbc.car.common.filter_simple import FirstOrderFilter

TransmissionType = structs.CarParams.TransmissionType
ButtonType = structs.CarState.ButtonEvent.Type

BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise, CruiseButtons.CANCEL: ButtonType.cancel}


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.shifter_values = can_define.dv["GEARBOX"]["GEAR_SHIFTER"]
    self.brake_switch_prev = False
    self.brake_switch_active = False
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0

    self.dash_speed_seen = False

    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 5.0, DT_CTRL, initialized=False)

    # self.angle_offset = 0
    # self.angle_offset_finalized = False
    # self.angle_offset_deque = deque(maxlen=500) # technically 5 seconds of driving perfectly straight, realistically will not be driving perfectly straight

    # self.wheel_speed_ratio = FirstOrderFilter(None, 0.5, DT_CTRL, initialized=False)
    # self.steering_angle = FirstOrderFilter(None, 0.5, DT_CTRL, initialized=False)
    self.offset_counter = 0

    self.main_button = 0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_actuator = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    prev_cruise_buttons = self.cruise_buttons
    prev_cruise_main = self.main_button
    self.cruise_buttons = cp.vl["CRUISE_CONTROL"]["CRUISE_BUTTONS"]
    self.main_button = cp.vl["CRUISE_CONTROL"]["CRUISE_MAIN"]

    # used for car hud message
    self.is_metric = True

    # ******************* parse out can *******************
    # STANDSTILL->WHEELS_MOVING bit can be noisy around zero, so use XMISSION_SPEED
    # panda checks if the signal is non-zero
    ret.standstill = cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] < 1e-5
    #   ret.doorOpen = any([cp.vl["DOORS_STATUS"]["DOOR_OPEN_FL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_FR"],
    #                       cp.vl["DOORS_STATUS"]["DOOR_OPEN_RL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_RR"]])
    ret.doorOpen = any([cp.vl["BODY"]["DRIVER_DOOR_OPEN"], cp.vl["BODY"]["PASSENGER_DOOR_OPEN"]])
    ret.seatbeltUnlatched = bool(cp.vl["BODY"]["SEATBELT_WARNING"])

    # ret.espDisabled = cp.vl["VSA_STATUS"]["ESP_DISABLED"] != 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    v_wheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.0

    # blend in transmission speed at low speed, since it has more low speed accuracy
    v_weight = float(np.interp(v_wheel, v_weight_bp, v_weight_v))
    ret.vEgoRaw = (1. - v_weight) * cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] * CV.KPH_TO_MS * self.CP.wheelSpeedFactor + v_weight * v_wheel
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    self.dash_speed_seen = self.dash_speed_seen or cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] > 1e-3
    if self.dash_speed_seen:
      conversion = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
      ret.vEgoCluster = cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] * conversion

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
       250, cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"], cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"])

    gear = int(cp.vl["GEARBOX"]["GEAR_SHIFTER"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear, None))


    ret.gas = cp.vl["DRIVER_THROTTLE_POSITION"]["DRIVER_THROTTLE_POSITION"]
    ret.gasPressed = ret.gas > 1 # for some reason sometimes `gas` = 1 even when not pressed...


    # ret.steeringPressed = False
    # ret.steeringTorque = 0

    # no torque sensor, so lightly pressing the gas indicates driver intention
    # ret.steeringPressed = ret.gasPressed
    # if ret.steeringPressed and ret.leftBlinker:
    #   ret.steeringTorque = 1
    # elif ret.steeringPressed and  ret.rightBlinker:
    #   ret.steeringTorque = -1
    # else:
    #   ret.steeringTorque = 0

    # no torque sensor, so lightly pressing the gas indicates driver intention (only when indicating)
    ret.steeringPressed = False
    ret.steeringTorque = 0
    if ret.leftBlinker or ret.rightBlinker:
        ret.steeringPressed = ret.gasPressed
        if ret.steeringPressed:
            ret.steeringTorque = 1 if ret.leftBlinker else -1
    # else:
    #     ret.steeringPressed = False
    #     ret.steeringTorque = 0

    ret.cruiseState.speed = cp.vl["CRUISE_CONTROL"]["CRUISE_SPEED"] * CV.KPH_TO_MS


    ret.brake = cp.vl["BRAKE_PRESSURE"]["BRAKE_PRESSURE"]
    ret.brakePressed = (cp.vl["POWERTRAIN_DATA"]["BRAKE_PRESSED"] != 0)
    ret.cruiseState.enabled = cp.vl["POWERTRAIN_DATA"]["CRUISE_ENGAGED"] != 0
    ret.cruiseState.available = bool(cp.vl["CRUISE_CONTROL"]["CRUISE_MAIN"])

    ret.steeringTorqueEps =  cp_actuator.vl['STEERING_STATUS']['STEERING_TORQUE']
    ret.steerFaultTemporary = int(cp_actuator.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4 != 0

    # if v_wheel > 3: # m/s ~= 6.7mph
    #   self.wheel_speed_ratio.update(
    #     (cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"]) /
    #     (cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"])
    #   )
    #   self.steering_angle.update(ssc_angle)
    #   if self.offset_counter < 50:
    #     self.offset_counter += 1
    # else:
    #   self.wheel_speed_ratio.initialized = False
    #   self.steering_angle.initialized = False
    #   self.offset_counter = 0

    # if self.offset_counter >= 50:
    #   self.accurate_steer_angle_seen = True

    # if self.accurate_steer_angle_seen:
    #   if self.wheel_speed_ratio.x > 0.9995 and self.wheel_speed_ratio.x < 1.0005 and self.offset_counter >= 50 and cp.can_valid:
    #     self.angle_offset.update(self.steering_angle.x)
    #     self.offset_counter = 0

    #   if self.angle_offset.initialized:
    #     ret.steeringAngleOffsetDeg = self.angle_offset.x
    #     ret.steeringAngleDeg = ssc_angle - self.angle_offset.x

    ssc_angle = cp_actuator.vl['STEERING_STATUS']['STEERING_ANGLE']

    if v_wheel > 3: # m/s ~= 6.7mph
      wheel_speed_ratio_live = ((cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"]) /
        (cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"]))
      if wheel_speed_ratio_live > 0.9995 and wheel_speed_ratio_live < 1.0005:
        self.angle_offset.update(ssc_angle)
        if self.offset_counter < 10:
          self.offset_counter += 1
        else:
          self.accurate_steer_angle_seen = True

    if self.angle_offset.initialized:
      ret.steeringAngleOffsetDeg = self.angle_offset.x
      ret.steeringAngleDeg = ssc_angle - self.angle_offset.x

    # if v_wheel > 3: # m/s ~= 6.7mph
    #   wheel_speed_ratio_live = ((cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"]) /
    #     (cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"]))
    #   if wheel_speed_ratio_live > 0.9995 and wheel_speed_ratio_live < 1.0005:
    #     self.angle_offset_deque.append(ssc_angle)
    #     if len(self.angle_offset_deque) > 20:
    #       self.accurate_steer_angle_seen = True
    #       current_offset = sum(self.angle_offset_deque) / len(self.angle_offset_deque)
    #       if len(self.angle_offset_deque) < self.angle_offset_deque.maxlen: # update angle_offset until maxlen samples collected
    #         self.angle_offset = current_offset
    #       elif abs(current_offset - self.angle_offset) > 5: # reset angle_offset on >5 deg belt slip, will result in alert on screen
    #         self.angle_offset = current_offset # make the angle_offset the current best estimate before clearing
    #         self.accurate_steer_angle_seen = False
    #         self.angle_offset_deque.clear() # we clear because otherwise all maxlen samples need to be cycled through to get accurate reading
    #     else:
    #       self.accurate_steer_angle_seen = False

    # ret.steeringAngleOffsetDeg = self.angle_offset
    # ret.steeringAngleDeg = ssc_angle - self.angle_offset

    ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    # if self.CP.enableBsm:
    #   # BSM messages are on B-CAN, requires a panda forwarding B-CAN messages to CAN 0
    #   # more info here: https://github.com/commaai/openpilot/pull/1867
    #   ret.leftBlindspot = cp_body.vl["BSM_STATUS_LEFT"]["BSM_ALERT"] == 1
    #   ret.rightBlindspot = cp_body.vl["BSM_STATUS_RIGHT"]["BSM_ALERT"] == 1

    ret.buttonEvents = [
      *create_button_events(self.cruise_buttons, prev_cruise_buttons, BUTTONS_DICT),
      *create_button_events(self.main_button, prev_cruise_main, {1: ButtonType.mainCruise}),
    ]


    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
    ("BODY", 3),
    ("BRAKE_PRESSURE", 143),
    ("CRUISE_CONTROL", 100),
    ("DRIVER_THROTTLE_POSITION", 100),
    ("ENGINE_DATA", 100),
    ("GEARBOX", 100),
    ("LIGHTS", 25),
    ("POWERTRAIN_DATA", 100),
    ("WHEEL_SPEEDS", 48),

  ]

    actuator_messages = [
      ("STEERING_STATUS", 100),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser('ocelot_controls', actuator_messages, 1),
    }
