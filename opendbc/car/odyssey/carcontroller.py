from collections import namedtuple

from opendbc.car import structs, apply_dist_to_meas_limits
from opendbc.car.odyssey.odysseycan import SteeringModes, create_steer_command
from opendbc.car.odyssey.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.honda.mads import MadsCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.stopping_counter = 0

    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.last_torque = 0.0

    self.apply_steer_last = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, self.CP, CC, CC_SP)
    actuators = CC.actuators

    # Send CAN commands
    can_sends = []

    # *** apply steering torque ***
    if CC.latActive:
      new_steer = actuators.torque * CarControllerParams.STEER_MAX
      # explicitly clip torque before sending on CAN
      apply_steer = apply_dist_to_meas_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps,
                                          CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                          CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
      can_sends.append(create_steer_command(self.frame, SteeringModes.TorqueControl, apply_steer))
    elif not CS.out.brakePressed and not CS.out.gasPressed and self.apply_steer_last != 0:
      can_sends.append(create_steer_command(self.frame, SteeringModes.SoftOff, self.apply_steer_last))
      apply_steer = CS.out.steeringTorqueEps
    else:
      apply_steer = 0
      can_sends.append(create_steer_command(self.frame, SteeringModes.Off))
    self.apply_steer_last = apply_steer

    # debug
    if CC.enabled and (self.frame % 10) == 0: #slow print
      frame_number = self.frame
      print(f"Steering req: {actuators.torque}, Speed: {CS.out.vEgoCluster}, Frame number: {frame_number}")

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_steer_last

    self.frame += 1

    return new_actuators, can_sends
