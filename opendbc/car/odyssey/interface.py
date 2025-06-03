#!/usr/bin/env python3
from math import exp

import numpy as np
from opendbc.car import get_safety_config, get_friction, structs
from opendbc.car.odyssey.values import CAR
from opendbc.car.odyssey.carcontroller import CarController
from opendbc.car.odyssey.carstate import CarState
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD, LatControlInputs

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  # from car.gm.interface
  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    # ToDo: To generalize to other GMs, explore tanh function as the nonlinear

    #a, b, c = torque_params.sigmoidSharpness, torque_params.sigmoidTorqueGain, torque_params.latAccelFactor

    # params for odyssey estimated with LiveTorqueParameter filtered points
    sigmoidSharpness = 4.5
    sigmoidTorqueGain = 0.9
    latAccelFactor = 0.2
    horizontalOffset = -0.13
    verticalOffset = -0.13

    latAccelWithOffset = latcontrol_inputs.lateral_acceleration + horizontalOffset
    steer_torque = (sig(latAccelWithOffset * sigmoidSharpness) * sigmoidTorqueGain) + (latAccelWithOffset * latAccelFactor)

    friction_mod = friction/(1 + abs(latcontrol_inputs.lateral_acceleration-horizontalOffset)) # decrease friction with higher latAccel

    siglinTorque = float(steer_torque) + friction_mod + verticalOffset

    lowSpeedLatAccelFactor = 1.0
    lowSpeedTorque = (latcontrol_inputs.lateral_acceleration / float(lowSpeedLatAccelFactor)) + friction
    return np.interp(latcontrol_inputs.vego, [0., 15.], [lowSpeedTorque, siglinTorque])

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint == CAR.HONDA_ODYSSEY_2005:
      return self.torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "odyssey"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hondaOdyssey)]

    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning) #, steering_angle_deadzone_deg=2.0) # deadzone is actually 3.5 deg on each side...
    # ret.lateralTuning.torque.kp = 0.6

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    return ret
