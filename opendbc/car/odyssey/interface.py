#!/usr/bin/env python3
from math import exp

import numpy as np
from opendbc.car import get_safety_config, structs
from opendbc.car.odyssey.values import CAR
from opendbc.car.odyssey.carcontroller import CarController
from opendbc.car.odyssey.carstate import CarState
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, LateralAccelFromTorqueCallbackType

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def get_lataccel_torque_siglin(self) -> float:

    def torque_from_lateral_accel_siglin_func(lateral_acceleration: float) -> float:
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

      def model(x, a, b, c, d, e):
        xs = x - d
        return sig(a * xs) * b + c * xs + e

      sigmoidSharpness = 5.0
      sigmoidTorqueGain = 1.0
      latAccelFactor = 0.13
      horizontalOffset = -0.10
      verticalOffset = -0.13

      return model(lateral_acceleration, sigmoidSharpness, sigmoidTorqueGain, latAccelFactor, horizontalOffset, verticalOffset)

      # lowSpeedTorque = model(lateral_acceleration, sigmoidSharpness*1.3, sigmoidTorqueGain*1.3, latAccelFactor*1.3, horizontalOffset, verticalOffset)

      # friction_mod = friction/(1 + abs(latcontrol_inputs.lateral_acceleration-horizontalOffset)) # decrease friction with higher latAccel

      # return np.interp(latcontrol_inputs.vego, [5., 17.], [lowSpeedTorque, torque]) #+ friction_mod

    lataccel_values = np.arange(-5.0, 5.0, 0.01)
    torque_values = [torque_from_lateral_accel_siglin_func(x) for x in lataccel_values]
    assert min(torque_values) < -1 and max(torque_values) > 1, "The torque values should cover the range [-1, 1]"
    return torque_values, lataccel_values

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint == CAR.HONDA_ODYSSEY_2005:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(lateral_acceleration, lataccel_values, torque_values)
      return torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

  def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
    if self.CP.carFingerprint == CAR.HONDA_ODYSSEY_2005:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def lateral_accel_from_torque_siglin(torque: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(torque, torque_values, lataccel_values)
      return lateral_accel_from_torque_siglin
    else:
      return self.lateral_accel_from_torque_linear

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
