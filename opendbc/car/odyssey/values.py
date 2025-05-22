from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, DbcDict, CarSpecs, PlatformConfig, Platforms, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, Device

Ecu = structs.CarParams.Ecu
VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter


class CarControllerParams:
  STEER_STEP = 1 # 100Hz
  STEER_MAX = 2.5  # Nm
  STEER_DELTA_UP = 3 / 100       # 3 Nm/s
  STEER_DELTA_DOWN = 5 / 100     # 3 Nm/s
  STEER_ERROR_MAX = 1     # max delta between torque cmd and torque motor

  def __init__(self, CP):
    pass


# Car button codes
class CruiseButtons:
  CANCEL = 3
  RES_ACCEL = 2
  DECEL_SET = 1


@dataclass
class HondaCarDocs(CarDocs):
  package: str = "StepperServoCAN"

  def init_make(self, CP: structs.CarParams):
    harness = CarHarness.custom
    self.car_parts = CarParts.common([Device.threex, harness])


@dataclass
class HondaOdysseyStepperServoConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'honda_odyssey_2005',
    # Bus.radar: RADAR.DELPHI_MRR,
  })
  def init(self):
    self.car_docs = []


class CAR(Platforms):
  HONDA_ODYSSEY_2005 = HondaOdysseyStepperServoConfig(
    [HondaCarDocs("Honda Odyssey 2005")],
    CarSpecs(mass=1700, wheelbase=3.0, steerRatio=23.5, centerToFrontRatio=0.45, tireStiffnessFactor=1.0), # steerRatio doesn't match actual car, not sure why
    # CarSpecs(mass=1700, wheelbase=3.0, steerRatio=16.2, centerToFrontRatio=0.45, tireStiffnessFactor=0.85),
  )


STEER_THRESHOLD = {
  # default is 1200, overrides go here
  # CAR.ACURA_RDX: 400,
  # CAR.HONDA_CRV_EU: 400,
}


DBC = CAR.create_dbc_map()
