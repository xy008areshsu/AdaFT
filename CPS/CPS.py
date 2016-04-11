import pandas as pd
from abc import ABCMeta, abstractmethod
import numpy as np

class CyberPhysicalSystem:
    def __init__(self, physical_sys, cyber_sys, sensors, actuator, h = 0.001):
        """

        :param physical_sys:
        :param cyber_sys:
        :param sensors: dict, one-to-one mapping
        :param actuators: dict, one-to-one mapping
        :param h, simulation step, in seconds
        :return:
        """
        self.physical_sys = physical_sys
        self.cyber_sys = cyber_sys
        self.sensors = sensors
        self.actuator = actuator
        self.h = h
        self.clock = 0



    @abstractmethod
    def should_stop(self):
        """stop condition"""


    def step_update(self):

        sensor_inputs = {}
        i = 0
        for sensor in self.sensors:
            z = sensor.sense(self.physical_sys.x)
            name = 'sensor' + str(i)
            sensor_inputs[name] = z[:, -1]
            i += 1

        self.cyber_sys.update(self.h, self.clock, sensor_inputs)

        # for control_name, control_val in self.cyber_sys.control_inputs.items():
        #     self.actuator[control_name] = self.actuator.actuator_commands(control_val)
        self.actuator.update(self.cyber_sys.control_inputs)

        self.physical_sys.update(self.h, self.clock, self.actuator.actuator_commands)



    def run(self):

        while not self.should_stop():
            self.step_update()
            self.clock += self.h

