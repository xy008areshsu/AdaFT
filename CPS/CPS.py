import pandas as pd

class CyberPhysicalSystem:
    def __init__(self, physical_sys, cyber_sys, sensors, actuators, h = 0.002):
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
        self.actuators = actuators
        self.h = h
        self.clock = 0
        self.all_physical_states = physical_sys.states.copy()


    def should_stop(self):
        raise NotImplementedError


    def step_update(self):
        for k in self.sensors:
            self.sensors[k].update(self.h, self.clock)

        self.cyber_sys.update(self.h, self.clock)

        for k in self.actuators:
            self.actuators[k].update(self.h, self.clock)

        self.physical_sys.update(self.h, self.clock)

        self.clock += self.h


    def run(self):

        while not self.should_stop():
            self.step_update()
            new_state = self.physical_sys.states.copy()
            new_state.index = [self.clock]
            self.all_physical_states = self.all_physical_states.append(new_state)
