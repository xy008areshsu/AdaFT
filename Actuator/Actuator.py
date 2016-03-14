from abc import abstractmethod
from numpy.random import randn

class Actuator:
    def __init__(self, noise_scale):
        self.noise_scale = noise_scale
        self.actuator_commands = {}

    def update(self, control_inputs):
        """
        actuator dynamics
        :param control_inputs: from cyber system
        :return: real actuator commands
        """
        for name, val in control_inputs.items():
            self.actuator_commands[name] = val + randn() * self.noise_scale[name]

