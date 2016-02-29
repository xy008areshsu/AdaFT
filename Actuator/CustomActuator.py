from Actuator.Actuator import Actuator
from Utilities.CustomExceptions import MyException
from numpy.random import randn
import numpy as np

class InvPenActuator(Actuator):
    def actuator_commands(self, control_input):
        return control_input + randn() * self.noise_scale


class ABSActuator(Actuator):

    def actuator_commands(self, control_input):
        return control_input + randn() * self.noise_scale

class NoABSActuator(Actuator):
    def __init__(self, noise_scale, upper_bound = 150.):
        """
        :param hydraulic_speed: m / s^3
        :return:
        """
        super().__init__(noise_scale)

        self.upper_bound = upper_bound


    def actuator_commands(self, control_input):

        command = self.upper_bound

        return command + randn() * self.noise_scale