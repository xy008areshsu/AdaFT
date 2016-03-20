from Actuator.Actuator import Actuator
from Utilities.CustomExceptions import MyException
from numpy.random import randn
import numpy as np

# class InvPenActuator(Actuator):
#     def actuator_commands(self, control_input):
#         commands = {'lqr' : control_input + randn() * self.noise_scale}
#         return commands
#
#
# class ABSActuator(Actuator):
#
#     def actuator_commands(self, control_input):
#         commands = {'abs' : control_input + randn() * self.noise_scale}
#         return commands
#
# class NoABSActuator(Actuator):
#     def __init__(self, noise_scale, upper_bound = 150.):
#         """
#         :param hydraulic_speed: m / s^3
#         :return:
#         """
#         super().__init__(noise_scale)
#
#         self.upper_bound = upper_bound
#
#
#     def actuator_commands(self, control_input):
#
#         command = self.upper_bound
#
#         return command + randn() * self.noise_scale


class RobotActuator(Actuator):
    def update(self, control_inputs):
        """
        actuator dynamics
        :param control_inputs: from cyber system
        :return: real actuator commands
        """
        if control_inputs['lqr'].shape[0] == 1:
            control_inputs['lqr'] = np.array([[0.], [0.], [0.]])
        for name, val in control_inputs.items():
            self.actuator_commands[name] = val + randn() * self.noise_scale[name]
