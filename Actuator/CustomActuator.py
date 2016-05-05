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
        if (type(control_inputs['lqr']) == int) or (control_inputs['lqr'].shape[0] == 1):
        # if control_inputs['lqr'].shape[0] == 1:
            control_inputs['lqr'] = np.array([[0.], [0.], [0.]])
        for name, val in control_inputs.items():
            if name != 'filter':
                self.actuator_commands[name] = val + randn() * self.noise_scale[name]


class RobotActuator_multi_control(Actuator):
    def update(self, control_inputs):
        """
        actuator dynamics
        :param control_inputs: from cyber system
        :return: real actuator commands
        """

        u1 = control_inputs['u1'] if type(control_inputs['u1']) == int else control_inputs['u1'][0]
        u2 = control_inputs['u2'] if type(control_inputs['u2']) == int else control_inputs['u2'][0]
        u3 = control_inputs['u3'] if type(control_inputs['u3']) == int else control_inputs['u3'][0]
        controls = np.array([[u1], [u2], [u3]])

        self.actuator_commands['lqr'] = controls + randn() * self.noise_scale['lqr']