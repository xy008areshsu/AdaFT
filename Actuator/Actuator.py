from abc import abstractmethod

class Actuator:
    def __init__(self, noise_scale):
        self.noise_scale = noise_scale

    @abstractmethod
    def actuator_commands(self, control_inputs):
        """
        actuator dynamics
        :param control_inputs: from cyber system
        :return: real actuator commands
        """

