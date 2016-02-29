from abc import abstractmethod

class Sensor:

    def __init__(self,noise_scale ):
        self.noise_scale = noise_scale

    @abstractmethod
    def sense(self, x):
        """
        measure the physical state vectors
        :param x: physical state vector
        :return: measured physical state vector values
        """


