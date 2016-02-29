
from Utilities.CustomExceptions import MyException
from abc import abstractmethod

class PhysicalSystem:

    def __init__(self, x0):
        """
        :param x0: init physical states, numpy array (n, 1), n dimensions
        :param actuators: dict, the keys should be consistent with the name of each state
        """
        self.x = x0


    @abstractmethod
    def update(self, h, clock, actuator_commands):
        """update the physical state vector"""

    @abstractmethod
    def is_safe(self):
        """check if the safety condition is met"""