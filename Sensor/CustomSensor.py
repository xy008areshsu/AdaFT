from Sensor.Sensor import Sensor
from Utilities.CustomExceptions import MyException
from numpy.random import randn
import numpy as np

class InvPenSensor(Sensor):
    def sense(self, x):
        if len(x) != 4:
            raise MyException('The Inverted Pendulum should have 4 physical state variables!')

        return np.array([[x[0][0] + randn() * self.noise_scale[0]],
                         [x[1][0] + randn() * self.noise_scale[1]],
                         [x[2][0] + randn() * self.noise_scale[2]],
                         [x[3][0] + randn() * self.noise_scale[3]]])

class ABSSensor(Sensor):
    def sense(self, x):
        """
        :param x: [vehicle speed, wheel speed, vehicle position]
        :return: sensor measurements of vehicle speed and wheel speed
        """
        if len(x) != 3:
            raise MyException('The ABS should have 3 physical state variables!')

        return np.array([[x[0][0] + randn() * self.noise_scale[0]],
                         [x[1][0] + randn() * self.noise_scale[1]],
                         [x[2][0] + randn() * self.noise_scale[2]]])