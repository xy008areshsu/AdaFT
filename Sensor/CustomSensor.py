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

        # return np.array([[x[0][0] + np.random.uniform(-3*self.noise_scale[0], 3*self.noise_scale[0])],
        #                  [x[1][0] + np.random.uniform(-3*self.noise_scale[1], 3*self.noise_scale[1])],
        #                  [x[2][0] + np.random.uniform(-3*self.noise_scale[2], 3*self.noise_scale[2])],
        #                  [x[3][0] + np.random.uniform(-3*self.noise_scale[3], 3*self.noise_scale[3])]])

class RobotSensor(Sensor):
    def sense(self, x):
        if len(x) != 6:
            raise MyException('The Robot should have 6 physical state variables!')

        return np.array([[x[0][0] + randn() * self.noise_scale[0]],
                         [x[1][0] + randn() * self.noise_scale[1]],
                         [x[2][0] + randn() * self.noise_scale[2]],
                         [x[3][0] + randn() * self.noise_scale[3]],
                         [x[4][0] + randn() * self.noise_scale[4]],
                         [x[5][0] + randn() * self.noise_scale[5]]])

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