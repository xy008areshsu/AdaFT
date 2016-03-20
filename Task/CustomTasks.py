from Task.TaskModel import TaskModel
from filterpy.kalman import KalmanFilter
import numpy as np
from Task.ParticleFilter import ParticleFilter, ABSParticleFilter
import copy

class ABS(TaskModel):
    def __init__(self, name, period, deadline, wcet, power, hydraulic_speed = 3300., upper_bound = 150., lower_bound = 100.):
        super().__init__(name, period, deadline, wcet, power)
        self.hydraulic_speed = hydraulic_speed
        self.upper_bound = upper_bound
        self.lower_bound = lower_bound
        self.output = max(self.lower_bound, min(self.upper_bound, self.output))

    def density(self):
        return self.wcet

    def run(self, inputs):
        optimal_slip = 0.2
        v = inputs[0][0]
        w = inputs[1][0]
        slip = self._slip_ratio(v, w)

        if slip > optimal_slip:
            brake = -1
        else:
            brake = 1

        self.output = self.output + self.period * brake * self.hydraulic_speed
        self.output = max(self.lower_bound, min(self.upper_bound, self.output))


    def _slip_ratio(self, v, w):
        return max(0., 1. - float(w) / float(v))

class NoABS(TaskModel):
    def __init__(self, name, period, deadline, wcet, power):
        super().__init__(name, period, deadline, wcet, power)
        self.output = 0

    def density(self):
        return self.wcet

    def run(self, inputs):
        self.output = 0


class ACC(TaskModel):
    def __init__(self, name, period, deadline, wcet, power):
        super().__init__(name, period, deadline, wcet, power)
        self.v = 1
        self.w = 1

    def density(self):
        return self.wcet

    def run(self, inputs):
        self.output = 2 * self.v


class LQRInvPen(TaskModel):
    def __init__(self, name, period, deadline, wcet, power, K):
        super().__init__(name, period, deadline, wcet, power)
        self.K = K
        self.output = np.array([[0]])

    def density(self):
        return self.wcet

    def run(self, inputs):
        self.output = -np.dot(self.K, inputs)

class LQRRobot(TaskModel):
    def __init__(self, name, period, deadline, wcet, power, K):
        super().__init__(name, period, deadline, wcet, power)
        self.K = K
        self.output = np.array([[0]])

    def density(self):
        return self.wcet

    def run(self, inputs):
        self.output = -np.dot(self.K, inputs)


class KalmanPredict(TaskModel):
    """
    Localization task
    """
    def __init__(self, name, period, deadline, wcet, power, kf, Rs):
        """
        Kalman Filter to predict linear systems
        :param kf: kalman filter object
        :param Rs: a dictionary of sensor noise matrix, R
        """
        super().__init__(name, period, deadline, wcet, power)
        self.kf = kf
        self.Rs = Rs
        self.output = self.kf.x

    def density(self):
        return self.wcet

    def run(self, inputs):
        """
        :param inputs: {'lqr':....., 'sensor': {dict of all sensor readings}}
        """
        threshold = 75
        inputs = inputs['sensor']
        self.kf.predict()
        predicted = copy.deepcopy(self.kf.x)
        for sensor_name, z in inputs.items():
            errors = abs(predicted - z)
            if errors[2] < threshold:
                self.kf.update(z, self.Rs[sensor_name])
            # self.kf.update(z, self.Rs[sensor_name])
        self.output = self.kf.x


class KalmanLQR(TaskModel):
    """
    Localization plut control task
    """
    def __init__(self, name, period, deadline, wcet, power, kf, Rs, K):
        """
        Kalman Filter to predict linear systems
        :param kf: kalman filter object
        :param Rs: a dictionary of sensor noise matrix, R
        """
        super().__init__(name, period, deadline, wcet, power)
        self.kf = kf
        self.Rs = Rs
        self.K = K
        self.output = -np.dot(self.K, self.kf.x)

    def density(self):
        return self.wcet

    def run(self, inputs):
        """
        :param inputs: dictionary of all sensor measurements
        """
        self.kf.predict()
        for sensor_name, z in inputs.items():
            self.kf.update(z, self.Rs[sensor_name])

        self.output = -np.dot(self.K, self.kf.x)

class ABSPF(TaskModel):
    """Localization task"""
    def __init__(self, name, period, deadline, wcet, power, pf):
        super().__init__(name, period, deadline, wcet, power)
        self.pf = pf
        self.output = self.pf.estimate()

    def density(self):
        return self.wcet

    def run(self, inputs):
        """
        :param inputs: dictionary of all sensor measurements, and actuator commands
               e.g. {'abs' : 100,
                     'sensor' : {'sensor1' : np.array[30., 30., 10.]
                                 'sensor2' : np.array[32., 31., 10.]
                                 }
                     }
        """
        self.pf.predict(inputs['abs'])

        self.pf.update(inputs['sensor'])
        if self.pf._neff() < self.pf.N / 2:
            self.pf.resample()

        self.output = self.pf.estimate()




def makeLinearKF(A, B, C, P, F, x, R = None, dim_x = 4, dim_z = 4):

    kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)
    kf.x = x
    kf.P = P
    kf.F = F
    kf.H = C
    kf.R = R

    return kf