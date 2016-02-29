from Task.TaskModel import TaskModel
import numpy as np
import scipy as sp
from numpy.random import uniform
from numpy.random import randn
from Utilities.CustomExceptions import MyException
from abc import abstractmethod
from filterpy.monte_carlo import systematic_resample, stratified_resample

class ParticleFilter():
    def __init__(self, physical_state_range, sensor_noise, state_update, h, guess_std = None, N = 5000, init_state_guess = None):
        """
        :param physical_state_range: define the range of each physical state, row represents each physical state range,
               column represents the lower and higher bound.
        :param sensor_noise: dictionary of noise level for each sensor, key is the sensor name, value is the np.array for each sensor
               e.g. {'sensor1': np.array([0.5, 1, 0.5]). 'sensor2' : np.array([1, 1, 1])}
        :param state_update: a function object defining the model of the dynamics
        :param N : total number of particles
        :param init_state_guess: initial state guess, used to generate gaussiant particles if not None
        :param guess_std: standard deviation for gaussian particles creation
        :param h: sampling period
        :return:
        """
        self.physical_state_range = physical_state_range
        self.sensor_noise = sensor_noise
        self.state_update = state_update
        self.N = N
        self.init_state_guess = init_state_guess
        self.guess_std = guess_std
        self.h = h
        if self.init_state_guess is not None:
            self.particles = self._create_gaussian_particles()
        else:
            self.particles = self._create_uniform_particles()

        self.weights = 1. / N * np.ones(N)

    def _create_uniform_particles(self):
        particles = np.empty((self.N, len(self.physical_state_range)))
        for i in np.arange(len(self.physical_state_range)):
            particles[:, i] = uniform(self.physical_state_range[i, 0], self.physical_state_range[i, 1], size=self.N)
        return particles

    def _create_gaussian_particles(self):
        if self.init_state_guess is None:
            raise MyException("Initial Guess is None!")

        if self.guess_std is None:
            raise MyException("Std is None!")

        if len(self.guess_std) != len(self.physical_state_range):
            raise MyException("Length of std is not consistent with lenght of physical state vector!")

        particles = np.empty((self.N, len(self.physical_state_range)))
        for i in np.arange(len(self.physical_state_range)):
            particles[:, i] = self.init_state_guess[i] + (randn(self.N) * self.guess_std[i])

        return particles

    def predict(self, actuator_commands):
        """
        move each particle according to the state_update function and the control actuator commands
        """
        # print(self.particles[0:5])
        self.particles = np.apply_along_axis(self.state_update, 1, self.particles, self.h, actuator_commands)
        # print(self.particles[0:5])

    def update(self, z):
        """
        importance sampling, calculate the likelihood of the sensor measurements, given each particle prediction.
        :param z: dictionary of each sensor readings, key is sensor name, value is a np array for each sensor reading
        """
        if len(z) != len(self.sensor_noise):
            raise MyException("number of sensors and numbers of sensor reading are not consistent!")

        self.weights.fill(1.)
        for name, val in z.items():
            for i , reading in enumerate(val):
                if self.sensor_noise[name][i] == 0:
                    self.sensor_noise[name][i] += 0.5 # avoid divide by 0, and don't use a too small value, which will cause
                                                    # worse perforamnce of particle filter
                self.weights *= sp.stats.norm(self.particles[:, i], self.sensor_noise[name][i]).pdf(reading)

        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= sum(self.weights)

    def estimate(self):
        """returns mean and variance of the weighted particles"""

        mean = np.average(self.particles, weights=self.weights, axis=0)
        var  = np.average((self.particles - mean)**2, weights=self.weights, axis=0)
        return np.reshape(mean, (-1, 1))

    def _neff(self):
        return 1. / np.sum(np.square(self.weights))

    def resample(self):
        """
        use systematic resample by default, can also use stratified resample as well, filterpy.monte_carlo provides this one too
        """
        idx = systematic_resample(self.weights)
        self._resample_from_index(idx)

    def _resample_from_index(self, indexes):
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights /= np.sum(self.weights)


class ABSParticleFilter(ParticleFilter):

    def __init__(self, road_friction_coeff, mass_quater_car, mass_effective_wheel, physical_state_range,
                 sensor_noise, h, guess_std = None, N = 5000, init_state_guess = None):
        super().__init__(physical_state_range, sensor_noise, self._state_update, h, guess_std, N, init_state_guess)
        self.road_friction_coeff = road_friction_coeff
        self.mass_quater_car = mass_quater_car
        self.mass_effective_wheel = mass_effective_wheel

    def resample(self):
        """
        use systematic resample by default, can also use stratified resample as well, filterpy.monte_carlo provides this one too
        """
        idx = systematic_resample(self.weights)
        self._resample_from_index(idx)

    def update(self, z):
        """
        :param z: e.g. z = {'sensor1' : np.array([vehicle speed1, wheel speed1, pos1]),
                            'sensor2' : np.array([vehicle speed2, wheel speed2, pos2])}
        """
        if len(z) != len(self.sensor_noise):
            raise MyException("number of sensors and numbers of sensor reading are not consistent!")

        self.weights.fill(1.)
        for name, val in z.items():
            for i , reading in enumerate(val):
                if self.sensor_noise[name][i] == 0:
                    self.sensor_noise[name][i] += 0.5 # avoid divide by 0, and don't use a too small value, which will cause
                                                    # worse perforamnce of particle filter
                self.weights *= sp.stats.norm(self.particles[:, i], self.sensor_noise[name][i]).pdf(reading)

        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= sum(self.weights)


    def _state_update(self, x, h, actuator_command):

        g = 9.81
        v = x[0]
        w = x[1]
        x = x[2]
        slip = self._slip_ratio(v, w)
        friction_force = self.road_friction_coeff * self._mu(slip) * self.mass_quater_car * g
        v = v - h * friction_force / self.mass_quater_car
        x = x + h * v
        w = w + h * (friction_force / self.mass_effective_wheel - actuator_command)
        w = max(0., w)

        return np.array([v, w, x])


    def _slip_ratio(self, v, w):
        return max(0., 1. - float(w) / float(v))

    def _mu(self, slip):
        return -1.1 * np.exp(-20 * slip) + 1.1 - 0.4 * slip;
