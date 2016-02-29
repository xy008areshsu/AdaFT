from Reliability.Reliability import ReliabilityModel
import math
from sympy import Symbol
from sympy import exp, integrate
import numpy as np

class TAAF(ReliabilityModel):
    def __init__(self,failure_rate, k = 8.6173324e-5, ambient_temperature = 45, abs_temperature = 45, R = 2, C = 0.5, Ea = 0.4):
        """
        :param failure_rate
        :param power : active power in watt
        :param power_idle : watt
        :param k: Boltzmann's constant
        :param ambient_temperature: Celsius
        :param abs_temperature: Celsius
        :param R: thermal resistance
        :param C: thermal capacitance
        :param Ea: activation energy
        :param A: factor
        :return:
        """
        super().__init__(failure_rate)
        kelvin = 273.15
        # self.power_idle = power_idle
        # self.power = power
        self.taaf = 1
        self.k = k
        self.ambient_temperature = ambient_temperature + kelvin
        self.abs_temperature = abs_temperature + kelvin
        self.R = R
        self.C = C
        self.Ea = Ea
        self.A = 1. / math.exp(-self.Ea / (self.k * self.ambient_temperature))
        self.desired_failure_rate = failure_rate
        self.desired_mttf = 1. / self.desired_failure_rate
        # self.Tss = self.steady_state_temperature(self.power)
        # self.Tss_idle = self.steady_state_temperature(self.power_idle)


    def steady_state_temperature(self, power):
        return self.ambient_temperature + self.R * power

    def update_temperature(self, Tss, h):
        self.abs_temperature = Tss + (self.abs_temperature - Tss) * math.exp(-h / (self.R * self.C))

    def update_taaf(self):
        self.taaf = self.A * math.exp(-self.Ea / (self.k * self.abs_temperature))

    def update_failure_rate(self):
        self.failure_rate = self.desired_failure_rate * self.taaf

    def update_reliability(self):
        # standard approach is: t = Symbol('t')
        # self.mttf = integrate(exp(-self.failure_rate * t), (t, 0, np.inf)) # this is the standard approach
        self.update_taaf()
        self.update_failure_rate()
        self.update_mttf()





