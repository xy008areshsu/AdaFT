from abc import ABCMeta, abstractmethod

class ReliabilityModel:
    def __init__(self, failure_rate):
        self.failure_rate = failure_rate
        self.mttf = 1. / self.failure_rate
        self.seconds_in_year = 3.154e+7
        self.mttf_in_years = self.mttf / self.seconds_in_year

    @abstractmethod
    def update_failure_rate(self):
        """ update failure rate"""

    def update_mttf(self):
        self.mttf = 1. / self.failure_rate
        self.mttf_in_years = self.mttf / self.seconds_in_year

    def update_reliability(self):
        self.update_failure_rate()
        self.update_mttf()
