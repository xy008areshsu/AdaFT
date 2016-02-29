
from abc import ABCMeta, abstractmethod

class TaskModel:
    def __init__(self, name, period, deadline, wcet, power):
        self.name = name
        self.period = period
        self.wcet = wcet
        self.power = power
        self.et = self.density()
        self.status = 'ready'
        self.deadline = deadline
        self.finished_time = 0
        self.intermediate_time_ = 0
        self.abs_deadline = 0
        self.start_time = 0
        self.output = 0

    @abstractmethod
    def density(self):
        """pdf of the execution time of the task"""


    @abstractmethod
    def run(self, inputs):
        """execute the task
        :param: a dictionary of all inputs
        """

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name