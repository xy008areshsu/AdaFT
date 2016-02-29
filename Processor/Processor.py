class Processor:
    def __init__(self, rtos, reliability_model, voltage = 3.3, frequency = 400, clock = 0):
        """
        :param voltage: V
        :param frequency: MHz
        :param temperature: C
        """
        import math
        self.clock = clock
        self.voltage = voltage
        self.frequency = frequency
        # self.temperature = temperature
        self.rtos = rtos
        # self.failure_rate = failure_rate
        self.reliability_model = reliability_model
        # self.mttf = 1. / self.failure_rate
        # self.old_temperature = temperature
        # self.control_inputs = rtos.task_outputs


    def update(self, h, clock):
        self.clock_sync(clock)
        self.rtos.update(h, clock)
        # self.control_inputs = self.rtos.task_outputs
        if self.rtos.running_task is None:
            power = 1 # idle power 1 watt
        else:
            power = self.rtos.running_task.power

        Tss = self.reliability_model.steady_state_temperature(power)
        self.reliability_model.update_temperature(Tss, h)
        self.reliability_model.update_reliability()

    def clock_sync(self, clock):
        self.clock = clock



