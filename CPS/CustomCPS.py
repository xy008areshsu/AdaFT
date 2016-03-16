from CPS.CPS import CyberPhysicalSystem
import numpy as np


class InvPenCPS(CyberPhysicalSystem):
    def __init__(self, physical_sys, cyber_sys, sensors, actuator, h = 0.001, end = 4):
        super().__init__(physical_sys, cyber_sys, sensors, actuator, h)
        self.end = end
        self.taaf = np.zeros([(end + 2 * h) / h, 1])
        self.taaf[0] = 1
        self.temp = np.zeros([(end + 2 * h) / h, 1])
        self.xs = self.physical_sys.x

        self.xtract = self.physical_sys.x



    def should_stop(self):
        return (self.clock >= self.end) or (not self.physical_sys.is_safe())


    def run(self):
        j = 0
        while not self.should_stop():
            self.step_update()
            self.clock += self.h

            self.taaf[j] = self.cyber_sys.processors[0].reliability_model.taaf
            self.temp[j] = self.cyber_sys.processors[0].reliability_model.abs_temperature
            j += 1
            self.xs = np.append(self.xs, self.physical_sys.x, axis=1)
            x_predict = np.reshape(self.cyber_sys.processors[0].rtos.task_outputs['filter'], (-1, 1))
            self.xtract = np.append(self.xtract, x_predict, axis=1)


class ABSCPS(CyberPhysicalSystem):
    def __init__(self, physical_sys, cyber_sys, sensors, actuator, h = 0.001, end = 4):
        super().__init__(physical_sys, cyber_sys, sensors, actuator, h)
        self.end = end
        self.taaf = np.zeros([(end + 2 * h) / h, 1])
        self.taaf[0] = 1
        self.temp = np.zeros([(end + 2 * h) / h, 1])
        self.xs = self.physical_sys.x

        self.xtract = self.physical_sys.x



    def should_stop(self):
        return (self.clock >= self.end) or (self.physical_sys.x[0] <= 1)


    def run(self):
        j = 0
        while not self.should_stop():
            self.step_update()
            self.clock += self.h

            self.taaf[j] = self.cyber_sys.processors[0].reliability_model.taaf
            self.temp[j] = self.cyber_sys.processors[0].reliability_model.abs_temperature
            j += 1
            self.xs = np.append(self.xs, self.physical_sys.x, axis=1)
            x_predict = np.reshape(self.cyber_sys.processors[0].rtos.task_outputs['filter'], (-1, 1))
            self.xtract = np.append(self.xtract, x_predict, axis=1)