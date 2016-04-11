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
            # print(self.clock)
            self.taaf[j] = self.cyber_sys.processors[0].reliability_model.taaf
            self.temp[j] = self.cyber_sys.processors[0].reliability_model.abs_temperature
            j += 1
            self.xs = np.append(self.xs, self.physical_sys.x, axis=1)
            x_predict = np.reshape(self.cyber_sys.processors[0].rtos.task_outputs['filter'], (-1, 1))
            self.xtract = np.append(self.xtract, x_predict, axis=1)

class RobotCPS(CyberPhysicalSystem):
    def __init__(self, physical_sys, cyber_sys, sensors, actuator, h = 0.001, end = 4):
        super().__init__(physical_sys, cyber_sys, sensors, actuator, h)
        self.end = end
        self.taaf = np.zeros([(end + 2 * h) / h, 1])
        self.taaf[0] = 1
        self.temp = np.zeros([(end + 2 * h) / h, 1])
        self.xs = np.zeros([6, (end + 2 * h) / h])
        self.xs[:, 0] = self.physical_sys.x[:, 0]
        self.xtract = np.zeros([6, (end + 2 * h) / h])
        self.xtract[:, 0] = self.physical_sys.x[:, 0]
        self.copies = np.zeros([1, (end + 2 * h) / h])
        self.version = np.zeros([1, (end + 2 * h) / h])



    def should_stop(self):
        return (self.clock >= self.end) or (not self.physical_sys.is_safe())


    def run(self):
        j = 0
        while not self.should_stop():
            self.step_update()
            self.clock += self.h
            print(self.clock)
            self.taaf[j] = self.cyber_sys.processors[0].reliability_model.taaf
            self.temp[j] = self.cyber_sys.processors[0].reliability_model.abs_temperature
            self.xs[:, j] = self.physical_sys.x[:, 0]
            x_predict = np.reshape(self.cyber_sys.processors[0].rtos.task_outputs['filter'], (-1, 1))
            self.xtract[:, j] = x_predict[:,0]
            self.copies[:, j] = self.cyber_sys.copies
            self.version[:, j] = self.cyber_sys.version
            j += 1


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