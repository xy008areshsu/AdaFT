from Cyber.CyberSystem import CyberSystem
import numpy as np
from operator import attrgetter
import pickle
import os

class RobotCyberSystem(CyberSystem):
    def __init__(self, processors, clock = 0, clf_file = 'subspace_clf_decision_tree.pkl'):
        super().__init__(processors, clock)
        self.clf = pickle.load(open(os.path.join('./Cyber/', clf_file), 'rb'))
        self.copies = 3
        self.version = 1
        self.K1 = np.array([[1054.367, 426.901, 153.864, 365.784, 173.577, 67.28],
                            [707.669, 610.181, 251.283, 263.836, 158.583, 72.18],
                            [12.129, 43.669, 132.469, 11.613, 16.447, 19.12]])


        self.K2 = np.array([[929.79, 322.082, 73.883, 317.782, 142.912, 49.865],
                            [521.09, 431.75, 74.787, 188.568, 105.95, 38.039],
                            [277., 239.326, 202.902, 108.11, 70.849, 42.213]])

        self.A = np.array([[0., 0., 0., 1., 0., 0.],
                          [0., 0., 0., 0., 1., 0.],
                          [0., 0., 0., 0., 0., 1.],
                          [18.337, -75.864, 6.395, 0., 0., 0.],
                          [-22.175, 230.549, -49.01, 0., 0., 0.],
                          [4.353, -175.393, 95.29, 0., 0., 0.]])


        self.B = np.array([[0., 0., 0.],
                          [0., 0., 0.],
                          [0., 0., 0.],
                          [0.292, -0.785, 0.558],
                          [-0.785, 2.457, -2.178],
                          [0.558, -2.178, 2.601]])


    def load_tuning(self, sensor_inputs):
            x = sensor_inputs['sensor1']
            idle_power = 0.5  # This is absurd, To Be Refactored!!!
            x = np.reshape(self.processors[0].rtos.task_outputs['filter'], (-1, 1))
            x = x.reshape(1, -1)
            theta1 = x[0][0]
            rate1 = x[0][3]
            copies = int(self.clf.predict(x) + 1) # clf from sklearn predicts class 1 as 0, class 2 as 1, and so on, so need to add 1 here
            # copies = 3

            power_version1 = self.processors[0].rtos.task_profiles['lqr'].power
            power_version2 = 6.0
            et_version1 = self.processors[0].rtos.task_profiles['lqr'].et
            et_version2 = 0.001
            QoC = 0.15
            self.copies = copies
            self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))

            kf_period = self.processors[0].rtos.task_list['filter'].period

            for i in range(copies):
                # List of processors need to run the tasks
                # for name in self.processors[i].rtos.task_list:
                #     if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                #
                #         if theta1 > QoC or theta1 < -QoC:
                #             self.processors[i].rtos.task_list[name].power = power_version1
                #             self.processors[i].rtos.task_list[name].et = et_version1
                #             self.version = 1
                #         else:
                #             self.processors[i].rtos.task_list[name].power = power_version2
                #             self.processors[i].rtos.task_list[name].et = et_version2
                #             self.version = 2

                if theta1 > QoC or theta1 < -QoC:
                    self.processors[i].rtos.task_list['lqr'].power = power_version1
                    self.processors[i].rtos.task_list['lqr'].et = et_version1
                    self.processors[i].rtos.task_list['lqr'].K = self.K1
                    self.version = 1
                    self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K1))
                else:
                    self.processors[i].rtos.task_list['lqr'].power = power_version2
                    self.processors[i].rtos.task_list['lqr'].et = et_version2
                    self.processors[i].rtos.task_list['lqr'].K = self.K2
                    self.version = 2
                    self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K2))


            for i in range(copies,len(self.processors)):
                # List of processor need to get some rest
                # for name in self.processors[i].rtos.task_list:
                #     if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                #         self.processors[i].rtos.task_list[name].power = idle_power
                #         self.processors[i].rtos.task_list[name].et = 0


                self.processors[i].rtos.task_list['lqr'].power = idle_power
                self.processors[i].rtos.task_list['lqr'].et = 0

                if theta1 > QoC or theta1 < -QoC:
                    self.processors[i].rtos.task_list['lqr'].K = self.K1
                    self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K1))
                else:
                    self.processors[i].rtos.task_list['lqr'].K = self.K2
                    self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K2))

