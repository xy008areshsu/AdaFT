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


    def load_tuning(self, sensor_inputs):
        x = sensor_inputs['sensor1']
        idle_power = 0.5  # This is absurd, To Be Refactored!!!
        x = np.reshape(self.processors[0].rtos.task_outputs['filter'], (-1, 1))
        x = x.reshape(1, -1)
        theta1 = x[0][0]
        rate1 = x[0][3]
        copies = int(self.clf.predict(x) + 1) # clf from sklearn predicts class 1 as 0, class 2 as 1, and so on, so need to add 1 here

        power_version1 = self.processors[0].rtos.task_profiles['lqr'].power
        power_version2 = 1.0
        et_version1 = self.processors[0].rtos.task_profiles['lqr'].et
        et_version2 = 0.01
        QoC = 0.45
        self.copies = copies
        self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))
        for i in range(copies):
            # List of processors need to run the tasks
            for name in self.processors[i].rtos.task_list:
                if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations

                    if theta1 > QoC or theta1 < -QoC:
                        self.processors[i].rtos.task_list[name].power = power_version1
                        self.processors[i].rtos.task_list[name].et = et_version1
                        self.version = 1
                    else:
                        self.processors[i].rtos.task_list[name].power = power_version2
                        self.processors[i].rtos.task_list[name].et = et_version2
                        self.version = 2



        for i in range(copies,len(self.processors)):
            # List of processor need to get some rest
            for name in self.processors[i].rtos.task_list:
                if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                    self.processors[i].rtos.task_list[name].power = idle_power
                    self.processors[i].rtos.task_list[name].et = 0

