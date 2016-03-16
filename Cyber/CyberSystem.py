import numpy as np
from operator import attrgetter

class CyberSystem:
    """
    Cyber System implements, for the simulation purpose, all of the components that we ignore the overhead, such as
    the voting component of different copies of a particular task.
    """

    def __init__(self, processors, clock = 0):

        self.clock = clock
        self.processors = processors
        self.control_inputs_candidates = {}
        self.update_control_inputs()
        self.control_inputs = {}
        for name, val in self.control_inputs_candidates.items():
            self.control_inputs[name] = 0


    def update(self, h, clock, sensor_inputs):
        self.clock_sync(clock)
        self.load_tuning()
        for processor in self.processors:
            for name in processor.rtos.task_list:
                if name == 'filter':
                    filter_inputs = {'sensor' : sensor_inputs}
                    for task_name, task_val in processor.rtos.task_outputs.items():
                        if task_name != 'filter':
                            filter_inputs[task_name] = task_val
                    processor.rtos.task_inputs[name] = filter_inputs
                else:
                    processor.rtos.task_inputs[name] = np.reshape(processor.rtos.task_outputs['filter'], (-1, 1))
            processor.update(h, self.clock)


        self.update_control_inputs()
        self.voting()


    def load_tuning(self):
        idle_power = 0.5  # This is absurd, To Be Refactored!!!
        x = np.reshape(self.processors[0].rtos.task_outputs['filter'], (-1, 1))
        copy = 3
        self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))
        for i in range(copy):
            # List of processors need to run the tasks
            for name in self.processors[i].rtos.task_list:
                if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                    self.processors[i].rtos.task_list[name].power = self.processors[i].rtos.task_profiles[name].power
                    # self.processors[i].rtos.task_list[name].et = self.processors[i].rtos.task_profiles[name].et

        for i in range(copy,len(self.processors)):
            # List of processor need to get some rest
            for name in self.processors[i].rtos.task_list:
                if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                    self.processors[i].rtos.task_list[name].power = idle_power
                    # self.processors[i].rtos.task_list[name].et = 0.001
                    # Don't change task et for now, since the voting algorithm would give wrong output,
                    # this is fine since the normal condition is to run 3 copies of the task, and now using AdaFT power will be idle,
                    # but the et will be unchanged, thus the schedulibility would not be affected.



    def clock_sync(self, clock):
        self.clock = clock

    def update_control_inputs(self):
        self.control_inputs_candidates = {}  # reset first
        for processor in self.processors:
            for control_name, control_val in processor.rtos.task_outputs.items():
                if control_name != 'filter':
                    if control_name in self.control_inputs_candidates:
                        self.control_inputs_candidates[control_name].append(control_val)
                    else:
                        self.control_inputs_candidates[control_name] = [control_val]

    @staticmethod
    def triple_vote(val_list, epsilon):
        if np.all((val_list[0] - val_list[1]) <= epsilon):
            return val_list[0]
        elif np.all(val_list[0] - val_list[2] <= epsilon):
            return val_list[0]
        return val_list[1]
        # out = {}
        # majority = None
        # out[majority] = 0
        # for k, v in val_list.items():
        #     if v not in out:
        #         out[v] = 1
        #     else:
        #         out[v] += 1
        #     if out[v] >= len(val_list) / 2.0:
        #         return v
        #     if out[v] > out[majority]:
        #         majority = v
        #
        # return majority

    @staticmethod
    def double_vote(val_list, epsilon):
        if np.all((val_list[0] - val_list[1]) <= epsilon):
            return val_list[0]
        return np.zeros(len(val_list))
        # vals = list(val_list)
        # if vals[0] == vals[1]:
        #     return vals[0]
        # return 0


    def voting(self):
        # for command in self.actuator_commands:
        #     running = 0
        #     finished = 0
        #     for processor in self.processors:
        #         if processor[command]['status'] == 'finished':
        #             self.actuator_commands[command]['val_list'][processor] = processor[command]['val']
        #             finished += 1
        #         elif processor[command]['status'] == 'running':
        #             running += 1
        #
        #     if running == 0 and finished != 0: # this means new iteration of this task is done for all copies
        #         #first we turn all finished status to idle
        #         for processor in self.processors:
        #             if processor[command]['status'] == 'finished':
        #                 processor[command]['status'] = 'idle'
        #
        #         # then we turn self.actuator_commands[command]['status'] to ready
        #         self.actuator_commands[command]['status'] = 'ready'
        #
        #     if self.actuator_commands[command]['status'] == 'ready':
        #         self.actuator_commands[command]['status'] = 'waiting'
        #         if len(self.actuator_commands[command]['val_list']) == 3:
        #             self.actuator_commands[command]['output'] = self.majority_vote(self.actuator_commands[command]['val_list'])
        #         elif len(self.actuator_commands[command]['val_list']) == 2:
        #             self.actuator_commands[command]['output'] = self.double_vote(self.actuator_commands[command]['val_list'])
        #         else:
        #             for p in self.actuator_commands[command]['val_list']:
        #                 self.actuator_commands[command]['output'] = self.actuator_commands[command]['val_list'][p]

        for control_name, control_val_list in self.control_inputs_candidates.items():
            if len(control_val_list) == 3:
                self.control_inputs[control_name] = self.triple_vote(control_val_list, 1e-6)
            elif len(control_val_list) == 2:
                self.control_inputs[control_name] = self.double_vote(control_val_list, 1e-6)
            else:
                self.control_inputs[control_name] = self.control_inputs_candidates[control_name][0]






