import numpy as np
from operator import attrgetter
from math import gcd
from functools import reduce
import copy
from abc import ABCMeta, abstractmethod


class CyberSystem:
    """
    Cyber System implements, for the simulation purpose, all of the components that we ignore the overhead, such as
    the voting component of different copies of a particular task.
    """

    def __init__(self, processors, clf = None, task_list = {}, clock = 0):

        self.clock = clock
        self.processors = processors
        self.control_inputs_candidates = {}
        self.update_control_inputs()
        self.control_inputs = {}
        self.task_list = {}   # key: name;  val: (task version 1, task version 2)
        self.task_profiles = {}
        self.task_iterations = {}
        self.n = 0
        self.ready_queue = []
        self.clf = clf  # dict, key: name, val: classifier
        # self.task_outputs = {}
        for name, t in task_list.items():
            self.create_task(name, t)
        for name, val in self.control_inputs_candidates.items():
            self.control_inputs[name] = 0
        self.control_inputs_candidates_iterations = {}
        # all_task_periods = [int(1000 * self.processors[0].rtos.task_profiles[name].period) for name in self.processors[0].rtos.task_profiles if name != 'filter']
        all_task_periods = [int(1000 * self.task_profiles[name][0].period) for name in self.task_profiles if name != 'filter']
        all_task_periods.append(int(1000 * self.task_profiles['filter'].period))
        self.load_tuning_period = reduce(gcd, all_task_periods) / 1000
        self.copies = 3
        self.adaft_on = True
        self.dvfs_on = False
        self.missed_task = {}

    def _remove_missed_tasks(self):
        """
        remove tasks in the ready queue if already missed the deadlines
        """
        if self.ready_queue:
            for i, t in enumerate(self.ready_queue):
                if round(t.abs_deadline, 3) < round(self.clock, 3):
                    del self.ready_queue[i]
                    if t.name in self.missed_task:
                        self.missed_task[t.name] += 1
                    else:
                        self.missed_task[t.name] = 1

    def release_new_task_iterations(self, sensor_inputs):
        for name, t in self.task_list.items():
            if name == 'filter':
                t = [t]
            if round((self.clock - t[0].start_time) * 1000, 0) % round((t[0].period * 1000), 0) == 0: # be careful to use round here to avoid precision errors
                self.n += 1
                x = np.reshape(self.control_inputs['filter'], (-1, 1))
                if x.shape[0] == 1:
                    x = sensor_inputs['sensor1']
                x = x.reshape(1, -1)

                #choose copies
                if name == 'filter':
                    copies = 3
                else:
                    if self.adaft_on:
                        if np.any(np.isnan(x)) or np.any(np.isinf(x)) or np.any(x > 65536):
                            copies = 3
                        else:
                            copies = int(self.clf[name].predict(x) + 1)
                    else:
                        copies = 3
                    self.copies = copies

                # pick version
                if name == 'filter':
                    new_t = copy.deepcopy(t[0])
                else:
                    if self.satisfy_QoC(sensor_inputs):
                        new_t = copy.deepcopy(t[1])
                    else:
                        new_t = copy.deepcopy(t[0])

                new_t.abs_deadline = round(self.clock + new_t.deadline, 3)
                self.task_iterations[new_t.name] += 1
                new_t.iterations = self.task_iterations[new_t.name]
                for i in range(copies):
                    self.ready_queue.append(copy.deepcopy(new_t))
                # new_t.run(self.task_inputs[new_t.name])

    @abstractmethod
    def satisfy_QoC(self, sensor_inputs):
        """is QoC constraint satisfied?"""
        x = sensor_inputs['sensor1']
        x = x.reshape(1, -1)
        theta1 = x[0][0]
        if -0.15 <= theta1 <= 0.15:
            return True
        return False



    def create_task(self, name, t):
        if name == 'filter':
            t.start_time = self.clock
        else:
            for version in t:
                version.start_time = self.clock

        self.task_list[name] = t
        self.control_inputs[name] = 0
        self.task_profiles[name] = copy.deepcopy(t)
        self.task_iterations[name] = 0


    def update1(self, h, clock, sensor_inputs):
        self.clock_sync(clock)
        if round((self.clock ) * 1000, 0) % round((self.load_tuning_period * 1000), 0) == 0: # be careful to use round here to avoid precision errors
            self.load_tuning(sensor_inputs)

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

    def update(self, h, clock, sensor_inputs):
        self.clock_sync(clock)
        if round((self.clock) * 1000, 0) % round((self.load_tuning_period * 1000),
                                                 0) == 0:  # be careful to use round here to avoid precision errors
            self.release_new_task_iterations(sensor_inputs)

        self.gedf(sensor_inputs)

        for processor in self.processors:
            processor.update(h, self.clock)

        self.update_control_inputs1()
        self.voting()

        if self.missed_task:
            print('missed task:' , self.missed_task)


    def update_control_inputs1(self):
        self.control_inputs_candidates = {}  # reset first
        self.control_inputs_candidates_iterations = {}  # reset first
        for processor in self.processors:
            for control_name, control_val in processor.rtos.task_outputs.items():
                if control_name in self.control_inputs_candidates:
                    if processor.rtos.task_iterations[control_name] == self.task_iterations[control_name]:
                        self.control_inputs_candidates[control_name].append(control_val)
                        self.control_inputs_candidates_iterations[control_name].append(processor.rtos.task_iterations[control_name])
                else:
                    if processor.rtos.task_iterations[control_name] == self.task_iterations[control_name]:
                        self.control_inputs_candidates[control_name] = [control_val]
                        self.control_inputs_candidates_iterations[control_name] = [processor.rtos.task_iterations[control_name]]

    def update_task_inputs(self, sensor_inputs):
        for processor in self.processors:
            for name in processor.rtos.task_list:
                if name == 'filter':
                    filter_inputs = {'sensor': sensor_inputs}
                    for task_name, task_val in processor.rtos.task_outputs.items():
                        if task_name != 'filter':
                            filter_inputs[task_name] = task_val
                    processor.rtos.task_inputs[name] = filter_inputs
                else:
                    processor.rtos.task_inputs[name] = np.reshape(self.control_inputs['filter'], (-1, 1))

    def gedf(self, sensor_inputs):
        self._remove_missed_tasks()
        self.ready_queue.sort(key = attrgetter('abs_deadline', 'name'))
        self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))
        processor_candidate = {i : 'not checked' for i in range(len(self.processors))}
        to_be_removed = []
        for t in self.ready_queue:
            assigned = False
            for i in processor_candidate:
                if processor_candidate[i] == 'not checked':
                    if self.processors[i].rtos.running_task is None:
                        if self.processors[i].rtos.task_list:
                            if 'filter' in self.processors[i].rtos.task_list:
                                self.task_list['filter'] = copy.deepcopy(self.processors[i].rtos.task_list['filter'])
                        new_t = copy.deepcopy(t)
                        self.processors[i].rtos.task_list = {t.name: new_t}
                        self.processors[i].rtos.ready_queue.append(new_t)
                        self.update_task_inputs(sensor_inputs)
                        new_t.run(self.processors[i].rtos.task_inputs[new_t.name])
                        to_be_removed.append(t)
                        processor_candidate[i] = 'checked'
                        assigned = True
                        break
            if assigned == False:
                for p in self.processors:
                    assigned_t = p.rtos.running_task if p.rtos.running_task else p.rtos.ready_queue[0]
                    if t.abs_deadline < assigned_t.abs_deadline:
                        # preempt least priority task that is running
                        self.processors.sort(key= lambda x : x.rtos.running_task.abs_deadline if x.rtos.running_task is not None else x.rtos.ready_queue[0].abs_deadline)

                        if self.processors[-1].rtos.running_task is not None:
                            self.processors[-1].rtos.running_task.finished_time += self.clock - self.processors[-1].rtos.running_task.intermediate_time_
                            if self.processors[-1].rtos.running_task.finished_time < self.processors[-1].rtos.running_task.et:
                                self.ready_queue.append(self.processors[-1].rtos.running_task)
                        else:
                            self.ready_queue.append(copy.deepcopy(self.processors[-1].rtos.ready_queue[0]))

                        self.processors[-1].rtos.task_list = {t.name : copy.deepcopy(t)}
                        self.processors[-1].rtos.running_task = None
                        new_t = copy.deepcopy(t)
                        self.processors[-1].rtos.ready_queue = [new_t]
                        self.update_task_inputs(sensor_inputs)
                        new_t.run(self.processors[-1].rtos.task_inputs[new_t.name])
                        to_be_removed.append(t)
                        self.processors.sort(key = attrgetter('reliability_model.abs_temperature'))
                        break

        for t in to_be_removed:
            self.ready_queue.remove(t)



    def load_tuning(self, sensor_inputs):
        idle_power = 0.5  # This is absurd, To Be Refactored!!!
        x = np.reshape(self.processors[0].rtos.task_outputs['filter'], (-1, 1))
        copy = 1
        self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))
        for i in range(copy):
            # List of processors need to run the tasks
            for name in self.processors[i].rtos.task_list:
                if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                    self.processors[i].rtos.task_list[name].power = self.processors[i].rtos.task_profiles[name].power
                    self.processors[i].rtos.task_list[name].et = self.processors[i].rtos.task_profiles[name].et

        for i in range(copy,len(self.processors)):
            # List of processor need to get some rest
            for name in self.processors[i].rtos.task_list:
                if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
                    self.processors[i].rtos.task_list[name].power = idle_power
                    self.processors[i].rtos.task_list[name].et = 0
                    # Don't change task et for now, since the voting algorithm would give wrong output,
                    # this is fine since the normal condition is to run 3 copies of the task, and now using AdaFT power will be idle,
                    # but the et will be unchanged, thus the schedulibility would not be affected.



    def clock_sync(self, clock):
        self.clock = clock

    def update_control_inputs(self):
        self.control_inputs_candidates = {}  # reset first
        self.control_inputs_candidates_iterations = {}  # reset first
        for processor in self.processors:
            for control_name, control_val in processor.rtos.task_outputs.items():
                if control_name != 'filter':
                    if control_name in self.control_inputs_candidates:
                        self.control_inputs_candidates[control_name].append(control_val)
                        self.control_inputs_candidates_iterations[control_name].append(processor.rtos.task_iterations[control_name])
                    else:
                        self.control_inputs_candidates[control_name] = [control_val]
                        self.control_inputs_candidates_iterations[control_name] = [processor.rtos.task_iterations[control_name]]

    @staticmethod
    def triple_vote(val_list, epsilon):
        if np.all((val_list[0] - val_list[1]) <= epsilon):
            return val_list[0]
        elif np.all(val_list[0] - val_list[2] <= epsilon):
            return val_list[0]
        return val_list[1]


    @staticmethod
    def double_vote(val_list, epsilon):
        if np.all((val_list[0] - val_list[1]) <= epsilon):
            return val_list[0]
        return np.zeros(len(val_list[0]))



    def voting(self):
        for control_name, control_val_list in self.control_inputs_candidates.items():
            # only when iterations numbers are equal to each other, it means all copies of a task have finished for this iteration, then we use voting
            if all(x==self.control_inputs_candidates_iterations[control_name][0] for x in self.control_inputs_candidates_iterations[control_name]):
                if len(control_val_list) == 3:
                    self.control_inputs[control_name] = self.triple_vote(control_val_list, 1e-6)
                elif len(control_val_list) == 2:
                    self.control_inputs[control_name] = self.double_vote(control_val_list, 1e-6)
                else:
                    self.control_inputs[control_name] = self.control_inputs_candidates[control_name][0]






