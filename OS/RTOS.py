import copy

class RTOS:
    def __init__(self, ready_queue = [], running_task = None, clock = 0):
        self.ready_queue = ready_queue
        self.running_task = running_task
        self.task_list = {}
        self.clock = clock
        self.task_outputs = {}
        self.task_inputs = {}
        self.n = 0
        self.missed_task = {}
        self.task_profiles = {} # original task profiles, parameters determined at design time


    def create_task(self, t):
        # import copy
        t.start_time = self.clock
        self.task_list[t.name] = t
        self.task_outputs[t.name] = t.output
        self.task_profiles[t.name] = copy.deepcopy(t)


    def schedule(self): # assume scheduler no overhead
        if self.ready_queue:
            self.prioritize_tasks('EDF')
            if self.running_task == None:
                self.run_highest_priority_task()
            elif self.compare_priorities(self.ready_queue[0], self.running_task, 'EDF') > 0:
                self.preempt()

    def preempt(self):
        self.running_task.finished_time += self.clock - self.running_task.intermediate_time_
        if self.running_task.finished_time < self.running_task.et:
            self.running_task.status = 'ready'
            self.ready_queue.append(self.running_task)
        self.run_highest_priority_task()

    def run_highest_priority_task(self):
        if self.ready_queue:
            self.running_task = self.ready_queue[0]
            self.running_task.status = 'running'
            del self.ready_queue[0]
            self.running_task.intermediate_time_ = self.clock

    def prioritize_tasks(self, policy):
        from operator import attrgetter
        if policy == 'EDF':
            self.ready_queue.sort(key=attrgetter('abs_deadline'))
            self._remove_missed_tasks()

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


    def compare_priorities(self, t1, t2, policy):
        # 1: t1 > t2;
        # -1:t1 < t2;
        # 0: t1 = t2;
        if policy == 'EDF':
            if t1.abs_deadline < t2.abs_deadline:
                return 1
            elif t1.abs_deadline == t2.abs_deadline:
                return 0
            else:
                return -1
        return 0

    def release_new_task_iterations(self):
        import math
        for name, t in self.task_list.items():
            if round((self.clock - t.start_time) * 1000, 0) % round((t.period * 1000), 0) == 0: # be careful to use round here to avoid precision errors
                self.n += 1
                new_t = copy.deepcopy(t)
                new_t.abs_deadline = round(self.clock + new_t.deadline, 3)
                self.ready_queue.append(new_t)
                new_t.run(self.task_inputs[new_t.name])

    def clock_sync(self, clock):
        self.clock = clock

    def check_running_task(self):
        # check if the running task has finished, if so, remove it from the running queue
        # and update the corresponding task output
        if self.running_task is not None:
            if round(self.clock - self.running_task.intermediate_time_ + self.running_task.finished_time, 3) >= round(self.running_task.et, 3):
                self.task_outputs[self.running_task.name] = self.running_task.output
                self.task_list[self.running_task.name] = copy.deepcopy(self.running_task)  # don't forget to update the output in the task_list, in case some task uses its previous iteration's output as its current input.
                self.task_list[self.running_task.name].finished_time = 0
                self.running_task = None





    def update(self, h, clock):
        self.clock_sync(clock)
        self.release_new_task_iterations()
        self.check_running_task()
        self.schedule()