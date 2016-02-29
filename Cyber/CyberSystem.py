
class CyberSystem:
    """
    Cyber System implements, for the simulation purpose, all of the components that we ignore the overhead, such as
    the voting component of different copies of a particular task.
    """

    def __init__(self, sensors, processors, actuators, clock = 0):
        """

        :param clock:
        :param sensors:
        :param processors:
        :return:
        """
        self.clock = clock
        self.processors = processors
        self.actuator_commands = {}
        for processor in self.processors:
            for command in processor.actuator_commands:
                self.actuator_commands[command] = {'status' : 'waiting', 'val_list' : {}, 'output' : 0}
            break

        self.sensors = sensors

        for processor in self.processors:
            processor.sensors = self.sensors

        self.actuators = actuators


    def update(self, h, clock):
        self.clock_sync(clock)
        for processor in self.processors:
            processor.update(h, self.clock)

        self.voting()


    def clock_sync(self, clock):
        self.clock = clock

    @staticmethod
    def majority_vote(val_list):
        out = {}
        majority = None
        out[majority] = 0
        for k, v in val_list.items():
            if v not in out:
                out[v] = 1
            else:
                out[v] += 1
            if out[v] >= len(val_list) / 2.0:
                return v
            if out[v] > out[majority]:
                majority = v

        return majority

    @staticmethod
    def double_vote(val_list):
        vals = list(val_list)
        if vals[0] == vals[1]:
            return vals[0]
        return 0


    def voting(self):
        for command in self.actuator_commands:
            running = 0
            finished = 0
            for processor in self.processors:
                if processor[command]['status'] == 'finished':
                    self.actuator_commands[command]['val_list'][processor] = processor[command]['val']
                    finished += 1
                elif processor[command]['status'] == 'running':
                    running += 1

            if running == 0 and finished != 0: # this means new iteration of this task is done for all copies
                #first we turn all finished status to idle
                for processor in self.processors:
                    if processor[command]['status'] == 'finished':
                        processor[command]['status'] = 'idle'

                # then we turn self.actuator_commands[command]['status'] to ready
                self.actuator_commands[command]['status'] = 'ready'

            if self.actuator_commands[command]['status'] == 'ready':
                self.actuator_commands[command]['status'] = 'waiting'
                if len(self.actuator_commands[command]['val_list']) == 3:
                    self.actuator_commands[command]['output'] = self.majority_vote(self.actuator_commands[command]['val_list'])
                elif len(self.actuator_commands[command]['val_list']) == 2:
                    self.actuator_commands[command]['output'] = self.double_vote(self.actuator_commands[command]['val_list'])
                else:
                    for p in self.actuator_commands[command]['val_list']:
                        self.actuator_commands[command]['output'] = self.actuator_commands[command]['val_list'][p]





