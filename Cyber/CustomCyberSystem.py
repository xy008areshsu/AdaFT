from Cyber.CyberSystem import CyberSystem
import numpy as np
from operator import attrgetter
import pickle
import os
import random


class RobotCyberSystem(CyberSystem):
    def __init__(self, processors, clock = 0, clf_file = 'subspace_clf_decision_tree.pkl'):
        super().__init__(processors, clock)
        self.clf = pickle.load(open(os.path.join('./Cyber/', clf_file), 'rb'))
        self.copies = 3
        self.version = 1
        self.idle_power = 0.5
        self.power_version2 = 6.0
        self.et_version2 = 0.002
        self.wcet_version2 = 0.002
        self.QoC = 0.15
        self.adaft_on = True
        self.dvfs_on = False
        self.old_q_state = (False, 3, 3)  # Q state (whether inside QoC constraint, what subspace, what taaf level,
                                                      # whether theta1 and rate1 are in different direction(True:different, False:same), planner's action)
        self.old_action = 's3_v1'
        # self.QLearning = QLearning(epsilon=0.9, alpha=0.95, gamma=0.7)
        # self.QLearning = pickle.load(open('./Cyber/QLearningRobot.pkl', 'rb'))
        self.QLearning = pickle.load(open('./Cyber/QLearningRobot_general.pkl', 'rb'))
        self.K0 = np.zeros((3, 6))
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

    def dvfs(self):
        """
        DSR algorithm, Dynamic Slack Reclamation
        :return: Scaled execution time and power
        """
        for i in range(len(self.processors)):
            for name in self.processors[i].rtos.task_list:
                ratio = self.processors[i].rtos.task_list[name].et / self.processors[i].rtos.task_list[name].wcet
                self.processors[i].rtos.task_list[name].power = max(self.idle_power, self.processors[i].rtos.task_list[name].power * (ratio ** 3))
                self.processors[i].rtos.task_list[name].et = self.processors[i].rtos.task_list[name].wcet

    def q_plan(self, QoC_constraint, x):
        greedy_prob = 0.3


        theta1 = x[0][0]
        rate1 = x[0][3]
        copies = int(self.clf.predict(x) + 1) # clf from sklearn predicts class 1 as 0, class 2 as 1, and so on, so need to add 1 here
        self.copies = copies
        prob = random.random()

        if (theta1 > QoC_constraint or theta1 < -QoC_constraint):
            if copies == 1:
                if prob > greedy_prob:
                    action = 's1_v1'
                else:
                    action = 'zero'
            elif copies == 2:
                if prob > greedy_prob:
                    action = 's2_v1'
                else:
                    action = 'zero'
            else:
                action = 's3_v1'
        else:
            if copies == 1:
                if prob > greedy_prob:
                    action = 's1_v2'
                else:
                    action = 'zero'
            elif copies == 2:
                if prob > greedy_prob:
                    action = 's2_v2'
                else:
                    action = 'zero'
            else:
                action = 's3_v2'

        return action, copies

    def act(self, action):
        power_version1 = self.processors[0].rtos.task_profiles['lqr'].power
        power_version2 = 6.0
        et_version1 = self.processors[0].rtos.task_profiles['lqr'].et
        et_version2 = 0.002
        wcet_version1 = self.processors[0].rtos.task_profiles['lqr'].wcet
        wcet_version2 = 0.004

        self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))

        kf_period = self.processors[0].rtos.task_list['filter'].period

        # schedule filter task, which is always 3 copies
        for i in range(3):
            # List of processors need to run the tasks
            if action in ['s1_v1', 's2_v1', 's3_v1']:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K1))
            elif action in ['s1_v2', 's2_v2', 's3_v2']:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K2))
            else:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * self.A
            self.processors[i].rtos.task_list['filter'].power = self.processors[i].rtos.task_profiles[
                'filter'].power
            self.processors[i].rtos.task_list['filter'].et = self.processors[i].rtos.task_profiles['filter'].et

        for i in range(3, len(self.processors)):
            # List of processor need to get some rest
            self.processors[i].rtos.task_list['filter'].power = self.idle_power
            self.processors[i].rtos.task_list['filter'].et = 0

            if action in ['s1_v1', 's2_v1', 's3_v1']:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K1))
            elif action in ['s1_v2', 's2_v2', 's3_v2']:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K2))
            else:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * self.A

        # schedule the remaining tasks
        if action in ['s1_v1', 's1_v2']:
            copies = 1
        elif action in ['s2_v1', 's2_v2']:
            copies = 2
        elif action in ['s3_v1', 's3_v2']:
            copies = 3
        else:
            copies = 0

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

            if action in ['s1_v1', 's2_v1', 's3_v1']:
                self.processors[i].rtos.task_list['lqr'].power = power_version1
                self.processors[i].rtos.task_list['lqr'].et = et_version1
                self.processors[i].rtos.task_list['lqr'].wcet = wcet_version1
                self.processors[i].rtos.task_list['lqr'].K = self.K1
                self.version = 1
            else:
                self.processors[i].rtos.task_list['lqr'].power = power_version2
                self.processors[i].rtos.task_list['lqr'].et = et_version2
                self.processors[i].rtos.task_list['lqr'].wcet = wcet_version2
                self.processors[i].rtos.task_list['lqr'].K = self.K2
                self.version = 2

        for i in range(copies, len(self.processors)):
            # List of processor need to get some rest
            # for name in self.processors[i].rtos.task_list:
            #     if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
            #         self.processors[i].rtos.task_list[name].power = idle_power
            #         self.processors[i].rtos.task_list[name].et = 0


            self.processors[i].rtos.task_list['lqr'].power = self.idle_power
            self.processors[i].rtos.task_list['lqr'].et = 0

            if action in ['s1_v1', 's2_v1', 's3_v1']:
                self.processors[i].rtos.task_list['lqr'].K = self.K1
            elif action in ['s1_v2', 's2_v2', 's3_v2']:
                self.processors[i].rtos.task_list['lqr'].K = self.K2
            else:
                self.processors[i].rtos.task_list['lqr'].K = self.K0

    def load_tuning1(self, sensor_inputs):
        x = sensor_inputs['sensor1']
        x = np.reshape(self.processors[0].rtos.task_outputs['filter'], (-1, 1))
        x = x.reshape(1, -1)
        theta1 = x[0][0]
        rate1 = x[0][3]

        QoC_constraint = 0.15
        (q_action, subspace) = self.q_plan(QoC_constraint=QoC_constraint, x=x)

        # update q state
        if 1.13 <= self.processors[0].reliability_model.taaf <= 1.14:
            taaf_state = 2
        elif 1.11 <= self.processors[0].reliability_model.taaf <= 1.13:
            taaf_state = 1
        elif self.processors[0].reliability_model.taaf < 1.11:
            taaf_state = 0
        else:
            taaf_state = 3
        cur_q_state = (-QoC_constraint <= theta1 <= QoC_constraint, subspace, taaf_state)

        # Get reward
        reward = 0
        # if self.old_action == self.old_q_state[2]:
        #     reward = 1

        if taaf_state == 2:
            reward += 1
        elif taaf_state == 1:
            reward += 2
        elif taaf_state == 0:
            reward += 3
        else:
            reward += 0

        if cur_q_state[0] == True:
            reward += 2

        # if self.old_action == self.old_q_state[2]:
        #     reward += 1

        if cur_q_state[0] == False:
            reward = -2
        if cur_q_state[1] == 2:
            reward =-3
        if cur_q_state[1] == 3:
            reward = -4


        # update Q Table
        self.QLearning.update_QTable(self.old_q_state, self.old_action, reward, cur_q_state)

        # select action from Q Learning
        action = self.QLearning.policy(cur_q_state)
        self.old_q_state = cur_q_state
        self.old_action = action

        self.act(action)
        # print(action)



    def load_tuning(self, sensor_inputs):

        x = sensor_inputs['sensor1']
        x = np.reshape(self.processors[0].rtos.task_outputs['filter'], (-1, 1))
        x = x.reshape(1, -1)
        theta1 = x[0][0]
        rate1 = x[0][3]
        if self.adaft_on:
            copies = int(self.clf.predict(x) + 1) # clf from sklearn predicts class 1 as 0, class 2 as 1, and so on, so need to add 1 here
        else:
            copies = 3

        power_version1 = self.processors[0].rtos.task_profiles['lqr'].power
        power_version2 = self.power_version2
        et_version1 = self.processors[0].rtos.task_profiles['lqr'].et
        et_version2 = self.et_version2
        wcet_version1 = self.processors[0].rtos.task_profiles['lqr'].wcet
        wcet_version2 = self.wcet_version2
        QoC = self.QoC
        self.copies = copies
        self.processors.sort(key=attrgetter('reliability_model.abs_temperature'))

        kf_period = self.processors[0].rtos.task_list['filter'].period

        #schedule filter task, which is always 3 copies
        for i in range(3):
            # List of processors need to run the tasks
            if theta1 > QoC or theta1 < -QoC:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K1))
            else:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K2))
            self.processors[i].rtos.task_list['filter'].power = self.processors[i].rtos.task_profiles[
                'filter'].power
            self.processors[i].rtos.task_list['filter'].et = self.processors[i].rtos.task_profiles['filter'].et


        for i in range(3, len(self.processors)):
            # List of processor need to get some rest
            self.processors[i].rtos.task_list['filter'].power = self.idle_power
            self.processors[i].rtos.task_list['filter'].et = 0

            if theta1 > QoC or theta1 < -QoC:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K1))
            else:
                self.processors[i].rtos.task_list['filter'].kf.F = 1 + kf_period * (self.A - np.dot(self.B, self.K2))

        #schedule the remaining tasks
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
                self.processors[i].rtos.task_list['lqr'].wcet = wcet_version1
                self.processors[i].rtos.task_list['lqr'].K = self.K1
                self.version = 1
            else:
                self.processors[i].rtos.task_list['lqr'].power = power_version2
                self.processors[i].rtos.task_list['lqr'].et = et_version2
                self.processors[i].rtos.task_list['lqr'].wcet = wcet_version2
                self.processors[i].rtos.task_list['lqr'].K = self.K2
                self.version = 2

        for i in range(copies,len(self.processors)):
            # List of processor need to get some rest
            # for name in self.processors[i].rtos.task_list:
            #     if name != 'filter': # filtering task always runs 3 copies, since it is the most important localizations
            #         self.processors[i].rtos.task_list[name].power = idle_power
            #         self.processors[i].rtos.task_list[name].et = 0


            self.processors[i].rtos.task_list['lqr'].power = self.idle_power
            self.processors[i].rtos.task_list['lqr'].et = 0

            if theta1 > QoC or theta1 < -QoC:
                self.processors[i].rtos.task_list['lqr'].K = self.K1
            else:
                self.processors[i].rtos.task_list['lqr'].K = self.K2

        if self.dvfs_on:
            self.dvfs()

class QLearning:
    def __init__(self, epsilon = 0.1, alpha = 0.5, gamma = 0.9):
        """
        :param epsilon: probability of doing random move to deal with local stuck
        :param alpha: learning rate
        :param gamma: discount factor for reward
        """
        self.QTable = {} # key: (state, action), value: the Q value
        self.epsilon = epsilon
        self.alpha = alpha
        self.gamma = gamma
        self.possible_actions = ['s1_v1', 's1_v2', 's2_v1', 's2_v2', 's3_v1', 's3_v2', 'zero']
        self.time = 0
        self.alpha_lower_bound = 0.7
        self.epsilon_lower_bound = 0.1
        self.learning_rate_decay_steps = 100
        self.epsilon_decay_steps = 200
        self.learning_rate_decay_factor = 0.95
        self.epsilon_decay_factor = 0.97
        self.init_utility = -10

    def policy(self, state):
        """
        :param state: current state
        :return: action: argmax over action of QTable[(state, action)], but also small probability of moving randomly
        """
        if random.random() < self.epsilon:
            action = random.choice(self.possible_actions)
        else:
            vals = [self.QTable[(state, a)] if (state, a) in self.QTable else -1000 for a in self.possible_actions]
            keys = [(state, a) for a in self.possible_actions]
            for k in keys:
                if k not in self.QTable:
                    self.QTable[k] = -10
            action =  keys[vals.index(max(vals))][1]

        return action

    def update_QTable(self, state, action, reward, next_state):
        if (state, action) not in self.QTable:
            self.QTable[(state, action)] = -10

        new_q = reward + self.gamma * max([self.QTable[(next_state, next_action)]
                                           if (next_state, next_action) in self.QTable else -10
                                           for next_action in self.possible_actions])
        old_q = self.QTable[(state, action)]
        self.QTable[(state, action)] = (1 - self.alpha) * self.QTable[(state, action)] + self.alpha * new_q
        self.time += 1

        # performance learning rate and probability decay
        if self.time % self.learning_rate_decay_steps == 0:
            if self.alpha > self.alpha_lower_bound:
                self.alpha *= self.learning_rate_decay_factor #learning rate decay

        if self.time % self.epsilon_decay_steps == 0:
            if self.epsilon > self.epsilon_lower_bound:
                self.epsilon *= self.epsilon_decay_factor # random move probability decay


        # if abs(self.QTable[(state, action)] - old_q) < 0.05:
        #     print("converged")
        # else:
        #     print('not converged')



class Planner(object):
    """Silly route planner that is meant for a perpendicular grid network."""

    def __init__(self, env, agent):
        self.env = env
        self.agent = agent
        self.destination = None

    def route_to(self, destination=None):
        self.destination = destination if destination is not None else random.choice(self.env.intersections.keys())
        # print "RoutePlanner.route_to(): destination = {}".format(destination)  # [debug]

    def next_waypoint(self):
        location = self.env.agent_states[self.agent]['location']
        heading = self.env.agent_states[self.agent]['heading']
        delta = (self.destination[0] - location[0], self.destination[1] - location[1])
        if delta[0] == 0 and delta[1] == 0:
            return None
        elif delta[0] != 0:  # EW difference
            if delta[0] * heading[0] > 0:  # facing correct EW direction
                return 'forward'
            elif delta[0] * heading[0] < 0:  # facing opposite EW direction
                return 'right'  # long U-turn
            elif delta[0] * heading[1] > 0:
                return 'left'
            else:
                return 'right'
        elif delta[1] != 0:  # NS difference (turn logic is slightly different)
            if delta[1] * heading[1] > 0:  # facing correct NS direction
                return 'forward'
            elif delta[1] * heading[1] < 0:  # facing opposite NS direction
                return 'right'  # long U-turn
            elif delta[1] * heading[0] > 0:
                return 'right'
            else:
                return 'left'
