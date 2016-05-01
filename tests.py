import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scipy as sp
from Task.TaskModel import TaskModel
from Task.CustomTasks import ABS, ACC, KalmanPredict, LQRInvPen, KalmanLQR, NoABS, ABSPF, LQRRobot
from OS.RTOS import RTOS
from Processor.Processor import Processor
from Reliability.CustomReliabilityModel import TAAF
from numpy.random import randn

import Task.CustomTasks as ct
from Sensor.CustomSensor import InvPenSensor, RobotSensor
from Actuator.Actuator import Actuator
from Actuator.CustomActuator import RobotActuator
from Physical.CustomPhysicalSystem import InvPenDynamics, Robot
from Cyber.CyberSystem import CyberSystem
from Cyber.CustomCyberSystem import RobotCyberSystem
from CPS.CustomCPS import InvPenCPS, ABSCPS, RobotCPS
import copy

from Sensor.CustomSensor import ABSSensor
# from Actuator.CustomActuator import ABSActuator, NoABSActuator
from Physical.CustomPhysicalSystem import VehicleABSDynamics
from Task.ParticleFilter import ABSParticleFilter, ParticleFilter
import pickle
import os
import random
import time

# ### Inverted Pendulum test  ###
# h = 0.001
# kf_period = 0.009
# kf_deadline = kf_period
# kf_wcet = 0.001
# kf_power = 6.5
#
# lqr_pen_period = 0.02
# lqr_pen_deadline = lqr_pen_period
# lqr_pen_wcet = 0.01
# lqr_pen_power = 3.5
#
# actuator_noise = {'lqr' : 3}
#
# cart_pos_noise = 0.001
# cart_vel_noise = 0.001
# angle_noise = 0.01
# rate_noise = 0.005
# cart_pos_noise1 = 0.001
# cart_vel_noise1 = 0.001
# angle_noise1 = 0.002
# rate_noise1 = 0.1
#
# A = np.array([[0., 1.0, 0., 0.],
#                [0., -0.18182, 2.6727, 0.],
#                [0., 0., 0., 1.],
#                [0., -0.45455, 31.1820, 0.]])
#
# B = np.array([[0.0],
#                [1.8182],
#                [0.],
#                [4.5455]])
#
# C = np.array([[1., 0., 0., 0.],
#               [0., 1., 0., 0.],
#               [0., 0., 1., 0.],
#               [0., 0., 0., 1.]])
#
# K = np.array([[-61.9930, -33.5054, 95.0600, 18.8300]])
# P = np.array([[1000., 0., 0., 0.],
#               [0., 1000., 0., 0.],
#               [0., 0., 1000., 0.],
#               [0., 0., 0., 1000.]])
#
#
# R0 = np.array([[cart_pos_noise, 0., 0., 0.],
#               [0., cart_vel_noise, 0., 0.],
#               [0., 0., angle_noise, 0.],
#               [0., 0., 0., rate_noise]])
# R1 = np.array([[cart_pos_noise1, 0., 0., 0.],
#               [0., cart_vel_noise1, 0., 0.],
#               [0., 0., angle_noise1, 0.],
#               [0., 0., 0., rate_noise1]])
#
# Rs = {'sensor0' : R0, 'sensor1' : R1}
#
# F = 1 + kf_period * (A - np.dot(B, K))
#
#
#
# sss = np.zeros((4, 1))
# c_sss = np.zeros((4, 1))
#
#
# cart_pos = [p for p in np.arange(0, 0.5, 0.5)]
# cart_vel = [p for p in np.arange(0, 1, 1)]
# angles = [p for p in np.arange(0.4, 0.5, 0.1)]
# rate = [p for p in np.arange(0.5, 1.5, 1)]
#
# count = 0
# for p in cart_pos:
#     for v in cart_vel:
#         for a in angles:
#             for r in rate:
#                 safe = True
#
#                 # print(count)
#                 count += 1
#                 x0 = np.array([[p],
#                                [v],
#                                [a],
#                                [r]])
#
#                 init_x = copy.deepcopy(x0)
#                 xs = x0
#
#                 xtract = np.array([[0.0],
#                                    [0.0],
#                                    [0.4],
#                                    [0.5]])
#
#
#                 actuator = Actuator(actuator_noise)
#
#                 pen = InvPenDynamics(x0, A, B)
#
#                 sensor = InvPenSensor([cart_pos_noise, cart_vel_noise, angle_noise, rate_noise])
#                 sensor1 = InvPenSensor([cart_pos_noise1, cart_vel_noise1, angle_noise1, rate_noise1])
#                 sensors = [sensor, sensor1]
#
#                 kf = ct.makeLinearKF(A, B, C, P, F, x0[:, -1])
#                 localizer = KalmanPredict('filter', kf_period, kf_deadline, kf_wcet, kf_power, kf, Rs)
#                 lqr = LQRInvPen('lqr', lqr_pen_period, lqr_pen_deadline, lqr_pen_wcet, lqr_pen_power, K)
#                 kalman_lqr = KalmanLQR('kalman_lqr', lqr_pen_period, lqr_pen_deadline, lqr_pen_wcet, lqr_pen_power, kf, Rs, K)
#
#                 queue = [localizer, lqr]
#                 queue2 = [kalman_lqr]
#                 rtos = RTOS()
#                 for task in queue:
#                     rtos.create_task(task)
#                 fail_rate = 3.1706e-09
#                 reliability_model = TAAF(fail_rate)
#                 processor = Processor(rtos, reliability_model)
#                 processor2 = copy.deepcopy(processor)
#                 processor3 = copy.deepcopy(processor)
#
#                 cyber = CyberSystem([processor, processor2, processor3])
#
#                 end = 4
#
#                 cps = InvPenCPS(pen, cyber, sensors, actuator, end=end)
#
#                 cps.run()
#
# #                 taaf = np.zeros([end / h, 1])
# #                 time = np.arange(0, end, h)
# #                 taaf[0] = 1
# #                 mttf = np.zeros([end / h, 1])
# #                 mttf[0] = 10
# #
# #
# #                 i = 0
# #                 for clock in np.arange(0, end, h):
# #
# #                     z0 = sensor.sense(pen.x)
# #                     z1 = sensor1.sense(pen.x)
# #                     # if 2 <= clock <= 3:
# #                     #     z2 = np.array([[0.], [0.], [100], [0.]]) # assign transient failures to sensor 2
# #                     sensor_inputs = {'sensor0' : z0[:, -1], 'sensor1' : z1[:, -1]}
# #                     # sensor_inputs = {'sensor1' : z1[:, -1]}
# #                     xs = np.append(xs, pen.x, axis=1)
# #                     # processor.rtos.task_inputs[lqr.name] = pen.x
# #                     # processor.rtos.task_inputs[lqr.name] = np.reshape(processor.rtos.task_outputs['filter'], (-1, 1))
# #                     # # processor.rtos.task_inputs[lqr.name] = z2
# #                     # processor.rtos.task_inputs['filter'] = all_z
# #                     # processor.update(h, clock)
# #                     cyber.update(h, clock, sensor_inputs)
# #                     # actuator_commands = actuator.actuator_commands(processor.rtos.task_outputs[lqr.name])
# #                     actuator.update(cyber.control_inputs)
# #                     # taaf[i] = processor.reliability_model.taaf
# #                     # mttf[i] = processor.reliability_model.mttf_in_years
# #                     taaf[i] = cyber.processors[0].reliability_model.taaf
# #                     mttf[i] = cyber.processors[0].reliability_model.mttf_in_years
# #                     i+=1
# #
# #                     pen.update(h, clock, actuator.actuator_commands)
# #
# #                     x_predict = np.reshape(cyber.processors[0].rtos.task_outputs['filter'], (-1, 1))
# #                     xtract = np.append(xtract, x_predict, axis=1)
# #
# #                     if not pen.is_safe():
# #                         safe = False
# #                         break
# #
# #                 if safe:
# #                     regression = np.append(init_x, [[pen.x[2][0]]], axis=0)
# #                     sss = np.append(sss, init_x, axis=1)
# #                 else:
# #                     c_sss = np.append(c_sss, init_x, axis=1)
# #
# #
# # # np.savetxt('sss.csv', sss, delimiter=',')
# # # np.savetxt('c_sss.csv', c_sss, delimiter=',')
# #
# # # print("Total number of tasks released: ", processor.rtos.n)
# # # print("Missed tasks: ", processor.rtos.missed_task)
# #
# # # np.savetxt('use_kalman.csv', u_from_task, delimiter=',')
#
# print("Total number of tasks released: ", cps.cyber_sys.processors[0].rtos.n)
# print("Missed tasks: ", cps.cyber_sys.processors[0].rtos.missed_task)
#
# xtract = cps.xtract
# xs = cps.xs
# taaf = cps.taaf[cps.taaf != 0]
# temp = cps.temp[cps.temp != 0]
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[2, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[2, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Angle (rad)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
# # plt.figure()
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(xtract[3, :], c = 'b', label='Kalman Filter', linewidth=2)
# # plt.plot(xs[3, :], c= 'r', label='True value', linewidth = 1)
# # plt.xlabel('Time (ms)', fontsize=18)
# # plt.ylabel('Angular Rate (rad/s)', fontsize=18)
# # plt.xticks(fontsize=18)
# # plt.yticks(fontsize=18)
# # plt.legend(loc=4)
# #
# # plt.figure()
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(xtract[0, :], c = 'b', label='Kalman Filter', linewidth=2)
# # plt.plot(xs[0, :], c= 'r', label='True value', linewidth = 1)
# # plt.xlabel('Time (ms)')
# # plt.ylabel('Cart pos')
# # plt.legend(loc=4)
# #
# # plt.figure()
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(xtract[1, :], c = 'b', label='Kalman Filter', linewidth=2)
# # plt.plot(xs[1, :], c= 'r', label='True value', linewidth = 1)
# # plt.xlabel('Time (ms)')
# # plt.ylabel('Cart vel')
# # plt.legend(loc=4)
# # plt.show()
#
# f, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
# ax1.plot(taaf)
# ax1.set_title('taaf')
# # ax1.set_ylim([1.0, 1.2])
# # ax1.set_xlim([0.0, 4000])
# ax2.plot(temp)
# # ax2.set_title('mttf in years')
#
# plt.show()
#
#
# ######### Car ABS  #########
#
# ### Define parameters
# h = 0.001
# x0 = np.array([[30.], [30.], [0.]])
# xs = x0
# mass_quater_car = 250
# mass_effective_wheel = 20
# road_friction_coeff = 0.8
#
# actuator_noise = {'abs' : 0}
# sensor_vehicle_speed_noise = 0.5 # don't use a very small value, which will lead to poor particle filter performance
# sensor_wheel_speed_noise = 0.5
# sensor_pos_noise = 0.5
# all_sensor_noise_levels = {'sensor0' : np.array([sensor_vehicle_speed_noise, sensor_wheel_speed_noise, sensor_pos_noise])}
#
# period = 0.02
# deadline = period
# wcet = 0.01
# power = 3.5
#
# period_pf = 0.01
# deadline_pf = period_pf
# wcet_pf = 0.001
# power_pf = 6.5
# N = 1000
#
#
# ### Create and Intialize Objects
# car = VehicleABSDynamics(x0, road_friction_coeff=road_friction_coeff)
# car_sensor = ABSSensor([sensor_vehicle_speed_noise, sensor_wheel_speed_noise, sensor_pos_noise])
# car_actuator = Actuator(actuator_noise)
# # car_actuator_no_abs = Actuator(actuator_noise)
# abs = ABS('abs', period, deadline, wcet, power)
#
# range = np.array([[0., 80.], [0., 80.], [0., 80.]])
# std = np.array([1, 1, 1])
# pf = ABSParticleFilter(road_friction_coeff, mass_quater_car, mass_effective_wheel, range, all_sensor_noise_levels, period_pf,
#                        std, N=N, init_state_guess= x0)
# localizer = ABSPF('filter', period_pf, deadline_pf, wcet_pf, power_pf, pf)
#
# xtract = x0
#
# queue = [abs, localizer]
# rtos = RTOS()
# for task in queue:
#     rtos.create_task(task)
# fail_rate = 3.1706e-09
# reliability_model = TAAF(fail_rate)
# processor = Processor(rtos, reliability_model)
# processor2 = copy.deepcopy(processor)
# processor3 = copy.deepcopy(processor)
#
# cyber = CyberSystem([processor, processor2, processor3])
# # cyber = CyberSystem([processor])
#
# cps = ABSCPS(car, cyber, [car_sensor], car_actuator, end=10)
# cps.run()
#
# # ### Simulation Parameters
# # end = 10
# # taaf = np.zeros([end / h, 1])
# # time = np.arange(0, end, h)
# # taaf[0] = 1
# # mttf = np.zeros([end / h, 1])
# # mttf[0] = 10
# # u = []
# #
# #
# # ### Run Simulations
# # i = 0
# # for clock in np.arange(0, end, h):
# #     if car.x[0] <= 1:
# #         break
# #
# #     z = car_sensor.sense(car.x)
# #     xs = np.append(xs, car.x, axis=1)
# #     # filter_inputs = {'control' : processor.rtos.task_outputs[abs.name],
# #     #                     'sensor' : {'sensor1' : z}}
# #     sensor_inputs = {'sensor1' : z}
# #     # processor.rtos.task_inputs[abs.name] = processor.rtos.task_outputs['filter']
# #     # processor.rtos.task_inputs['filter'] = filter_inputs
# #     # processor.update(h, clock)
# #     cyber.update(h, clock, sensor_inputs)
# #     # actuator_commands = car_actuator.actuator_commands(processor.rtos.task_outputs[abs.name])
# #     car_actuator.update(cyber.control_inputs)
# #     # actuator_commands = car_actuator_no_abs.actuator_commands(processor.rtos.task_outputs[abs.name])
# #     u.append(car_actuator.actuator_commands['abs'])
# #     taaf[i] = cyber.processors[0].reliability_model.taaf
# #     mttf[i] = cyber.processors[0].reliability_model.mttf_in_years
# #     i+=1
# #
# #     car.update(h, clock, car_actuator.actuator_commands)
# #
# #     x_predict = np.reshape(cyber.processors[0].rtos.task_outputs['filter'], (-1, 1))
# #     xtract = np.append(xtract, x_predict, axis=1)
#
# taaf = cps.taaf[cps.taaf != 0]
# temp = cps.temp[cps.temp != 0]
# # mttf = cps.mttf[mttf != 0]
# xtract = cps.xtract
# xs = cps.xs
#
# print("Total number of tasks released: ", processor.rtos.n)
# print("Missed tasks: ", processor.rtos.missed_task)
#
# ### Plot Results
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[0, :], c = 'c', label='Particle Filter Vehicle Speed', linewidth=4)
# plt.plot(xtract[1, :], c = 'g', label='Particle Filter Wheel Speed', linewidth=4)
# plt.plot(xs[0, :], c = 'b', label='Vehicle Speed', linewidth=1)
# plt.plot(xs[1, :], c= 'r', label='Wheel Speed', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Speed m/s', fontsize=18)
# plt.xticks(fontsize = 15)
# plt.yticks(fontsize = 15)
# plt.legend(loc=1, fontsize = 15)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xs[2, :], c = 'b', label='distance', linewidth=2)
# plt.xlabel('Time (ms)')
# plt.ylabel('Distance (m)')
# plt.legend(loc=4)
#
# # plt.figure()
# # plt.hold(True)
# # plt.grid(True)
# # plt.plot(u, c = 'b', label='control', linewidth=2)
# # plt.xlabel('Time (ms)')
# # plt.ylabel('control actuator')
# # plt.legend(loc=4)
#
# f, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
# ax1.plot(taaf)
# ax1.set_title('taaf')
# ax2.plot(temp)
# ax2.set_title('mttf in years')
#
# plt.show()
#
# # np.savetxt('temp009.csv', temp, delimiter=',')


############ Humanoid Robot ########

from numpy import array, zeros, eye, asarray, dot, rad2deg
from numpy.linalg import inv
from matplotlib.pyplot import plot, xlabel, ylabel, legend, rcParams
rcParams['figure.figsize'] = (14, 8)
from sympy import simplify, Matrix, matrix2numpy
from sympy.physics.vector import init_vprinting, vlatex
init_vprinting(use_latex='mathjax')
from Utilities.Controller import controllable
from scipy.linalg import solve_continuous_are
from scipy.integrate import odeint

A = np.array([[   0.   ,    0.   ,    0.   ,    1.   ,    0.   ,    0.   ],
               [   0.   ,    0.   ,    0.   ,    0.   ,    1.   ,    0.   ],
               [   0.   ,    0.   ,    0.   ,    0.   ,    0.   ,    1.   ],
               [  18.337,  -75.864,    6.395,    0.   ,    0.   ,    0.   ],
               [ -22.175,  230.549,  -49.01 ,    0.   ,    0.   ,    0.   ],
               [   4.353, -175.393,   95.29 ,    0.   ,    0.   ,    0.   ]])

B = np.array([[ 0.   ,  0.   ,  0.   ],
               [ 0.   ,  0.   ,  0.   ],
               [ 0.   ,  0.   ,  0.   ],
               [ 0.292, -0.785,  0.558],
               [-0.785,  2.457, -2.178],
               [ 0.558, -2.178,  2.601]])

C = eye(6)


K = np.array([[1054.367, 426.901, 153.864, 365.784, 173.577, 67.28 ],
              [707.669, 610.181, 251.283, 263.836, 158.583, 72.18 ],
              [12.129,  43.669, 132.469,  11.613,  16.447,  19.12 ]])

K1 = np.array([[ 929.79 ,  322.082,   73.883,  317.782,  142.912,   49.865],
       [ 521.09 ,  431.75 ,   74.787,  188.568,  105.95 ,   38.039],
       [ 277.   ,  239.326,  202.902,  108.11 ,   70.849,   42.213]])

# x0 = np.array([-0.087,  0.087, -0.087, -0.087, -0.087, -0.087])

# y = odeint(right_hand_side, x0, t, args=(controller, numerical_constants))

h = 0.001
kf_period = 0.01
kf_deadline = kf_period
kf_wcet = 0.001
kf_power = 6.5

lqr_period = 0.02
lqr_deadline = lqr_period
lqr_wcet = 0.008
lqr_power = 10

actuator_noise = {'lqr' : 0}

theta1_noise = 0.0001
theta2_noise = 0.0001
theta3_noise = 0.0001
rate1_noise = 0.0001
rate2_noise = 0.0001
rate3_noise = 0.0001

theta1_noise1 = 0.0001
theta2_noise1 = 0.0001
theta3_noise1 = 0.0001
rate1_noise1 = 0.0001
rate2_noise1 = 0.0001
rate3_noise1 =0.0001

P = 1000 * eye(6)


R0 = np.array([[theta1_noise, 0., 0., 0., 0., 0.],
               [0., theta2_noise, 0., 0., 0., 0.],
               [0., 0., theta3_noise, 0., 0., 0.],
               [0., 0., 0., rate1_noise,  0., 0.],
               [0., 0., 0., 0., rate2_noise,  0.],
               [0., 0., 0., 0., 0., rate3_noise ]])


R1 = np.array([[theta1_noise1, 0., 0., 0., 0., 0.],
               [0., theta2_noise1, 0., 0., 0., 0.],
               [0., 0., theta3_noise1, 0., 0., 0.],
               [0., 0., 0., rate1_noise1,  0., 0.],
               [0., 0., 0., 0., rate2_noise1,  0.],
               [0., 0., 0., 0., 0., rate3_noise1 ]])

Rs = {'sensor0' : R0, 'sensor1' : R1}

F = 1 + kf_period * (A - np.dot(B, K))


x0 = np.array([[-0.45],
               [0.087],
               [-0.087],
               [-0.087],
               [-0.087],
               [-0.087]])

init_x = copy.deepcopy(x0)
xs = x0

xtract = copy.deepcopy(x0)


actuator = RobotActuator(actuator_noise)

robot = Robot(x0, A, B)

sensor = RobotSensor([theta1_noise, theta2_noise, theta3_noise, rate1_noise, rate2_noise, rate3_noise])
sensor1 = RobotSensor([theta1_noise1, theta2_noise1, theta3_noise1, rate1_noise1, rate2_noise1, rate3_noise1])
sensors = [sensor, sensor1]

kf = ct.makeLinearKF(A, B, C, P, F, x0[:, -1], 6, 6)
localizer = KalmanPredict('filter', kf_period, kf_deadline, kf_wcet, kf_power, kf, Rs)
lqr = LQRRobot('lqr', lqr_period, lqr_deadline, lqr_wcet, lqr_power, K)
lqr1 = LQRRobot('lqr1', lqr_period, lqr_deadline, lqr_wcet, lqr_power/10, K1)

queue = [localizer, lqr]
rtos = RTOS()
for task in queue:
    rtos.create_task(task)
fail_rate = 3.1706e-09
reliability_model = TAAF(fail_rate)
processor = Processor(rtos, reliability_model)
processor2 = copy.deepcopy(processor)
processor3 = copy.deepcopy(processor)
processor4 = copy.deepcopy(processor)
processor5 = copy.deepcopy(processor)
processor6 = copy.deepcopy(processor)
processor7 = copy.deepcopy(processor)
processor8 = copy.deepcopy(processor)
processor9 = copy.deepcopy(processor)
processor10 = copy.deepcopy(processor)

processor_list = [copy.deepcopy(processor) for i in range(3)]

cyber = RobotCyberSystem(processor_list)

# cyber = RobotCyberSystem([processor, processor2, processor3, processor4])

end = 5

# cps = RobotCPS(robot, cyber, sensors, actuator, end=end)
#
# cps.run()

for i in range(1):
    # x0 = np.array([[random.uniform(-1.0, 1.0)],
    #            [random.uniform(-0.1, 0.1)],
    #            [random.uniform(-0.1, 0.1)],
    #            [random.uniform(-0.1, 0.1)],
    #            [random.uniform(-0.1, 0.1)],
    #            [random.uniform(-0.1, 0.1)]])

    x1 = np.array([[-0.45],
                   [0.087],
                   [-0.087],
                   [-0.087],
                   [-0.087],
                   [-0.087]])

    # version2_power_list = [1., 3., 5., 7., 9.]
    # version2_wcet_list = [0.001, 0.002, 0.003, 0.004, 0.005]
    # num_processors_list = [3, 4, 5, 6]
    # QoC_list = [0.15, 0.35, 0.55, 0.75]

    version2_power_list = [6.]
    version2_wcet_list = [0.002]
    num_processors_list = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
    QoC_list = [0.15]

    dict_list = []
    count = 0

    for version2_power in version2_power_list:
        for version2_wcet in version2_wcet_list:
            for version2_et in np.arange(0.001,version2_wcet + 0.001,0.001):
                for num_processors in num_processors_list:
                    for QoC in QoC_list:

                        x0 = copy.deepcopy(x1)
                        robot = Robot(x0, A, B)
                        processor_list = [copy.deepcopy(processor) for i in range(num_processors)]
                        cyber = RobotCyberSystem(processor_list)
                        cyber.power_version2 = version2_power
                        cyber.wcet_version2 = version2_wcet
                        cyber.et_version2 = version2_et
                        cyber.QoC = QoC
                        cyber.adaft_on = False
                        cyber.dvfs_on = False
                        cps = RobotCPS(robot, cyber, sensors, actuator, end=end)
                        cps.run()
                        mttf = cps.mttf[cps.mttf != 0]
                        mttf_all_on = np.mean(mttf)
                        energy_all_on = cps.energy[cps.energy != 0]
                        energy_all_on_final = energy_all_on[-1]


                        x0 = copy.deepcopy(x1)
                        robot = Robot(x0, A, B)
                        processor_list = [copy.deepcopy(processor) for i in range(num_processors)]
                        cyber = RobotCyberSystem(processor_list)
                        cyber.power_version2 = version2_power
                        cyber.wcet_version2 = version2_wcet
                        cyber.et_version2 = version2_et
                        cyber.QoC = QoC
                        cyber.adaft_on = False
                        cyber.dvfs_on = True
                        cps = RobotCPS(robot, cyber, sensors, actuator, end=end)
                        cps.run()
                        mttf = cps.mttf[cps.mttf != 0]
                        mttf_dvfs = np.mean(mttf)
                        energy_dvfs = cps.energy[cps.energy != 0]
                        energy_dvfs_final = energy_dvfs[-1]


                        x0 = copy.deepcopy(x1)
                        robot = Robot(x0, A, B)
                        processor_list = [copy.deepcopy(processor) for i in range(num_processors)]
                        cyber = RobotCyberSystem(processor_list)
                        cyber.power_version2 = version2_power
                        cyber.wcet_version2 = version2_wcet
                        cyber.et_version2 = version2_et
                        cyber.QoC = QoC
                        cyber.adaft_on = True
                        cyber.dvfs_on = True
                        cps = RobotCPS(robot, cyber, sensors, actuator, end=end)
                        cps.run()
                        mttf = cps.mttf[cps.mttf != 0]
                        mttf_adaft_dvfs = np.mean(mttf)
                        energy_adaft_dvfs = cps.energy[cps.energy != 0]
                        energy_adaft_dvfs_final = energy_adaft_dvfs[-1]


                        x0 = copy.deepcopy(x1)
                        robot = Robot(x0, A, B)
                        processor_list = [copy.deepcopy(processor) for i in range(num_processors)]
                        cyber = RobotCyberSystem(processor_list)
                        cyber.power_version2 = version2_power
                        cyber.wcet_version2 = version2_wcet
                        cyber.et_version2 = version2_et
                        cyber.QoC = QoC
                        cyber.adaft_on = True
                        cyber.dvfs_on = False
                        cps = RobotCPS(robot, cyber, sensors, actuator, end=end)
                        cps.run()
                        mttf = cps.mttf[cps.mttf != 0]
                        mttf_adaft = np.mean(mttf)
                        energy_adaft = cps.energy[cps.energy != 0]
                        energy_adaft_final = energy_adaft[-1]


                        one_dict = {'Version 1 Power': 10, 'Version 1 Execution Time' : 0.004, 'Version 1 WCET': 0.008, 'Version 2 Power': version2_power,
                                    'Version 2 Execution Time' : version2_et, 'Version 2 WCET': version2_wcet, 'Number of Processors': num_processors,
                                    'QoC Constraint': QoC, 'MTTF All On': mttf_all_on, 'MTTF AdaFT': mttf_adaft, 'MTTF DVFS': mttf_dvfs, 'MTTF AdaFT DVFS': mttf_adaft_dvfs,
                                    'Energy All On' : energy_all_on_final, 'Energy AdaFT' : energy_adaft_final, 'Energy DVFS' : energy_dvfs_final, 'Energy AdaFT DVFS' : energy_adaft_dvfs_final}
                        dict_list.append(one_dict)
                        print(one_dict)
                        print(count)
                        count += 1
                        # print('MTTF:', np.mean(mttf))
                        # pickle.dump(cps.cyber_sys.QLearning, open(os.path.join('./Cyber/', 'QLearningRobot_general.pkl'), 'wb'))
                        # time.sleep(2)

    df = pd.DataFrame(dict_list)
    df.to_csv('./Data/HumanRobot/hardware_provisioning.csv')

print("Total number of tasks released: ", cps.cyber_sys.processors[0].rtos.n)
print("Missed tasks: ", cps.cyber_sys.processors[0].rtos.missed_task)

xtract = cps.xtract
xs = cps.xs

taaf = cps.taaf[cps.taaf != 0]
temp = cps.temp[cps.temp != 0]
mttf = cps.mttf[cps.mttf != 0]

print('MTTF:', np.mean(mttf))
print('lqr power:', cps.cyber_sys.processors[0].rtos.task_list['lqr'].power)
print('kf power:', cps.cyber_sys.processors[0].rtos.task_list['filter'].power)

copies = cps.copies
version = cps.version

# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[0, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[0, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Theta 1 (rad)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[1, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[1, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Theta 2 (rad)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[2, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[2, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Theta 3 (rad)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[3, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[3, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Rate 1 (rad/s)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[4, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[4, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Rate 2 (rad/s)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[5, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[5, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('Rate 3 (rad/s)', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)
#
plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(xtract[0, :], c = 'm', label='Kalman Filter Theta 1', linewidth=3)
plt.plot(xtract[1, :], c = 'c', label='Kalman Filter Theta 2', linewidth=3)
plt.plot(xtract[2, :], c = 'black', label='Kalman Filter Theta 3', linewidth=3)
plt.plot(xs[0, :], c = 'r', label='Theta 1', linewidth=1)
plt.plot(xs[1, :], c = 'g', label='Theta 2', linewidth = 1)
plt.plot(xs[2, :], c = 'b', label='Theta 3', linewidth = 1)
plt.xlabel('Time (ms)', fontsize=18)
plt.ylabel('Angle (rad)', fontsize=18)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(loc=4)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(xtract[3, :], c = 'm', label='Kalman Filter Rate 1', linewidth=3)
plt.plot(xtract[4, :], c = 'c', label='Kalman Filter Rate 2', linewidth=3)
plt.plot(xtract[5, :], c = 'black', label='Kalman Filter Rate 3', linewidth=3)
plt.plot(xs[3, :], c = 'r', label='Rate 1', linewidth=1)
plt.plot(xs[4, :], c = 'g', label='Rate 2', linewidth = 1)
plt.plot(xs[5, :], c = 'b', label='Rate 3', linewidth = 1)
plt.xlabel('Time (ms)', fontsize=18)
plt.ylabel('Rate (rad)', fontsize=18)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(loc=4)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(copies[0, :], c = 'r', label='Copies', linewidth=2)
plt.xlabel('Time (ms)', fontsize=18)
plt.ylabel('Copies', fontsize=18)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(loc=4)

# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(version[0, :], c = 'r', label='version', linewidth=2)
# plt.xlabel('Time (ms)', fontsize=18)
# plt.ylabel('version', fontsize=18)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.legend(loc=4)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(taaf, c = 'r', label='TAAF', linewidth=2)
plt.xlabel('Time (ms)', fontsize=18)
plt.ylabel('TAAF (1x)', fontsize=18)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(loc=4)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(temp, c = 'r', label='Temperature', linewidth=2)
plt.xlabel('Time (ms)', fontsize=18)
plt.ylabel('Temperature (K)', fontsize=18)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(loc=4)


plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(mttf, c = 'r', label='MTTF', linewidth=2)
plt.xlabel('Time (ms)', fontsize=18)
plt.ylabel('MTTF (Years)', fontsize=18)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.legend(loc=4)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(energy_all_on, '-m', label='All On', linewidth=4, )
plt.plot(energy_adaft, '-c', label='AdaFT', linewidth=4, )
plt.plot(energy_dvfs, '-b', label='DVFS', linewidth=2, )
plt.plot(energy_adaft_dvfs, '-r', label='AdaFT DVFS', linewidth=2, )
plt.xlabel('Time (ms)', fontsize=30)
plt.ylabel('Energy (joule)', fontsize=30)
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.legend(loc=4, fontsize=30)

plt.show()



# np.savetxt('./Data/HumanRobot/xtrack_vmix_ada_rf1.csv', xtract, delimiter=',')
# np.savetxt('./Data/HumanRobot/xs_vmix_ada_rf1.csv', xs, delimiter=',')
# np.savetxt('./Data/HumanRobot/copies_vmix_ada_rf1.csv', copies, delimiter=',')
# np.savetxt('./Data/HumanRobot/temp_vmix_ada_rf1.csv', temp, delimiter=',')
# np.savetxt('./Data/HumanRobot/taaf_vmix_ada_rf1.csv', taaf, delimiter=',')




