import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scipy as sp
from Task.TaskModel import TaskModel
from Task.CustomTasks import ABS, ACC, KalmanPredict, LQRInvPen, KalmanLQR, NoABS, ABSPF
from OS.RTOS import RTOS
from Processor.Processor import Processor
from Reliability.CustomReliabilityModel import TAAF
from numpy.random import randn
import book_plots as bp
import Task.CustomTasks as ct
from Sensor.CustomSensor import InvPenSensor
from Actuator.CustomActuator import InvPenActuator
from Physical.CustomPhysicalSystem import InvPenDynamics

from Sensor.CustomSensor import ABSSensor
from Actuator.CustomActuator import ABSActuator, NoABSActuator
from Physical.CustomPhysicalSystem import VehicleABSDynamics
from Task.ParticleFilter import ABSParticleFilter, ParticleFilter

# ### Inverted Pendulum test  ###
# h = 0.001
# kf_period = 0.01
# kf_deadline = kf_period
# kf_wcet = 0.001
# kf_power = 3.5
#
# lqr_pen_period = 0.02
# lqr_pen_deadline = lqr_pen_period
# lqr_pen_wcet = 0.001
# lqr_pen_power = 6.5
#
# actuator_noise = 0.06
#
# cart_pos_noise = 0.05
# cart_vel_noise = 0.05
# angle_noise = 0.01
# rate_noise = 0.01
# cart_pos_noise2 = 0.005
# cart_vel_noise2 = 0.05
# angle_noise2 = 0.01
# rate_noise2 = 0.01
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
# R = np.array([[cart_pos_noise, 0., 0., 0.],
#               [0., cart_vel_noise, 0., 0.],
#               [0., 0., angle_noise, 0.],
#               [0., 0., 0., rate_noise]])
# R2 = np.array([[cart_pos_noise2, 0., 0., 0.],
#               [0., cart_vel_noise2, 0., 0.],
#               [0., 0., angle_noise2, 0.],
#               [0., 0., 0., rate_noise2]])
#
# Rs = {'sensor1' : R, 'sensor2' : R2}
#
# F = 1 + kf_period * (A - np.dot(B, K))
#
# x0 = np.array([[0.0],
#                [0.0],
#                [0.4],
#                [0.5]])
# xs = x0
#
# xtract = np.array([[0.0],
#                    [0.0],
#                    [0.0],
#                    [0.0]])
#
#
# actuator = InvPenActuator(actuator_noise)
#
# pen = InvPenDynamics(x0, A, B)
#
# sensor = InvPenSensor([cart_pos_noise, cart_vel_noise, angle_noise, rate_noise])
# sensor2 = InvPenSensor([cart_pos_noise2, cart_vel_noise2, angle_noise2, rate_noise2])
#
# kf = ct.makeLinearKF(A, B, C, P, F, x0[:, -1])
# localizer = KalmanPredict('kalman', kf_period, kf_deadline, kf_wcet, kf_power, kf, Rs)
# lqr = LQRInvPen('lqr', lqr_pen_period, lqr_pen_deadline, lqr_pen_wcet, lqr_pen_power, K)
# kalman_lqr = KalmanLQR('kalman_lqr', lqr_pen_period, lqr_pen_deadline, lqr_pen_wcet, lqr_pen_power, kf, Rs, K)
#
# queue = [localizer, lqr]
# queue2 = [kalman_lqr]
# rtos = RTOS()
# fail_rate = 3.1706e-09
# reliability_model = TAAF(fail_rate)
# processor = Processor(rtos, reliability_model)
# for task in queue:
#     rtos.create_task(task)
#
#
# end = 10
# taaf = np.zeros([end / h, 1])
# time = np.arange(0, end, h)
# taaf[0] = 1
# mttf = np.zeros([end / h, 1])
# mttf[0] = 10
#
#
# i = 0
# for clock in np.arange(0, end, h):
#
#     z = sensor.sense(pen.x)
#     z2 = sensor2.sense(pen.x)
#     all_z = {'sensor1' : z[:, -1], 'sensor2' : z2[:, -1]}
#     xs = np.append(xs, pen.x, axis=1)
#     # processor.rtos.task_inputs[lqr.name] = pen.x
#     processor.rtos.task_inputs[lqr.name] = np.reshape(processor.rtos.task_outputs[localizer.name], (-1, 1))
#     # processor.rtos.task_inputs[lqr.name] = z2
#     processor.rtos.task_inputs[localizer.name] = all_z
#     processor.update(h, clock)
#     actuator_commands = actuator.actuator_commands(processor.rtos.task_outputs[lqr.name])
#     taaf[i] = processor.reliability_model.taaf
#     mttf[i] = processor.reliability_model.mttf_in_years
#     i+=1
#
#     pen.update(h, clock, actuator_commands)
#
#     x_predict = np.reshape(processor.rtos.task_outputs[localizer.name], (-1, 1))
#     xtract = np.append(xtract, x_predict, axis=1)
#
# print(processor.rtos.n)
# print(processor.rtos.missed_task)
#
# # np.savetxt('use_kalman.csv', u_from_task, delimiter=',')
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[2, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[2, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)')
# plt.ylabel('Angle (rad)')
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[3, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[3, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)')
# plt.ylabel('Angular Rate (rad/s)')
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[0, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[0, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)')
# plt.ylabel('Cart pos')
# plt.legend(loc=4)
#
# plt.figure()
# plt.hold(True)
# plt.grid(True)
# plt.plot(xtract[1, :], c = 'b', label='Kalman Filter', linewidth=2)
# plt.plot(xs[1, :], c= 'r', label='True value', linewidth = 1)
# plt.xlabel('Time (ms)')
# plt.ylabel('Cart vel')
# plt.legend(loc=4)
# plt.show()
#
# f, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
# ax1.plot(taaf)
# ax1.set_title('taaf')
# ax2.plot(mttf)
# ax2.set_title('mttf in years')
#
# plt.show()


######### Car ABS  #########

### Define parameters
h = 0.001
x0 = np.array([[33.], [33.], [0.]])
xs = x0
mass_quater_car = 250
mass_effective_wheel = 20
road_friction_coeff = 1.0

actuator_noise = 0
sensor_vehicle_speed_noise = 0.5 # don't use a very small value, which will lead to poor particle filter performance
sensor_wheel_speed_noise = 0.5
sensor_pos_noise = 0.5
all_sensor_noise_levels = {'sensor1' : np.array([sensor_vehicle_speed_noise, sensor_wheel_speed_noise, sensor_pos_noise])}

period = 0.02
deadline = period
wcet = 0.001
power = 3.5

period_pf = 0.02
deadline_pf = period_pf
wcet_pf = 0.001
power_pf = 6.5
N = 1000


### Create and Intialize Objects
car = VehicleABSDynamics(x0)
car_sensor = ABSSensor([sensor_vehicle_speed_noise, sensor_wheel_speed_noise, sensor_pos_noise])
car_actuator = ABSActuator(actuator_noise)
car_actuator_no_abs = NoABSActuator(actuator_noise)
abs = ABS('abs', period, deadline, wcet, power)

range = np.array([[0., 80.], [0., 80.], [0., 80.]])
std = np.array([1, 1, 1])
pf = ABSParticleFilter(road_friction_coeff, mass_quater_car, mass_effective_wheel, range, all_sensor_noise_levels, period_pf,
                       std, init_state_guess= x0, N=N)
localizer = ABSPF('pf', period_pf, deadline_pf, wcet_pf, power_pf, pf)

xtract = x0

queue = [abs, localizer]
rtos = RTOS()

fail_rate = 3.1706e-09
reliability_model = TAAF(fail_rate)
processor = Processor(rtos, reliability_model)
for task in queue:
    rtos.create_task(task)

### Simulation Parameters
end = 10
taaf = np.zeros([end / h, 1])
time = np.arange(0, end, h)
taaf[0] = 1
mttf = np.zeros([end / h, 1])
mttf[0] = 10
u = []
old_estimations = x0

### Run Simulations
i = 0
for clock in np.arange(0, end, h):
    if car.x[0] <= 1:
        break

    z = car_sensor.sense(car.x)
    xs = np.append(xs, car.x, axis=1)
    localizer_inputs = {'control' : processor.rtos.task_outputs[abs.name],
                        'sensor' : {'sensor1' : z}}
    processor.rtos.task_inputs[abs.name] = processor.rtos.task_outputs[localizer.name]
    processor.rtos.task_inputs[localizer.name] = localizer_inputs
    processor.update(h, clock)
    actuator_commands = car_actuator.actuator_commands(processor.rtos.task_outputs[abs.name])
    # actuator_commands = car_actuator_no_abs.actuator_commands(processor.rtos.task_outputs[abs.name])
    u.append(actuator_commands)
    taaf[i] = processor.reliability_model.taaf
    mttf[i] = processor.reliability_model.mttf_in_years
    i+=1

    car.update(h, clock, actuator_commands)

    x_predict = np.reshape(processor.rtos.task_outputs[localizer.name], (-1, 1))
    xtract = np.append(xtract, x_predict, axis=1)

taaf = taaf[taaf != 0]
mttf = mttf[mttf != 0]


### Plot Results
plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(xtract[0, :], c = 'c', label='Particle Filter Vehicle Speed', linewidth=4)
plt.plot(xtract[1, :], c = 'g', label='Particle Filter Wheel Speed', linewidth=4)
plt.plot(xs[0, :], c = 'b', label='Vehicle Speed', linewidth=1)
plt.plot(xs[1, :], c= 'r', label='Wheel Speed', linewidth = 1)
plt.xlabel('Time (ms)')
plt.ylabel('speed m/s')
plt.legend(loc=1)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(xs[2, :], c = 'b', label='distance', linewidth=2)
plt.xlabel('Time (ms)')
plt.ylabel('Stop Dis (m)')
plt.legend(loc=4)

plt.figure()
plt.hold(True)
plt.grid(True)
plt.plot(u, c = 'b', label='control', linewidth=2)
plt.xlabel('Time (ms)')
plt.ylabel('control actuator')
plt.legend(loc=4)

f, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
ax1.plot(taaf)
ax1.set_title('taaf')
ax2.plot(mttf)
ax2.set_title('mttf in years')

plt.show()
