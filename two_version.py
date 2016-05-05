import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

A = np.array([[0., 0., 0., 1., 0., 0.],
              [0., 0., 0., 0., 1., 0.],
              [0., 0., 0., 0., 0., 1.],
              [18.337, -75.864, 6.395, 0., 0., 0.],
              [-22.175, 230.549, -49.01, 0., 0., 0.],
              [4.353, -175.393, 95.29, 0., 0., 0.]])

B = np.array([[0., 0., 0.],
              [0., 0., 0.],
              [0., 0., 0.],
              [0.292, -0.785, 0.558],
              [-0.785, 2.457, -2.178],
              [0.558, -2.178, 2.601]])

C = np.eye(6)

K = np.array([[1054.367, 426.901, 153.864, 365.784, 173.577, 67.28],
              [707.669, 610.181, 251.283, 263.836, 158.583, 72.18],
              [12.129, 43.669, 132.469, 11.613, 16.447, 19.12]])

K1 = np.array([[929.79, 322.082, 73.883, 317.782, 142.912, 49.865],
               [521.09, 431.75, 74.787, 188.568, 105.95, 38.039],
               [277., 239.326, 202.902, 108.11, 70.849, 42.213]])

time = 5.
step = 0.001

x0 = np.array([[-0.45],
               [0.087],
               [-0.087],
               [-0.087],
               [-0.087],
               [-0.087]])

x = np.zeros((time / step, 6))
x[0, :] = x0.T

p_u1 = 10
p_u2 = 10
p_u3 = 20

u = np.zeros((3, 1))
new_u1 = -np.dot(K, x0)
new_u2 = -np.dot(K1, x0)

new_u = np.array([[new_u1[0][0]], [new_u2[1][0]], [new_u2[2][0]]])

i = 0
for t in np.arange(0, time - step, step):
    x_dot = np.dot(A, x[i, :].reshape(6, 1)) + np.dot(B, u)
    new_x = x[i, :].reshape(6, 1) + step * x_dot

    if i % p_u1 == 0:
        control = -np.dot(K, x[i, :].reshape(6, 1))
        u[0][0] = new_u[0][0]
        new_u[0][0] = control[0][0]
        if i % p_u2 == 0:
            control1 = -np.dot(K1, x[i, :].reshape(6, 1))
            u[1][0] = new_u[1][0]
            new_u[1][0] = control1[1][0]

        if i % p_u3 == 0:
            control1 = -np.dot(K1, x[i, :].reshape(6, 1))
            u[2][0] = new_u[2][0]
            new_u[2][0] = control1[2][0]

    i += 1
    x[i, :] = new_x.T



plt.figure()
plt.hold(True)
plt.grid(True)

plt.plot(x[:, 0], '-r')
plt.plot(x[:, 1], '-g')
plt.plot(x[:, 2], '-b')


plt.figure()
plt.hold(True)
plt.grid(True)

plt.plot(x[:, 3], '-y')
plt.plot(x[:, 4], '-k')
plt.plot(x[:, 5], '-m')

plt.show()