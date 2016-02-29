from Task.ParticleFilter import ParticleFilter
import numpy as np


range = np.array([[0. , 10.], [1., 5.]])
a_noise= 1.
s_noise = np.array([[1.], [1.]])

def state_update(x, h, u, noise):
    x[0] += u
    x[1] = x[1]
    return x


h = 0.01

guess_std = np.array([[1.], [1.]])

pf = ParticleFilter(range, a_noise, s_noise, state_update, h, guess_std=guess_std, init_state_guess=np.array([[2.], [2.]]))
pf2 = ParticleFilter(range, a_noise, s_noise, state_update, h)

print(pf.particles)
# print(pf2.particles)

pf.predict(1)
print(pf.particles)