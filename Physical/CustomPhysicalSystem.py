from Physical.PhysicalSystem import PhysicalSystem
import numpy as np


class InvPenDynamics(PhysicalSystem):
    def __init__(self, x0, A, B):
        super().__init__(x0)
        self.A = A
        self.B = B

    def update(self, h, clock, actuator_commands):
        self.u = actuator_commands['lqr']
        self.u = np.reshape(self.u, (-1, 1))
        x_dot = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        self.x += h * x_dot

    def is_safe(self):
        return -0.5 <= self.x[2] <= 0.5


class VehicleABSDynamics(PhysicalSystem):
    def __init__(self, x0, mass_quater_car = 250, mass_effective_wheel = 20, road_friction_coeff = 1.0):
        """
        :param x0: [vehicle speed, wheel speed, vehcile position]
        :param mass_quater_car: kg
        :param mass_effective_wheel: kg
        :param road_friction_coeff:
        """
        super().__init__(x0)
        self.mass_quater_car = mass_quater_car
        self.mass_effective_wheel = mass_effective_wheel
        self.road_friction_coeff = road_friction_coeff

    def update(self, h, clock, actuator_commands):
        actuator_commands = actuator_commands['abs']
        g = 9.81
        v = self.x[0][0]
        w = self.x[1][0]
        x = self.x[2][0]
        slip = self._slip_ratio(v, w)
        friction_force = self.road_friction_coeff * self._mu(slip) * self.mass_quater_car * g
        v = v - h * friction_force / self.mass_quater_car
        x = x + h * v
        w = w + h * (friction_force / self.mass_effective_wheel - actuator_commands)
        w = max(0., w)

        self.x = np.array([[v], [w], [x]])

    def _slip_ratio(self, v, w):
        return max(0., 1. - float(w) / float(v))

    def _mu(self, slip):
        return -1.1 * np.exp(-20 * slip) + 1.1 - 0.4 * slip

    def is_safe(self):
        v = self.x[0][0]
        w = self.x[1][0]
        slip = self._slip_ratio(v, w)
        return slip <= 0.25 and slip >= 0.05