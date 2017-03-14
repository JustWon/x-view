from . import AbstractIntegrator

import numpy as np


class ExplicitEuler(AbstractIntegrator):
    """Implements an explicit euler integration scheme
    """

    def __init__(self, dt=0.01, mu=0.001):
        super(ExplicitEuler, self).__init__(dt=dt, mu=mu)

    def step(self, position_list, force_list):

        if len(position_list) != len(force_list):
            raise IndexError("Positions list size and Forces list size do not agree")

        iterable_check = [isinstance(par, list) for par in [position_list, force_list]]
        if not all(iterable_check):
            raise TypeError("A parameter passed to integrator is not iterable")

        if len(self.velocity_list) == 0:
            self.velocity_list = [[np.zeros(len(position_list[0][0])) for i in positions] for positions in
                                  position_list]

        for positions, forces, velocities in zip(position_list, force_list, self.velocity_list):
            for pos, force, vel in zip(positions, forces, velocities):
                vel = (1.0 - self.mu * self.dt) * (vel + force * self.dt)
                pos += vel * self.dt
