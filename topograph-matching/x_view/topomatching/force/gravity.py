from . import AbstractForce

import numpy as np


class Gravity(AbstractForce):
    """
    Force class based on Gravitational law, where two entities are attracted towards each other proportionally
    to their relative iverse squared distance with a constant proportionality factor 'G'
    """

    def __init__(self, G):
        """
        Sets the Gravitational constants
        :param G: Proportionality factor
        """
        super(Gravity, self).__init__()
        self.G = G

    def interact(self, pos1: np.array, pos2: np.array, mass1=1.0, mass2=1.0, **kwargs):
        """
        Computes gravitational's law based on two points in 2D/3D space
        :param pos1: numpy array specifying position of first entity
        :param pos2: numpy array specifying position of second entity
        :param mass1: scalar value indicating the first entity mass
        :param mass2: scalar value indicating the second entity mass
        :return: force acting on second entity from first entity
        """
        dist = Gravity.norm(pos1 - pos2)
        direction = (pos1 - pos2) / (1.0e-6 + dist)
        return self.G * mass1 * mass2 / dist ** 2 * direction
