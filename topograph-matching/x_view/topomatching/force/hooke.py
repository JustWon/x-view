from . import AbstractForce

import numpy as np


class Hooke(AbstractForce):
    """
    Force class based on Hooke's law, where two entities are attracted towards each other proportionally
    to their relative distance with a constant proportionality factor 'k'
    """

    def __init__(self, k):
        """
        Sets the Hooke's constants
        :param k: Proportionality factor
        """
        super(Hooke, self).__init__()
        self.k = k

    def interact(self, pos1: np.array, pos2: np.array, rest_length=0.0, stiffness=1.0, **kwargs):

        """
        Computes hooke's law based on two points in 2D/3D space
        :param pos1: numpy array specifying position of first entity
        :param pos2: numpy array specifying position of second entity
        :param rest_length: scalar value defining rest_length
        :param stiffness: edge-specific stiffness
        :return: force acting on second entity from first entity
        """
        dist = Hooke.norm(pos1 - pos2)
        off = dist - rest_length
        direction = (pos2 - pos1) / (1.0e-6 + dist)
        return -self.k * stiffness * direction * off
