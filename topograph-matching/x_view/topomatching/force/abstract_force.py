import numpy as np

class AbstractForce(object):
    """Base class of all forces
    """

    def __init__(self, *args, **kwargs):
        pass

    def interact(self, pos1, pos2, mass1, mass2, rest_length, stiffness):
        """Interaction force between two entities"""
        raise NotImplementedError

    @staticmethod
    def norm(v):
        return np.sqrt(np.dot(v, v))
