class AbstractIntegrator(object):
    """Base class of all integrators
    """

    def __init__(self, dt, mu):
        """
        Initializes the parameters of the integrators
        :param dt: simulation time step
        :param mu: velocity damping
        """
        self.dt = dt
        self.mu = mu

        self.velocity_list = []

    def step(self, position_list, force_list):
        """Integrates the positions and velocities passed as argument using the associated forces
        for a single timestep
        :param position_list: a list of numpy arrays (2D/3D) coordinates referring to the current positions
        :param force_list: a list of numpy arrays(2D/3D) referring to the forces to be applied to the entities
        """
        raise NotImplementedError
