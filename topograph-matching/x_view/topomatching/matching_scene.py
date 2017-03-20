from ..topograph import TopoGraph
from . import AbstractForce, AbstractIntegrator, WordnetForceManager, RestrictedForceManager


class MatchingScene(object):
    """Class handling the simulation of the matching scene
    """

    def __init__(self, topograph_list, integrator, inner_force, outer_force, dim=3):
        if not all(isinstance(topograph, TopoGraph) for topograph in topograph_list):
            raise TypeError("'topograph_list' parameter passed to 'MatchingScene' must be a list of TopoGraph objects")
        self.topograph_list = topograph_list

        if not isinstance(integrator, AbstractIntegrator):
            raise TypeError("integrator parameter passed to MatchingScene is not derived from 'AbstractIntegrator'")
        self.integrator = integrator

        if not (isinstance(inner_force, AbstractForce)) or not (isinstance(outer_force, AbstractForce)):
            raise TypeError("force parameter passed to MatchingScene is not derived from 'AbstractForce'")
        self.inner_force = inner_force
        self.outer_force = outer_force

        self.dim = dim

        self.node_list = []
        self.dict_list = []
        for topograph in topograph_list:
            nodes, dicts = zip(*topograph.nodes(data=True))
            self.node_list.append(nodes)
            self.dict_list.append(dicts)

        self.force_manager = RestrictedForceManager(topograph_list=self.topograph_list, node_list=self.node_list,
                                                 dict_list=self.dict_list, inner_force=self.inner_force,
                                                 outer_force=self.outer_force, dim=self.dim)

    def step(self):

        self.force_manager.reset()
        self.force_manager.compute_forces()

        # create a position_list to be passed to the integrator
        position_list = []
        for dicts in self.dict_list:
            positions = [dic['pos'] for dic in dicts]
            position_list.append(positions)

        self.integrator.step(position_list=position_list, force_list=self.force_manager.force_list)
