import numpy as np
import itertools

from . import Hooke, AbstractForce



class ForceManager(object):
    def __init__(self, topograph_list, node_list, dict_list, inner_force=Hooke(k=10), outer_force=Hooke(k=20), dim=3):
        """Initializes the force manager class by generating a force list
        :param topograph_list: a python list of topographs
        :param node_list: a python list (usually of size 2 if there are 2 graphs in the scene)
        each containing a list with graph nodes
        :param dict_list: a list of dictionary lists containing parameters of nodes
        :param inner_force: an instance of class AbstractForce which will be called to compute pairwise interactions
        between nodes belonging to the same graph
        :param outer_force: an instance of a class AbstractForce which will be called to compute pairwise interactions
        between nodes belonging to different graphs
        :param dim: dimension in which the scene takes place (either 2D or 3D)
        """
        self.topograph_list = topograph_list
        self.node_list = node_list
        self.dict_list = dict_list
        if len(self.node_list) != len(self.dict_list):
            raise IndexError("node_list adn dict_list have size mismatch")
        for nodes, dicts in zip(self.node_list, self.dict_list):
            if len(nodes) != len(dicts):
                raise IndexError("node_list and dict_list have a size mismatch in internal size")

        if not isinstance(inner_force, AbstractForce) or not isinstance(outer_force, AbstractForce):
            raise TypeError(
                "force objects passed to '{}' are not inherited by 'AbstractForce".format(self.__class__.__name__))
        self.inner_force = inner_force
        self.outer_force = outer_force
        self.dim = dim
        self.force_list = [np.zeros((len(nodes), self.dim)) for nodes in node_list]

    def reset(self):
        self.force_list = [np.zeros((len(forces), self.dim)) for forces in self.force_list]


    def compute_forces(self):
        self.compute_inner_forces()
        self.compute_outer_forces()

    def compute_inner_forces(self):
        # iterate through the list of topographs and compute inner forces
        for topograph_index, (topograph, dicts, nodes) in enumerate(
                zip(self.topograph_list, self.dict_list, self.node_list)):
            # loop over all possible combinations of nodes represented in the current list of dictionaries
            for ((idx1, (dict1, node1)), (idx2, (dict2, node2))) in itertools.combinations(enumerate(zip(dicts, nodes)),
                                                                                           2):
                # only interact if there is an edge between the two nodes
                if node1 in topograph[node2]:
                    pos1, pos2 = dict1['pos'], dict2['pos']
                    rest_length = topograph[node1][node2]['rest_length']
                    stiffness = topograph[node1][node2]['stiffness']
                    f12 = self.inner_force.interact(pos1=pos1, pos2=pos2, rest_length=rest_length, stiffness=stiffness)
                    self.force_list[topograph_index][idx1, :] -= f12
                    self.force_list[topograph_index][idx2, :] += f12

    def compute_outer_forces(self):
        # iterate over pairs of topographs and compute outer forces
        for ((topo1_idx, (topo1, dicts1, nodes1)), (topo2_idx, (topo2, dicts2, nodes2))) in \
                itertools.combinations(enumerate(zip(self.topograph_list, self.dict_list, self.node_list)), 2):
            # iterate over all nodes in nodes1
            for idx1, (node1, dict1) in enumerate(zip(nodes1, dicts1)):
                for idx2, (node2, dict2) in enumerate(zip(nodes2, dicts2)):
                    # compute semantic graph_node
                    sim12 = node1.similarity(node2)
                    sim12 = 0 if sim12 < 0.7 else sim12
                    pos1, pos2 = dict1['pos'], dict2['pos']

                    # scale the force based on graph_node
                    f12 = self.outer_force.interact(pos1=pos1, pos2=pos2, rest_length=0, stiffness=sim12)

                    self.force_list[topo1_idx][idx1, :] -= f12
                    self.force_list[topo2_idx][idx2, :] += f12
