import numpy as np

class AbstractGraphNode(object):
    """Class exposed to the user which must be used to add new nodes
    to the graph structure represented by TopoGraph"""

    def __init__(self, node, pos: np.array, label="node", node_color='42C2ED'):
        """Simply constructs a node used to generate internal representation of TopoGraph
        :param node: real entity stored as node in the graph structure
        :param pos: 2D/3D numpy array of spatial position of node
        :param label: text label to be plotted
        :param node_color: optional, if available the associated node will be plotted in that color
        """

        self.node = node

        self.pos = pos
        assert isinstance(label, str)
        self.label = label

        self.node_color = node_color

    def similarity(self, other):
        raise NotImplementedError("Each GraphNode object must implement the 'similarity' method")
