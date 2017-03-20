import numpy as np
from .topograph import TopoGraph


class IdFactory(object):
    """IdFactory class used to generate unique ids for the graph nodes. This is necessary as
    when merging two graphs together, even if they have the same node we don't want the nodes to collapse together
    """
    __instance = None
    __current_id = 0

    def __new__(cls, *args, **kwargs):
        if not cls.__instance:
            cls.__instance = super(IdFactory, cls).__new__(cls, *args, **kwargs)
        return cls.__instance

    def next_id(self):
        """Generates a unique id each time it is called
        :return: a unique id (sequentially increasing) whenever this function is called
        """
        self.__current_id += 1
        return self.__current_id


class NodeEntity(object):
    """Each TopoGraph node must be an instance of this class, as it is associated to a unique id
    when constructed.
    """

    def __init__(self, value):
        """Constructs an object which contains 'value' and associates a unique integer id 'node_id' which is needed to
        have a unique hash in the networkx structure
        :param value: any object one wants to store in the networkx structure
        """
        self.value = value
        self.node_id = IdFactory().next_id()


class AbstractGraphNode(object):
    """Class exposed to the user which must be used to add new nodes
    to the graph structure represented by TopoGraph"""

    def __init__(self, node, pos: np.array, label="node", lemma="", plot_pos=None, node_color='42C2ED'):
        """Simply constructs a node used to generate internal representation of TopoGraph
        :param node: real entity stored as node in the graph structure
        :param pos: 2D/3D numpy array of spatial position of node
        :param label: text label to be plotted
        :param lemma: semantic representation of the node
        :param plot_pos: optional 2D numpy array, if available will be used for plotting instead than position[0:2]
        :param node_color: optional, if available the associated node will be plotted in that color
        """
        if not isinstance(node, NodeEntity):
            raise TypeError("node parameter passed to '{}' constructor must be an instance"
                            "of 'NodeEntity' (or be an instance of a subclass of 'NodeEntity'".format(self.__class__.__name__))
        self.node = node

        self.pos = pos
        assert isinstance(label, str)
        self.label = label
        self.lemma = lemma

        self.plot_pos = plot_pos if plot_pos is not None else pos[0:2]
        self.node_color = node_color

    def similarity(self, other):
        raise NotImplementedError("Each GraphNode object must implement the 'similarity' method")


class GraphEdge(object):
    """Class exposed to the user which must be used to add edges between the nodes in the graph structure
    """

    def __init__(self, node1: AbstractGraphNode, node2: AbstractGraphNode, stiffness=1.0, rest_length=1.0, edge_color='#125E87'):
        """Simply constructs an edge in the graph structure linking node1 and node2
        """

        if not isinstance(node1, AbstractGraphNode) or not isinstance(node2, AbstractGraphNode):
            raise TypeError("Nodes passed to 'GraphEdge' constructor must be instances of class 'AbstractGraphNode'")

        self.node1 = node1
        self.node2 = node2

        self.stiffness = stiffness
        self.rest_length = rest_length
        self.edge_color = edge_color


def generate_topograph(graph_node_list, graph_edge_list, name="TopoGraph"):
    if not all(isinstance(graph_node, AbstractGraphNode) for graph_node in graph_node_list):
        raise TypeError("All elements in 'graph_node_list' must be of type 'GraphNode'")

    if not all(isinstance(graph_edge, GraphEdge) for graph_edge in graph_edge_list):
        raise TypeError("All elements in 'graph_edge_list' must be of type 'GraphEdge'")

    G = TopoGraph(name=name)

    for graph_node in graph_node_list:
        G.add_node(n=graph_node, pos=graph_node.pos, label=graph_node.label, lemma=graph_node.lemma,
                   plot_pos=graph_node.plot_pos, node_color=graph_node.node_color)

    for graph_edge in graph_edge_list:
        G.add_edge(u=graph_edge.node1, v=graph_edge.node2, stiffness=graph_edge.stiffness,
                   rest_length=graph_edge.rest_length, edge_color=graph_edge.edge_color)

    return G
