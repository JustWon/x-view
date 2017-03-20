from . import AbstractGraphNode
from .topograph import TopoGraph


class GraphEdge(object):
    """Class exposed to the user which must be used to add edges between the nodes in the graph structure
    """

    def __init__(self, node1: AbstractGraphNode, node2: AbstractGraphNode, stiffness=1.0, rest_length=1.0,
                 edge_color='#125E87'):
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
    """Builds a networkx graph given the list of graph nodes and edges
    :param graph_node_list: a python list of graph nodes, all instances of classes derived from AbstractGraphNode
    :param graph_edge_list: a python list of graph edges, all instance of class GraphEdge
    :param name: optional graph name
    :return: A topograph containing the nodes linked by edges passed as parameters
    """
    if not all(isinstance(graph_node, AbstractGraphNode) for graph_node in graph_node_list):
        raise TypeError("All elements in 'graph_node_list' must be of type 'AbstractGraphNode'")

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
