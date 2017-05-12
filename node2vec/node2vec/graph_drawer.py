import networkx as nx
from . import GraphProperties


class GraphDrawer:
    """Class implementing static methods for drawing a graph using the attributes associated to the nodes.
    """

    @staticmethod
    def draw_graph(graph):
        # type (nx.Graph) -> None
        """Draws the graph passed as argument into the current matplotlib figure.
        The position of the graph nodes are the ones associated to the node attributes."""
        nodes = graph.nodes(data=True)
        positions = dict((n[0], n[1][GraphProperties.node_drawing_position_key]) for n in nodes)
        nx.draw(graph, pos=positions)

    @staticmethod
    def draw_graph_node_labels(graph):
        # type (ny.Graph) -> None
        """Renders labels into the current matplotlib figure. The label rendered for each node is the one associated
        to the 'GraphProperties.node_semantic_label_key' key.
        """
        nodes = graph.nodes(data=True)
        positions = dict((n[0], n[1][GraphProperties.node_drawing_position_key]) for n in nodes)
        labels = dict((n[0], n[1][GraphProperties.node_semantic_label_key]) for n in nodes)
        nx.draw_networkx_labels(graph, pos=positions, labels=labels)
