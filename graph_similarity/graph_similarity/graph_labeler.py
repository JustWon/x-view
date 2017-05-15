import networkx as nx
import random
from . import GraphProperties


class GraphLabeler:
    """Class containing functions which add node properties to a graph
    """

    @staticmethod
    def add_unique_node_identifiers(graph):
        # type (nx.Graph) -> None
        """Adds a unique identifier (integer) to each node of the graph. This property is keyed in the node
        attributes keyed with 'GraphProperties.node_unique_identifier_key
        """
        graph_nodes = graph.nodes(data=True)
        for i, node in enumerate(graph_nodes):
            node[1][GraphProperties.node_unique_identifier_key] = i

    @staticmethod
    def add_graph_membership(graph, graph_id):
        # type (nx.Graph, int) -> None
        """Adds a graph membership id to each node, such that each node belonging to the same graph is tagged with
        the same id. This allows to distinguish nodes in a subgrpah from nodes belonging to an other subgraph.
        """
        graph_nodes = graph.nodes(data=True)
        for node in graph_nodes:
            node[1][GraphProperties.graph_membership_key] = graph_id

    @staticmethod
    def add_random_semantic_labels(graph, semantic_label_list):
        # type (nx.Graph) -> None
        """Adds a semantic label and a semantic label index to each node of the graph. This property is keyed in the
        node attributes keyed with 'GraphProperties.node_semantic_label_key' and
        'GraphProperties.node_semantic_label_id_key' respectively.
        :param semantic_label_list: list of semantic labels to associate to each node.
        """
        graph_nodes = graph.nodes(data=True)
        num_labels = len(semantic_label_list)
        for node in graph_nodes:
            label_index = random.randint(0, num_labels - 1)
            label = semantic_label_list[label_index]
            node[1][GraphProperties.node_semantic_label_key] = label
            node[1][GraphProperties.node_semantic_label_id_key] = label_index

    @staticmethod
    def add_drawing_position(graph):
        # type (nx.Graph) -> None
        """Adds a drawing position (2D coordinate) to each node of the graph. This property is keyed in the node
        attributes keyed with 'GraphProperties.node_drawing_position_key
        """
        positions = nx.spring_layout(graph)
        graph_nodes = graph.nodes(data=True)
        for node in graph_nodes:
            node[1][GraphProperties.node_drawing_position_key] = positions[node[0]]
