import networkx as nx
import random


class SubgraphGenerator:
    """Class responsible for generating subgraphs of a base graph
    """

    def __init__(self, base_graph):
        self.base_graph = base_graph

    def generate_random_subgraph_from_node_indices(self, node_indices):
        # type: (list) -> nx.Graph
        """A subgraph is extracted by the base_graph stored in the calling object such that the extracted nodes
        correspond to the ones indicated by the passed array of indices.
        :param node_indices: list of node indices to extract from the base graph.
        :return: nx.Graph containing all nodes specified by the passed argument. Existing edges between nodes are
        retained, while new ones might be added in case the resulting nodes don't form a single connected component.
        """
        # extract the node from the graph given their indices
        nodes = [node for node in self.base_graph.nodes(data=False) if int(node) in node_indices]

        # create a subgraph containing those nodes
        subgraph = self.base_graph.subgraph(nodes).copy()
        if not nx.is_connected(subgraph):
            SubgraphGenerator.connect_disconnected_graph(subgraph)

        return subgraph

    def generate_random_subgraph(self, num_nodes):
        # type (int) -> nx.Graph
        """Generates a random subgraph of the base_graph stored in the calling object with num_nodes nodes.
        :param num_nodes: number of nodes to extract from the base_graph.
        :return: Subgraph of base_graph containing num_nodes.
        """
        assert 0 < num_nodes < self.base_graph.number_of_nodes()
        # generate random indices to keep
        node_indices = random.sample(range(0, self.base_graph.number_of_nodes()), num_nodes)
        return self.generate_random_subgraph_from_node_indices(node_indices)

    def generate_subgraph_centered_at_with_radius(self, center_node_id, radius):
        # type (int, int) -> nx.Graph
        """Generates a subgraph of the base_graph stored in the calling object starting at the node indicated by the
        center_node_id and defined by the neighborhood specified by the radius parameter.
        :param center_node_id: Index referring to the node used to define the subgraph.
        :param radius: Maximal edge-distance between extracted nodes and central node.
        :return: Subgraph of base_graph centered ate center_node_id with nodes being at maximal distance of radius to the center node.
        """
        assert 0 <= center_node_id < self.base_graph.number_of_nodes()
        center_node = self.base_graph.nodes()[center_node_id]
        return nx.ego_graph(self.base_graph, n=center_node, radius=radius)

    @staticmethod
    def connect_disconnected_graph(disconnected_graph):
        # type (nx.Graph)
        """Given a disconnected graph, this function connects all disconnected component in a naive way,
        i.e. by adding an edge between the first node of the i-th component to the first node of the (i+1)-th component.
        :param disconnected_graph: Graph presenting disconnected components
        """
        disconnected_components = list(nx.connected_component_subgraphs(disconnected_graph))

        # connect the components
        for i in range(0, len(disconnected_components) - 1):
            disconnected_graph.add_edge(disconnected_components[i].nodes()[0],
                                        disconnected_components[i + 1].nodes()[0])
