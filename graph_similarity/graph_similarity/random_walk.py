import numpy as np
import networkx as nx
from . import GraphProperties


class RandomWalk:
    """Class providing methods to compute random walks starting from a node in the graph
    """

    def __init__(self, graph):
        # type (nx.Graph) -> None
        """Initializes a RandomWalk class for the graph passed as argument
        """
        self.graph = graph
        adj_matrix = nx.adjacency_matrix(graph).todense()
        trans_prob = adj_matrix.astype(float)

        for i in range(trans_prob.shape[1]):
            if np.sum(trans_prob[i]) > 0:
                trans_prob[i] /= np.sum(trans_prob[i])

        # a matrix of size #nodes x #nodes containing used to compute the random walk over the graph
        self.cum_sum = np.cumsum(trans_prob, axis=1)

        # dictionary keyed by the graph nodes having as values the random walks starting at the keyed node.
        # The values contained in the random walk refer to the unique node identifier shared by all subgraphs.
        # This values are usually unknown and shall only be used to test the correctness of the matching
        self.unique_id_random_walks = {}

        # dictionary keyed by the graph nodes having as value the random walks starting a the keyed node.
        # The values contained in the random walk refer to the semantic label id stored in the nodes traversed by the
        # random walk. This information shall be used to test the matching.
        self.semantic_id_walks = {}

    def generate_random_walks(self, num_walks, walk_length):
        # type (int, int) -> None
        """Generate num_walks random walks of length walk_length for each node of the graph passed to the constructor
        of the calling object.
        :param num_walks: number of random walks to generate for each node of the graph.
        :param walk_length: length of the random walk.
        """
        graph_nodes = self.graph.nodes(data=False)
        for i, node in enumerate(graph_nodes):
            node_unique_id_walks, node_semantic_id_walks \
                = self.generate_random_walk_starting_at(num_walks=num_walks, walk_length=walk_length,
                                                        start_node_index=i)

            self.unique_id_random_walks[node] = node_unique_id_walks
            self.semantic_id_walks[node] = node_semantic_id_walks

    def generate_random_walk_starting_at(self, num_walks, walk_length, start_node_index):
        # type (int, int, int) -> numpy.ndarray(), numpy.ndarray()
        """Generats num_walks random walks of length walk_length for the 'start_node_index'-th node contained in the
        node passed to the constructor of the calling object.
        :param num_walks: number of random walks to generate.
        :param walk_length: length of each generated random walk.
        :param start_node_index: local index of the node where the random walk should start from. Note that this
        index is purely local, this index is a simple iterator between 0 and len(self.G.nodes())-1
        :return: two numpy ndarray of shape [num_walks x (walk_length+1)] containing the unique node id of the
        generated random walks and the associated semantic label id.
        """
        unique_id_walks = np.empty([num_walks, walk_length + 1], dtype=int)
        semantic_label_id_walks = np.empty(unique_id_walks.shape, dtype=int)

        graph_nodes = self.graph.nodes(data=True)
        for i in range(num_walks):
            walk = [start_node_index]
            for j in range(walk_length):
                curr_node = walk[-1]

                node_cum_sum = self.cum_sum[curr_node, :].T
                r = np.random.random()

                next_node_index = [n for n, l in enumerate(node_cum_sum) if l > r][0]
                walk.append(next_node_index)

            semantic_label_id_walk = []
            unique_id_walk = []
            for node_index in walk:
                node_semantic_label_id = graph_nodes[node_index][1][GraphProperties.node_semantic_label_id_key]
                node_unique_label_id = graph_nodes[node_index][1][GraphProperties.node_unique_identifier_key]
                semantic_label_id_walk.append(node_semantic_label_id)
                unique_id_walk.append(node_unique_label_id)

            unique_id_walks[i, :] = unique_id_walk
            semantic_label_id_walks[i, :] = semantic_label_id_walk

        return unique_id_walks, semantic_label_id_walks
