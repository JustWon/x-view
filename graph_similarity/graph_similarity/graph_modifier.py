import networkx as nx
import random
from . import GraphChecker


class GraphModifier:
    """Class responsible for modifying the structure and topology of a graph maintaining the connectivity of the
    entire graph (the graph remains a single connected component).
    """

    @staticmethod
    def remove_n_edges_from_graph(graph, n, max_iter=None):
        if max_iter is None:
            max_iter = graph.number_of_edges()
        edge_list = graph.edges()
        removed_edges = 0
        iter = 0
        while removed_edges < n and iter < max_iter:
            candidate_edge = random.choice(edge_list)
            if graph.has_edge(*candidate_edge):
                if GraphChecker.would_still_remain_connected_if_edge_removed(graph, candidate_edge):
                    graph.remove_edge(*candidate_edge)
                    removed_edges += 1
            iter += 1

    @staticmethod
    def add_n_edges_to_graph(graph, n, max_iter=None):
        if max_iter is None:
            max_iter = graph.number_of_edges()
        num_nodes = graph.number_of_nodes()
        nodes = graph.nodes()

        added_edges = 0
        iter = 0
        while added_edges < n and iter < max_iter:
            candidate_node_indices = random.sample(range(0, num_nodes), 2)
            candidate_edge = [nodes[candidate_node_indices[0]], nodes[candidate_node_indices[1]]]
            if not graph.has_edge(*candidate_edge):
                graph.add_edge(*candidate_edge)
                added_edges += 1
            iter += 1
