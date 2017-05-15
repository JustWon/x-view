import networkx as nx

class GraphChecker:
    @staticmethod
    def would_still_remain_connected_if_edge_removed(graph, candidate_edge):
        graph_copy = graph.copy()
        graph_copy.remove_edge(*candidate_edge)

        if nx.is_connected(graph_copy):
            return True
        return False
