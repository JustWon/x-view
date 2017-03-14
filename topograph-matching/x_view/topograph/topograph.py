import networkx as nx


class TopoGraph(nx.Graph):
    def __init__(self, name):
        super(TopoGraph, self).__init__(name=name)

    def print(self):
        for node in self.nodes(data=True):
            print(node)

    @staticmethod
    def dump_to_file(graph, filename):
        nx.write_gpickle(graph, filename)

    @staticmethod
    def load_from_file(path):
        return nx.read_gpickle(path)
