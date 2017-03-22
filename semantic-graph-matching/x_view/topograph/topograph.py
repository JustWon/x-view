import networkx as nx


class TopoGraph(nx.Graph):
    def __init__(self, name):
        super(TopoGraph, self).__init__(name=name)