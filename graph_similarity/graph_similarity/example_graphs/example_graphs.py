import os
from ..gml_loader import GmlLoader


class ExampleGraphs:
    graph_names = ["adjnoun.gml",
                   "as-22july06.gml",
                   "astro-ph.gml",
                   "celegansneural.gml",
                   "cond-mat.gml",
                   "dolphins.gml",
                   "football.gml",
                   "hep-th.gml",
                   "karate.gml",
                   "lesmis.gml",
                   "netscience.gml",
                   "polbooks.gml"]

    current_directory_path = os.path.dirname(os.path.abspath(__file__))

    @staticmethod
    def load_example_graph(graph_name):
        if graph_name not in ExampleGraphs.graph_names:
            raise Exception("Example graph '{}' does not exist".format(graph_name))
        return GmlLoader.load_named_graph_in_directory(ExampleGraphs.current_directory_path, graph_name)
