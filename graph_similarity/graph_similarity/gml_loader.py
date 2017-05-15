import networkx as nx
import os


class GmlLoader:
    def __init__(self):
        pass

    @staticmethod
    def list_gml_files_in_directory(directory_path):
        file_names_in_directory = os.listdir(directory_path)
        graph_name_list = [os.path.join(directory_path, graph_name) for graph_name in file_names_in_directory if
                           graph_name.endswith(".gml")]
        return graph_name_list

    @staticmethod
    def convert_gml_to_nx_graph(gml_file_name):
        return nx.read_gml(gml_file_name)

    @staticmethod
    def convert_gml_list_to_nx_graph(gml_file_names):
        graphs = []
        for gml_file_name in gml_file_names:
            print("Reading {}".format(gml_file_name))
            graphs.append(GmlLoader.convert_gml_to_nx_graph(gml_file_name))
        return graphs

    @staticmethod
    def load_all_graphs_in_directory(directory_path):
        graph_name_list = GmlLoader.list_gml_files_in_directory(directory_path)
        graphs = GmlLoader.convert_gml_list_to_nx_graph(graph_name_list)

        return graphs, graph_name_list

    @staticmethod
    def load_named_graph_in_directory(directory_path, graph_name):
        gml_file_name = os.path.join(directory_path, graph_name)
        return GmlLoader.convert_gml_to_nx_graph(gml_file_name), gml_file_name
