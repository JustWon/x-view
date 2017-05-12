import argparse
import networkx as nx
from node2vec import ExampleGraphs, SubgraphGenerator
import matplotlib.pylab as plt


def parse_args():
    """Parses the node2vec arguments.
    """
    parser = argparse.ArgumentParser(description="Run node2vec.")

    return parser.parse_args()


def main(arguments, base_graph):
    # type (argparse.ArgumentParsere, nx.Graph)
    """Runs the main process
    """
    subgraph_generator = SubgraphGenerator(base_graph)
    subgraph_generator.generate_n_subgraphs_centered_around_nodes(num_subgraphs=5, remove_edge_fraction=0.8,
                                                                  add_edge_fraction=0)
    for g in subgraph_generator.subgraphs:
        plt.figure()
        nx.draw(g)
    plt.show()


if __name__ == '__main__':
    args = parse_args()
    current_base_graph_name = ExampleGraphs.graph_names[5]
    graph, graph_path = ExampleGraphs.load_example_graph(current_base_graph_name)
    print("Working with '{}' graph".format(graph_path))
    main(args, graph)
