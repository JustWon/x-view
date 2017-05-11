import argparse
import networkx as nx
from node2vec import ExampleGraphs, SubgraphGenerator, GraphModifier
import matplotlib.pylab as plt


def parse_args():
    '''
    Parses the node2vec arguments.
    '''
    parser = argparse.ArgumentParser(description="Run node2vec.")

    return parser.parse_args()


def main(arguments, base_graph):
    # type (argparse.ArgumentParsere, nx.Graph)
    """Runs the main process
    """
    subgraph_generator = SubgraphGenerator(base_graph)
    base_graph_num_nodes = base_graph.number_of_nodes()
    base_graph_num_edges = base_graph.number_of_edges()

    subgraph = subgraph_generator.generate_subgraph_centered_at_with_radius(15, 2)
    subgraph_num_nodes = subgraph.number_of_nodes()
    subgraph_num_edges = subgraph.number_of_edges()

    plt.figure()
    nx.draw(base_graph)
    plt.show()

    plt.figure()
    nx.draw(subgraph)
    plt.show()

    plt.figure()
    GraphModifier.remove_n_edges_from_graph(subgraph, subgraph_num_edges)
    nx.draw(subgraph)
    plt.show()

    plt.figure()
    GraphModifier.add_n_edges_to_graph(subgraph, subgraph_num_edges)
    nx.draw(subgraph)
    plt.show()

if __name__ == '__main__':
    args = parse_args()
    current_base_graph_name = ExampleGraphs.graph_names[5]
    graph, graph_path = ExampleGraphs.load_example_graph(current_base_graph_name)
    print("Working with '{}' graph".format(graph_path))
    main(args, graph)
