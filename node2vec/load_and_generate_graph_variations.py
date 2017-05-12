import argparse
import networkx as nx
from node2vec import ExampleGraphs, SubgraphGenerator, GraphModifier
import matplotlib.pylab as plt


def parse_args():
    """Parses the node2vec arguments.
    """
    parser = argparse.ArgumentParser(description="Run node2vec.")

    return parser.parse_args()


def main(arguments, base_graph):
    """Runs the main process
    """
    subgraph_generator = SubgraphGenerator(base_graph)
    base_graph_num_nodes = base_graph.number_of_nodes()
    base_graph_num_edges = base_graph.number_of_edges()

    subgraph = subgraph_generator.generate_subgraph_centered_at_with_radius(15, 2)
    subgraph_num_nodes = subgraph.number_of_nodes()
    subgraph_num_edges = subgraph.number_of_edges()

    plt.figure(0)
    plt.title("Loaded graph")
    nx.draw(base_graph)

    # compute node positions
    node_position = nx.spring_layout(subgraph)

    plt.figure(1)
    plt.title("Extracted subgraph")
    nx.draw(subgraph, pos=node_position)

    plt.figure(2)
    plt.title("Subgraph with removed edges")
    num_edges_to_remove = int(subgraph_num_edges / 2)
    GraphModifier.remove_n_edges_from_graph(subgraph, num_edges_to_remove)
    nx.draw(subgraph, pos=node_position)

    plt.figure(3)
    plt.title("Subgraph with new edges")
    num_edges_to_add = int(subgraph_num_edges / 2)
    GraphModifier.add_n_edges_to_graph(subgraph, num_edges_to_add)
    nx.draw(subgraph, pos=node_position)

    plt.show()


if __name__ == '__main__':
    args = parse_args()
    current_base_graph_name = ExampleGraphs.graph_names[5]
    graph, graph_path = ExampleGraphs.load_example_graph(current_base_graph_name)
    print("Working with '{}' graph".format(graph_path))
    main(args, graph)
