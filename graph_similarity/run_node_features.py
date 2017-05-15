import argparse
from graph_similarity import GraphProperties, SubgraphGenerator, GraphLabeler, GraphDrawer, RandomWalk
from example_graphs import ExampleGraphLoader
from gensim.models import Word2Vec
from matplotlib import pylab as plt
import numpy as np


def parse_args():
    """Parses the cmd line arguments.
    """
    parser = argparse.ArgumentParser(description="Run node2vec.")

    parser.add_argument('--input', type=int, default=5, help='Base graph index')

    parser.add_argument('--output', type=str, default='emb/embeddings.emb', help='Embeddings path')

    parser.add_argument('--walk-length', type=int, default=5, help='Length of walk per source. Default is 80.')

    parser.add_argument('--num-walks', type=int, default=10, help='Number of walks per source. Default is 10.')

    return parser.parse_args()


def draw_all_subgraphs(graph_list):
    """Draws all graphs passed as argument into a new figure each.
    """
    for i, g in enumerate(graph_list):
        plt.figure()
        plt.title("Subgraph {}".format(i))
        GraphDrawer.draw_graph(g)
        # GraphDrawer.draw_graph_node_labels(g)
        GraphDrawer.draw_graph_unique_identifier(g)


def run_node_features(arguments, base_graph):
    """Runs the main process
    """

    print("Generating {} random subgraphs".format(1))
    subgraph_generator = SubgraphGenerator(base_graph)
    subgraph_generator.generate_n_subgraphs_centered_around_nodes(num_subgraphs=1, min_radius=1, max_radius=1,
                                                                  remove_edge_fraction=0.2, add_edge_fraction=0.2)

    num_walks = 10
    walk_length = 5
    subgraph_walks = []
    for i, subgraph in enumerate(subgraph_generator.subgraphs):
        print("Subgraph {} has the following node descriptors: ".format(i))
        plt.figure()
        GraphDrawer.draw_graph(subgraph)
        GraphDrawer.draw_graph_labels(subgraph, GraphProperties.node_unique_identifier_key)

        r = RandomWalk(subgraph)
        r.generate_random_walks(num_walks=num_walks, walk_length=walk_length)

        print(r.unique_id_random_walks)

        subgraph_walks.append(r.unique_id_random_walks)

    plt.show()


if __name__ == '__main__':
    args = parse_args()

    # load the base graph
    current_base_graph_name = ExampleGraphLoader.graph_names[args.input]
    graph, graph_path = ExampleGraphLoader.load_example_graph(current_base_graph_name)

    print("Working with '{}' graph".format(graph_path))

    # add some properties to the base graph's nodes
    GraphLabeler.add_unique_node_identifiers(graph)
    GraphLabeler.add_graph_membership(graph, graph_id=0)
    GraphLabeler.add_random_semantic_labels(graph, ["car", "tree", "pedestrian", "sky", "road", "building"])
    GraphLabeler.add_drawing_position(graph)

    # display the base graph
    plt.figure()
    plt.title("Base graph")
    GraphDrawer.draw_graph(graph)
    GraphDrawer.draw_graph_labels(graph, GraphProperties.node_unique_identifier_key)

    # run the node_features program, which generates random subgraphs of the base graph and computes
    # and embedding in which similar nodes are close to each other
    run_node_features(args, graph)
