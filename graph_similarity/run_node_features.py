from __future__ import print_function
import argparse
from matplotlib import pylab as plt
import random
import numpy as np
import itertools
from scipy.spatial.distance import cdist
from graph_similarity import GraphProperties, SubgraphGenerator, GraphLabeler, GraphDrawer, RandomWalk
from example_graphs import ExampleGraphLoader


def parse_args():
    """Parses the cmd line arguments.
    """
    parser = argparse.ArgumentParser(description="Run node2vec.")

    parser.add_argument('--input', type=int, default=5, help='Base graph index')

    parser.add_argument('--walk-length', type=int, default=3, help='Length of walk per source. Default is 3.')

    parser.add_argument('--num-walks', type=int, default=100, help='Number of walks per source. Default is 100.')

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


def run_node_features(base_graph):
    """Runs the main process
    """

    # compute the indices of two nodes in the base_graph linked by an edge
    random_edge = random.sample(base_graph.edges(), 1)

    index1 = base_graph.nodes().index(random_edge[0][0])
    index2 = base_graph.nodes().index(random_edge[0][1])

    center_node_indices = [index1, index2]

    print("Generating {} random subgraphs".format(len(center_node_indices)))
    subgraph_generator = SubgraphGenerator(base_graph)
    subgraph_generator.generate_n_subgraphs_centered_around_nodes(num_subgraphs=len(center_node_indices), min_radius=1,
                                                                  max_radius=1, remove_edge_fraction=0.2,
                                                                  add_edge_fraction=0.2,
                                                                  node_indices=center_node_indices)

    num_walks = args.num_walks
    walk_length = args.walk_length
    unique_id_random_walks = []
    semantic_id_random_walks = []
    print("Generating {} random walks of length {} for each node in each graph.".format(num_walks, walk_length))
    for i, subgraph in enumerate(subgraph_generator.subgraphs):
        f, axarr = plt.subplots(2)

        axarr[0].set_title("Unique node identifiers")
        GraphDrawer.draw_graph(subgraph, ax=axarr[0])
        GraphDrawer.draw_graph_labels(subgraph, GraphProperties.node_unique_identifier_key, ax=axarr[0])

        axarr[1].set_title("Semantic label")
        GraphDrawer.draw_graph(subgraph, ax=axarr[1])
        GraphDrawer.draw_graph_labels(subgraph, GraphProperties.node_semantic_label_id_key, ax=axarr[1])


        r = RandomWalk(subgraph)
        r.generate_random_walks(num_walks=num_walks, walk_length=walk_length)

        unique_id_random_walks.append(r.unique_id_random_walks)
        semantic_id_random_walks.append(r.semantic_id_walks)

    # perform a matching between the subgraphs and check if there is a matching
    for (i1, subgraph1), (i2, subgraph2) in itertools.combinations(enumerate(subgraph_generator.subgraphs), 2):
        distances = np.zeros([subgraph1.number_of_nodes(), subgraph2.number_of_nodes()])
        for (n1, node1) in enumerate(subgraph1.nodes()):
            for (n2, node2) in enumerate(subgraph2.nodes()):
                # if the two semantic labels are different, then set the relative distance to infinity,
                # otherwise compute the similarity according to the random paths
                if(subgraph1.node[node1][GraphProperties.node_semantic_label_id_key] ==
                   subgraph2.node[node2][GraphProperties.node_semantic_label_id_key]):
                    # Comparing node1 of graph i1 with node2 of graph i2
                    descriptors1 = semantic_id_random_walks[i1][node1]
                    descriptors2 = semantic_id_random_walks[i2][node2]

                    pairwise_distance = cdist(descriptors1, descriptors2)

                    sort_dist = np.sort(pairwise_distance.flatten())
                    weights = -np.log(np.linspace(1.0e-5, 1, num=len(sort_dist)))

                    weight_dist = sort_dist.dot(weights)

                    distances[n1, n2] = weight_dist
                else:
                    distances[n1, n2] = np.finfo(float).max

        min_dist_for_i1 = np.argsort(distances, axis=1)
        min_dist_for_i2 = np.argsort(distances, axis=0).T

        print("Matches according to subgraph {}".format(i1))
        for (n1, node1) in enumerate(subgraph1.nodes(data=True)):
            min_dist_ind = min_dist_for_i1[n1, :]
            matching_node_in_i2 = [subgraph2.nodes(data=True)[sorted_index] for sorted_index in min_dist_ind]
            print("Min dist for node {} in subgraph {} are nodes: "
                  .format(node1[1][GraphProperties.node_unique_identifier_key], i1), end=' ')
            for match in matching_node_in_i2:
                print("{}, ".format(match[1][GraphProperties.node_unique_identifier_key]), end=' ')
            print()

        for (n2, node2) in enumerate(subgraph2.nodes(data=True)):
            min_dist_ind = min_dist_for_i2[n2, :]
            matching_node_in_i1 = [subgraph1.nodes(data=True)[sorted_index] for sorted_index in min_dist_ind]
            print("Min dist for node {} in subgraph {} are nodes: "
                  .format(node2[1][GraphProperties.node_unique_identifier_key], i2), end=' ')
            for match in matching_node_in_i1:
                print("{}, ".format(match[1][GraphProperties.node_unique_identifier_key]), end=' ')
            print()

    plt.show()

if __name__ == '__main__':

    random.seed(0)
    np.random.seed(0)

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
    run_node_features(graph)
