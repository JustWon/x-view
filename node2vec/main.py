import argparse
from node2vec import ExampleGraphs, SubgraphGenerator, GraphLabeler, GraphDrawer
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
    subgraph_generator.generate_n_subgraphs_centered_around_nodes(num_subgraphs=2, min_radius=1, max_radius=1,
                                                                  remove_edge_fraction=0.2, add_edge_fraction=0.2)
    for i, g in enumerate(subgraph_generator.subgraphs):
        plt.figure()
        plt.title("Subgraph {}".format(i))
        GraphDrawer.draw_graph(g)
        GraphDrawer.draw_graph_node_labels(g)

    plt.show()


if __name__ == '__main__':
    args = parse_args()
    current_base_graph_name = ExampleGraphs.graph_names[5]
    graph, graph_path = ExampleGraphs.load_example_graph(current_base_graph_name)
    print("Working with '{}' graph".format(graph_path))


    GraphLabeler.add_unique_node_identifiers(graph)
    GraphLabeler.add_random_semantic_labels(graph, ["car", "tree", "pedestrian", "sky", "road", "building"])
    GraphLabeler.add_drawing_position(graph)
    plt.figure()
    plt.title("Base graph")
    GraphDrawer.draw_graph(graph)
    GraphDrawer.draw_graph_node_labels(graph)

    main(args, graph)
