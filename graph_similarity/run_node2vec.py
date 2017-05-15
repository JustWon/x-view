import argparse
from graph_similarity import SubgraphGenerator, GraphLabeler, GraphDrawer, Node2VecGraph, GraphProperties
from example_graphs import ExampleGraphLoader
import matplotlib.pylab as plt
from gensim.models import Word2Vec


def parse_args():
    """Parses the node2vec arguments.
    """
    parser = argparse.ArgumentParser(description="Run node2vec.")

    parser.add_argument('--input', type=int, default=5, help='Base graph index')

    parser.add_argument('--n', type=int, default=4, help='Number of subgrpahs to be generated')

    parser.add_argument('--output', type=str, default='emb/embeddings.emb', help='Embeddings path')

    parser.add_argument('--dimensions', type=int, default=128,
                        help='Number of dimensions od the embedding. Default is 128.')

    parser.add_argument('--walk-length', type=int, default=5, help='Length of walk per source. Default is 80.')

    parser.add_argument('--num-walks', type=int, default=10, help='Number of walks per source. Default is 10.')

    parser.add_argument('--window-size', type=int, default=10, help='Context size for optimization. Default is 10.')

    parser.add_argument('--iter', type=int, default=10, help='Number of epochs in SGD')

    parser.add_argument('--workers', type=int, default=8, help='Number of parallel workers. Default is 8.')

    parser.add_argument('--p', type=float, default=1, help='Return hyperparameter. Default is 1.')

    parser.add_argument('--q', type=float, default=1, help='Inout hyperparameter. Default is 1.')

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


def predict_embeddings(model, walks):
    # type(Word2Vec, list(list(int))) -> list(list(np.array(2)))
    """Predict embeddings using the model passed as argument for the random walks
    """
    # convert the walks into strings ([1,2,3] --> ['1', '2', '3'])
    string_walks = [map(str, walk) for walk in walks]

    # how to make predictions?
    predictions = []
    return predictions


def learn_embeddings_from_base_graph(base_graph, p, q, num_walks, walk_length, embedding_dimensions,
                                     window_size, iterations):
    """Learn embeddings by optimizing the Skipgram objective using SGD.
    """
    g = Node2VecGraph(base_graph, p, q)
    g.preprocess_transition_probs()
    walks = g.simulate_walks(num_walks, walk_length)

    # IDEA: the string_walks of the base graphs are pf the form ['0_base', '2_base', '12_base', ...] and the ones of
    # the subgr5aphs are ['1_subgraph_#id', '53_subgraph_#id', ...]
    # then we can train the entire walks togetherm hoping that '0_base' will be close to '0_subgraph_#id'
    string_walks = [map(str, walk) for walk in walks]

    model = Word2Vec(string_walks, size=embedding_dimensions, window=window_size, iter=iterations, min_count=0, sg=1,
                     workers=8)


    model.wv.save_word2vec_format(args.output)

    return model


def run_node2vec(arguments, base_graph):
    """Runs the main process
    """

    print("Training word2vec on the base graph")
    base_graph = base_graph

    model = learn_embeddings_from_base_graph(base_graph=base_graph, p=arguments.p, q=arguments.q,
                                             num_walks=arguments.num_walks, walk_length=arguments.walk_length,
                                             window_size=arguments.window_size, iterations=arguments.iter,
                                             embedding_dimensions=arguments.dimensions)
    print("Training complete")

    print("Generating {} random subgraphs".format(arguments.n))
    subgraph_generator = SubgraphGenerator(base_graph)
    subgraph_generator.generate_n_subgraphs_centered_around_nodes(num_subgraphs=arguments.n, min_radius=1, max_radius=1,
                                                                  remove_edge_fraction=0.2, add_edge_fraction=0.2)

    # dict keyed with subgraphs containing the corresponding random walks
    subgraph_walks = {}
    for i, subgraph in enumerate(subgraph_generator.subgraphs):
        print("Preprocessing subgraph {} of {}".format(i + 1, len(subgraph_generator.subgraphs)))
        g = Node2VecGraph(subgraph, arguments.p, arguments.q)
        g.preprocess_transition_probs()
        walks = g.simulate_walks(arguments.num_walks, arguments.walk_length)
        subgraph_walks[subgraph] = walks

    print("Performing predictions on subgraphs")
    for walks in subgraph_walks.itervalues():
        predictions = predict_embeddings(model, walks)

    draw_all_subgraphs(subgraph_generator.subgraphs)
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
    GraphDrawer.draw_graph_unique_identifier(graph)

    # run the node2vec program, which generates random subgraphs of the base graph and computes
    # and embedding in which similar nodes are close to each other
    run_node2vec(args, graph)
