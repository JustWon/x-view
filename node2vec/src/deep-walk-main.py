'''
Reference implementation of node2vec. 

Author: Aditya Grover

For more details, refer to the paper:
node2vec: Scalable Feature Learning for Networks
Aditya Grover and Jure Leskovec 
Knowledge Discovery and Data Mining (KDD), 2016
'''

import argparse

import networkx as nx
import node2vec
from matplotlib import pylab as plt
from gensim.models import Word2Vec


def parse_args():
    '''
    Parses the node2vec arguments.
    '''
    parser = argparse.ArgumentParser(description="Run node2vec.")

    parser.add_argument('--input', nargs='?', default='graphs/karate.edgelist', help='Input graph path')

    parser.add_argument('--output', nargs='?', default='emb/karate.emb', help='Embeddings path')

    parser.add_argument('--dimensions', type=int, default=2, help='Number of dimensions. Default is 128.')

    parser.add_argument('--walk-length', type=int, default=80, help='Length of walk per source. Default is 80.')

    parser.add_argument('--num-walks', type=int, default=10, help='Number of walks per source. Default is 10.')

    parser.add_argument('--window-size', type=int, default=10, help='Context size for optimization. Default is 10.')

    parser.add_argument('--iter', default=100, type=int, help='Number of epochs in SGD')

    parser.add_argument('--workers', type=int, default=8, help='Number of parallel workers. Default is 8.')

    parser.add_argument('--p', type=float, default=1, help='Return hyperparameter. Default is 1.')

    parser.add_argument('--q', type=float, default=1, help='Inout hyperparameter. Default is 1.')

    parser.add_argument('--weighted', dest='weighted', action='store_true',
                        help='Boolean specifying (un)weighted. Default is unweighted.')

    parser.add_argument('--unweighted', dest='unweighted', action='store_false')

    parser.set_defaults(weighted=False)

    parser.add_argument('--directed', dest='directed', action='store_true',
                        help='Graph is (un)directed. Default is undirected.')
    parser.add_argument('--undirected', dest='undirected', action='store_false')

    parser.set_defaults(directed=False)

    return parser.parse_args()


def read_graph():
    '''
    Reads the input network in networkx.
    '''
    if args.weighted:
        G = nx.read_edgelist(args.input, nodetype=int,
                             data=(('weight', float),),
                             create_using=nx.DiGraph())
    else:
        G = nx.read_edgelist(args.input, nodetype=int,
                             create_using=nx.DiGraph())
        for edge in G.edges():
            G[edge[0]][edge[1]]['weight'] = 1

    if not args.directed:
        G = G.to_undirected()
    return G


def learn_embeddings(walks):
    '''
    Learn embeddings by optimizing the Skipgram objective using SGD.
    '''
    walks = [map(str, walk) for walk in walks]
    model = Word2Vec(walks, size=args.dimensions, window=args.window_size,
                     min_count=0, sg=1, workers=args.workers, iter=args.iter)
    model.wv.save_word2vec_format(args.output)

    return model


def plot_embedding(model):
    word_to_vec = model.wv
    embedded_data = word_to_vec.syn0
    fig, ax = plt.subplots()
    ax.scatter(embedded_data[:, 0], embedded_data[:, 1])
    for i, txt in enumerate(range(embedded_data.shape[0])):
        ax.annotate(txt, (embedded_data[i, 0], embedded_data[i, 1]))
    plt.show()

def main(args):
    '''
    Pipeline for representational learning for all nodes in a graph.
    '''
    nx_G = read_graph()
    G = node2vec.Graph(nx_G, args.directed, args.p, args.q)
    G.preprocess_transition_probs()
    walks = G.simulate_walks(args.num_walks, args.walk_length)
    model = learn_embeddings(walks)
    plot_embedding(model)


if __name__ == "__main__":
    args = parse_args()
    main(args)
