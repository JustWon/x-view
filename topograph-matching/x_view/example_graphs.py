from .topograph import NodeEntity, GraphNode, GraphEdge, TopoGraph, generate_topograph
from . import semantic_concepts

import itertools
import numpy as np
import random

from nltk.corpus import wordnet as wn


def generate_edges(nodes, positions, fill_factor=0.4, stretch_variation=0.02):
    """Generates edges on the nodes passed as argument
    """
    edges = []
    for (node1, pos1), (node2, pos2) in itertools.combinations(zip(nodes, positions), 2):
        # measure the distance between the nodes of the edge
        dist = pos1 - pos2
        dist = np.sqrt(np.dot(dist, dist))
        edges.append(GraphEdge(node1=node1, node2=node2, rest_length=dist + stretch_variation * np.random.randn(),
                               stiffness=1.0 + np.random.rand(), edge_color='#696969'))

    num_edges_to_remove = int((1.0 - fill_factor) * len(edges))
    edges_to_remove = random.sample(range(0, len(edges)), num_edges_to_remove)
    for index in sorted(edges_to_remove, reverse=True):
        del edges[index]

    return edges


def get_wordnet_topograph1(dim=3):
    semantic = ["tree", "person", "dog", "child", "house"]
    # generate node entity having a unique ID with associated parameters
    nodes = [NodeEntity(wn.synsets(sem)[0]) for sem in semantic]
    colors = ["y" for _ in semantic]
    positions = [3 * np.random.rand(dim) for _ in semantic]
    plot_positions = [pos[0:2] for pos in positions]

    # generate graph nodes
    graph_nodes = []
    for node, sem, col, pos, plot_pos in zip(nodes, semantic, colors, positions, plot_positions):
        graph_nodes.append(GraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

    edges = generate_edges(nodes=graph_nodes, positions=positions, fill_factor=0.8, stretch_variation=0.05)

    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Wordnet example Graph 1")


def get_wordnet_topograph2(dim=3):
    semantic = ["tree", "man", "cat", "sun", "school", "hospital"]
    # generate node entity having a unique ID with associated parameters
    nodes = [NodeEntity(wn.synsets(sem)[0]) for sem in semantic]
    colors = ["b" for _ in semantic]
    positions = [4 * np.ones(dim) + 3 * np.random.rand(dim) for _ in semantic]
    plot_positions = [pos[0:2] for pos in positions]

    # generate graph nodes
    graph_nodes = []
    for node, sem, col, pos, plot_pos in zip(nodes, semantic, colors, positions, plot_positions):
        graph_nodes.append(GraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

    edges = generate_edges(nodes=graph_nodes, positions=positions, fill_factor=0.6, stretch_variation=0.1)

    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Wordnet example Graph 2")


def get_restricted_topograph1(dim=3, num_elements = 5):
    indices = random.sample(range(0, len(semantic_concepts)), num_elements)
    semantic = [semantic_concepts[id] for id in indices]
    nodes = [NodeEntity(sem) for sem in semantic]
    colors = ["y" for _ in semantic]
    positions = [3 * np.random.rand(dim) for _ in semantic]
    plot_positions = [pos[0:2] for pos in positions]

    graph_nodes = []
    for node, sem, col, pos, plot_pos in zip(nodes, semantic, colors, positions, plot_positions):
        graph_nodes.append(GraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

    edges = generate_edges(nodes=graph_nodes, positions = positions, fill_factor=0.8, stretch_variation=0.1)
    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Restricted example graph 1")

def get_restricted_topograph2(dim=3, num_elements = 5):
    indices = random.sample(range(0, len(semantic_concepts)), num_elements)
    semantic = [semantic_concepts[id] for id in indices]
    nodes = [NodeEntity(sem) for sem in semantic]
    colors = ["b" for _ in semantic]
    positions = [4 * np.random.rand(dim) + 3 * np.random.rand(dim) for _ in semantic]
    plot_positions = [pos[0:2] for pos in positions]

    graph_nodes = []
    for node, sem, col, pos, plot_pos in zip(nodes, semantic, colors, positions, plot_positions):
        graph_nodes.append(GraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

    edges = generate_edges(nodes=graph_nodes, positions = positions, fill_factor=0.8, stretch_variation=0.1)
    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Restricted example graph 2")
