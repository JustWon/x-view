from .topograph import NodeEntity, GraphEdge, generate_topograph
from .topograph.graph_node import WordNetGraphNode, RestrictedGraphNode
from . import RestrictedSimilarityTable

import itertools
import numpy as np
import random

from nltk.corpus import wordnet as wn


def generate_edges(nodes, positions, fill_factor=0.4, stretch_variation=0.02, edge_color='#696969'):
    """Generates edges on the nodes passed as argument
    """
    edges = []
    for (node1, pos1), (node2, pos2) in itertools.combinations(zip(nodes, positions), 2):
        # measure the distance between the nodes of the edge
        dist = pos1 - pos2
        dist = np.sqrt(np.dot(dist, dist))
        edges.append(GraphEdge(node1=node1, node2=node2, rest_length=dist + stretch_variation * np.random.randn(),
                               stiffness=1.0 + np.random.rand(), edge_color=edge_color))

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
        graph_nodes.append(
            WordNetGraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

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
        graph_nodes.append(
            WordNetGraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

    edges = generate_edges(nodes=graph_nodes, positions=positions, fill_factor=0.6, stretch_variation=0.1)

    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Wordnet example Graph 2")


def add_edge(id1, id2, nodes, edges, positions, stretch_variation, edge_color):
    dist = positions[id1] - positions[id2]
    dist = np.sqrt(np.dot(dist, dist))
    edges.append(GraphEdge(node1=nodes[id1], node2=nodes[id2],
                           rest_length=dist + stretch_variation * np.random.randn(),
                           stiffness=1.0 + np.random.rand(), edge_color=edge_color))


def get_restricted_topograph1(node_color="#2EC221", edge_color="#299C1F"):
    semantic_concepts = ["vegetation", "sky", "road", "traffic sign", "car", "pedestrian", "sidewalk", "building",
                         "car", "building"]
    positions = [np.array([0.0, 0.5]), np.array([1.0, 1.0]), np.array([1.0, 0.0]), np.array([1.0, -1.0]),
                 np.array([2.0, 0.0]), np.array([2.0, -1.0]), np.array([3.0, -1.0]), np.array([3.0, 0.0]),
                 np.array([3.0, 1.0]), np.array([4.0, 0.0])]
    descriptions = [{}, {}, {}, {}, {'color': 'red'}, {'sex': 'female'}, {}, {}, {'color': 'blue'}, {}]
    nodes = [NodeEntity(sem) for sem in semantic_concepts]

    colors = [node_color for _ in semantic_concepts]
    plot_positions = [pos[0:2] for pos in positions]

    graph_nodes = []
    for node, sem, col, pos, plot_pos, desc in zip(nodes, semantic_concepts, colors, positions, plot_positions,
                                                   descriptions):
        graph_nodes.append(
            RestrictedGraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col,
                                properties=desc))

    graph_nodes[4].node_color = '#A61414' # red car
    graph_nodes[8].node_color = '#2441E3' # blue car
    graph_nodes[5].node_color = '#ED05E9' # female pedestrian

    edges = []
    stretch_variation = 0.1

    add_edge(id1=0, id2=1, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=1, id2=2, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=0, id2=2, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=2, id2=3, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=3, id2=4, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=1, id2=4, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=2, id2=4, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=4, id2=5, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=5, id2=6, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=6, id2=7, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=4, id2=7, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=4, id2=6, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=7, id2=7, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=7, id2=8, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=4, id2=8, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=9, id2=8, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=3, id2=5, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=6, id2=9, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=1, id2=8, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)

    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Restricted example graph 1")


def get_restricted_topograph2(node_color="#2EC221", edge_color="#299C1F"):
    semantic_concepts = ["car", "car", "pedestrian", "pedestrian", "traffic sign"]
    positions = [np.array([3.0, 1.0]), np.array([2.0, 0.0]), np.array([2.0, 1.0]), np.array([2.0, -1.0]),
                 np.array([1.0, -1.0])]
    angle = np.pi * 0.5
    rot = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    positions = [rot.dot(pos) for pos in positions]
    positions += np.array([3,3])
    descriptions = [{'color': 'blue'}, {'color': 'red'}, {'sex': 'male'}, {'sex': 'female'}, {}]
    nodes = [NodeEntity(sem) for sem in semantic_concepts]

    colors = [node_color for _ in semantic_concepts]
    plot_positions = [pos[0:2] for pos in positions]

    graph_nodes = []
    for node, sem, col, pos, plot_pos, desc in zip(nodes, semantic_concepts, colors, positions, plot_positions,
                                                   descriptions):
        graph_nodes.append(
            RestrictedGraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col,
                                properties=desc))

    graph_nodes[0].node_color = '#2441E3'  # blue car
    graph_nodes[1].node_color = '#A61414'  # red car
    graph_nodes[2].node_color = '#25DEE8'  # male pedestrian
    graph_nodes[3].node_color = '#ED05E9'  # female pedestrian

    edges = []
    stretch_variation = 0.1

    add_edge(id1=0, id2=1, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=1, id2=2, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=1, id2=3, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=1, id2=4, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=2, id2=4, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=3, id2=4, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=0, id2=3, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)
    add_edge(id1=0, id2=2, nodes=graph_nodes, edges=edges, positions=positions, stretch_variation=stretch_variation,
             edge_color=edge_color)

    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Restricted example graph 2")


def get_random_restricted_topograph(dim=3, num_elements=5, edge_fill_factor=0.8, edge_stretch_variation=0.1):
    semantic_concepts = RestrictedSimilarityTable.semantic_concepts
    indices = np.random.choice(len(semantic_concepts), num_elements)
    semantic = [semantic_concepts[id] for id in indices]
    nodes = [NodeEntity(sem) for sem in semantic]
    colors = ["#2EC221" for _ in semantic]
    positions = [3 * np.random.rand(dim) for _ in semantic]
    plot_positions = [pos[0:2] for pos in positions]

    graph_nodes = []
    for node, sem, col, pos, plot_pos in zip(nodes, semantic, colors, positions, plot_positions):
        graph_nodes.append(
            RestrictedGraphNode(node=node, pos=pos, label=sem, plot_pos=plot_pos, lemma=sem, node_color=col))

    edges = generate_edges(nodes=graph_nodes, positions=positions, fill_factor=edge_fill_factor,
                           stretch_variation=edge_stretch_variation, edge_color="#299C1F")
    return generate_topograph(graph_node_list=graph_nodes, graph_edge_list=edges, name="Restricted example graph 1")
