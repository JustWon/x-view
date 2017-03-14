import networkx as nx
import os
from matplotlib import pylab as plt
from matplotlib.patches import FancyArrowPatch
import itertools

import numpy as np

from enum import Enum


class EdgeWidth(Enum):
    STIFFNESS = 1
    NODE_SIMILARITY = 2


class EdgeColor(Enum):
    LENGTH_DEVIATION = 1
    EDGE_COLOR = 2


class TopoGraphPlotter(object):
    def __init__(self, topograph_list, labels_visible=True, nodes_visible=True, edges_visible=True,
                 outer_connections_visible=False, directorty_name="fig/"):

        plt.figure()
        if not os.path.exists(directorty_name):
            os.makedirs(directorty_name)

        self.directory_name = directorty_name

        if not isinstance(topograph_list, list):
            topograph_list = [topograph_list]

        self.topograph_list = topograph_list

        self.labels_visible = labels_visible
        self.node_visible = nodes_visible
        self.edges_visible = edges_visible
        self.outer_connections_visible = outer_connections_visible

        self.label_color = 'k'
        self.font_size = 16

        self.edge_width_type = EdgeWidth.STIFFNESS
        self.edge_color_type = EdgeColor.LENGTH_DEVIATION

    def draw(self):

        for topograph in self.topograph_list:
            _, node_dicts = zip(*topograph.nodes(data=True))
            _, _, edge_dicts = zip(*topograph.edges(data=True))

            plot_pos = nx.get_node_attributes(topograph, 'plot_pos')
            labels = nx.get_node_attributes(topograph, 'label')
            if self.node_visible:
                positions = [d['pos'] for d in node_dicts]
                # simulate perspective by setting node_sizes based on 'z' coordinate
                if len(positions[0]) == 3:
                    depth = [pos[2] for pos in positions]
                    max_depth = max(depth)
                    min_depth = min(depth)
                    depth_range = max_depth - min_depth
                    node_sizes = [600.00 / (0.9 + 3 * (pos[2] - min_depth) / depth_range) for pos in positions]
                else:
                    node_sizes = [200 for _ in positions]

                color = [d['node_color'] for d in node_dicts]
                nx.draw_networkx_nodes(topograph, pos=plot_pos, node_color=color, node_size=node_sizes)

            if self.edges_visible:
                edge_list = topograph.edges(data=False)
                rest_length = [e['rest_length'] for e in edge_dicts]
                stiffness = [e['stiffness'] ** 2 for e in edge_dicts]

                if self.edge_color_type is EdgeColor.EDGE_COLOR:
                    color = [e['edge_color'] for e in edge_dicts]
                    nx.draw_networkx_edges(topograph, pos=plot_pos, edgelist=topograph.edges(), width=stiffness,
                                           edge_color=color)

                elif self.edge_color_type is EdgeColor.LENGTH_DEVIATION:
                    length_deviation = [0 for _ in rest_length]
                    for i, edge in enumerate(edge_list):
                        e = topograph.node[edge[0]]['pos'] - topograph.node[edge[1]]['pos']
                        current_edge_length = np.sqrt(np.dot(e, e))
                        length_deviation[i] = current_edge_length - rest_length[i]

                    color = [-abs(deviation) for deviation in length_deviation]

                    nx.draw_networkx_edges(topograph, pos=plot_pos, edgelist=topograph.edges(), width=stiffness,
                                           edge_color=color, edge_cmap=plt.cm.RdYlGn,
                                           edge_vmin=-0.5, edge_vmax=0.0)

            if self.labels_visible:
                nx.draw_networkx_labels(topograph, pos=plot_pos, labels=labels, font_color=self.label_color,
                                        font_size=self.font_size)

        if self.outer_connections_visible:
            nodes_list = []
            dicts_list = []
            for topograph in self.topograph_list:
                nodes, dicts = zip(*topograph.nodes(data=True))
                nodes_list.append(nodes)
                dicts_list.append(dicts)

            for ((dicts1, nodes1), (dicts2, nodes2)) in itertools.combinations(zip(dicts_list, nodes_list), 2):
                # iterate over all nodes in nodes1 and nodes2
                for idx1, (node1, dict1) in enumerate(zip(nodes1, dicts1)):
                    for idx2, (node2, dict2) in enumerate(zip(nodes2, dicts2)):
                        # compute semantic similarity
                        sim12 = node1.node.value.path_similarity(node2.node.value)
                        if sim12 > 0.3:
                            pos1, pos2 = dict1['pos'][0:2], dict2['pos'][0:2]
                            plt.gca().add_patch(FancyArrowPatch((pos1[0], pos1[1]), (pos2[0], pos2[1]),
                                                                arrowstyle='<->', mutation_scale=30,
                                                                connectionstyle='arc3, rad=-0.1', color='#B54343',
                                                                linestyle='dotted', linewidth=2.5 * sim12 ** 0.6))

        plt.grid()
        plt.ylim([-1, 7])
        plt.xlim([0, 7])

    def show(self):
        self.draw()
        plt.show()
        plt.close()

    def save(self, filename):
        self.draw()
        plt.savefig(filename)
        plt.close()

    def set_label_color(self, col):
        self.label_color = col

    def set_font_size(self, size):
        self.font_size = size
