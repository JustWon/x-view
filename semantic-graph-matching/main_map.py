import x_view as xv
import numpy as np
from matplotlib import pylab as plt


def main():
    # generate a 2D map (grid style) containing landmarks
    map2D = xv.generate_map(nx=6, ny=6, landmarks_per_crossing=4)

    def generate_pose_graph(path_length, node_color="#4286f4", edge_color="#1d3e72", pos_offset=np.array([0, 0])):
        positions, orientations = map2D.generate_random_path(path_length=path_length)
        pose_node_list = []
        edge_list = []
        for i, (pos, ori) in enumerate(zip(positions, orientations)):
            pose_node_list.append(
                xv.PoseGraphNode(node=xv.next_id(), label=str(i), pos=pos + pos_offset, orientation=ori,
                                 node_color=node_color, landmarks=map2D.observe(pos, detection_probability=0.9)))

        for i in range(1, len(pose_node_list)):
            edge_list.append(
                xv.GraphEdge(node1=pose_node_list[i - 1].node, node2=pose_node_list[i].node, edge_color=edge_color))

        return xv.generate_topograph(graph_node_list=pose_node_list, graph_edge_list=edge_list)

    g1 = generate_pose_graph(path_length=15)
    g2 = generate_pose_graph(path_length=20, node_color="#d6159f", edge_color="#23dd39",
                             pos_offset=np.array([-0.2, 0.2]))

    similarity = xv.SemanticSimilarity(pose_graph1=g1, pose_graph2=g2)
    similarity.compute_similarities()

    # Plot the similarity computed between all nodes of the two graphs
    drawer = xv.TopoGraphDrawer([g1, g2])
    drawer.font_size = 22
    drawer.node_base_size = 400
    drawer.show()

    plt.figure()
    plt.imshow(similarity.similarity_scores, interpolation='none', vmin=0, vmax=1, aspect='equal')

    ax = plt.gca()

    # Minor ticks
    ax.set_xticks(np.arange(-.5, len(g2.nodes()), 1), minor=True)
    ax.set_yticks(np.arange(-.5, len(g1.nodes()), 1), minor=True)

    ax.grid(which='minor', color='w', linestyle='-', linewidth=2)

    for (j, i), label in np.ndenumerate(similarity.similarity_scores):
        if label != 0:
            ax.text(i, j, "{0:.2f}".format(round(label, 2)), ha='center', va='center')

    plt.title("Cross similarity computation\nbetween nodes of two graphs")
    plt.xlabel("Graph 2")
    plt.ylabel("Graph 1")
    plt.show()


if __name__ == '__main__':
    np.random.seed(0)
    main()
