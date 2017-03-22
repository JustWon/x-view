import numpy as np
import itertools

from ..restricted_classes import RestrictedSimilarityTable


def jaccard_similarity(semanticBag1, semanticBag2):
    """Computes the binary similarity using Jaccard similarity for sets [https://en.wikipedia.org/wiki/Jaccard_index]
    """
    if len(semanticBag1) == 0 or len(semanticBag2) == 0:
        return 0

    num_intersection = 0
    for semanticConcept in RestrictedSimilarityTable.semantic_concepts:
        if semanticConcept in semanticBag1 and semanticConcept in semanticBag2:
            num_intersection += 1

    return 2 * num_intersection / (len(set(semanticBag1)) + len(set(semanticBag2)))


class SemanticSimilarity(object):
    def __init__(self, pose_graph1, pose_graph2):
        """Computes pairwise semantic similarities between the nodes contained in the pose graphs passed as argument
        """
        self.pose_graphs = [pose_graph1, pose_graph2]

        number_of_nodes = [len(topograph.nodes()) for topograph in self.pose_graphs]
        self.similarity_scores = np.zeros(number_of_nodes)

    def reset(self):
        self.similarity_scores.fill(0.0)

    def compute_similarities(self):
        # iterate over pairs of topographs and compute outer forces
        for ((topo1_idx, topo1), (topo2_idx, topo2)) in itertools.combinations(enumerate(self.pose_graphs), 2):
            # iterate over all nodes in topo1
            nodes1 = topo1.nodes(data=True)
            for idx1, (node1, dict1) in enumerate(nodes1):
                nodes2 = topo2.nodes(data=True)
                for idx2, (node2, dict2) in enumerate(nodes2):
                    # compute semantic graph_node
                    sim12 = jaccard_similarity(dict1['landmarks'], dict2['landmarks'])
                    self.similarity_scores[int(dict1['label']), int(dict2['label'])] = sim12


        self.similarity_scores /= self.similarity_scores.max()
        self.similarity_scores[np.array(self.similarity_scores) < 0.5] = 0
