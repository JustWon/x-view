from . import AbstractGraphNode
import numpy as np

from nltk.corpus.reader.wordnet import Synset
from ...restricted_classes import RestrictedSimilarityTable


class WordNetGraphNode(AbstractGraphNode):
    def __init__(self, node, pos: np.array, label="wordnet_node", lemma="", plot_pos=None, node_color='42C2ED'):
        super(WordNetGraphNode, self).__init__(node=node, pos=pos, label=label, lemma=lemma, plot_pos=plot_pos,
                                               node_color=node_color)
        if not isinstance(node.value, Synset):
            raise TypeError(
                "'node.value' param passed as argument to constructor of '{}' must be an instance of 'wn.synset'".format(
                    self.__class__.__name__))

    def similarity(self, other):
        if not isinstance(other, WordNetGraphNode):
            return 0
        return self.node.value.path_similarity(other.node.value)


class RestrictedGraphNode(AbstractGraphNode):
    def __init__(self, node, pos: np.array, label="restricted_node", lemma="", plot_pos=None, node_color='42C2ED'):
        super(RestrictedGraphNode, self).__init__(node=node, pos=pos, label=label, lemma=lemma, plot_pos=plot_pos,
                                                  node_color=node_color)
        if node.value not in RestrictedSimilarityTable.semantic_concepts:
            raise ValueError(
                "'node.value' param passed as argument to constructor of '{}' must be contained in 'semantic_concepts'".format(
                    self.__class__.__name__))

    def similarity(self, other):
        if not isinstance(other, RestrictedGraphNode):
            return 0
        return RestrictedSimilarityTable.similarity(self.node.value, other.node.value)

