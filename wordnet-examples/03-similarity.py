"""Direct interface to wordnet.

This is an example script which directly interacts with wordnet library and shows how one can compute
semantic similarities between objects using the wordnet library
"""

from nltk.corpus import wordnet as wn
import argparse
import itertools


def similarityTest(syns):
    """Given a list of synonymous, this function performs a list of similarity tests for each pair in the list
    """

    print("Similarity test:")
    print("")

    def similarityTestPair(synsPair):
        print("\t(shortest) Path Similarity:   {}".format(synsPair[0].path_similarity(synsPair[1])))
        print("\tLeacock-Chodorow Similarity:  {}".format(synsPair[0].lch_similarity(synsPair[1])))
        print("\tWu-Palmer Similarity:         {}".format(synsPair[0].wup_similarity(synsPair[1])))
        # print("\tResnik Similarity:            {}".format(synsPair[0].res_similarity(synsPair[1])))
        # print("\tJiang-Conrath Similarity:     {}".format(synsPair[0].jcn_similarity(synsPair[1])))
        # print("\tLin Similarity:               {}".format(synsPair[0].lin_similarity(synsPair[1])))

    for comb in itertools.combinations(syns, 2):
        print("Similarity between {} and {}".format(comb[0], comb[1]))
        similarityTestPair(comb)
        print("")


def main():
    print(__doc__)

    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--list', nargs='+', dest='conceptList')
    prs = parser.parse_args()

    conceptList = prs.conceptList if (prs.conceptList and len(prs.conceptList) > 1) else ["cat", "dog"]

    syns = []
    for concept in conceptList:
        syns.append(wn.synsets(concept)[0])

    print("Semantic concepts to be compared:")

    print(*("{}: {}".format(i, syn.definition()) for i, syn in enumerate(syns)), sep="\n")

    print("")

    similarityTest(syns=syns)


if __name__ == '__main__':
    main()
