"""Direct interface to wordnet.

This is an example script which directly interacts with wordnet library
"""

from nltk.corpus import wordnet as wn
import argparse


def processSynonyms(word, synsetIndex=0):
    print("Definitions of synonyms of word '{0}' are: ".format(word))
    for i, syn in enumerate(wn.synsets(word)):
        if i == synsetIndex:
            print("[ --> ] ", end="")
        print("{0}: {1}".format(i, syn.definition()), end="")
        print("\t(with corresponding example: {})".format(syn.examples()), end="")
        if i == synsetIndex:
            print(" [ <-- ]", end="")
        print("")

    print("")


def processIsARelationship(word, synsetIndex=0):
    def getMoreGeneralSynSet(nodeSynSet):
        # hypernyms are synsets that are more general
        return nodeSynSet.hypernyms()

    def getModeSpecificSynSet(nodeSynSet):
        # hyponyms are synsets that are more specific
        return nodeSynSet.hyponyms()

    nodeSynSet = wn.synsets(word)[synsetIndex]

    print("Hypernyms (parent) of synonymous set n. {} ({}) of {} is:".format(str(synsetIndex),
                                                                             nodeSynSet, word))
    hypernyms = getMoreGeneralSynSet(nodeSynSet=nodeSynSet)
    for i, hypernym in enumerate(hypernyms):
        print("{0}: {1}".format(i, hypernym))

    print("")

    print("Hyponyms (children) of synonymous set n. {} ({}) of {} is:".format(str(synsetIndex),
                                                                              nodeSynSet, word))
    hyponyms = getModeSpecificSynSet(nodeSynSet=nodeSynSet)
    for i, hyponym in enumerate(hyponyms):
        print("{0}: {1}".format(i, hyponym))


def processIsMadeOfRelationship(word, synsetIndex=0):
    def getThingsMadeOf(nodeSynSet):
        # holonyms are things that the item is contained in
        return nodeSynSet.member_holonyms()

    def getThingsWhichMake(nodeSynSet):
        # meronyms are components or substances that make up the item
        return nodeSynSet.part_meronyms()

    nodeSynSet = wn.synsets(word)[synsetIndex]

    print("Holonyms (object is part of) of synonymous set n. {} ({}) of {} is:".format(str(synsetIndex),
                                                                                       nodeSynSet,
                                                                                       word))
    holonyms = getThingsMadeOf(nodeSynSet=nodeSynSet)
    for i, holonym in enumerate(holonyms):
        print("{0}: {1}".format(i, holonym))

    print("")

    print("Meronyms (components) of synonymous set n. {} ({}) of {} is:".format(str(synsetIndex),
                                                                                nodeSynSet, word))
    meronyms = getThingsWhichMake(nodeSynSet=nodeSynSet)
    for i, meronym in enumerate(meronyms):
        print("{0}: {1}".format(i, meronym))


def main():
    print(__doc__)

    parser = argparse.ArgumentParser()
    parser.add_argument('-w', '--word', action='store', dest='word',
                        help='Semantic concept to be analyzed')

    parser.add_argument('-s', '--synsetIndex', action='store', dest='synsetIndex', type=int,
                        help='Synset index to be analyzed')

    prs = parser.parse_args()

    semanticConcept = prs.word if prs.word else "plant"
    synsetIndex = prs.synsetIndex if prs.synsetIndex else 0

    # in this case we don't need to use the Word object provided by textblob
    word = semanticConcept

    processSynonyms(word=word, synsetIndex=synsetIndex)
    print("")

    # Semantic analysis
    processIsARelationship(word=word, synsetIndex=synsetIndex)
    print("")

    processIsMadeOfRelationship(word=word, synsetIndex=synsetIndex)
    print("")


if __name__ == '__main__':
    main()
