
class RestrictedSimilarityTable(object):
    """Facility class to handle and query similarities between semantic concepts
    """
    __instance = None
    semantic_concepts = ['car', 'street', 'vehicle', 'window', 'wheel', 'chair', 'kitchen', 'office', 'computer',
                         'desk']

    def __new__(cls):
        if cls.__instance == None:
            cls.__instance = object.__new__(cls)
            cls.__instance.name = "IdFactory instance"
            cls.__table = RestrictedSimilarityTable.create_similarity_table()
        return cls.__instance

    @staticmethod
    def create_similarity_table():
        """Manual definition of similarity measures implemented as a dictionary of dictionaries accessible as
        'table[id1][id2]' where 'id1' and 'id2' are elements of 'semantic_concepts'
        :return:
        """
        return {"street": {"street": 1, "wheel": 0.2, "office": 0, "window": 0.2, "computer": 0, "desk": 0, "car": 0.7,
                           "vehicle": 0.7, "chair": 0.1, "kitchen": 0},
                "wheel": {"street": 0.2, "wheel": 1, "office": 0, "window": 0.1, "computer": 0, "desk": 0.1,
                          "car": 0.65,
                          "vehicle": 0.65, "chair": 0.4, "kitchen": 0},
                "office": {"street": 0, "wheel": 0, "office": 1, "window": 0.4, "computer": 0.9, "desk": 0.9, "car": 0,
                           "vehicle": 0, "chair": 0.3, "kitchen": 0.8},
                "window": {"street": 0.2, "wheel": 0.1, "office": 0.4, "window": 1, "computer": 0.7, "desk": 0.3,
                           "car": 0.4, "vehicle": 0.5, "chair": 0.3, "kitchen": 0.3},
                "computer": {"street": 0, "wheel": 0, "office": 0.9, "window": 0.7, "computer": 1, "desk": 0.95,
                             "car": 0,
                             "vehicle": 0, "chair": 0.6, "kitchen": 0.2},
                "desk": {"street": 0, "wheel": 0.1, "office": 0.9, "window": 0.3, "computer": 0.95, "desk": 1, "car": 0,
                         "vehicle": 0, "chair": 0.9, "kitchen": 0.3},
                "car": {"street": 0.7, "wheel": 0.65, "office": 0, "window": 0.4, "computer": 0, "desk": 0, "car": 1,
                        "vehicle": 0.95, "chair": 0.1, "kitchen": 0},
                "vehicle": {"street": 0.7, "wheel": 0.65, "office": 0, "window": 0.5, "computer": 0, "desk": 0,
                            "car": 0.95,
                            "vehicle": 1, "chair": 0.1, "kitchen": 0},
                "chair": {"street": 0.1, "wheel": 0.4, "office": 0.3, "window": 0.3, "computer": 0.6, "desk": 0.9,
                          "car": 0.1, "vehicle": 0.1, "chair": 1, "kitchen": 0.4},
                "kitchen": {"street": 0, "wheel": 0, "office": 0.8, "window": 0.3, "computer": 0.2, "desk": 0.3,
                            "car": 0,
                            "vehicle": 0, "chair": 0.4, "kitchen": 1}}

    @staticmethod
    def similarity(concept1, concept2):
        """Returns the similarity between two concepts
        :param concept1: first concept to be compared
        :param concept2: second concept to be compared
        :return: a graph_node measure between concept1 and concept2, where '1' corresponds to maximal graph_node, and '0'
        to complete dissimilarity
        """
        return RestrictedSimilarityTable().__table[concept1][concept2]
