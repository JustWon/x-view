class RestrictedSimilarityTable(object):
    """Facility class to handle and query similarities between semantic concepts
    """
    __instance = None
    semantic_concepts = ['sky', 'building', 'road', 'sidewalk', 'vegetation', 'pole', 'car', 'traffic sign',
                         'pedestrian', 'cyclist', 'lane-marking', 'miscellaneous']

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
        return {"lane-marking": {"lane-marking": 1, "cyclist": 0, "pole": 0, "vegetation": 0, "sky": 0, "car": 0.1,
                                 "sidewalk": 0.1, "building": 0, "miscellaneous": 0, "pedestrian": 0, "traffic sign": 0,
                                 "road": 0.95},
                "pole": {"pole": 1, "cyclist": 0.1, "lane-marking": 0, "vegetation": 0, "sky": 0, "car": 0.7,
                         "sidewalk": 0.5, "building": 0, "miscellaneous": 0, "pedestrian": 0.1, "traffic sign": 0.8,
                         "road": 0.5},
                "sky": {"sky": 0, "miscellaneous": 0, "cyclist": 0, "lane-marking": 0, "pole": 0, "car": 0,
                        "sidewalk": 0, "building": 0, "vegetation": 0, "pedestrian": 0, "traffic sign": 0, "road": 0},
                "sidewalk": {"sidewalk": 1, "miscellaneous": 0, "cyclist": 0.4, "lane-marking": 0.1, "pole": 0.5,
                             "sky": 0, "car": 0.7, "building": 0, "vegetation": 0, "pedestrian": 0.7,
                             "traffic sign": 0.7, "road": 0.7},
                "cyclist": {"cyclist": 1, "miscellaneous": 0, "pole": 0.1, "lane-marking": 0, "vegetation": 0, "sky": 0,
                            "car": 0.5, "sidewalk": 0.4, "building": 0, "pedestrian": 0.7, "traffic sign": 0,
                            "road": 0.7},
                "building": {"building": 1, "cyclist": 0, "lane-marking": 0, "pole": 0, "sky": 0, "car": 0,
                             "sidewalk": 0, "road": 0, "miscellaneous": 0, "pedestrian": 0, "traffic sign": 0,
                             "vegetation": 0},
                "miscellaneous": {"miscellaneous": 0, "cyclist": 0, "pole": 0, "lane-marking": 0, "vegetation": 0,
                                  "sky": 0, "car": 0, "sidewalk": 0, "building": 0, "pedestrian": 0, "traffic sign": 0,
                                  "road": 0},
                "vegetation": {"vegetation": 0.2, "cyclist": 0, "lane-marking": 0, "pole": 0, "sky": 0, "car": 0,
                               "sidewalk": 0, "building": 0, "miscellaneous": 0, "pedestrian": 0, "traffic sign": 0,
                               "road": 0},
                "car": {"car": 1, "cyclist": 0.5, "pole": 0.7, "lane-marking": 0.1, "vegetation": 0, "sky": 0,
                        "sidewalk": 0.7, "building": 0, "miscellaneous": 0, "pedestrian": 0.3, "traffic sign": 0.3,
                        "road": 0.8},
                "road": {"road": 1, "cyclist": 0.7, "lane-marking": 0.95, "pole": 0.5, "sky": 0, "car": 0.8,
                         "sidewalk": 0.7, "building": 0, "miscellaneous": 0, "pedestrian": 0.4, "traffic sign": 0.8,
                         "vegetation": 0},
                "pedestrian": {"pedestrian": 1, "miscellaneous": 0, "pole": 0.1, "lane-marking": 0, "vegetation": 0,
                               "sky": 0, "car": 0.3, "sidewalk": 0.7, "building": 0, "cyclist": 0.7, "traffic sign": 0,
                               "road": 0.4},
                "traffic sign": {"traffic sign": 1, "cyclist": 0, "pole": 0.8, "lane-marking": 0, "vegetation": 0,
                                 "sky": 0, "car": 0.3, "sidewalk": 0.7, "building": 0, "miscellaneous": 0,
                                 "pedestrian": 0, "road": 0.8}}

    @staticmethod
    def similarity(concept1, concept2):
        """Returns the similarity between two concepts
        :param concept1: first concept to be compared
        :param concept2: second concept to be compared
        :return: a graph_node measure between concept1 and concept2, where '1' corresponds to maximal graph_node, and '0'
        to complete dissimilarity
        """
        return RestrictedSimilarityTable().__table[concept1][concept2]
