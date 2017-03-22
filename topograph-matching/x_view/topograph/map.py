from ..restricted_classes import RestrictedSimilarityTable
import numpy as np


class Crossing(object):
    def __init__(self, pos_x, pos_y):
        self.__pos_x = pos_x
        self.__pos_y = pos_y
        self.__landmarks = []

    def pos(self):
        return (self.__pos_x, self.__pos_y)

    def add_landmark(self, landmark):
        if landmark not in self.__landmarks:
            self.__landmarks.append(landmark)

    @property
    def landmarks(self):
        return self.__landmarks

    @landmarks.setter
    def landmarks(self, landmarks):
        # Do something if you want
        self.__landmarks = landmarks

    def contains(self, landmark):
        return landmark in self.__landmarks


class Map2D(object):
    def __init__(self, nx, ny):
        self.__nx, self.__ny = nx, ny
        self.__map = [[Crossing(j, i) for i in range(ny)] for j in range(nx)]

    def __getitem__(self, item):
        return self.__map[item[0]][item[1]]

    @property
    def size(self):
        return (self.__nx, self.__ny)


def generate_map(nx, ny, landmarks_per_crossing=3):
    map2d = Map2D(nx, ny)
    semantic_concepts = RestrictedSimilarityTable.semantic_concepts
    for i in range(map2d.size[1]):
        for j in range(map2d.size[0]):
            num_landmarks = landmarks_per_crossing + np.random.randint(-landmarks_per_crossing + 1,
                                                                       landmarks_per_crossing)
            for landmark_idx in range(num_landmarks):
                semantic_idx = np.random.randint(0, len(semantic_concepts))
                map2d[j, i].add_landmark(semantic_concepts[semantic_idx])

    return map2d
