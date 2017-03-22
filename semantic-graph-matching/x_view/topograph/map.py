from ..restricted_classes import RestrictedSimilarityTable
import numpy as np
from enum import Enum


class GridMap2D(object):
    class Orientation(Enum):
        UP = 0
        RIGHT = 1
        DOWN = 2
        LEFT = 3

    class Pose(object):
        def __init__(self, x, y, orientation):
            self.__x, self.__y = x, y
            self.__orientation = orientation

        def next_orientation(self):
            orientation_change = np.random.randint(-1, 2)
            next_orientation = GridMap2D.Orientation((self.__orientation.value + orientation_change) % 4)
            return next_orientation

        def next_pose(self):
            next_x = self.__x + (self.__orientation == GridMap2D.Orientation.RIGHT) - (
                self.__orientation == GridMap2D.Orientation.LEFT)
            next_y = self.__y + (self.__orientation == GridMap2D.Orientation.UP) - (
                self.__orientation == GridMap2D.Orientation.DOWN)

            next_orientation = self.next_orientation()

            next_pose = GridMap2D.Pose(next_x, next_y, next_orientation)
            return next_pose

        @property
        def position(self):
            return np.array([self.__x, self.__y])

        @property
        def orientation(self):
            return self.__orientation.value

    class Crossing(object):
        """Class to represent crossings in a 2D grid-map
        """

        def __init__(self, pos_x, pos_y):
            self.__pos_x = pos_x
            self.__pos_y = pos_y
            self.__landmarks = []

        # property
        def coord(self):
            return np.array([self.__pos_x, self.__pos_y])

        def add_landmark(self, landmark):
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

        def __str__(self):
            string = "Pos: ({}, {})".format(self.__pos_x, self.__pos_y)
            string += "\nLandmarks: "
            for landmark in self.__landmarks:
                string += " <{}>, ".format(landmark)
            return string

    def __init__(self, nx, ny):
        self.__nx, self.__ny = nx, ny
        self.__map = [[GridMap2D.Crossing(j, i) for i in range(ny)] for j in range(nx)]

    def __getitem__(self, item):
        if not (0<=item[0]<self.size[0]):
            print("X range: ", item)
        if not (0<=item[1]<self.size[1]):
            print("Y range: ", item)
            print(self.size)
        return self.__map[item[0]][item[1]]

    def __str__(self):
        string = ""
        for i in range(self.__ny):
            for j in range(self.__nx):
                string += self[j, i].__str__() + "\n"

        return string

    @property
    def size(self):
        return self.__nx, self.__ny

    def observe(self, coord, detection_probability=0.8):
        if len(coord) != 2:
            raise TypeError("coord parameter passed to 'observe' function must be 2-dimensional")

        crossing = self[coord]
        all_landmarks = crossing.landmarks
        detected_landmarks = []
        for landmark in all_landmarks:
            prob = np.random.rand()
            if prob < detection_probability:
                detected_landmarks.append(landmark)

        return detected_landmarks

    def generate_random_path(self, path_length=None):
        if path_length is None:
            path_length = self.__nx + self.__ny
        pose_path = []
        while len(pose_path) < path_length:
            pose_path = []
            reset = False

            initial_pose = GridMap2D.Pose(x=np.random.randint(0, self.__nx), y=np.random.randint(0, self.__ny),
                                          orientation=GridMap2D.Orientation(np.random.randint(0, 4)))

            pose_path.append(initial_pose)

            for i in range(1, path_length):
                if reset is False:
                    last_pose = pose_path[i - 1]
                    new_pose = last_pose.next_pose()

                    if not (0 <= new_pose.position[0] < self.size[0] and 0 <= new_pose.position[1] < self.size[1]):
                        reset = True
                    elif any(all(new_pose.position == pos.position) for pos in pose_path):
                        reset = True
                    else:
                        pose_path.append(new_pose)

        return [pose.position for pose in pose_path], [pose.orientation for pose in pose_path]


def generate_map(nx, ny, landmarks_per_crossing=3):
    map2d = GridMap2D(nx, ny)
    semantic_concepts = RestrictedSimilarityTable.semantic_concepts
    for i in range(map2d.size[1]):
        for j in range(map2d.size[0]):
            num_landmarks = int(landmarks_per_crossing + 0.5 * np.random.randint(-landmarks_per_crossing + 1,
                                                                       landmarks_per_crossing))
            for landmark_idx in range(num_landmarks):
                semantic_idx = np.random.randint(0, len(semantic_concepts))
                map2d[j, i].add_landmark(semantic_concepts[semantic_idx])

    return map2d
