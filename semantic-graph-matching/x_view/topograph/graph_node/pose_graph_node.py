from . import AbstractGraphNode
import numpy as np


class PoseGraphNode(AbstractGraphNode):
    def __init__(self, node, pos: np.array, orientation: int, label="map_node", node_color='#42C2ED', landmarks=None):
        """Pose graph node containing information about pose and observed landmarks
        :param node: unique index given to the node, it must be different also between other
        :param pos: 3D/3D pose of the node
        :param orientation: integer referring to the orientation of the robot
        :param label: string to be rendered by the viewer
        :param node_color: color to be used by the renderer to render the node
        :param landmarks: set of observed landmarks from this node
        """
        super(PoseGraphNode, self).__init__(node=node, pos=pos, label=label, node_color=node_color)

        self.orientation = orientation
        self.landmarks = landmarks

    def similarity(self, other):
        if not isinstance(other, PoseGraphNode):
            return 0
        return 0.1
