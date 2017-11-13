import os


class XViewConfig:
    def __init__(self, x_view_config_file, config_file_skeleton=None):
        self._config_file_name = x_view_config_file

        config_file_skeleton_ = config_file_skeleton
        if config_file_skeleton_ is None:
            config_file_skeleton_ = self._loadDefaultConfigFile()

        self.current_config_file = None
        self._config_file_skeleton = config_file_skeleton_

        self.current_args = None
        self._default_args = {
            "dataset_name": "SYNTHIA",
            "min_blob_size": 2000,
            "num_dilate_erode": 4,
            "extraction_type": "3D_SPACE",
            "max_euclidean_distance": 12.0,
            "blob_neighbor_distance": 10,
            "depth_clip": 50.0,
            "vertex_similarity_score": "WEIGHTED",
            "random_walk_sampling_type": "NON_RETURNING",
            "num_walks": 300,
            "walk_length": 3,
            "time_window": 10,
            "similarity_threshold": 0.3,
            "distance_threshold": 5.0,
            "merge_close_vertices": True,
            "merge_distance": 2.0,
            "link_close_vertices": True,
            "max_link_distance": 5.0,
            "outlier_rejection": True,
            "num_candidate_matches": 3,
            "consistency_threshold": 2,
            "consistency_size": 2.5,
            "localizer_type": "OPTIMIZATION",
            "use_robust_noise": False,
            "start_frame": 0,
            "end_frame": 200,
            "relabeling_percentage": 0.0,
            "graph_construction_camera": "FRONT",
            "localization_camera": "BACK",
            "local_graph_steps": 5,
            "run_name": "Example_run"

        }

    def _loadDefaultConfigFile(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        default_config_path = os.path.join(os.path.join(current_dir, "resources"), "default_config_skeleton.txt")
        with open(default_config_path, 'r') as file:
            file_content = file.read()
            return file_content

    def writeConfigFile(self, arguments):

        for key in arguments:
            if key not in self._default_args:
                Warning("Key {} does not exist.".format(key))
        merged_args = self._default_args.copy()
        merged_args.update(arguments)

        self.current_args = merged_args.copy()

        self.current_config_file = self._config_file_skeleton.format(**merged_args)

        with open(self._config_file_name, 'w') as file:
            file.write(self.current_config_file)
