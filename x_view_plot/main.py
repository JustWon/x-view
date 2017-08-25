from x_view_run import XViewRun, XViewConfig
from x_view_data import getTimes, getLastResultsDir, getVertices, getEdges, XViewPR

import time
from matplotlib import pylab as plt
import os
import sys
import numpy as np
import seaborn as sns

# Seaborn setup.
sns.set_style("darkgrid", {"axes.facecolor": ".9", "axes.labelcolor": "0"})
sns.set_context("paper", font_scale=1.5, rc={"lines.linewidth": 1.5, "lines.markersize": 5,
                                             "legend.fontsize": 6})

line_color = "#218092"
error_color = "#ed8d30"

# Path from where to launch roslaunch.
x_view_launch_dir = "/home/carlo/x-view_ws_release"

# File path of config file read by XView, overwritten by this script using new parameters.
x_view_cfg_file = "/home/carlo/x-view_ws_release/src/x-view/x_view_ros/x_view_bag_reader/cfg/x_view_bag_reader.yaml"

current_time = time.strftime('%H-%M-%S')
run_name = "AutoRun-{}".format(current_time)

# Path to folder containing results of evaluation written by XView.
x_view_evaluation_dir = "/home/carlo/x-view_ws_release/src/x-view/x_view_core/output/{}".format(run_name)

# Path to folder containing generated graphs (.dot files)
x_view_produced_graph_dir = "/home/carlo/x-view_ws_release/src/x-view/x_view_core/output"

# Path to directory containing this file
current_dir = os.path.abspath(os.path.dirname(sys.argv[0]))

# Resource directory containing all destination_dir s
resources_dir = os.path.join(current_dir, "front-back-comparison")

# Path to folder used to collect generated data (local path).
destination_dir = os.path.join(resources_dir, run_name)

# Path to folder used for generated graphs.
output_folder = os.path.join(current_dir, "results")


def launchXView(runs):
    # Create a config file generator.
    x_view_config = XViewConfig(x_view_config_file=x_view_cfg_file)

    """
    extraction_types = ["3D_SPACE"]  # ["IMAGE", "3D_SPACE"]
    similarity_scores = ["WEIGHTED"]  # ["WEIGHTED", "SURFACE"]
    sampling_types = ["UNIFORM", "WEIGHTED"]  # ["UNIFORM", "AVOIDING", "NON_RETURNING", "WEIGHTED"]
    localization_steps = [1, 5, 10]
    construction_cameras = ["FRONT", "BACK"]
    localization_cameras = ["FRONT", "BACK"]

    for extraction_type in extraction_types:
        for similarity_score in similarity_scores:
            for sampling_type in sampling_types:
                for localization_step in localization_steps:
                    for construction_camera in construction_cameras:
                        for localization_camera in localization_cameras:
                            custom_arguments = {
                                "run_name": run_name,
                                "extraction_type": extraction_type,
                                "vertex_similarity_score": similarity_score,
                                "random_walk_sampling_type": sampling_type,
                                "local_graph_steps": localization_step,
                                "graph_construction_camera": construction_camera,
                                "localization_camera": localization_camera,
                                "end_frame": 100
                            }
    """

    cameras = [["FRONT", "FRONT"], ["FRONT", "BACK"]]
    local_graph_steps = [1, 5, 10]
    sampling_types = ["UNIFORM", "WEIGHTED"]

    for local_graph_step in local_graph_steps:
        for camera in cameras:
            for sampling_type in sampling_types:
                custom_arguments = {
                    "run_name": run_name,
                    "extraction_type": "3D_SPACE",
                    "local_graph_steps": local_graph_step,
                    "random_walk_sampling_type": sampling_type,
                    "graph_construction_camera": camera[0],
                    "localization_camera": camera[1],
                    "end_frame": 600,
                    "use_robust_noise": True
                }

                # Write the config file to the x_view_config_file.
                x_view_config.writeConfigFile(custom_arguments)

                # Create an XView executer.
                folder_suffix = ""
                for key in sorted(custom_arguments.keys()):
                    if key is not "run_name":
                        folder_suffix += "_" + str(custom_arguments[key])

                run_folder_name = run_name + folder_suffix
                run_folder_name = os.path.join(resources_dir, run_folder_name)
                x_view_run = XViewRun(x_view_run_dir=x_view_launch_dir,
                                      x_view_evaluation_output_dir=x_view_evaluation_dir,
                                      x_view_graph_output_dir=x_view_produced_graph_dir,
                                      x_view_config_file=x_view_cfg_file,
                                      evaluation_storage_dir=run_folder_name)

                # Run XView with the current arguments for num_runs times and store the evaluations.
                x_view_run.run(num_runs=runs, store_eval=True)




def plotLastTimings():
    last_directory = getLastResultsDir(resources_dir)
    print("Plotting results of {}".format(last_directory))

    num_vertices = getVertices(base_path=last_directory)
    # num_edges = getEdges(base_path=lastDirectory)

    time_names = ["DuplicateMerging",
                  "GraphGrowing",
                  "VertexLinking",
                  "ProcessFrameData",
                  "SemanticLandmarkExtraction",
                  "VertexMerging",
                  "GlobalRandomWalksGeneration",
                  "BlobExtraction",
                  "GraphBuilding",
                  "GraphMatching",
                  "QueryRandomWalksGeneration",
                  "VertexMatching"
                  ]

    for time_name in time_names:
        times = getTimes(base_path=last_directory, time_name=time_name,
                         timings_file_name="all_timings_graph_building_.dat")

        f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))

        means = np.mean(times, axis=0)
        errors = np.std(times, axis=0)

        num_vertices = np.sort(num_vertices, axis=1)

        ax.errorbar(num_vertices[0][-len(means):], means, yerr=errors, fmt='o', color=error_color)
        ax.plot(num_vertices[0][-len(means):], means, color=line_color)

        plt.title("Num vertices VS {}".format(time_name))
        plt.xlabel("Num vertices")
        plt.ylabel("Execution time [s]")

        plt.xlim([np.min(np.min(num_vertices)) - 3, np.max(np.max(num_vertices)) + 3])

        f.savefig(os.path.join(output_folder, "num_vertices_vs_{}.pdf".format(time_name)), bbox_inches='tight')


def plotPR():

    f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))

    for d in sorted(os.listdir(resources_dir)):
        print("Computing PR curve for {}".format(d))
        full_config_dir = os.path.join(resources_dir, d)
        run_directory = getLastResultsDir(full_config_dir)
        eval_directory = os.path.join(run_directory, "eval")
        localization_file_name = os.path.join(eval_directory, "all_localizations_localization_.dat")

        x_view_pr = XViewPR(filename=localization_file_name, true_threshold=0.06)
        PR = x_view_pr.computePR()

        order = PR[0:, 1].argsort()
        ax.plot(PR[order, 1], PR[order, 0], label=d)

    plt.title("PR curves")
    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.legend()

    plt.ylim([0, 1.05])

    plt.tight_layout()
    f.savefig(os.path.join(output_folder, "PR_curves.pdf"))
    plt.close()


if __name__ == '__main__':
    # launchXView(runs=1)

    # plotLastTimings()

    plotPR()
