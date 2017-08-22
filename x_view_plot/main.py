from x_view_run import XViewRun, XViewConfig
from x_view_data import getTimes, getLastResultsDir, getVertices, getEdges, getLocalizations

import time
from matplotlib import pylab as plt
import os
import sys
import numpy as np
import seaborn as sns

# Seaborn setup.
sns.set_style("darkgrid", {"axes.facecolor": ".9", "axes.labelcolor": "0"})
sns.set_context("paper", font_scale=1.5, rc={"lines.linewidth": 1.5, "lines.markersize": 5})

line_color = "#218092"
error_color = "#ed8d30"

# Path from where to launch roslaunch.
x_view_launch_dir = "/home/carlo/x-view_ws_release"

# File path of config file read by XView
x_view_cfg_file = "/home/carlo/x-view_ws_release/src/x-view/x_view_ros/x_view_bag_reader/cfg/x_view_bag_reader.yaml"

# Path to folder containing results of evaluation.
x_view_produced_dir = "/home/carlo/x-view_ws_release/src/x-view/x_view_core/output/Example_run"

# Path to folder containing generated graphs (.dot files)
x_view_produced_graph_dir = "/home/carlo/x-view_ws_release/src/x-view/x_view_core/output"

# Path to directory containing this file
current_dir = os.path.abspath(os.path.dirname(sys.argv[0]))

# Resource directory containing all destination_dir s
resources_dir = os.path.join(current_dir, "resources")

# Path to folder used to collect generated data (local path).
destination_dir = os.path.join(resources_dir, "auto_run{}/".format(time.strftime('%H-%M-%S')))

# Path to folder used for generated graphs.
output_folder = os.path.join(current_dir, "results")


def launchXView(runs):
    # Create a config file generator.
    x_view_config = XViewConfig(x_view_config_file=x_view_cfg_file)

    # With custom arguments.
    custom_arguments = {
        "vertex_similarity_score": "WEIGHTED"
    }

    # Write the config file to the x_view_config_file.
    x_view_config.writeConfigFile(custom_arguments)

    # Create an XView executer.
    x_view_run = XViewRun(x_view_run_dir=x_view_launch_dir, x_view_evaluation_output_dir=x_view_produced_dir,
                          x_view_graph_output_dir=x_view_produced_graph_dir, x_view_config_file=x_view_cfg_file,
                          evaluation_storage_dir=destination_dir)

    # Run XView with the current arguments for num_runs times and store the evaluations.
    x_view_run.run(num_runs=runs, store_eval=True)


def plotLastResults():
    lastDirectory = getLastResultsDir(resources_dir)

    num_vertices = getVertices(base_path=lastDirectory)
    num_edges = getEdges(base_path=lastDirectory)

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
        times = getTimes(base_path=lastDirectory, time_name=time_name,
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

    gt, es = getLocalizations(base_path=lastDirectory, localization_file_name="all_localizations_localization_.dat")
    print(gt[0]["position"], es[0]["rotation"])


if __name__ == '__main__':
    # launchXView(runs = 1)

    plotLastResults()
