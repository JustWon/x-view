from x_view_run import XViewRun, XViewConfig
from x_view_data import getTimes, getLastResultsDir, getLocalizations, getVertices, getEdges, XViewPR, \
    getMeanDistance, computeSuccessRate, getSimilarities

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
resources_dir = os.path.join(current_dir, "test")

# Path to folder used to collect generated data (local path).
destination_dir = os.path.join(resources_dir, run_name)

# Path to folder used for generated graphs.
output_folder = os.path.join(current_dir, "results")


def launchXView(runs):
    # Create a config file generator.
    x_view_config = XViewConfig(x_view_config_file=x_view_cfg_file)

    outlier_rejections = [True, False]
    consistency_sizes = [3, 7]


    for outlier_rejection in outlier_rejections:
        for consistency_size in consistency_sizes:
            custom_arguments = {
                "run_name": run_name,
                "local_graph_steps": 8,
                "end_frame": 400,
                "outlier_rejection": outlier_rejection,
                "consistency_size": consistency_size
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


def plotLocalizationPR():
    f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))
    for d in sorted(os.listdir(resources_dir)):
        if d == ".keep":
            continue

        print("Computing PR curve for {}".format(d))

        full_config_dir = os.path.join(resources_dir, d)
        run_directory = getLastResultsDir(full_config_dir)
        eval_directory = os.path.join(run_directory, "eval")
        localization_file_name = os.path.join(eval_directory, "all_localizations_localization_.dat")

        x_view_pr = XViewPR(filename=localization_file_name, true_threshold=13.0)
        PR = x_view_pr.computePR()
        ax.plot(PR[:, 1], PR[:, 0], label=d)

    plt.title("PR curves")
    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.legend()

    plt.ylim([0, 1.05])
    plt.xlim([0, 1.0])

    plt.tight_layout()
    f.savefig(os.path.join(output_folder, "PR_curves.pdf"))
    plt.show()

    plt.close()


def plotSuccessRate():
    def hasCandidateNumber(d, candidate_number):
        return "{}_True".format(candidate_number) in d

    def hasConsistencyThreshold(d, consistency_threshold):
        return "{}_250".format(consistency_threshold) in d

    def hasConsistencySize(d, consistency_size):
        return "{}".format(consistency_size) in d

    def hasLocalGraphSteps(d, local_graph_steps):
        return "250_{}".format(local_graph_steps) in d

    def hasSamplingType(d, sampling_type):
        return sampling_type in d

    distance_thresholds = np.linspace(0, 100, 100)

    f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))

    for d in sorted(os.listdir(resources_dir)):
        if d == ".keep" in d:
            continue

        print("Computing success rate for {}".format(d))

        full_config_dir = os.path.join(resources_dir, d)
        run_directory = getLastResultsDir(full_config_dir)
        eval_directory = os.path.join(run_directory, "eval")
        localization_file_name = os.path.join(eval_directory, "all_localizations_localization_.dat")

        ground_truths, estimations, _ = getLocalizations(localization_file_name)
        success_rate, filtered = computeSuccessRate(distance_thresholds, ground_truths, estimations)

        ax.plot(distance_thresholds, success_rate, label=d + "-{}%".format(int(filtered * 100)))

    plt.title("Success rate")
    plt.xlabel("Success distance")
    plt.ylabel("Success rate")
    plt.legend()

    plt.ylim([0, 1])

    plt.tight_layout()
    plt.show()

    # f.savefig(os.path.join(output_folder, "Success Rate consistency thresh {} candidates {}.pdf".format(
    #    consistency_thresh, candidates)))


def plotVertexSimilarityPR():

    f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))
    for d in sorted(os.listdir(resources_dir)):
        if d == ".keep":
            continue


        print("Computing PR curve for {}".format(d))
        full_config_dir = os.path.join(resources_dir, d)
        run_directory = getLastResultsDir(full_config_dir)
        eval_directory = os.path.join(run_directory, "eval")
        localization_file_name = os.path.join(eval_directory, "similarities_similarity_.dat")

        db_vertices, query_vertices, similarities = getSimilarities(localization_file_name)

        # two vertices are a true match if their euclidean distance is smaller than true_radius
        true_radius = 10.0
        similarity_sweep = np.linspace(0.0, 1.0, 101)
        precisions = []
        recalls = []
        distances = [np.linalg.norm(query_vertex - db_vertex) for query_vertex, db_vertex in
                     zip(query_vertices, db_vertices)]

        for similarity in similarity_sweep:
            true_positives = 0
            true_negatives = 0
            false_positives = 0
            false_negatives = 0

            for dist, sim in zip(distances, similarities):
                if sim > similarity:
                    if dist < true_radius:
                        true_positives += 1
                    else:
                        false_positives += 1
                else:
                    if dist < true_radius:
                        false_negatives += 1
                    else:
                        true_negatives += 1

            precision = 1
            if true_positives + false_positives > 0:
                precision = (1.0 * true_positives) / (true_positives + false_positives)

            recall = 0
            if true_positives + false_negatives > 0:
                recall = (1.0 * true_positives) / (true_positives + false_negatives)

            precisions.append(precision)
            recalls.append(recall)

        precisions = np.array(precisions)
        recalls = np.array(recalls)

        order = recalls.argsort()
        ax.plot(recalls[order], precisions[order], label=d)

        # Also plot the similarity score associate to the recall
        step = 1000
        for i in range(int(len(similarity_sweep) / step)):
            index = i * step
            s = similarity_sweep[index]
            recall = recalls[index]
            precision = precisions[index]
            ax.text(recall, precision, "{:.2}".format(s))

    plt.title("PR curve Semantic Similarity")
    plt.xlabel("Recall")
    plt.ylabel("Precision")
    plt.legend()

    plt.ylim([0, 1.05])
    plt.xlim([0, 1])

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # launchXView(runs=1)

    # Seems working best: "/home/carlo/polybox/Master/Master
    # thesis/x-view/x_view_plot/new_candidate_comparison/AutoRun-18-48-21_2.0_5.0_400_5_3_True_WEIGHTED"

    # plotLastTimings()

    plotLocalizationPR()

    plotSuccessRate()

    plotVertexSimilarityPR()
