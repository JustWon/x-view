import seaborn as sns
from matplotlib import pylab as plt
import numpy as np
import os


# Seaborn setup.
sns.set_style("darkgrid", {"axes.facecolor": ".9", "axes.labelcolor": "0"})
sns.set_context("paper", font_scale=1.5, rc={"lines.linewidth": 1.5, "lines.markersize": 5})

line_color = "#218092"
error_color = "#ed8d30"


def convertLineToArray(line):
    arrayOfStrings = line.split(" ")
    arrayOfStrings = [val for val in arrayOfStrings if val is not "\t"]
    arrayOfStrings = [val for val in arrayOfStrings if val is not "\n"]
    arrayOfStrings = arrayOfStrings[1:]
    arrayOfTimes = list(map(float, arrayOfStrings))

    return arrayOfTimes


def getNumNodes(line):
    arrayOfStrings = line.split(" ")

    last_string = arrayOfStrings[-1]
    if last_string[:-1] == '\n':
        last_string = last_string[0:-1]

    return int(last_string)


def getNumEdges(line):
    arrayOfStrings = line.split(" ")

    last_string = arrayOfStrings[-1]
    if last_string[:-1] == '\n':
        last_string = last_string[0:-1]

    return int(last_string)


def plotTimes(x_axis, time_name, x_label, y_label, title, times, output_folder):
    f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))
    num_data_points_x = len(x_axis)
    num_data_points_y = min(len(samples) for samples in times[time_name])
    means = []
    for sample in range(num_data_points_y):
        mean = 0
        for evaluation in range(len(times[time_name])):
            mean += times[time_name][evaluation][sample]
        means.append(mean / len(times[time_name]))
    errors = []
    for sample in range(num_data_points_y):
        std = 0
        for evaluation in range(len(times[time_name])):
            std += (means[sample] - times[time_name][evaluation][sample]) ** 2
        errors.append(1.0 / (len(times[time_name]) - 1) * np.sqrt(std))

    num_data_points = min([num_data_points_y, num_data_points_x])
    ax.errorbar(x_axis[-num_data_points:], means[-num_data_points:], yerr=errors, fmt='o', color=error_color)
    g = ax.plot(x_axis[-num_data_points:], means[-num_data_points:], color=line_color)
    plt.title("{} ({} rep)\n\n".format(title, len(times[time_name])))
    plt.xlabel(x_label)
    plt.ylabel("{}".format(y_label))
    plt.xlim(0, max(x_axis) + 20)


    f.savefig(os.path.join(output_folder, "{}_vs_{}".format(x_label, time_name)) + ".pdf", bbox_inches='tight')


def main():
    # Folder containing all timer evaluations. This folder must contain a set of N subfolders all related to the same
    # configuration, this allows to perform statistics on the timings.
    times_folder = "/home/carlo/Desktop/x_view_plot/ressources/auto_run/evaluation"
    all_times_filename = "all_timings_graph_building_.dat"

    # Folder containing the merged graphs. The number of vertices and edges are queried by this files.
    graphs_folder = "/home/carlo/Desktop/x_view_plot/ressources/auto_run/graphs"

    # Folder used to generate figures.
    output_folder = "/home/carlo/Desktop/x_view_plot/results"

    # Dictionary of times keyed by the timer names.
    times = {"DuplicateMerging": [],
             "GraphGrowing": [],
             "VertexLinking": [],
             "ProcessFrameData": [],
             "SemanticLandmarkExtraction": [],
             "VertexMerging": [],
             "GlobalRandomWalksGeneration": [],
             "BlobExtraction": [],
             "GraphBuilding": [],
             "GraphMatching": [],
             "QueryRandomWalksGeneration": [],
             "VertexMatching": []
             }

    # Arrays of node and edge evolution over frames.
    nodes_over_time = []
    edges_over_time = []

    # Iterate over all evaluation folders contained in the times_folder.
    # Append each evaluation to the corresponding timer.
    for eval in os.listdir(times_folder):
        f = open(os.path.join(os.path.join(times_folder, eval), all_times_filename), 'r')
        for line in f:
            for key in times:
                if key in line:
                    times[key].append(convertLineToArray(line))

    # Count the number of vertices and edges over the frames
    for graph_file in sorted(os.listdir(graphs_folder)):
        with open(os.path.join(graphs_folder, graph_file), 'r') as g:
            num_nodes = -1
            num_edges = -1
            for line in g:
                if "Num vertices" in line:
                    num_nodes = getNumNodes(line)
                elif "Num edges" in line:
                    num_edges = getNumEdges(line)
            if num_nodes == -1 or num_edges == -1:
                raise ValueError("Num edges was {}, Num vertices was {}".format(num_edges, num_nodes))

            nodes_over_time.append(num_nodes)
            edges_over_time.append(num_edges)

    for key in times:
        plotTimes(x_axis=nodes_over_time, time_name=key, x_label="Number of nodes", y_label="Execution time [s]",
                  title="Num nodes VS {}".format(key), times=times, output_folder=output_folder)


if __name__ == '__main__':
    main()
