import os
import numpy as np

def getLastResultsDir(resource_path):

    all_dirs = [os.path.join(resource_path, d) for d in os.listdir(resource_path)]
    all_dirs.sort(key=lambda x: -os.path.getmtime(x))
    last_result = all_dirs[0]
    return last_result


def getTimes(base_path, time_name, timings_file_name):

    times = []

    for run_folder in os.listdir(base_path):
        run_folder_path = os.path.join(base_path, run_folder)
        eval_path = os.path.join(run_folder_path, "eval")
        all_timings_path = os.path.join(eval_path, timings_file_name)

        if not os.path.exists(all_timings_path):
            print("Path {} does not exist.".format(all_timings_path))

        with open(all_timings_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if time_name in line:
                    array_of_strings = line.split(" ")
                    array_of_strings = [val for val in array_of_strings if val not in ["\t", "\n"]]
                    array_of_strings = array_of_strings[1:]
                    array_of_times = list(map(float, array_of_strings))
                    times.append(array_of_times)

    if len(times) == 0:
        print("Could not extract timing from folder {}".format(base_path))
        return []

    times_array = np.array(times)
    return times_array


def getVertices(base_path):

    vertices = []

    for run_folder in os.listdir(base_path):
        run_folder_path = os.path.join(base_path, run_folder)
        graph_path = os.path.join(run_folder_path, "graphs")

        if not os.path.exists(graph_path):
            print("Path {} does not exist.".format(graph_path))
        else:
            # Count the number of vertices over the frames.
            local_vertices = []
            for graph_file in sorted(os.listdir(graph_path)):
                with open(os.path.join(graph_path, graph_file), 'r') as g:
                    num_vertices = -1
                    for line in g:
                        if "Num vertices" in line:
                            array_of_strings = line.split(" ")
                            last_string = array_of_strings[-1]
                            if last_string[:-1] == '\n':
                                last_string = last_string[0:-1]
                            num_vertices = int(last_string)

                    local_vertices.append(num_vertices)

            vertices.append(local_vertices)

    if len(vertices) == 0:
        print("Could not extract vertices from folder {}".format(base_path))
        return []

    vertices_array = np.array(vertices)

    return vertices_array

def getEdges(base_path):

    edges = []

    for run_folder in os.listdir(base_path):
        run_folder_path = os.path.join(base_path, run_folder)
        graph_path = os.path.join(run_folder_path, "graphs")

        if not os.path.exists(graph_path):
            print("Path {} does not exist.".format(graph_path))
        else:
            # Count the number of edges over the frames.
            local_edges = []
            for graph_file in sorted(os.listdir(graph_path)):
                with open(os.path.join(graph_path, graph_file), 'r') as g:
                    num_vertices = -1
                    for line in g:
                        if "Num edges" in line:
                            array_of_strings = line.split(" ")
                            last_string = array_of_strings[-1]
                            if last_string[:-1] == '\n':
                                last_string = last_string[0:-1]
                            num_vertices = int(last_string)

                    local_edges.append(num_vertices)

            edges.append(local_edges)

    if len(edges) == 0:
        print("Could not extract edges from folder {}".format(base_path))
        return []

    edges_array = np.array(edges)

    return edges_array

