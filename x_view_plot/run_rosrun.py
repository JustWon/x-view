import os


def main():
    x_view_launch_dir = "/home/carlo/x-view_ws_release"

    x_view_cfg_file = "/home/carlo/x-view_ws_release/src/x-view/x_view_ros/x_view_bag_reader/cfg/x_view_bag_reader.yaml"

    x_view_produced_dir = "/home/carlo/x-view_ws_release/src/x-view/x_view_core/output/Example_run"

    x_view_produced_graph_dir = "/home/carlo/x-view_ws_release/src/x-view/x_view_core/output"

    destination_dir = "/home/carlo/Desktop/x_view_plot/ressources/auto_run/"

    num_runs = 50

    for run in range(num_runs):
        command = ""
        command += "cd {}".format(x_view_launch_dir)
        command += " && "
        command += "roslaunch x_view_bag_reader x_view_bag_node.launch"

        os.system(command)

        # Prepare a new directory for this run.
        run_directory = os.path.join(os.path.join(destination_dir, "evaluation"), "eval{0:03d}".format(run))
        if os.path.exists(run_directory):
            os.system("rm -rf {}".format(run_directory))
        os.mkdir(run_directory)

        os.system("cp {} {}".format(x_view_cfg_file, run_directory))
        os.system("cp {} {}".format(os.path.join(x_view_produced_dir, "*"), run_directory))

        # Prepare a directory for the graphs.
        graph_directory = os.path.join(destination_dir, "graphs")
        if not os.path.exists(graph_directory):
            os.mkdir(graph_directory)

        os.system("cp {} {}".format(os.path.join(x_view_produced_graph_dir, "*.dot"), graph_directory))


if __name__ == '__main__':
    main()
