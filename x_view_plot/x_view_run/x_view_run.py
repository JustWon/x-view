import os
import select
import sys


class XViewRun:
    def __init__(self, x_view_run_dir, x_view_evaluation_output_dir, x_view_graph_output_dir, x_view_config_file,
                 evaluation_storage_dir, run_command=None):

        self._x_view_run_dir = x_view_run_dir
        self._x_view_evaluation_output_dir = x_view_evaluation_output_dir
        self._x_view_graph_output_dir = x_view_graph_output_dir
        self._x_view_config_file = x_view_config_file
        self._evaluation_storage_dir = evaluation_storage_dir

        self._createStorageDir()

        command = "cd {}".format(self._x_view_run_dir)
        command += " && "
        if run_command is None:
            command += "roslaunch x_view_bag_reader x_view_bag_node.launch"
        else:
            command += run_command
        self._run_command = command

    def run(self, num_runs=1, store_eval=True):

        for i in range(num_runs):
            os.system(self._run_command)
            if store_eval is True:
                self._copyEvaluationResults(run_number=i)

    def _createStorageDir(self):

        if not os.path.exists(self._evaluation_storage_dir):
            os.mkdir(self._evaluation_storage_dir)
        else:
            print("Output directory {} is existing.\nContinuing will potentially overwrite its content.".format(
                self._evaluation_storage_dir))
            wait_time = 10.0
            print("Do you want to continue? ({} seconds to answer) [y/n]".format(wait_time))
            i, _, _ = select.select([sys.stdin], [], [], 10)
            if i:
                input_string = sys.stdin.readline().strip()
                if input_string in ['n', 'N']:
                    print("Aborting")
                    exit(0)
                else:
                    print("Continuing on current directory.")
            else:
                print("No input given, continuing on current directory.")

    def _copyEvaluationResults(self, run_number):

        current_run_dir = os.path.join(self._evaluation_storage_dir, "run{0:03d}".format(run_number))
        if os.path.exists(current_run_dir):
            os.system("rm -rf {}".format(current_run_dir))
        os.mkdir(current_run_dir)

        # Copy the current config file.
        os.system("cp {} {}".format(self._x_view_config_file, current_run_dir))

        # Copy the evaluation folder.
        os.system("cp -rf {} {}".format(self._x_view_evaluation_output_dir, os.path.join(current_run_dir, "eval")))

        # Copy the generated graphs.
        graph_dir = os.path.join(current_run_dir, "graphs")
        os.mkdir(graph_dir)
        os.system("cp -rf {} {}".format(os.path.join(self._x_view_graph_output_dir, "*.dot"), graph_dir))
