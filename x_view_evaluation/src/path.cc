#include <x_view_evaluation/path.h>

#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

#include <glog/logging.h>

namespace x_view_evaluation {

const std::string Path::generateDirectoryPath(const ros::NodeHandle& nh,
                                              const std::string& dir_name) {

  // The base directory is the default output directory.
  std::string base_evaluation_output_dir = x_view::getOutputDirectory();

  // If specified in the rosparam server, use that as base directory.
  std::string base_evaluation_output_dir_ros;
  if (nh.getParam("/XViewBagReader/evaluation_directory",
                  base_evaluation_output_dir_ros)) {
    base_evaluation_output_dir = base_evaluation_output_dir_ros;
  } else {
    LOG(WARNING) << "Failed to get param "
        "'/XViewBagReader/evaluation_directory'. Using default directory <"
                 << base_evaluation_output_dir << ".";
  }

  // Make sure the directory ends with a '/' char.
  if (base_evaluation_output_dir.back() != '/')
    base_evaluation_output_dir.push_back('/');

  std::string dir_name_ = dir_name;
  if(dir_name.back() != '/')
    dir_name_.push_back('/');

  return base_evaluation_output_dir + dir_name_;

}

}

