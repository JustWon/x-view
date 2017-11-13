#ifndef X_VIEW_EVALUATION_PATH_H
#define X_VIEW_EVALUATION_PATH_H

#include <ros/ros.h>

#include <string>

namespace x_view_evaluation {

class Path {

 public:

  /**
   * \brief Generates a string representing the full path of the directory to
   * be used to write all evaluation results.
   * \param nh Node handle reference to rosnode containing all parameters.
   * \param dir_name Internal directory name.
   * \return Full path to the directory where to write the evaluation results.
   * \note The structure of the generated path is the following:
   *  "/{....}/outer_directory/dir_name/"
   *  where the outer_directory is either specified as a ros param as
   *  "/XViewBagReader/evaluation_directory", or if not specified the default
   *  x_view::getOutputDirectory() will be used.
   */
  static const std::string generateDirectoryPath(const ros::NodeHandle& nh,
                                                 const std::string& dir_name);
 private:

};

}

#endif //X_VIEW_EVALUATION_DIRECTORY_H
