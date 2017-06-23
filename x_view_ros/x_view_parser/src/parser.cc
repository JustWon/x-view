#include <x_view_parser/parser.h>

#include <glog/logging.h>

using namespace x_view;

namespace x_view_ros {

void addString(const ros::NodeHandle& nh, const std::string& ros_key,
               const std::string& param_key,
               std::unique_ptr<x_view::Parameters>& parameters,
               const std::string& default_string) {
  std::string retrieved_string;
  if (nh.getParam(ros_key, retrieved_string)) {
    parameters->setString(param_key, retrieved_string);
  } else {
    LOG(WARNING) << "Could not parse string parameter <" << ros_key << ">\n"
        << "Setting default value <" << default_string << ">.";
    parameters->setString(param_key, default_string);
  }
}

void addInt(const ros::NodeHandle& nh, const std::string& ros_key,
            const std::string& param_key,
            std::unique_ptr<x_view::Parameters>& parameters,
            const int default_int) {
  int retrieved_int;
  if (nh.getParam(ros_key, retrieved_int)) {
    parameters->setInteger(param_key, retrieved_int);
  } else {
    LOG(WARNING) << "Could not parse int parameter <" << ros_key << ">\n"
                 << "Setting default value <" << default_int << ">.";
    parameters->setInteger(param_key, default_int);
  }
}

void addFloat(const ros::NodeHandle& nh, const std::string& ros_key,
              const std::string& param_key,
              std::unique_ptr<x_view::Parameters>& parameters,
              const float default_float) {
  float retrieved_float;
  if (nh.getParam(ros_key, retrieved_float)) {
    parameters->setFloat(param_key, retrieved_float);
  } else {
    LOG(WARNING) << "Could not parse float parameter <" << ros_key << ">\n"
                 << "Setting default value <" << default_float << ">.";
    parameters->setFloat(param_key, default_float);
  }
}

void addBool(const ros::NodeHandle& nh, const std::string& ros_key,
             const std::string& param_key,
             std::unique_ptr<x_view::Parameters>& parameters,
             const bool default_bool) {
  bool retrieved_bool;
  if (nh.getParam(ros_key, retrieved_bool)) {
    parameters->setBoolean(param_key, retrieved_bool);
  } else {
    LOG(WARNING) << "Could not parse bool parameter <" << ros_key << ">\n"
                 << "Setting default value <"
                 << (default_bool ? "True" : "False") << ">.";
    parameters->setBoolean(param_key, default_bool);
  }
}

std::unique_ptr<Parameters> parseParameters(const ros::NodeHandle& nh) {
  std::unique_ptr<Parameters> parameters(new Parameters("Root Parameters"));

  auto dataset_parameters = parseDataset(nh);
  auto landmark_parameters = parseLandmark(nh);
  auto matcher_parameters = parseMatcher(nh);

  parameters->addChildPropertyList("dataset", std::move(dataset_parameters));
  parameters->addChildPropertyList("landmark", std::move(landmark_parameters));
  parameters->addChildPropertyList("matcher", std::move(matcher_parameters));

  return std::move(parameters);
}
std::unique_ptr<Parameters> parseDataset(const ros::NodeHandle& nh) {
  std::unique_ptr<Parameters> dataset_parameters(
      new Parameters("Dataset Parameters"));

  addString(nh, "/Dataset/name", "name", dataset_parameters, "SYNTHIA");
  addString(nh, "/Dataset/rosbag_path", "rosbag_path", dataset_parameters);
  addString(nh, "/Dataset/semantics_image_topic", "semantics_image_topic",
            dataset_parameters);

  return std::move(dataset_parameters);
}

std::unique_ptr<Parameters> parseLandmark(const ros::NodeHandle& nh) {
  std::string landmark_type;
  if (!nh.getParam("/Landmark/type", landmark_type)) {
    LOG(WARNING) << "Could not parse parameter </Landmark/type>.\n"
                 << "Setting landmark type to <GRAPH>.";
    landmark_type = "GRAPH";
  }

  if (landmark_type == "GRAPH") {
    std::unique_ptr<Parameters> graph_landmark_parameters =
        parseGraphLandmark(nh);
    return std::move(graph_landmark_parameters);
  } else if (landmark_type == "ORB") {
    std::unique_ptr<Parameters> ORB_landmark_parameters =
        parseORBLandmark(nh);
    return std::move(ORB_landmark_parameters);
  } else if (landmark_type == "SIFT") {
    std::unique_ptr<Parameters> SIFT_landmark_parameters =
        parseSIFTLandmark(nh);
    return std::move(SIFT_landmark_parameters);
  } else if (landmark_type == "SURF") {
    std::unique_ptr<Parameters> SURF_landmark_parameters =
        parseSURFLandmark(nh);
    return std::move(SURF_landmark_parameters);
  } else if (landmark_type == "HISTOGRAM") {
    std::unique_ptr<Parameters> histogram_landmark_parameters =
        parseHistogramLandmark(nh);
    return std::move(histogram_landmark_parameters);
  } else {
    LOG(ERROR) << "Unrecognized landmark type <" << landmark_type << ">."
               << std::endl;
    return std::unique_ptr<Parameters>();
  }
}

std::unique_ptr<Parameters> parseHistogramLandmark(const ros::NodeHandle& nh) {
  std::unique_ptr<Parameters> histogram_landmark_parameters(
      new Parameters("Histogram landmark parameters"));

  // Set the landmark type.
  histogram_landmark_parameters->setString("type", "HISTOGRAM");

  return std::move(histogram_landmark_parameters);
}

std::unique_ptr<Parameters> parseORBLandmark(const ros::NodeHandle& nh) {

  std::unique_ptr<Parameters> ORBParameters(
      new Parameters("ORB landmark Parameters"));

  // Set the landmark type.
  ORBParameters->setString("type", "ORB");

  // Parse other parameters.
  addInt(nh, "/Landmark/num_features", "num_features", ORBParameters, 1000);
  return std::move(ORBParameters);
}

std::unique_ptr<Parameters> parseSIFTLandmark(const ros::NodeHandle& nh) {
  std::unique_ptr<Parameters> SIFTParameters(
      new Parameters("SIFT landmark Parameters"));

  // Set the landmark type.
  SIFTParameters->setString("type", "SIFT");

  // Parse other parameters.
  addInt(nh, "/Landmark/num_features", "num_features", SIFTParameters, 1000);
  return std::move(SIFTParameters);
}

std::unique_ptr<Parameters> parseSURFLandmark(const ros::NodeHandle& nh) {

  std::unique_ptr<Parameters> SURFParameters(
      new Parameters("SURF landmark Parameters"));

  // Set the landmark type.
  SURFParameters->setString("type", "SURF");

  // Parse other parameters.
  addInt(nh, "/Landmark/num_features", "num_features", SURFParameters, 1000);
  addFloat(nh, "/Landmark/hessian_threshold", "hessian_threshold",
           SURFParameters, 0.2f);

  return std::move(SURFParameters);
}

std::unique_ptr<Parameters> parseGraphLandmark(const ros::NodeHandle& nh) {
  std::unique_ptr<Parameters> graph_landmark_parameters(
      new Parameters("Graph landmark Parameters"));

  // Set the landmark type.
  graph_landmark_parameters->setString("type", "GRAPH");

  // Parse other parameters.
  addString(nh, "/Landmark/blob_filter_type", "blob_filter_type",
            graph_landmark_parameters, "ABSOLUTE");
  if (graph_landmark_parameters->getString("blob_filter_type") == "ABSOLUTE")
    addInt(nh, "/Landmark/min_blob_size", "min_blob_size",
           graph_landmark_parameters, 500);
  else if (graph_landmark_parameters->getString("blob_filter_type") ==
      "RELATIVE") {
    addFloat(nh, "/Landmark/min_blob_size", "min_blob_size",
             graph_landmark_parameters, 0.01f);
  } else {
    LOG(ERROR) << "Unrecognized blob filtering type, choose between "
        "\"ABSOLUTE\" and \"RELATIVE\"";
  }

  addBool(nh, "/Landmark/dilate_and_erode", "dilate_and_erode",
          graph_landmark_parameters, true);
  if (graph_landmark_parameters->getBoolean("dilate_and_erode") == true) {
    addInt(nh, "/Landmark/num_dilate", "num_dilate",
           graph_landmark_parameters, 5);
    addInt(nh, "/Landmark/num_erode", "num_erode",
           graph_landmark_parameters, 5);
  }

  addInt(nh, "/Landmark/blob_neighbor_distance", "blob_neighbor_distance",
         graph_landmark_parameters, 10);

  return std::move(graph_landmark_parameters);
}

std::unique_ptr<Parameters> parseMatcher(const ros::NodeHandle& nh) {
  std::string matcher_type;
  if (!nh.getParam("/Matcher/type", matcher_type)) {
    LOG(WARNING) << "Could not parse parameter </Matcher/type>.\n"
                 << "Setting matcher type to <GRAPH>.";
    matcher_type = "GRAPH";
  }

  if (matcher_type == "GRAPH") {
    std::unique_ptr<Parameters> graph_matcher_parameters =
        parseGraphMatcher(nh);
    return std::move(graph_matcher_parameters);
  } else if (matcher_type == "VECTOR") {
    std::unique_ptr<Parameters> vector_matcher_parameters =
        parseVectorMatcher(nh);
    return std::move(vector_matcher_parameters);
  } else {
    LOG(ERROR) << "Unrecognized matcher type <" << matcher_type << ">."
               << std::endl;
    return std::move(std::unique_ptr<Parameters>(new Parameters));
  }
}

std::unique_ptr<Parameters> parseGraphMatcher(const ros::NodeHandle& nh) {

  std::unique_ptr<Parameters> graph_matcher_parameters(
      new Parameters("Graph matcher Parameters"));

  // Set matcher type.
  graph_matcher_parameters->setString("type", "GRAPH");

  // Parse other parameters.
  addString(nh, "/Matcher/vertex_similarity_score",
            "vertex_similarity_score", graph_matcher_parameters, "WEIGHTED");

  addString(nh, "/Matcher/random_walk_sampling_type",
            "random_walk_sampling_type", graph_matcher_parameters, "UNIFORM");

  addInt(nh, "/Matcher/num_walks", "num_walks", graph_matcher_parameters, 200);
  addInt(nh, "/Matcher/walk_length", "walk_length",
         graph_matcher_parameters, 3);

  return std::move(graph_matcher_parameters);
}

std::unique_ptr<Parameters> parseVectorMatcher(const ros::NodeHandle& nh) {
  std::unique_ptr<Parameters> vector_matcher_parameters(
      new Parameters("Vector matcher Parameters"));

  // Set the matcher type.
  vector_matcher_parameters->setString("type", "VECTOR");

  // Parse other parameters.
  addInt(nh, "Matcher/num_retained_matches", "num_retained_matches",
         vector_matcher_parameters, 1);

  return std::move(vector_matcher_parameters);
}

}