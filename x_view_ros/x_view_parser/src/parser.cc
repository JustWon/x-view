#include <x_view_parser/parser.h>

#include <glog/logging.h>

using namespace x_view;

namespace x_view_ros {

std::unique_ptr<Parameters> Parser::parseParameters() const {
  std::unique_ptr<Parameters> parameters(new Parameters("Root Parameters"));

  auto dataset_parameters = parseDataset();
  auto landmark_parameters = parseLandmark();
  auto matcher_parameters = parseMatcher();
  auto localizer_parameters = parseLocalizer();

  parameters->addChildPropertyList("dataset", std::move(dataset_parameters));
  parameters->addChildPropertyList("landmark", std::move(landmark_parameters));
  parameters->addChildPropertyList("matcher", std::move(matcher_parameters));
  parameters->addChildPropertyList("localizer", std::move(localizer_parameters));

  return std::move(parameters);
}

void Parser::addString(const ros::NodeHandle& nh, const std::string& ros_key,
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

void Parser::addInt(const ros::NodeHandle& nh, const std::string& ros_key,
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

void Parser::addFloat(const ros::NodeHandle& nh, const std::string& ros_key,
                      const std::string& param_key,
                      std::unique_ptr<x_view::Parameters>& parameters,
                      const real_t default_float) {
  real_t retrieved_float;
  if (nh.getParam(ros_key, retrieved_float)) {
    parameters->setFloat(param_key, retrieved_float);
  } else {
    LOG(WARNING) << "Could not parse float parameter <" << ros_key << ">\n"
                 << "Setting default value <" << default_float << ">.";
    parameters->setFloat(param_key, default_float);
  }
}

void Parser::addBool(const ros::NodeHandle& nh, const std::string& ros_key,
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

std::unique_ptr<Parameters> Parser::parseDataset() const {
  std::unique_ptr<Parameters> dataset_parameters(
      new Parameters("Dataset Parameters"));

  addString(nh_, "/Dataset/name", "name", dataset_parameters, "SYNTHIA");

  return std::move(dataset_parameters);
}

std::unique_ptr<Parameters> Parser::parseLandmark() const {
  std::string landmark_type;
  if (!nh_.getParam("/Landmark/type", landmark_type)) {
    LOG(WARNING) << "Could not parse parameter </Landmark/type>.\n"
                 << "Setting landmark type to <GRAPH>.";
    landmark_type = "GRAPH";
  }

  if (landmark_type == "GRAPH") {
    std::unique_ptr<Parameters> graph_landmark_parameters =
        parseGraphLandmark();
    return std::move(graph_landmark_parameters);
  } else if (landmark_type == "ORB") {
    std::unique_ptr<Parameters> ORB_landmark_parameters =
        parseORBLandmark();
    return std::move(ORB_landmark_parameters);
  } else if (landmark_type == "SIFT") {
    std::unique_ptr<Parameters> SIFT_landmark_parameters =
        parseSIFTLandmark();
    return std::move(SIFT_landmark_parameters);
  } else if (landmark_type == "SURF") {
    std::unique_ptr<Parameters> SURF_landmark_parameters =
        parseSURFLandmark();
    return std::move(SURF_landmark_parameters);
  } else if (landmark_type == "HISTOGRAM") {
    std::unique_ptr<Parameters> histogram_landmark_parameters =
        parseHistogramLandmark();
    return std::move(histogram_landmark_parameters);
  } else {
    LOG(ERROR) << "Unrecognized landmark type <" << landmark_type << ">."
               << std::endl;
    return std::unique_ptr<Parameters>();
  }
}

std::unique_ptr<Parameters> Parser::parseHistogramLandmark() const {
  std::unique_ptr<Parameters> histogram_landmark_parameters(
      new Parameters("Histogram landmark parameters"));

  // Set the landmark type.
  histogram_landmark_parameters->setString("type", "HISTOGRAM");

  return std::move(histogram_landmark_parameters);
}

std::unique_ptr<Parameters> Parser::parseORBLandmark() const {

  std::unique_ptr<Parameters> ORBParameters(
      new Parameters("ORB landmark Parameters"));

  // Set the landmark type.
  ORBParameters->setString("type", "ORB");

  // Parse other parameters.
  addInt(nh_, "/Landmark/num_features", "num_features", ORBParameters, 1000);
  return std::move(ORBParameters);
}

std::unique_ptr<Parameters> Parser::parseSIFTLandmark() const {
  std::unique_ptr<Parameters> SIFTParameters(
      new Parameters("SIFT landmark Parameters"));

  // Set the landmark type.
  SIFTParameters->setString("type", "SIFT");

  // Parse other parameters.
  addInt(nh_, "/Landmark/num_features", "num_features", SIFTParameters, 1000);
  return std::move(SIFTParameters);
}

std::unique_ptr<Parameters> Parser::parseSURFLandmark() const {

  std::unique_ptr<Parameters> SURFParameters(
      new Parameters("SURF landmark Parameters"));

  // Set the landmark type.
  SURFParameters->setString("type", "SURF");

  // Parse other parameters.
  addInt(nh_, "/Landmark/num_features", "num_features", SURFParameters, 1000);
  addFloat(nh_, "/Landmark/hessian_threshold", "hessian_threshold",
           SURFParameters, 0.2f);

  return std::move(SURFParameters);
}

std::unique_ptr<Parameters> Parser::parseGraphLandmark() const {
  std::unique_ptr<Parameters> graph_landmark_parameters(
      new Parameters("Graph landmark Parameters"));

  // Set the landmark type.
  graph_landmark_parameters->setString("type", "GRAPH");

  // Parse other parameters.
  addString(nh_, "/Landmark/blob_filter_type", "blob_filter_type",
            graph_landmark_parameters, "ABSOLUTE");
  if (graph_landmark_parameters->getString("blob_filter_type") == "ABSOLUTE")
    addInt(nh_, "/Landmark/min_blob_size", "min_blob_size",
           graph_landmark_parameters, 500);
  else if (graph_landmark_parameters->getString("blob_filter_type") ==
      "RELATIVE") {
    addFloat(nh_, "/Landmark/min_blob_size", "min_blob_size",
             graph_landmark_parameters, 0.01f);
  } else {
    LOG(ERROR) << "Unrecognized blob filtering type, choose between "
        "\"ABSOLUTE\" and \"RELATIVE\"";
  }

  addBool(nh_, "/Landmark/dilate_and_erode", "dilate_and_erode",
          graph_landmark_parameters, true);
  if (graph_landmark_parameters->getBoolean("dilate_and_erode") == true) {
    addInt(nh_, "/Landmark/num_dilate", "num_dilate",
           graph_landmark_parameters, 5);
    addInt(nh_, "/Landmark/num_erode", "num_erode",
           graph_landmark_parameters, 5);
  }

  addInt(nh_, "/Landmark/blob_neighbor_distance", "blob_neighbor_distance",
         graph_landmark_parameters, 10);

  addFloat(nh_, "/Landmark/depth_clip", "depth_clip", graph_landmark_parameters);

  return std::move(graph_landmark_parameters);
}

std::unique_ptr<Parameters> Parser::parseMatcher() const {
  std::string matcher_type;
  if (!nh_.getParam("/Matcher/type", matcher_type)) {
    LOG(WARNING) << "Could not parse parameter </Matcher/type>.\n"
                 << "Setting matcher type to <GRAPH>.";
    matcher_type = "GRAPH";
  }

  if (matcher_type == "GRAPH") {
    std::unique_ptr<Parameters> graph_matcher_parameters =
        parseGraphMatcher();
    return std::move(graph_matcher_parameters);
  } else if (matcher_type == "VECTOR") {
    std::unique_ptr<Parameters> vector_matcher_parameters =
        parseVectorMatcher();
    return std::move(vector_matcher_parameters);
  } else {
    LOG(ERROR) << "Unrecognized matcher type <" << matcher_type << ">."
               << std::endl;
    return std::move(std::unique_ptr<Parameters>(new Parameters));
  }
}

std::unique_ptr<Parameters> Parser::parseGraphMatcher() const {

  std::unique_ptr<Parameters> graph_matcher_parameters(
      new Parameters("Graph matcher Parameters"));

  // Set matcher type.
  graph_matcher_parameters->setString("type", "GRAPH");

  // Parse other parameters.
  addString(nh_, "/Matcher/vertex_similarity_score",
            "vertex_similarity_score", graph_matcher_parameters, "WEIGHTED");

  addString(nh_, "/Matcher/random_walk_sampling_type",
            "random_walk_sampling_type", graph_matcher_parameters, "UNIFORM");

  addInt(nh_, "/Matcher/num_walks", "num_walks", graph_matcher_parameters, 200);
  addInt(nh_, "/Matcher/walk_length", "walk_length",
         graph_matcher_parameters, 3);
  addBool(nh_, "/Matcher/outlier_rejection", "outlier_rejection",
          graph_matcher_parameters, false);
  addFloat(nh_, "/Matcher/consistency_threshold", "consistency_threshold",
           graph_matcher_parameters, 5.0);
  addFloat(nh_, "/Matcher/consistency_size", "consistency_size",
           graph_matcher_parameters, 2.0);

  // Parameters related to graph merger.
  addInt(nh_, "/Matcher/time_window", "time_window",
         graph_matcher_parameters, std::numeric_limits<int>::max());
  addFloat(nh_, "/Matcher/similarity_threshold", "similarity_threshold",
           graph_matcher_parameters, 0.0);
  addFloat(nh_, "/Matcher/distance_threshold", "distance_threshold",
           graph_matcher_parameters, std::numeric_limits<real_t>::max());
  addFloat(nh_, "/Matcher/merge_distance", "merge_distance",
           graph_matcher_parameters, 2.0);

  return std::move(graph_matcher_parameters);
}

std::unique_ptr<Parameters> Parser::parseVectorMatcher() const {
  std::unique_ptr<Parameters> vector_matcher_parameters(
      new Parameters("Vector matcher Parameters"));

  // Set the matcher type.
  vector_matcher_parameters->setString("type", "VECTOR");

  // Parse other parameters.
  addInt(nh_, "Matcher/num_retained_matches", "num_retained_matches",
         vector_matcher_parameters, 1);

  return std::move(vector_matcher_parameters);
}

std::unique_ptr<Parameters> Parser::parseLocalizer() const {

  std::unique_ptr<Parameters> localizer_parameters(
      new Parameters("Localizer Parameters"));

  // Set localizer type.
  addString(nh_, "/Localizer/type", "type", localizer_parameters,
            "OPTIMIZATION");

  return std::move(localizer_parameters);
}

}
