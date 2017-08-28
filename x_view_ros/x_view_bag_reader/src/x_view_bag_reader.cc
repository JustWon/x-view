#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_bag_reader/x_view_pause.h>
#include <x_view_core/datasets/airsim_dataset.h>
#include <x_view_core/datasets/synthia_dataset.h>
#include <x_view_core/timer/timer.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include <iostream>

namespace x_view_ros {

XViewBagReader::XViewBagReader(ros::NodeHandle& n)
    : nh_(n), parser_(nh_), graph_publisher_(n) {

  // Load parameters used by XViewBagReader.
  getXViewBagReaderParameters();

  // Parse all parameters used by XView.
  std::unique_ptr<x_view::Parameters> parameters = parser_.parseParameters();

  // Register the parameters into the locator.
  x_view::Locator::registerParameters(std::move(parameters));

  // Set the usage of the specified dataset.
  const auto& params = x_view::Locator::getParameters();
  const auto& dataset_params = params->getChildPropertyList("dataset");
  const std::string dataset_name = dataset_params->getString("name");
  if (dataset_name == "SYNTHIA") {
    std::unique_ptr<x_view::AbstractDataset> dataset(
        new x_view::SynthiaDataset());
    x_view::Locator::registerDataset(std::move(dataset));
  } else if (dataset_name == "AIRSIM") {
    std::unique_ptr<x_view::AbstractDataset> dataset(
        new x_view::AirsimDataset());
    x_view::Locator::registerDataset(std::move(dataset));
  } else {
    CHECK(false) << "Dataset '" << dataset_name
        << "' is not supported" << std::endl;
  }

  // Create x_view only now because it has access to the parser parameters.
  x_view_ = std::unique_ptr<x_view::XView>(new x_view::XView());

  // Register the vertex publisher.
  vertex_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/localization/position",
                                                10000);
}

void XViewBagReader::loadCurrentTopic(const CameraTopics& current_topics) {

  bag_.open(params_.bag_file_name, rosbag::bagmode::Read);

  // Dataset used to parse the images.
  const auto& dataset = x_view::Locator::getDataset();

  // Remove the '/' char at the beginning of the topic names and create
  // corresponding topic views (this might be a bug in ros, as for creating
  // listeners and publishers the '/' char is needed, while to access the
  // data in a bag file one needs to remove it).
  std::string semantic_image_topic = current_topics.semantics_image_topic;
  if (semantic_image_topic.front() == '/')
    semantic_image_topic.erase(semantic_image_topic.begin());

  std::string depth_image_topic = current_topics.depth_image_topic;
  if (depth_image_topic.front() == '/')
    depth_image_topic.erase(depth_image_topic.begin());

  semantic_topic_view_ = std::unique_ptr<SemanticImageView>(
      new SemanticImageView(bag_, semantic_image_topic));
  depth_topic_view_ = std::unique_ptr<DepthImageView>(
      new DepthImageView(bag_, depth_image_topic));
  transform_view_ = std::unique_ptr<TransformView>(
      new TransformView(bag_, params_.transform_topic, params_.world_frame,
                        current_topics.sensor_frame));

}

void XViewBagReader::iterateBagFromTo(const CAMERA camera_type,
                                      const int from, const int to) {
  loadCurrentTopic(getTopics(camera_type));
  const int step = (from <= to ? +1 : -1);
  Pause pause;
  for (int i = from; step * i < step * to; ) {
    if(!pause.isPaused()) {
      std::cout << "Processing semantic image at index " << i << std::endl;
      parseParameters();
      const cv::Mat semantic_image = semantic_topic_view_->getDataAtFrame(i);
      const cv::Mat depth_image = depth_topic_view_->getDataAtFrame(i);
      const tf::StampedTransform trans = transform_view_->getDataAtFrame(i);
      x_view::PoseId pose_id;
      pose_id.id = x_view::KeyGenerator::getNextKey();
      tfTransformToSE3(trans, &pose_id.pose);

      x_view::FrameData frame_data(semantic_image, depth_image, pose_id, i);
      x_view_->processFrameData(frame_data);
      x_view_->writeGraphToFile();

      graph_publisher_.publish(x_view_->getSemanticGraph(), ros::Time());
      i += step;
    }
  }
  bag_.close();
  pause.terminate();
}

void XViewBagReader::relabelGlobalGraphVertices(const x_view::real_t percentage,
                                                const uint64_t seed) {
  x_view_->relabelGlobalGraphVertices(percentage, seed);
}

x_view::real_t XViewBagReader::localizeGraph(
    const CAMERA camera_type, const int start_frame, const int steps,
    x_view::LocalizationPair* locations,
    x_view::GraphMatcher::IndexMatrixType* candidate_matches,
    x_view::Graph* local_graph) {

  CHECK_NOTNULL(locations);

  loadCurrentTopic(getTopics(camera_type));

  const auto& timer = x_view::Locator::getTimer();

  // Local X-View object used to generate local graph which will be localized
  // against the global semantic graph built by x_view_.
  x_view::XView local_x_view;
  ros::Time time;
  std::vector<x_view::PoseId> pose_ids;

  // Write pose ids.
  for (int i = start_frame; i < start_frame + steps; ++i) {
    x_view::PoseId pose_id;
    pose_id.id = x_view::KeyGenerator::getNextKey();
    pose_ids.push_back(pose_id);
  }

  timer->registerTimer("QueryGraphConstruction");
  timer->start("QueryGraphConstruction");

  for(int i = start_frame; i < start_frame + steps; ++i) {
    parseParameters();
    const cv::Mat semantic_image = semantic_topic_view_->getDataAtFrame(i);
    const cv::Mat depth_image = depth_topic_view_->getDataAtFrame(i);
    const tf::StampedTransform trans = transform_view_->getDataAtFrame(i);
    tfTransformToSE3(trans, &pose_ids[i - start_frame].pose);
    // Use the start frame as ground truth.
    if (i == start_frame) {
      locations->true_pose = pose_ids[i - start_frame].pose;
    }
    x_view::FrameData frame_data(semantic_image, depth_image,
                                 pose_ids[i - start_frame], i);
    local_x_view.processFrameData(frame_data);
    // Publish all ground truth poses that contribute to the estimation.
    const x_view::Vector3r ground_truth_color(0.15, 0.7, 0.15);
    publishRobotPosition(
        pose_ids[i - start_frame].pose.getPosition().cast<x_view::real_t>(),
        ground_truth_color, trans.stamp_,
        "true_position_" + x_view::PaddedInt(i - start_frame, 3).str());
  }

  timer->stop("QueryGraphConstruction");

  (*local_graph) = local_x_view.getSemanticGraph();

  timer->registerTimer("GraphLocalization");
  timer->start("GraphLocalization");
  x_view::real_t error = x_view_->localizeGraph(*local_graph, pose_ids,
                                                candidate_matches,
                                                &(locations->estimated_pose));
  timer->stop("GraphLocalization");

  const x_view::Vector3r estimated_color(0.7, 0.15, 0.15);
  publishRobotPosition(locations->estimated_pose.getPosition()
                           .cast<x_view::real_t>(),
                       estimated_color, time,
                       "estimated_position");

  graph_publisher_.publish(*local_graph, ros::Time(), 70.0);
  graph_publisher_.publishMatches(*local_graph, x_view_->getSemanticGraph(),
                                  *candidate_matches, ros::Time(), 70.0);

  bag_.close();

  return error;
}

void XViewBagReader::parseParameters() const {
  // Parse all parameters.
  std::unique_ptr<x_view::Parameters> parameters = parser_.parseParameters();

  // Register the parameters into the locator.
  x_view::Locator::registerParameters(std::move(parameters));

  // Set the usage of the specified dataset.
  const auto& params = x_view::Locator::getParameters();
  const auto& dataset_params = params->getChildPropertyList("dataset");
  const std::string dataset_name = dataset_params->getString("name");
  if (dataset_name == "SYNTHIA") {
    std::unique_ptr<x_view::AbstractDataset> dataset(
        new x_view::SynthiaDataset());
    x_view::Locator::registerDataset(std::move(dataset));
  } else if (dataset_name == "AIRSIM") {
    std::unique_ptr<x_view::AbstractDataset> dataset(
        new x_view::AirsimDataset());
    x_view::Locator::registerDataset(std::move(dataset));
  } else {
    CHECK(false) << "Dataset '" << dataset_name
        << "' is not supported" << std::endl;
  }
}

void XViewBagReader::getXViewBagReaderParameters() {

  if (!nh_.getParam("/XViewBagReader/bag_file_name", params_.bag_file_name)) {
    LOG(ERROR) << "Failed to get param '/XViewBagReader/bag_file_name'";
  }
  if (!nh_.getParam("/XViewBagReader/back/semantics_image_topic",
                    params_.back.semantics_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/back/semantics_image_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/back/depth_image_topic",
                    params_.back.depth_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/back/depth_image_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/back/sensor_frame",
                    params_.back.sensor_frame)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/back/sensor_frame'";
  }
  if (!nh_.getParam("/XViewBagReader/right/semantics_image_topic",
                    params_.right.semantics_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/right/semantics_image_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/right/depth_image_topic",
                    params_.right.depth_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/right/depth_image_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/right/sensor_frame",
                    params_.right.sensor_frame)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/right/sensor_frame'";
  }
  if (!nh_.getParam("/XViewBagReader/front/semantics_image_topic",
                    params_.front.semantics_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/front/semantics_image_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/front/depth_image_topic",
                    params_.front.depth_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/back/depth_image_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/front/sensor_frame",
                    params_.front.sensor_frame)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/front/sensor_frame'";
  }
  if (!nh_.getParam("/XViewBagReader/transform_topic",
                    params_.transform_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/transform_topic'";
  }
  if (!nh_.getParam("/XViewBagReader/world_frame",
                    params_.world_frame)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/world_frame'";
  }
}

void XViewBagReader::tfTransformToSE3(const tf::StampedTransform& tf_transform,
                                   x_view::SE3* pose) {
  // Register new pose.
  x_view::SE3::Position pos(
      tf_transform.getOrigin().getX(),
      tf_transform.getOrigin().getY(),
      tf_transform.getOrigin().getZ());
  x_view::SE3::Rotation::Implementation rot(
      tf_transform.getRotation().getW(),
      tf_transform.getRotation().getX(),
      tf_transform.getRotation().getY(),
      tf_transform.getRotation().getZ());
  *pose = x_view::SE3(pos, rot);
}


void XViewBagReader::publishRobotPosition(const x_view::Vector3r& pos,
                                          const x_view::Vector3r& color,
                                          const ros::Time& stamp,
                                          const std::string ns) {

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = stamp;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = ns;
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.position.z = pos[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 4.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = static_cast<float>(color[0]);
  marker.color.g = static_cast<float>(color[1]);
  marker.color.b = static_cast<float>(color[2]);
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  vertex_publisher_.publish(marker);
}

}
