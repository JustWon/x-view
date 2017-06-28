#include <x_view_bag_reader/x_view_bag_reader.h>

#include <x_view_core/datasets/synthia_dataset.h>
#include <x_view_core/x_view_locator.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace x_view_ros {

XViewBagReader::XViewBagReader(ros::NodeHandle& n)
    : nh_(n), parser_(nh_) {

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
  } else
    CHECK(false) << "Dataset '" << dataset_name
                 << "' is not supported" << std::endl;

  // Create x_view only now because it has access to the parser parameters.
  x_view_ = std::unique_ptr<x_view::XView>(new x_view::XView());
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
  for (int i = from; step * i < step * to; i += step) {
    std::cout << "Processing semantic image at index " << i << std::endl;
    parseParameters();
    const cv::Mat semantic_image = semantic_topic_view_->getDataAtFrame(i);
    const cv::Mat depth_image = depth_topic_view_->getDataAtFrame(i);
    const tf::StampedTransform trans = transform_view_->getDataAtFrame(i);
    x_view::SE3 pose;
    tfTransformToSE3(trans, &pose);
    x_view::FrameData frame_data(semantic_image, depth_image, pose);
    x_view_->processFrameData(frame_data);
  }
  bag_.close();
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
  } else
    CHECK(false) << "Dataset '" << dataset_name
                 << "' is not supported" << std::endl;
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

}