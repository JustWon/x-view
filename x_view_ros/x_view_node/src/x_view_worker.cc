#include <x_view_node/x_view_worker.h>
#include <x_view_core/datasets/synthia_dataset.h>
#include <x_view_core/x_view_locator.h>

#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view_ros {

XViewWorker::XViewWorker(ros::NodeHandle& n) : nh_(n), parser_(nh_) {

  // Load parameters used by XViewBagReader.
  getXViewWorkerParameters();

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

  semantics_image_sub_ = nh_.subscribe(params_.semantics_image_topic, 1,
                                       &XViewWorker::semanticsImageCallback,
                                       this);

  // Create XView object.
  x_view_ = std::unique_ptr<x_view::XView>(new x_view::XView());

}

XViewWorker::~XViewWorker() {
}

void XViewWorker::semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  parseParameters();

  // Preprocess the ros message.
  const auto& dataset = x_view::Locator::getDataset();
  cv::Mat semantic_image = dataset->convertSemanticImage(msg);

  const cv::Mat depth_image = cv::Mat();

  // Read in pose in world frame.
  tf::StampedTransform tf_transform;
  if (tf_listener_.waitForTransform(params_.world_frame, params_.sensor_frame,
                                    msg->header.stamp,
                                    ros::Duration(0.2))) {
    // Get the tf transform.
    tf_listener_.lookupTransform(params_.world_frame, params_.sensor_frame,
                                 msg->header.stamp, tf_transform);
  } else {
    LOG(ERROR) << "Failed to get transformation between "
               << params_.world_frame << " and " << params_.sensor_frame;
  }

  x_view::SE3 pose;
  tf_transform.getRotation().normalize();
  tfTransformToSE3(tf_transform, &pose);
  x_view::FrameData frame_data(semantic_image, depth_image, pose);
  x_view_->processFrameData(frame_data);
}

void XViewWorker::tfTransformToSE3(const tf::StampedTransform& tf_transform,
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

void XViewWorker::parseParameters() const {
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

void XViewWorker::getXViewWorkerParameters() {

  if (!nh_.getParam("/XViewWorker/semantics_image_topic",
                    params_.semantics_image_topic)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewWorker/semantics_image_topic'";
  }
  if (!nh_.getParam("/XViewWorker/sensor_frame",
                    params_.sensor_frame)) {
    LOG(ERROR) << "Failed to get param '/XViewWorker/sensor_frame'";
  }
  if (!nh_.getParam("/XViewWorker/world_frame",
                    params_.world_frame)) {
    LOG(ERROR) << "Failed to get param '/XViewWorker/world_frame'";
  }
}

}
