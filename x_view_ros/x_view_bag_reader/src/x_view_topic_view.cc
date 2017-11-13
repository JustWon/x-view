#include <x_view_bag_reader/x_view_topic_view.h>

#include <x_view_core/x_view_locator.h>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <tf/tfMessage.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view_ros {


cv::Mat SemanticImageView::getDataAtFrame(const int frame_index) const {
  const auto& dataset = x_view::Locator::getDataset();

  // Retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < iterators_.size())
  << "Index passed to 'SemanticImageView::getDataAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  sensor_msgs::ImageConstPtr msg = iter->instantiate<sensor_msgs::Image>();
  return dataset->convertSemanticImage(msg);
}

sensor_msgs::ImageConstPtr SemanticImageView::getMessageAtFrame(const int frame_index) const {
  const auto& dataset = x_view::Locator::getDataset();

  // Retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < iterators_.size())
  << "Index passed to 'SemanticImageView::getDataAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  return iter->instantiate<sensor_msgs::Image>();
}

cv::Mat DepthImageView::getDataAtFrame(const int frame_index) const {
  const auto& dataset = x_view::Locator::getDataset();

  // Retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < iterators_.size())
  << "Index passed to 'DepthImageView::getDataAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  sensor_msgs::ImageConstPtr msg = iter->instantiate<sensor_msgs::Image>();
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO16);

  return cv_ptr->image;
}

sensor_msgs::ImageConstPtr DepthImageView::getMessageAtFrame(const int frame_index) const {
  const auto& dataset = x_view::Locator::getDataset();

  // Retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < iterators_.size())
  << "Index passed to 'DepthImageView::getDataAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  return iter->instantiate<sensor_msgs::Image>();
}

tf::StampedTransform TransformView::getDataAtFrame(const int frame_index) const {
  // Retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < iterators_.size())
  << "Index passed to 'TransformView::getDataAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  tf::tfMessage::ConstPtr tfPtr = iter->instantiate<tf::tfMessage>();

  // Retrieve the transforms which link the world frame to the one associated
  // to the sensor.
  tf::StampedTransform world_to_imu;
  tf::StampedTransform imu_to_sensor;
  bool world_to_imu_found = false;
  bool imu_to_sensors_found = false;

  for(int i = 0; i < tfPtr->transforms.size(); ++i) {
    tf::StampedTransform trans;
    tf::transformStampedMsgToTF(tfPtr->transforms[i], trans);
    if(!world_to_imu_found &&
        trans.frame_id_ == world_frame_ && trans.child_frame_id_ =="imu") {
      world_to_imu = trans;
      world_to_imu_found = true;
    } else if( !imu_to_sensors_found &&
        trans.frame_id_ == "imu" && trans.child_frame_id_ == sensor_frame_) {
      imu_to_sensor = trans;
      imu_to_sensors_found = true;
    }
  }

  CHECK(world_to_imu_found) << "Could not find transform between 'world' frame "
                            << world_frame_ << " and 'imu' frame.";
  CHECK(imu_to_sensors_found) << "Could not find transform between 'imu' frame "
      "and 'sensor' frame " << sensor_frame_;


  // Create a StampedTransform object which contains the transform between
  // the world frame and the sensor frame.
  tf::Transform result = world_to_imu * imu_to_sensor;
  tf::StampedTransform result_stamped(world_to_imu);
  result_stamped.getBasis() = result.getBasis();
  result_stamped.getOrigin() = result.getOrigin();
  result_stamped.child_frame_id_ = sensor_frame_;
  result_stamped.frame_id_ = world_frame_;
  return result_stamped;
}


}
