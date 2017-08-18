#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <tf/tfMessage.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "airsim_to_rosbag/airsim_parser.h"
#include "airsim_to_rosbag/airsim_ros_conversions.h"

namespace airsim {

class AirsimBagConverter {
 public:
  AirsimBagConverter(const std::string& dataset_path,
                     const std::string& output_filename);

  void convertAll();
  bool convertEntry(uint64_t entry);
  void convertTf(uint64_t timestamp_ns, const airsim::Transformation& imu_pose,
                 const std::vector<airsim::Transformation>& camera_poses);

 private:
  airsim::AirsimParser parser_;

  rosbag::Bag bag_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;

  std::string path_topic_;
  std::string pose_topic_;
  std::string transform_topic_;
  std::string pointcloud_topic_;

  std::vector<geometry_msgs::PoseStamped> poses_;
};

AirsimBagConverter::AirsimBagConverter(const std::string& dataset_path,
                                       const std::string& output_filename)
: parser_(dataset_path, true),
  world_frame_id_("world"),
  imu_frame_id_("imu"),
  cam_frame_id_prefix_("cam"),
  path_topic_("path"),
  pose_topic_("pose_imu"),
  transform_topic_("transform_imu") {
  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  parser_.loadTimestampMaps();

  bag_.open(output_filename, rosbag::bagmode::Write);
}

void AirsimBagConverter::convertAll() {
  uint64_t entry = 0;
  while (convertEntry(entry)) {
    entry++;
  }
  std::cout << "Converted " << entry << " entries into a rosbag.\n";
}

bool AirsimBagConverter::convertEntry(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;

  // Convert poses + TF transforms + path.
  airsim::Transformation pose;
  std::vector<airsim::Transformation> camera_poses;
  if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TransformStamped transform_msg;
    nav_msgs::Path path_msg;

    airsim::timestampToRos(timestamp_ns, &timestamp_ros);
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = timestamp_ros;
    transform_msg.header.frame_id = world_frame_id_;
    transform_msg.header.stamp = timestamp_ros;
    path_msg.header.frame_id = world_frame_id_;
    path_msg.header.stamp = timestamp_ros;

    airsim::poseToRos(pose, &pose_msg);
    airsim::transformToRos(pose, &transform_msg);
    bag_.write(pose_topic_, timestamp_ros, pose_msg);
    bag_.write(transform_topic_, timestamp_ros, transform_msg);

    poses_.push_back(pose_msg);
    airsim::posesToPath(poses_, &path_msg);
    bag_.write(path_topic_, timestamp_ros, path_msg);

    // Get all camera poses.
    for (size_t id = 0u; id < parser_.getNumCameras(); ++id) {
      airsim::Transformation camera_pose;
      if(!parser_.getCameraPoseAtEntry(entry, id, &camera_pose)) {
        return false;
      }
      camera_poses.push_back(camera_pose);
    }

    convertTf(timestamp_ns, pose, camera_poses);
  } else {
    return false;
  }

  // Convert images.
  cv::Mat image, depth_image, labels_image, labels;
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image) &&
        parser_.getDepthImageAtEntry(entry, cam_id, &timestamp_ns, &depth_image) &&
        parser_.getLabelImageAtEntry(entry, cam_id, &timestamp_ns, &labels_image) &&
        parser_.getLabelsAtEntry(entry, cam_id, &timestamp_ns, &labels)) {
      airsim::timestampToRos(timestamp_ns, &timestamp_ros);

      sensor_msgs::Image image_msg, depth_image_msg, labels_image_msg, labels_msg;
      airsim::imageToRos(image, &image_msg);
      airsim::depthImageToRos(depth_image, &depth_image_msg);
      airsim::imageToRos(labels_image, &labels_image_msg);
      airsim::imageToRos(labels, &labels_msg);
      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = airsim::getCameraFrameId(cam_id);
      depth_image_msg.header.stamp = timestamp_ros;
      depth_image_msg.header.frame_id = airsim::getCameraFrameId(cam_id);

      labels_image_msg.header.stamp = timestamp_ros;
      labels_image_msg.header.frame_id = airsim::getCameraFrameId(cam_id);

      labels_msg.header.stamp = timestamp_ros;
      labels_msg.header.frame_id = airsim::getCameraFrameId(cam_id);

      // Get the calibration info for this camera.
      airsim::CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      airsim::calibrationToRos(cam_id, cam_calib, &cam_info);
      cam_info.header = image_msg.header;

      // Convert depth image to pointloud.
      sensor_msgs::PointCloud2 ptcloud_msg;
      ptcloud_msg.header.stamp = timestamp_ros;
      ptcloud_msg.header.frame_id = airsim::getCameraFrameId(cam_id);
      ptcloud_msg.height = depth_image_msg.height;
      ptcloud_msg.width  = depth_image_msg.width;
      ptcloud_msg.is_dense = false;
      ptcloud_msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier pcd_modifier(ptcloud_msg);
      pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
      parser_.convertDepthImageToDepthCloud(depth_image_msg, image_msg,
                                            cam_info, &ptcloud_msg);

      bag_.write(parser_.getCameraPath(cam_id) + "/image_raw", timestamp_ros,
                 image_msg);
      bag_.write(parser_.getCameraPath(cam_id) + "/depth", timestamp_ros,
                 depth_image_msg);
      bag_.write(parser_.getCameraPath(cam_id) + "/labels_image", timestamp_ros,
                 labels_image_msg);
      bag_.write(parser_.getCameraPath(cam_id) + "/labels", timestamp_ros,
                 labels_msg);
      bag_.write(parser_.getCameraPath(cam_id) + "/camera_info", timestamp_ros,
                 cam_info);
      bag_.write(parser_.getCameraPath(cam_id) + "/depth_cloud", timestamp_ros,
                 ptcloud_msg);
    }
  }

  return true;
}

void AirsimBagConverter::convertTf(uint64_t timestamp_ns,
                                   const airsim::Transformation& imu_pose,
                                   const std::vector<airsim::Transformation>& camera_poses) {
  tf::tfMessage tf_msg;
  ros::Time timestamp_ros;
  airsim::timestampToRos(timestamp_ns, &timestamp_ros);

  // Create the full transform chain.
  Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  airsim::Transformation identity_transform(identity);
  airsim::Transformation T_imu_world = imu_pose;
  airsim::Transformation T_vel_imu = identity_transform;
  airsim::Transformation T_cam_world = identity_transform;
  airsim::Transformation T_cam_imu = identity_transform;

  geometry_msgs::TransformStamped tf_imu_world, tf_vel_imu, tf_cam_imu;
  airsim::transformToRos(T_imu_world, &tf_imu_world);
  tf_imu_world.header.frame_id = world_frame_id_;
  tf_imu_world.child_frame_id = imu_frame_id_;
  tf_imu_world.header.stamp = timestamp_ros;

  // Put them into one tf_msg.
  tf_msg.transforms.push_back(tf_imu_world);

  // Get all of the camera transformations as well.
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    airsim::CameraCalibration calibration;
    parser_.getCameraCalibration(cam_id, &calibration);
    T_cam_world = camera_poses[cam_id].inverse();
    T_cam_imu = T_cam_world * T_imu_world;
    airsim::transformToRos(T_cam_imu.inverse(), &tf_cam_imu);
    tf_cam_imu.header.frame_id = imu_frame_id_;
    tf_cam_imu.child_frame_id = airsim::getCameraFrameId(cam_id);
    tf_cam_imu.header.stamp = timestamp_ros;
    tf_msg.transforms.push_back(tf_cam_imu);
  }

  bag_.write("/tf", timestamp_ros, tf_msg);
}

}  // namespace airsim

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  if (argc < 3) {
    std::cout << "Usage: rosrun airsim_to_rosbag airsim_rosbag_converter "
        "dataset_path output_path\n";
    std::cout << "Note: no trailing slashes.\n";
    return 0;
  }

  const std::string dataset_path = argv[1];
  const std::string output_path = argv[2];

  airsim::AirsimBagConverter converter(dataset_path,
                                       output_path);
  converter.convertAll();

  return 0;
}
