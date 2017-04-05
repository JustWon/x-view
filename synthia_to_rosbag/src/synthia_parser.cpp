#include "synthia_to_rosbag/synthia_parser.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include "ros/ros.h"

#include <eigen-checks/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <opencv2/highgui/highgui.hpp>


namespace synthia {

const std::string SynthiaParser::kDepthFolder = "Depth";
const std::string SynthiaParser::kSemanticsGTFolder = "GT";
const std::string SynthiaParser::kRGBFolder = "RGB";
const std::string SynthiaParser::kPosesFolder = "CameraParams";
const std::string SynthiaParser::kLeftCameraFolder = "Stereo_Left";
const std::string SynthiaParser::kRightCameraFolder = "Stereo_Right";
const std::string SynthiaParser::kBackFacingFolder = "Omni_B";
const std::string SynthiaParser::kFrontFacingFolder = "Omni_F";
const std::string SynthiaParser::kLeftFacingFolder = "Omni_L";
const std::string SynthiaParser::kRightFacingFolder = "Omni_R";
const std::string SynthiaParser::kIntrinsicsFilename = "intrinsics.txt";

SynthiaParser::SynthiaParser(const std::string& dataset_path, bool rectified)
: dataset_path_(dataset_path),
  rectified_(rectified),
  initial_pose_set_(false) {
  cam_paths_.push_back(kLeftCameraFolder + "/" + kFrontFacingFolder);
  cam_paths_.push_back(kLeftCameraFolder + "/" + kRightFacingFolder);
  cam_paths_.push_back(kLeftCameraFolder + "/" + kBackFacingFolder);
  cam_paths_.push_back(kLeftCameraFolder + "/" + kLeftFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kFrontFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kRightFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kBackFacingFolder);
  cam_paths_.push_back(kRightCameraFolder + "/" + kLeftFacingFolder);

}

bool SynthiaParser::loadCalibration() {

  camera_calibrations_.resize(8);
  size_t cam_id = 0u;
  for (auto&& calibration : camera_calibrations_) {
    // Intrinsics.
    calibration.image_size << 1280, 760;
    calibration.rect_mat = Eigen::Matrix3d::Identity();
    calibration.projection_mat = Eigen::Matrix<double, 3, 4>::Identity();
    calibration.K = Eigen::Matrix3d::Zero();
    calibration.D = Eigen::Matrix<double, 1, 5>::Zero();
    // Extrinsics.
    Eigen::Matrix3d cam_orientation = Eigen::Matrix3d::Identity();
    cam_orientation(0,0) = cos(cam_id * M_PI / 2);
    cam_orientation(0,2) = -sin(cam_id * M_PI / 2);
    cam_orientation(2,0) = sin(cam_id * M_PI / 2);
    cam_orientation(2,2) = cos(cam_id * M_PI / 2);
    calibration.T_cam0_cam.getRotation() =
        synthia::Rotation::fromApproximateRotationMatrix(cam_orientation);
    Eigen::Vector3d translation(0,0,0);
    if (cam_id < 4) {
      if (cam_id % 2 == 0) {
        translation(1) = -0.4;
      } else {
        translation(0) = -0.4;
      }
    } else {
      if (cam_id % 2 == 0) {
        translation(1) = 0.4;
      } else {
        translation(0) = 0.4;
      }
    }
    calibration.T_cam0_cam.getPosition() = translation;
    ++cam_id;
  }
  return true;
}

bool SynthiaParser::parseVectorOfDoubles(const std::string& input,
                                         std::vector<double>* output) const {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.empty()) {
      continue;
    }
    try {
      output->emplace_back(std::stod(element));
    } catch (const std::exception& exception) {
      std::cout << "Could not parse number in import file.\n";
      return false;
    }
  }
  return true;
}

void SynthiaParser::loadTimestampMaps() {

  std::string file_path = dataset_path_ + "/CameraParams/Stereo_Right/Omni_F";
  size_t num_files = 0u;
  for(boost::filesystem::directory_iterator it(file_path);
      it != boost::filesystem::directory_iterator(); ++it)
  {
    timestamps_ns_.push_back(1 + num_files * 2e8);
    num_files++;
  }
}

bool SynthiaParser::getCameraCalibration(uint64_t cam_id,
                                         synthia::CameraCalibration* cam) const {
  if (cam_id >= camera_calibrations_.size()) {
    return false;
  }
  *cam = camera_calibrations_[cam_id];
  return true;
}

bool SynthiaParser::getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
                                   synthia::Transformation* pose) {
  // Read in both camera poses and spawn one in between.
  std::string filename_cam0 = dataset_path_ + "/" + kPosesFolder +
      "/Stereo_Left/Omni_F/" + getFilenameForEntry(entry) + ".txt";
  std::string filename_cam1 = dataset_path_ + "/" + kPosesFolder +
      "/Stereo_Right/Omni_F/" + getFilenameForEntry(entry) + ".txt";

  std::ifstream import_file_cam0(filename_cam0, std::ios::in);
  std::ifstream import_file_cam1(filename_cam1, std::ios::in);
  if (!import_file_cam0 || !import_file_cam1) {
    return false;
  }
  if (timestamps_ns_.size() <= entry) {
    return false;
  }
  *timestamp = timestamps_ns_[entry];

  // todo(gawela) pose is presently on cam0, need to add the intermediate frame
  // between am0 and cam1
  std::vector<double> parsed_doubles_0, parsed_doubles_1;
  std::string line_0, line_1;
  std::getline(import_file_cam0, line_0);
  std::getline(import_file_cam1, line_1);
  if (parseVectorOfDoubles(line_0, &parsed_doubles_0) &&
      parseVectorOfDoubles(line_1, &parsed_doubles_1)) {
    // Flip z,y.
    if (!flipYZ(&parsed_doubles_0) || !flipYZ(&parsed_doubles_1)) {
      return false;
    }
    // Make position between two cameras.
    parsed_doubles_0[12] = (parsed_doubles_0[12] + parsed_doubles_1[12]) / 2;
    parsed_doubles_0[13] = (parsed_doubles_0[13] + parsed_doubles_1[13]) / 2;
    parsed_doubles_0[14] = (parsed_doubles_0[14] + parsed_doubles_1[14]) / 2;

    if (convertVectorToPose(parsed_doubles_0, pose)) {
      return true;
    }
  }
  return false;
}

bool SynthiaParser::flipYZ(std::vector<double>* parsed_doubles) {
  if (parsed_doubles->size() == 16) {
    std::iter_swap(parsed_doubles->begin() + 1,parsed_doubles->begin() + 2);
    std::iter_swap(parsed_doubles->begin() + 5,parsed_doubles->begin() + 6);
    std::iter_swap(parsed_doubles->begin() + 9,parsed_doubles->begin() + 10);
    std::iter_swap(parsed_doubles->begin() + 13,parsed_doubles->begin() + 14);
    return true;
  } else {
    return false;
  }
}

bool SynthiaParser::getPoseAtEntry2(uint64_t entry, uint64_t* timestamp,
                                    synthia::Transformation* pose) {
  // Read in front camera (global) poses and interpolate base between.
  std::string filename_cam_left_front = dataset_path_ + "/" + kPosesFolder + "/" +
      cam_paths_[1] + "/" + getFilenameForEntry(entry) + ".txt";
  std::ifstream import_file_cam_left_front(filename_cam_left_front, std::ios::in);
  std::string filename_cam_right_front = dataset_path_ + "/" + kPosesFolder + "/" +
      cam_paths_[5] + "/" + getFilenameForEntry(entry) + ".txt";
  std::ifstream import_file_cam_right_front(filename_cam_right_front, std::ios::in);

  if (!import_file_cam_left_front || !import_file_cam_right_front) {
    return false;
  }
  if (timestamps_ns_.size() <= entry) {
    return false;
  }
  *timestamp = timestamps_ns_[entry];
  synthia::Transformation pose_left_front, pose_right_front;
  std::vector<double> parsed_doubles;
  std::string line;
  std::getline(import_file_cam_left_front, line);
  if (parseVectorOfDoubles(line, &parsed_doubles)) {
    synthia::Transformation pose_left_front;
    if (convertVectorToPose(parsed_doubles, &pose_left_front)) {
      std::getline(import_file_cam_right_front, line);
      if (parseVectorOfDoubles(line, &parsed_doubles)) {
        synthia::Transformation pose_right_front;
        if (convertVectorToPose(parsed_doubles, &pose_right_front)) {
          //          CHECK(EIGEN_MATRIX_NEAR(pose_left_front.getRotationMatrix(),
          //                          pose_right_front.getRotationMatrix(),1e-5))
          //                                    << "Rotation of cameras do not agree.";
          Eigen::Vector3d base_position = pose_right_front.getPosition() -
              pose_left_front.getPosition();
          synthia::Transformation base_pose(pose_right_front.getRotation(),
                                            base_position);
          *pose = base_pose;
          return true;
        }
      }
    }
  }
}

uint64_t SynthiaParser::getPoseTimestampAtEntry(uint64_t entry) {
  if (timestamps_ns_.size() <= entry) {
    return 0;
  }
  return timestamps_ns_[entry];
}

bool SynthiaParser::getImageAtEntry(uint64_t entry, uint64_t cam_id,
                                    uint64_t* timestamp, cv::Mat* image) {
  // Get the timestamp for this first.
  std::string filename = dataset_path_ + "/RGB/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);

  if (!image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::getDepthImageAtEntry(uint64_t entry, uint64_t cam_id,
                                         uint64_t* timestamp, cv::Mat* depth_image) {
  // Get the timestamp for this first.

  std::string filename = dataset_path_ + "/Depth/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *depth_image = cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  cv::imshow("window", *depth_image);

  if (!depth_image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::getLabelImageAtEntry(uint64_t entry, uint64_t cam_id,
                                         uint64_t* timestamp, cv::Mat* label_image) {
  // Get the timestamp for this first.
  *timestamp = timestamps_ns_[entry];

  std::string filename = dataset_path_ + "/GT/COLOR/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *label_image = cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  cv::imshow("window", *label_image);

  if (!label_image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::getLabelsAtEntry(uint64_t entry, uint64_t cam_id,
                                     uint64_t* timestamp, cv::Mat* label_image) {
  // Get the timestamp for this first.
  std::cout << "entry: " << entry << std::endl;

  std::string filename = dataset_path_ + "/GT/LABELS/" + cam_paths_[cam_id] + "/" +
      getFilenameForEntry(entry) + ".png";

  *label_image = cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
  cv::imshow("window", *label_image);

  if (!label_image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool SynthiaParser::convertVectorToPose(const std::vector<double>& pose_16,
                                        synthia::Transformation* pose) {
  if (pose_16.size() < 16) {
    return false;
  }
  std::vector<double> pose_temp = pose_16;
  typedef Eigen::Matrix<double, 4, 4, Eigen::ColMajor> TransformationMatrix;
  Eigen::Map<TransformationMatrix> m1(pose_temp.data(), 4, 4);
  TransformationMatrix mat = m1;
  synthia::Transformation t_temp(mat);
  *pose = t_temp;

  return true;
}

// From the MATLAB raw data dev kit.
double SynthiaParser::latToScale(double lat) const {
  return cos(lat * M_PI / 180.0);
}

// From the MATLAB raw data dev kit.
void SynthiaParser::latlonToMercator(double lat, double lon, double scale,
                                     Eigen::Vector2d* mercator) const {
  double er = 6378137;
  mercator->x() = scale * lon * M_PI * er / 180.0;
  mercator->y() = scale * er * log(tan((90.0 + lat) * M_PI / 360.0));
}

std::string SynthiaParser::getFilenameForEntry(uint64_t entry) const {
  char buffer[20];
  sprintf(buffer, "%06d", entry);
  return std::string(buffer);
}

synthia::Transformation SynthiaParser::T_camN_vel(int cam_number) const {
  return camera_calibrations_[cam_number].T_cam0_cam.inverse() * T_cam0_vel_;
}

synthia::Transformation SynthiaParser::T_camN_imu(int cam_number) const {
  return T_camN_vel(cam_number) * T_vel_imu_;
}

synthia::Transformation SynthiaParser::T_cam0_vel() const { return T_cam0_vel_; }

synthia::Transformation SynthiaParser::T_vel_imu() const { return T_vel_imu_; }

bool SynthiaParser::interpolatePoseAtTimestamp(uint64_t timestamp,
                                               synthia::Transformation* pose) {
  // Look up the closest 2 timestamps to this.
  size_t left_index = timestamps_ns_.size();
  for (size_t i = 0; i < timestamps_ns_.size(); ++i) {
    if (timestamps_ns_[i] > timestamp) {
      if (i == 0) {
        // Then we can't interpolate the pose since we're outside the range.
        return false;
      }
      left_index = i - 1;
      break;
    }
  }
  if (left_index >= timestamps_ns_.size()) {
    return false;
  }
  // Figure out what 't' should be, where t = 0 means 100% left boundary,
  // and t = 1 means 100% right boundary.
  double t = (timestamp - timestamps_ns_[left_index]) /
      static_cast<double>(timestamps_ns_[left_index + 1] -
                          timestamps_ns_[left_index]);

  // Load the two transformations.
  uint64_t timestamp_left, timestamp_right;
  synthia::Transformation transform_left, transform_right;
  if (!getPoseAtEntry(left_index, &timestamp_left, &transform_left) ||
      !getPoseAtEntry(left_index + 1, &timestamp_right, &transform_right)) {
    // For some reason couldn't load the poses.
    return false;
  }

  // Interpolate between them.
  *pose = synthia::interpolateTransformations(transform_left, transform_right, t);
  return true;
}

size_t SynthiaParser::getNumCameras() const {
  return camera_calibrations_.size();
}

}  // namespace synthia
