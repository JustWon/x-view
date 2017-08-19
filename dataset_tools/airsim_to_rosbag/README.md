# airsim_to_rosbag
Dataset tools for working with the Airsim dataset raw data (todo: host data somewhere, or give instructions on generating it), as we extracted it from the Airsim tool ( https://github.com/Microsoft/AirSim ) and converting it to a ROS bag. Also allows a library for direct access to poses and images. This is an adaptation of the kitti_to_rosbag tool by Helen Oleynikova ( https://github.com/ethz-asl/kitti_to_rosbag ).

## Rosbag converter usage example
```
rosrun airsim_to_rosbag airsim_rosbag_converter dataset_path output_path
```
(No trailing slashes). Note: the library currently assumes the original Airsim structure of the dataset subfolders.

```
rosrun airsim_to_rosbag airsim_rosbag_converter /media/johnny/ASL_ext_1/airsim_datasets/neighbourhood /media/johnny/ASL_ext_1/airsim_datasets/neighbourhood/outbag.bag
```

## Library API Example
```C++
#include <opencv2/highgui/highgui.hpp>

#include "airsim_to_rosbag/airsim_parser.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  
  const std::string dataset_path =
      "/media/johnny/ASL_ext_1/airsim_datasets/neighbourhood";

  airsim::AirsimParser parser(dataset_path, true);

  parser.loadCalibration();
  parser.loadTimestampMaps();

  uint64_t timestamp;
  airsim::Transformation pose;
  parser.getPoseAtEntry(0, &timestamp, &pose);

  std::cout << "Timestamp: " << timestamp << " Pose: " << pose << std::endl;

  pcl::PointCloud<pcl::PointXYZI> ptcloud;
  parser.getPointcloudAtEntry(0, &timestamp, &ptcloud);

  std::cout << "Timestamp: " << timestamp << " Num points: " << ptcloud.size()
            << std::endl;

  cv::Mat image;
  parser.getImageAtEntry(0, 3, &timestamp, &image);

  cv::imshow("Display window", image);
  cv::waitKey(0);

  return 0;
}
  
```

## Airsim dataset "Neighbourhood"

Consisting of several UAV flights over a simulated static rural environment with different viewpoints (front / bird's eye).
The environment features the following semantic classes with respective labels:

misc: 0
street: 1
building: 2
car: 3
sign: 4
fence: 5
hedge: 6
tree: 7
wall: 8
bench: 9
powerline: 10
rock: 11
pool: 12

The camera intrinsic parameters are:
kx = ky = 512
px = 512
py = 288
