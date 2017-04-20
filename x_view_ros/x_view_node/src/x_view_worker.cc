#include <x_view_node/x_view_worker.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <x_view_core/datasets/synthia_dataset.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view_ros {

XViewWorker::XViewWorker(ros::NodeHandle& n) : nh_(n) {
  getParameters();
  semantics_image_sub_ = nh_.subscribe(params_.semantics_image_topic, 1,
                                       &XViewWorker::semanticsImageCallback,
                                       this);

  x_view_ = x_view::XView(params_.x_view_params);
}

XViewWorker::~XViewWorker() {
}

void XViewWorker::semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  std::cout << "msg size: " << msg->data.size() << std::endl;
  std::cout << "step size: " << msg->step << std::endl;
  std::cout << "little_endian?: "
            << (unsigned(msg->is_bigendian) ? "No" : "Yes") << std::endl;

  const int COLS = msg->width;
  const int ROWS = msg->height;

  std::cout << "COLS: " << COLS << std::endl;
  std::cout << "ROWS: " << ROWS << std::endl;

  auto toInt = [&](int index) -> unsigned char {
    unsigned char s = ((msg->data[index + 1] << 8) | msg->data[index]);
    return s;
  };

  x_view::SynthiaDataset synthia;

  for (int interest_label = 0; interest_label < synthia.numSemanticClasses();
       ++interest_label) {
    // initialize greyscale image
    cv::Mat newImage(ROWS, COLS, CV_8UC3, cv::Scalar(255, 255, 255));

    // loop over the rows
    for (int i = 0; i < ROWS; ++i) {
      // loop over the cols
      for (int j = 0; j < COLS; ++j) {
        // index of the pixel, need to have "6*j" because each pixel value is
        // stored into two consecutive bytes and there are three channels
        int idx = msg->step * i + 6 * j;
        // loop over the channels
        cv::Vec3b values;
        for (int c = 0; c < 3; ++c) {
          // the class label is stored into the last two bytes, thus we need
          // to add "+4" to the index such that the bytes 4 and 5 are considered
          values[c] = (uchar) (toInt(idx + 4) == interest_label ? 255 : 0);
        }
        newImage.at<cv::Vec3b>(cv::Point(j, i)) = values;
      }
    }
    cv::imshow("LabelsImage: " + synthia.label(interest_label), newImage);
    cv::waitKey(1000000);
  }

  /*
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    cv::Mat image = cv_ptr->image;

    x_view_.

        process(image, x_view::SE3()

    );
// TODO: Process image using x-view functions.

  }
  catch (

      cv_bridge::Exception& e

  ) {
    ROS_ERROR_STREAM("Could not convert from '" << msg->encoding
                                                << "' to 'bgr8'.\nError: "
                                                << e.what());
  }
   */
}

void XViewWorker::getParameters() {

  // XView parameters.
  if (nh_.getParam("/XView/semantics/dataset", params_
      .x_view_params.semantic_dataset_name_)) {
    ROS_INFO_STREAM(
        "XView is working on the following semantic dataset: <"
            << params_.x_view_params.semantic_dataset_name_ << ">");
  } else {
    ROS_FATAL_STREAM("Failed to get param '/XView/semantics/dataset'");
  }

  if (nh_.getParam("/XView/landmarks/type", params_
      .x_view_params.semantic_landmark_type_)) {
    ROS_INFO_STREAM(
        "XView is using the following semantic landmark type: <"
            << params_.x_view_params.semantic_landmark_type_ << ">");
  } else {
    ROS_ERROR_STREAM("Failed to get param '/XView/landmarks/type'\nUsing "
                         "default <SURF> landmark type.");
    params_.x_view_params.semantic_landmark_type_ = "SURF";
  }

  if (nh_.getParam("/XView/matcher/type", params_
      .x_view_params.landmark_matching_type_)) {
    ROS_INFO_STREAM(
        "XView is using the following landmark matcher type: <"
            << params_.x_view_params.landmark_matching_type_ << ">");
  } else {
    ROS_ERROR_STREAM("Failed to get param '/XView/matcher/type'\nUsing "
                         "default <VISUAL> landmark matcher.");
    params_.x_view_params.landmark_matching_type_ = "VECTOR";
  }


  // XViewWorker parameters.
  nh_.getParam("/XViewWorker/semantics_image_topic",
               params_.semantics_image_topic);
}

}
