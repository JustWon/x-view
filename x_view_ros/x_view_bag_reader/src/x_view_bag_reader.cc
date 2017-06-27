#include <x_view_bag_reader/x_view_bag_reader.h>

#include <x_view_core/datasets/synthia_dataset.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_types.h>
#include <x_view_parser/parser.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>

namespace x_view_ros {

XViewBagReader::RosbagTopicView::RosbagTopicView(const rosbag::Bag& bag,
                                                 const std::string& topic)
    : topic_name_(topic),
      view_(new rosbag::View(bag, rosbag::TopicQuery(topic))),
      size_(0) {

  // create a copy of the iterators locally which allows random access to the
  // data instead of looping through the view each time an object is queried.
  for (rosbag::View::iterator iter = view_->begin();
       iter != view_->end(); ++iter) {
    iterators_.push_back(rosbag::View::iterator(iter));
    ++size_;
  }

}

cv::Mat XViewBagReader::RosbagTopicView::getSemanticImageAtFrame(const int frame_index) const {
  x_view::SynthiaDataset dataset;

  // retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < size_) << "Index passed to "
      "'RosbagTopicView::getSemanticImageAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  sensor_msgs::ImageConstPtr msg = iter->instantiate<sensor_msgs::Image>();
  return dataset.convertSemanticImage(msg);
}

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

  loadBagFile();

  // Create x_view only now because it has access to the parser parameters.
  x_view_ = std::unique_ptr<x_view::XView>(new x_view::XView());
}

void XViewBagReader::loadBagFile() {
  bag_.open(params_.bag_file_name, rosbag::bagmode::Read);

  // dataset used to parse the images
  x_view::SynthiaDataset synthiaDataset;

  // list of topics which the XViewBagReader observes through its views.
  std::vector<std::string> topic_names = {params_.semantics_image_topic_back,
                                          params_.semantics_image_topic_front,
                                          params_.semantics_image_topic_right};


  // remove the '/' char at the beginning of the topic names and create
  // corresponding topic views (this might be a bug in ros, as for creating
  // listeners and publishers the '/' char is needed, while to access the
  // data in a bag file one needs to remove it)
  for (auto& s : topic_names) {
    if (s.front() == '/')
      s.erase(s.begin());
    // create a view on the topic contained in the bag file
    topic_views_[s] = RosbagTopicView(bag_, s);
  }
}

void XViewBagReader::iterateBagForwards(const std::string& image_topic) {
  auto const& view = topic_views_[image_topic];
  for (int i = 0; i < view.size_; ++i) {
    LOG(INFO) << "Processing semantic image at index " << i;
    parseParameters();
    const cv::Mat semantic_image = view.getSemanticImageAtFrame(i);
    const cv::Mat depth_image;
    const x_view::SE3 pose;
    x_view::FrameData frame_data(semantic_image, depth_image, pose);
    x_view_->processFrameData(frame_data);
  }
}
void XViewBagReader::iterateBagBackwards(const std::string& image_topic) {
  auto const& view = topic_views_[image_topic];
  for (int i = view.size_ - 1; i >= 0; --i) {
    LOG(INFO) << "Processing semantic image at index " << i;
    parseParameters();
    const cv::Mat semantic_image = view.getSemanticImageAtFrame(i);
    const cv::Mat depth_image;
    const x_view::SE3 pose;
    x_view::FrameData frame_data(semantic_image, depth_image, pose);
    x_view_->processFrameData(frame_data);
  }
}
void XViewBagReader::iterateBagFromTo(const std::string& image_topic,
                                      const int from, const int to) {
  auto const& view = topic_views_[image_topic];
  const int step = (from <= to ? +1 : -1);
  for (int i = from; step * i < step * to; i += step) {
    std::cout << "Processing semantic image at index " << i << std::endl;
    parseParameters();
    const cv::Mat semantic_image = view.getSemanticImageAtFrame(i);
    const cv::Mat depth_image;
    const x_view::SE3 pose;
    x_view::FrameData frame_data(semantic_image, depth_image, pose);
    x_view_->processFrameData(frame_data);
  }
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
  if (!nh_.getParam("/XViewBagReader/semantics_image_topic_back",
                    params_.semantics_image_topic_back)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/semantics_image_topic_back'";
  }
  if (!nh_.getParam("/XViewBagReader/semantics_image_topic_front",
                    params_.semantics_image_topic_front)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/semantics_image_topic_front'";
  }
  if (!nh_.getParam("/XViewBagReader/semantics_image_topic_right",
                    params_.semantics_image_topic_right)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/semantics_image_topic_right'";
  }
  if (!nh_.getParam("/XViewBagReader/sensor_frame",
                    params_.sensor_frame)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/sensor_frame'";
  }
  if (!nh_.getParam("/XViewBagReader/world_frame",
                    params_.world_frame)) {
    LOG(ERROR) << "Failed to get param "
        "'/XViewBagReader/world_frame'";
  }
}

}