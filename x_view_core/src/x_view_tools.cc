#include <x_view_core/x_view_tools.h>

#include <glog/logging.h>

#include <iostream>

namespace x_view {

int twoBytesToInt(const unsigned char b1, const unsigned char b2) {
  return static_cast<int>((b2 << 8) | b1);
};

int twoBytesToInt(const unsigned char* b) {
  return twoBytesToInt(b[0], b[1]);
}

cv::Mat extractChannelFromImage(const cv::Mat& image, const int channel) {
  if (image.channels() == 1)
    return image;

  CHECK_LT(channel, image.channels()) << "Image has less channels than the "
        "requested one. Make sure the input image has multiple channels";

  std::vector<cv::Mat> image_channels(image.channels());
  cv::split(image, image_channels);
  return image_channels[channel];
}

const std::string& getRootDirectory() {
  static std::string x_view_root = std::string(X_VIEW_XSTR(X_VIEW_ROOT_DIR));
  return x_view_root;
}

const std::string& getLogDirectory() {
  static std::string x_view_log = std::string(X_VIEW_XSTR(X_VIEW_LOG_DIR));
  return x_view_log;
}

void setupLogging(char** argv) {

  const std::string& log_dir_name = x_view::getLogDirectory();

  std::vector<std::pair<const int, std::string> > log_file_names =
      {{google::INFO, "log_INFO"},
       {google::WARNING, "log_WARN"},
       {google::ERROR, "log_ERR"},
       {google::FATAL, "log_FATAL"}};

  for (const auto& level : log_file_names) {
    google::SetLogDestination(level.first,
                              (log_dir_name + level.second).c_str());

    google::SetLogSymlink(level.first, "__LAST");
  }

  // Print logs also to the console if their level is greater than
  // min_console_level;
  const int min_console_level = google::ERROR;
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(min_console_level);

#ifdef X_VIEW_DEBUG
  FLAGS_alsologtostderr = true;
#endif

  google::InitGoogleLogging(argv[0]);

  std::cout << "X-View is logging to <" << log_dir_name << ">" << std::endl;

}

void finalizeLogging() {
  google::FlushLogFiles(google::INFO);
  google::FlushLogFiles(google::WARNING);
  google::FlushLogFiles(google::ERROR);
  google::FlushLogFiles(google::FATAL);
}

};
