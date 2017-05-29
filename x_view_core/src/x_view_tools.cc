#include <x_view_core/x_view_tools.h>

#include <glog/logging.h>

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

};
