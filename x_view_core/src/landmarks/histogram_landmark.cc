#include <x_view_core/landmarks/histogram_landmark.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/vector_descriptor.h>
#include <x_view_core/x_view_tools.h>

namespace x_view {
HistogramLandmark::HistogramLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {

  const int dataset_size = global_dataset_ptr->numSemanticClasses();
  std::vector<int> histogram_count(dataset_size, 0);

  // we are only interested in the labels of the image, contained in the
  // first channel of the semantic_image_ object
  const cv::Mat label_image = extractChannelFromImage(semantic_image_, 0);

  // iterate through the semantic image and determine how many pixels vote
  // for each semantic class by inspecting the first channel of each pixel
  for (int i = 0; i < semantic_image_.rows; ++i) {
    for (int j = 0; j < semantic_image_.cols; ++j) {
      // get the label associated to this pixel
      int label = static_cast<int>(label_image.at<uchar>(cv::Point(j, i)));
      if (label < dataset_size && label >= 0)
        ++histogram_count[label];
    }
  }

  // since the feature representation must be a cv::Mat (due to the
  // vector-matcher which accepts only cv::Mats as features), convert the
  // integer histogram into a normalized 1D cv::Mat containing the voting
  // frequencies as percentages
  const int votes = std::accumulate(histogram_count.begin(),
                                    histogram_count.end(), int(0));
  cv::Mat descriptor(1, histogram_count.size(), CV_32FC1);
  for (int k = 0; k < histogram_count.size(); ++k) {
    descriptor.at<float>(k) = float(histogram_count[k]) / votes;
  }

  // create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the histogram data
  descriptor_ =
      std::make_shared<VectorDescriptor>(VectorDescriptor(descriptor));

}
}

