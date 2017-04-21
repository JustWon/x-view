#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/features/vector_feature.h>

namespace x_view {
HistogramLandmark::HistogramLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {


  // FIXME: size of histogram must depend on dataset
  const int dataset_size = 13;
  std::vector<int> histogram_count(dataset_size, 0);

  // iterate through the semantic image and determine how many pixels vote
  // for each semantic class by inspecting the first channel of each pixel
  for (int i = 0; i < image.rows; ++i) {
    for (int j = 0; j < image.cols; ++j) {
      // get the label associated to this pixel
      int label = image.at<cv::Vec3b>(cv::Point(j, i)).val[0];
      if (label < 13 && label > 0)
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

  // create the feature stored in this landmark by generating a VectorFeature
  // containing the histogram data
  feature_ = std::make_shared<VectorFeature>(VectorFeature(descriptor));

}
}

