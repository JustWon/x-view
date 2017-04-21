#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/features/vector_feature.h>

namespace x_view {
HistogramLandmark::HistogramLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {


  // FIXME: size of histogram must depend on dataset
  const int dataset_size = 13;
  std::vector<int> histogram_count(dataset_size, 0);

  for (int i = 0; i < image.rows; ++i) {
    for (int j = 0; j < image.cols; ++j) {
      // get the label associated to this pixel
      int label = image.at<cv::Vec3b>(cv::Point(j, i)).val[0];
      if (label < 13 && label > 0)
        ++histogram_count[label];
    }
  }

  // copy the vector to a cv::Mat
  cv::Mat descriptor(1, histogram_count.size(), CV_32FC1);
  for (int k = 0; k < histogram_count.size(); ++k) {
    descriptor.at<float>(k) = float(histogram_count[k]);
  }

  std::cout << descriptor << std::endl;

  feature_ = std::make_shared<VectorFeature>(VectorFeature(descriptor));

}
}

