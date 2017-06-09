#include <x_view_core/matchers/graph_matcher/similarity_plotter.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

namespace x_view {

int SimilarityPlotter::desired_image_size_ = 500;
int SimilarityPlotter::colormap_ = cv::COLORMAP_OCEAN;

cv::Mat SimilarityPlotter::getImageFromSimilarityMatrix(
    const Eigen::MatrixXf& similarity_matrix) {

  // Normalize score matrix and convert them to opencv image.
  const float max_score = similarity_matrix.maxCoeff();

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> normalized_similarity =
      (similarity_matrix / max_score * 255).cast<unsigned char>();

  cv::Mat cv_scores, color_scores;
  cv::eigen2cv(normalized_similarity, cv_scores);
  cv::resize(cv_scores, cv_scores,
             SimilarityPlotter::computeSize(cv_scores.size()),  0,
             0, cv::INTER_NEAREST);

  cv::applyColorMap(cv_scores, color_scores, SimilarityPlotter::colormap_);

  return color_scores;

}

cv::Mat SimilarityPlotter::getMaxColwiseImageFromSimilarityMatrix(
    const Eigen::MatrixXf& similarity_matrix) {

  // Normalize similarity matrix
  const float max_score = similarity_matrix.maxCoeff();
  Eigen::MatrixXf normalized_similarity = similarity_matrix / max_score;

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
      scores_max(normalized_similarity.rows(), normalized_similarity.cols());
  scores_max.setZero();

  for (int i = 0; i < normalized_similarity.cols(); ++i) {
    int max_row_index;
    normalized_similarity.col(i).maxCoeff(&max_row_index);
    scores_max(max_row_index, i) = 255;
  }
  cv::Mat show_scores_max;
  cv::eigen2cv(scores_max, show_scores_max);
  cv::resize(show_scores_max, show_scores_max,
             SimilarityPlotter::computeSize(show_scores_max.size()),
             0, 0, cv::INTER_NEAREST);

  return show_scores_max;
}


cv::Mat SimilarityPlotter::getMaxRowwiseImageFromSimilarityMatrix(
    const Eigen::MatrixXf& similarity_matrix) {

  // Normalize similarity matrix
  const float max_score = similarity_matrix.maxCoeff();
  Eigen::MatrixXf normalized_similarity = similarity_matrix / max_score;

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
      scores_max(normalized_similarity.rows(), normalized_similarity.cols());
  scores_max.setZero();

  for (int i = 0; i < normalized_similarity.rows(); ++i) {
    int max_col_index;
    normalized_similarity.row(i).maxCoeff(&max_col_index);
    scores_max(i, max_col_index) = 255;
  }
  cv::Mat show_scores_max;
  cv::eigen2cv(scores_max, show_scores_max);
  cv::resize(show_scores_max, show_scores_max,
             SimilarityPlotter::computeSize(show_scores_max.size()),
             0, 0, cv::INTER_NEAREST);

  return show_scores_max;
}

const cv::Size SimilarityPlotter::computeSize(const cv::Size& original_size) {
  const int original_height = original_size.height;
  const int original_width = original_size.width;

  auto roundToClosestMultiple = [](const int number, const int multiple) {
    return ((number + multiple / 2) / multiple) * multiple;
  };

  const int resulting_height = roundToClosestMultiple
      (SimilarityPlotter::desired_image_size_, original_height);
  const int resulting_width = roundToClosestMultiple
      (SimilarityPlotter::desired_image_size_, original_width);

  if (resulting_height != 0 && resulting_width != 0)
    return cv::Size(resulting_width, resulting_height);
  else
    return original_size;
}


}