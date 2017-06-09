#ifndef X_VIEW_SIMILARITY_PLOTTER_H
#define X_VIEW_SIMILARITY_PLOTTER_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace x_view {

/**
 * \brief Class responsible for converting an Eigen Matrix containing
 * floating point elements (similarity scores) to a viewable opencv image.
 */
class SimilarityPlotter {
 public:

  /**
   * \brief Sets the desired image size for the images generated by this class.
   * \param desired_image_size Desired image size.
   * \note The desired image size passed as argument is only a hint for
   * computing the dimension of the generated images.
   */
  static void setDesiredSize(const int desired_image_size) {
    SimilarityPlotter::desired_image_size_ = desired_image_size;
  }

  /**
   * \brief Generates an image which displays the matrix passed as argument
   * as a heat map.
   * \param similarity_matrix Matrix used to generate the image.
   * \return An image representing the matrix passed as argument.
   * \note If a sparse matrix is passed as argument, then it is automatically
   * converted to a dense format. For large matrices this should be avoided.
   */
  static cv::Mat getImageFromSimilarityMatrix(
      const Eigen::MatrixXf& similarity_matrix);

  /**
   * \brief Generates an image which a pixel (i,j) is white only if the
   * associated value is the larges of the i-th column.
   * \param similarity_matrix Matrix used to generate the image.
   * \return An image representing the max similarity of the similarity
   * matrix passed as argument.
   * \note If a sparse matrix is passed as argument, then it is automatically
   * converted to a dense format. For large matrices this should be avoided
   */
  static cv::Mat getMaxColwiseImageFromSimilarityMatrix(
      const Eigen::MatrixXf& similarity_matrix);

  /**
 * \brief Generates an image which a pixel (i,j) is white only if the
 * associated value is the larges of the j-th row.
 * \param similarity_matrix Matrix used to generate the image.
 * \return An image representing the max similarity of the similarity
 * matrix passed as argument.
 * \note If a sparse matrix is passed as argument, then it is automatically
 * converted to a dense format. For large matrices this should be avoided
 */
  static cv::Mat getMaxRowwiseImageFromSimilarityMatrix(
      const Eigen::MatrixXf& similarity_matrix);

  /**
   * \brief Sets the colormap of the plotted similarity matrix.
   * \param colormap Enum (int) referring to the opencv colormap to be used
   * while plotting the similarity matrix.
   */
  static void setColormap(const int colormap) {
    SimilarityPlotter::colormap_ = colormap;
  }

 private:
  static int desired_image_size_;
  static int colormap_;
  static const cv::Size computeSize(const cv::Size& original_shape);
};

}

#endif //X_VIEW_SIMILARITY_PLOTTER_H
