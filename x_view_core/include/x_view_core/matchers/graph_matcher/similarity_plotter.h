#ifndef X_VIEW_SIMILARITY_PLOTTER_H
#define X_VIEW_SIMILARITY_PLOTTER_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>


namespace Eigen {

typedef Matrix<uchar, Eigen::Dynamic, Eigen::Dynamic> MatrixXuc;
typedef Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;

}

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
   * \param auto_size If true, then the generated image will be resized
   * conveniently for displaying purposes, otherwise the generated image will
   * be of the same size of the similarity matrix.
   * \return An image representing the matrix passed as argument.
   * \note If a sparse matrix is passed as argument, then it is automatically
   * converted to a dense format. For large matrices this should be avoided.
   */
  static cv::Mat getImageFromSimilarityMatrix(
      const Eigen::MatrixXf& similarity_matrix, bool auto_size = true);

  static cv::Mat getImageFromSimilarityMatrix(
      const Eigen::MatrixXb& max_similarity_matrix, bool auto_size = true);

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
