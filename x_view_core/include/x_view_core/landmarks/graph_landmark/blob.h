#ifndef X_VIEW_BLOB_H
#define X_VIEW_BLOB_H

#include <x_view_core/x_view_types.h>

#include <Eigen/Core>
#include <opencvblobslib/blob.h>

namespace x_view {

/**
* \brief A Blob contains all pixels belonging to a connected component
* determined by their class labels.
*/
class Blob {

 public:
  Blob();
  Blob(const int semantic_label, const int instance, const CBlob& c_blob);

  /// \brief Semantic label associated to all pixels contained in this blob.
  int semantic_label;

  /// \brief Instance value associated to the blob. If a blob has no
  /// instance (e.g. static objects in Synthia dataset), then the value
  /// associated to this member is instance = -1.
  int instance;

  /// \brief Vector of pixels representing the contour of this blob.
  std::vector<cv::Point2i> external_contour_pixels;
  std::vector<std::vector<cv::Point2i>> internal_contour_pixels;

  /// \brief Number of pixels in this blob.
  int num_pixels;

  /// \brief Pixel representing the center of the blob.
  cv::Point2i pixel_center;

  /// \brief Ellipse fitted to external contours of blob.
  cv::RotatedRect ellipse;

  /// \brief Blob axis aligned bounding box used to perform early termination
  /// for blobs overlap test.
  cv::Rect bounding_box;

  /**
   * \brief Determines if the two blobs passed as argument are neighbors.
   * \param bi First queried blob.
   * \param bj Second queried blob.
   * \param distance A pixel pi is neighbor of pixel pj if their distance is
   * smaller or equal to this parameter.
   * \return True if the blobs are neighbors, false otherwise.
   * \details A blob bi is neighbor of blob bj if at least one pixel pi
   * belonging to the contours ci of blob bi is a neighbor pixel of a pixel pj
   * belonging to the contours cj of blob bj.
   */
  static bool areNeighbors(const Blob& bi, const Blob& bj, const int distance);

 private:
  /// \brief Object generated by the opencvblobslib containing data used to
  /// execute the functions provided by this struct.
  CBlob c_blob_;

  /// \brief Fits the contour pixels and extracts geometrical properties.
  void computeContours();

  /// \brief Computes the number of pixels belonging to the blob.
  void computeArea() {
    num_pixels = static_cast<int>(c_blob_.Area(AreaMode::PIXELWISE));
  }

  /// \brief Computes the external bounding boxes of the blob. The bounding
  /// boxes are used for early termination during neighbor check.
  void computeBoundingBox();

  /// \brief Computes the blob center by trying to fit an ellipse to the
  /// contours and taking its center.
  void computeBlobCenter();

};

/// \brief Overloaded operator to print a Blob.
std::ostream& operator<<(std::ostream& out, const Blob& blob);

}

#endif //X_VIEW_BLOB_H
