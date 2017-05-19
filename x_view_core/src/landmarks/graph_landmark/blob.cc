#include <x_view_core/landmarks/graph_landmark/blob.h>

namespace x_view {

Blob::Blob() : semantic_label_(-1), c_blob_() {
}

Blob::Blob(const int semantic_label, const CBlob& c_blob)
    : semantic_label_(semantic_label),
      c_blob_(c_blob) {
  computeContours();
  computeArea();
  computeCenter();
  computeBoundingBox();
}

bool Blob::areNeighbors(const Blob& bi, const Blob& bj, const int distance) {

  const int distance_squared = distance * distance;

  const cv::Rect& bounding_box_i = bi.bounding_box_;
  const cv::Rect& bounding_box_j = bj.bounding_box_;

  if ((bounding_box_i & bounding_box_j).area() == 0)
    return false;

  // external contour
  const std::vector<cv::Point>& external_contour_i = bi
      .external_contour_pixels_;
  const std::vector<cv::Point>& external_contour_j = bj
      .external_contour_pixels_;

  for (const cv::Point& pi : external_contour_i)
    for (const cv::Point& pj : external_contour_j) {
      // compute pixel distance
      cv::Point diff = pi - pj;
      const int dist2 = diff.dot(diff);
      if (dist2 <= distance_squared)
        return true;
    }

  // internal contours
  const std::vector<std::vector<cv::Point>>& internal_contours_i =
      bi.internal_contour_pixels_;
  const std::vector<std::vector<cv::Point>>& internal_contours_j =
      bj.internal_contour_pixels_;

  for (const auto& internal_contour_i : internal_contours_i)
    for (const cv::Point& pi : internal_contour_i)
      for (const cv::Point& pj : external_contour_j) {
        // compute pixel distance
        cv::Point diff = pi - pj;
        const int dist2 = diff.dot(diff);
        if (dist2 <= distance_squared)
          return true;
      }

  for (const auto& internal_contour_j : internal_contours_j)
    for (const cv::Point& pj : internal_contour_j)
      for (const cv::Point& pi : external_contour_i) {
        // compute pixel distance
        cv::Point diff = pi - pj;
        const int dist2 = diff.dot(diff);
        if (dist2 <= distance_squared)
          return true;
      }

  return false;
}

void Blob::computeContours() {
  external_contour_pixels_ =
      c_blob_.GetExternalContour()->GetContourPoints();
  internal_contour_pixels_.clear();
  for (auto& internal_contour : c_blob_.GetInternalContours()) {
    internal_contour_pixels_.push_back(internal_contour->GetContourPoints());
  }
}

void Blob::computeBoundingBox() {
  if (external_contour_pixels_.size() == 0)
    computeContours();

  // iterate over the pixels belonging to the blob contour and determine the
  // minimum and maximum value for the x and y coordinate.
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();

  for (const cv::Point& p : external_contour_pixels_) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  // grow the bb in order to have intersections also for bounding boxes that
  // are aligned with the axes.
  min_x -= 1;
  min_y -= 1;
  max_x += 1;
  max_y += 1;

  bounding_box_.x = min_x;
  bounding_box_.y = min_y;
  bounding_box_.width = max_x - min_x;
  bounding_box_.height = max_y - min_y;
}

}

