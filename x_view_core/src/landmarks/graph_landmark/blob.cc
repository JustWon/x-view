#include <x_view_core/landmarks/graph_landmark/blob.h>

#include <glog/logging.h>

namespace x_view {

Blob::Blob() : semantic_label(-1), c_blob_() {
}

Blob::Blob(const int semantic_label, const int instance, const CBlob& c_blob)
    : semantic_label(semantic_label),
      instance(instance),
      c_blob_(c_blob) {
  computeContours();
  computeArea();
  computeBoundingBox();
  computeFittingEllipse();
}

bool Blob::areNeighbors(const Blob& bi, const Blob& bj, const int distance) {

  const int distance_squared = distance * distance;

  // Increase the box dimension to avoid rejecting matches between blobs in
  // case they are close but their original bounding_box's don't touch.
  const cv::Point box_shift(distance, distance);
  const cv::Size box_expansion(2 * distance, 2 * distance);

  const cv::Rect& bounding_box_i =
      (bi.bounding_box - box_shift) + box_expansion;
  const cv::Rect& bounding_box_j =
      (bj.bounding_box - box_shift) + box_expansion;

  if ((bounding_box_i & bounding_box_j).area() == 0)
    return false;

  // External contour.
  const std::vector<cv::Point>& external_contour_i = bi
      .external_contour_pixels;
  const std::vector<cv::Point>& external_contour_j = bj
      .external_contour_pixels;

  for (const cv::Point& pi : external_contour_i)
    for (const cv::Point& pj : external_contour_j) {
      // Compute pixel distance.
      cv::Point diff = pi - pj;
      const int dist2 = diff.dot(diff);
      if (dist2 <= distance_squared)
        return true;
    }

  // Internal contours.
  const std::vector<std::vector<cv::Point>>& internal_contours_i =
      bi.internal_contour_pixels;
  const std::vector<std::vector<cv::Point>>& internal_contours_j =
      bj.internal_contour_pixels;

  for (const auto& internal_contour_i : internal_contours_i)
    for (const cv::Point& pi : internal_contour_i)
      for (const cv::Point& pj : external_contour_j) {
        // Compute pixel distance.
        cv::Point diff = pi - pj;
        const int dist2 = diff.dot(diff);
        if (dist2 <= distance_squared)
          return true;
      }

  for (const auto& internal_contour_j : internal_contours_j)
    for (const cv::Point& pj : internal_contour_j)
      for (const cv::Point& pi : external_contour_i) {
        // Compute pixel distance.
        cv::Point diff = pi - pj;
        const int dist2 = diff.dot(diff);
        if (dist2 <= distance_squared)
          return true;
      }

  return false;
}

void Blob::computeContours() {
  external_contour_pixels =
      c_blob_.GetExternalContour()->GetContourPoints();
  internal_contour_pixels.clear();
  for (auto& internal_contour : c_blob_.GetInternalContours()) {
    internal_contour_pixels.push_back(internal_contour->GetContourPoints());
  }
}

void Blob::computeBoundingBox() {
  if (external_contour_pixels.size() == 0)
    computeContours();

  bounding_box = cv::boundingRect(external_contour_pixels);
}

void Blob::computeFittingEllipse() {
  if (c_blob_.Area(AreaMode::PIXELWISE) >= 5)
    ellipse = cv::fitEllipse(external_contour_pixels);

  // If the distance of the fitted ellipse's center to the contour of the
  // blob is larger than 'max_distance_thresh', then simply compute the center
  // of the blob by fitting a circle to the contour. This threshold is set
  // empirically and should be adapted in case of different input image
  // resolution.
  const int max_distance_thresh = 30;
  if (cv::pointPolygonTest(external_contour_pixels, ellipse.center,
  true) < -max_distance_thresh) {
    LOG(WARNING) << "Computing the center of blob using the minEnclosingCircle"
                 << " algorithm as the ellipse fitting was not effective.";
    float r;
    cv::minEnclosingCircle(external_contour_pixels, ellipse.center, r);
    ellipse.size = cv::Size(r, r);
  }
  // Scale down the ellipse by a factor of two.
  ellipse.size.height *= 0.5;
  ellipse.size.width *= 0.5;

  center = ellipse.center;
}

std::ostream& operator<<(std::ostream& out, const Blob& blob) {
  out << "label: " << blob.semantic_label
      << ", instance: " << blob.instance
      << ", num pixels: " << blob.num_pixels
      << ", center: " << blob.center
      << ", bounding box: " << blob.bounding_box;
  return out;
}

}

