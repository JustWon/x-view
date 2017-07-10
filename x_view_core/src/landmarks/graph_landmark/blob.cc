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
  computeBlobCenter();
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

void Blob::computeBlobCenter() {
  if (c_blob_.Area(AreaMode::PIXELWISE) >= 5) {
    ellipse = cv::fitEllipse(external_contour_pixels);
  } else {
    float r;
    cv::minEnclosingCircle(external_contour_pixels, ellipse.center, r);
    ellipse.size = cv::Size(r, r);
  }

  // Make sure the center of the fitted ellipse/circle is contained in the
  // blob, as this pixel coordinate will be used to compute the 3D position
  // of the associated semantic entity.
  // pointPolygonTest returns positive (inside), negative (outside), or zero (on
  // an edge) value.
  if(cv::pointPolygonTest(external_contour_pixels, ellipse.center, true) < 0) {
    LOG(WARNING) << "Computed blob center is not inside blob. Shifting center"
        " to closest contour point.";
    int closest_index = -1;
    int min_distance_squared = std::numeric_limits<int>::max();
    const cv::Point2i old_center(ellipse.center.x, ellipse.center.y);
    for(int i = 0; i < external_contour_pixels.size(); ++i) {
      const cv::Point2i& pixel = external_contour_pixels[i];
      const cv::Point2i diff = pixel - old_center;
      const int dist_squared = diff.dot(diff);
      if(dist_squared < min_distance_squared) {
        min_distance_squared = dist_squared;
        closest_index = i;
      }
    }

    CHECK(closest_index != -1);
    // Assign the pixel associated to the closest index to the pixel center.
    pixel_center = ellipse.center = external_contour_pixels[closest_index];
  }

  // Scale down the ellipse by a factor of two.
  ellipse.size.height *= 0.5;
  ellipse.size.width *= 0.5;

  pixel_center = ellipse.center;
}

std::ostream& operator<<(std::ostream& out, const Blob& blob) {
  out << "label: " << blob.semantic_label
      << ", instance: " << blob.instance
      << ", num pixels: " << blob.num_pixels
      << ", center: " << blob.pixel_center
      << ", bounding box: " << blob.bounding_box;
  return out;
}

}

