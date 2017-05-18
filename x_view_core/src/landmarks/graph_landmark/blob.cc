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
}

void Blob::computeContours() {
  external_contour_pixels_ =
      c_blob_.GetExternalContour()->GetContourPoints();
  internal_contour_pixels_.clear();
  for (auto& internal_contour : c_blob_.GetInternalContours()) {
    internal_contour_pixels_.push_back(internal_contour->GetContourPoints());
  }
}

}

