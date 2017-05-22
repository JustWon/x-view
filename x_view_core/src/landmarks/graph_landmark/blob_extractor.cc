#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_tools.h>

#include <opencvblobslib/BlobResult.h>

namespace x_view {

int BlobExtractorParams::BlobSizeFiltering::minimumBlobSize(const cv::Size&
image_size) const {
  if (type_ == MIN_BLOB_SIZE_TYPE::ABSOLUTE) {
    return static_cast<int>(n_min_pixels_);
  } else if (type_ == MIN_BLOB_SIZE_TYPE::RELATIVE_TO_IMAGE_SIZE) {
    const int num_total_pixels = image_size.width * image_size.height;
    const int min_blob_size = static_cast<int>(num_total_pixels * n_min_pixels_);
    return min_blob_size;
  }
}

ImageBlobs BlobExtractor::findBlobsWithContour(const cv::Mat& image,
                                               const BlobExtractorParams&
                                               params) {

  const cv::Mat all_labels_image = extractChannelFromImage(image, 0);
  ImageBlobs image_blobs(global_dataset_ptr->numSemanticClasses());

  // extract only the pixels associated with each class, and compute their
  // corresponding contours
  for (int c = 0; c < global_dataset_ptr->numSemanticClasses(); ++c) {
    cv::Mat current_class_layer;
    cv::inRange(all_labels_image, cv::Scalar(c), cv::Scalar(c),
                current_class_layer);

    // compute a smoother version of the image by performing dilation
    // followed by erosion
    cv::Mat dilated, eroded;
    if (params.dilate_and_erode_) {

      cv::dilate(current_class_layer, dilated, cv::Mat(), cv::Point(-1, -1),
                 params.num_dilate_reps_);
      cv::erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1),
                params.num_erode_reps_);
    } else
      eroded = current_class_layer;

    const int num_threads = params.num_threads_;
    const int minimum_blob_size = params.minimumBlobSize(image.size());
    CBlobResult res(eroded, cv::Mat(), num_threads);
    for (int b = 0; b < res.GetNumBlobs(); ++b) {
      if (res.GetBlob(b)->Area(AreaMode::PIXELWISE) >= minimum_blob_size)
        image_blobs[c].push_back(Blob(c, *(res.GetBlob(b))));
    }

  }

  return image_blobs;
}
}

