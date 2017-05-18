#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>

#include <x_view_core/x_view_tools.h>
#include <x_view_core/datasets/abstract_dataset.h>

#include <opencvblobslib/BlobResult.h>

namespace x_view {

ImageBlobs BlobExtractor::findBlobsWithContour(const cv::Mat& image,
                                               bool dilate_and_erode,
                                               int min_blob_size) {

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
    if (dilate_and_erode) {

      cv::dilate(current_class_layer, dilated, cv::Mat(), cv::Point(-1, -1), 3);
      cv::erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1), 3);
    } else
      eroded = current_class_layer;

    const int num_threads = 4;
    CBlobResult res(eroded, cv::Mat(), num_threads);
    for (int b = 0; b < res.GetNumBlobs(); ++b) {
      if (res.GetBlob(b)->Area(AreaMode::PIXELWISE) >= min_blob_size)
        image_blobs[c].push_back(Blob(c, *(res.GetBlob(b))));
    }

  }

  return image_blobs;
}
}

