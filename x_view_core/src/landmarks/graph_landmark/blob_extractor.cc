#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_tools.h>

#include <opencvblobslib/BlobResult.h>

#include <unordered_set>

namespace x_view {

int BlobExtractorParams::BlobSizeFiltering::minimumBlobSize(const cv::Size&
image_size) const {
  if (type_ == MIN_BLOB_SIZE_TYPE::ABSOLUTE) {
    return static_cast<int>(n_min_pixels_);
  } else if (type_ == MIN_BLOB_SIZE_TYPE::RELATIVE_TO_IMAGE_SIZE) {
    const int num_total_pixels = image_size.width * image_size.height;
    const int
        min_blob_size = static_cast<int>(num_total_pixels * n_min_pixels_);
    return min_blob_size;
  }
}

ImageBlobs BlobExtractor::findBlobsWithContour(const cv::Mat& image,
                                               const BlobExtractorParams&
                                               params) {

  // The pixels in this image indicate the semantic label.
  const cv::Mat all_labels_image = extractChannelFromImage(image, 0);
  // The pixels in this image indicate the instance id associated to the blob.
  const cv::Mat all_instances_image = extractChannelFromImage(image, 1);

  ImageBlobs image_blobs(global_dataset_ptr->numSemanticClasses());

  // Iterate over the semantic classes and process only the pixels which
  // belong to the semantic class.
  for (int c = 0; c < global_dataset_ptr->numSemanticClasses(); ++c) {
    // This image is a binary image, where a pixel is set to 1 only if its
    // corresponding semantic label is equal to c. Otherwise the pixel is set
    // to 0.
    cv::Mat current_class_layer;
    cv::inRange(all_labels_image, cv::Scalar(c), cv::Scalar(c),
                current_class_layer);

    // Create an image containing instance information of pixels belonging to
    // the current semantic class. This is accomplished by performing and AND
    // operation on the all_instances_image with the current_class_layer
    // binary mask.
    cv::Mat instance_layer = all_instances_image & current_class_layer;

    // If the current semantic class has some distinguishable instances,
    // process them.
    if (cv::countNonZero(instance_layer) > 0) {
      BlobExtractor::extractBlobsConsideringInstances(instance_layer,
                                                      &(image_blobs[c]),
                                                      c, params);
    }
      // Otherwise the pixels associated to the current semantic label have
      // no instance information.
    else {
      BlobExtractor::extractBlobsWithoutInstances(current_class_layer,
                                                  &(image_blobs[c]),
                                                  c, params);
    }
  }
  return image_blobs;
}

void BlobExtractor::extractBlobsConsideringInstances(const cv::Mat& instance_layer,
                                                     ClassBlobs* class_blobs,
                                                     const int current_semantic_class,
                                                     const BlobExtractorParams& params) {
  // Collect all different instances of the current label.
  std::unordered_set<unsigned char> instance_set;
  for (int i = 0; i < instance_layer.rows; ++i) {
    const unsigned char* row_ptr = instance_layer.ptr<uchar>(i);
    for (int j = 0; j < instance_layer.cols; j++)
      instance_set.insert(row_ptr[j]);
  }

  // Remove the instance '0' as it is not a real instance, but rather it
  // comes from the masking operation when creating instance_layer.
  const unsigned long removed_zero =
      instance_set.erase(static_cast<unsigned char>(0));
  CHECK(removed_zero > 0) << "instance_layer image should contain zero "
      "elements resulting from the masking with the current_class_layer "
      "image.";

#ifdef X_VIEW_DEBUG
  std::cout << "Class " << current_semantic_class << " has "
            << instance_set.size() << " instances" << std::endl;
#endif
  // Iterate over all instance found in the current semantic class.
  for (const unsigned char instance_value : instance_set) {
    // Extract only the instance with instance value equal to
    // instance_value.
    cv::Mat current_instance;
    cv::inRange(instance_layer, cv::Scalar(instance_value),
                cv::Scalar(instance_value), current_instance);

    // compute a smoother version of the image by performing dilation
    // followed by erosion.
    cv::Mat dilated, eroded;
    if (params.dilate_and_erode_) {
      cv::dilate(current_instance, dilated, cv::Mat(), cv::Point(-1, -1),
                 params.num_dilate_reps_);
      cv::erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1),
                params.num_erode_reps_);
    } else
      eroded = current_instance;

    // Extract the blobs associated to the current instance. Usually
    // there is only one blob per instance, but due to occlusions in
    // the scene, a single instance could also be composed by two blobs.
    const int num_threads = params.num_threads_;
    const int minimum_blob_size = params.minimumBlobSize(instance_layer.size());
    CBlobResult res(eroded, cv::Mat(), num_threads);
    for (int b = 0; b < res.GetNumBlobs(); ++b) {
      if (res.GetBlob(b)->Area(AreaMode::PIXELWISE) >= minimum_blob_size) {
        class_blobs->push_back(
            Blob(current_semantic_class,
                 static_cast<int>(instance_value),
                 *(res.GetBlob(b))));
#ifdef X_VIEW_DEBUG
        std::cout << "Added blob with size "
                  << class_blobs->back().num_pixels_
                  << " and instance id: "
                  << class_blobs->back().instance_
                  << std::endl;
#endif
      }
    }
  }
}

void BlobExtractor::extractBlobsWithoutInstances(const cv::Mat& current_class_layer,
                                                 ClassBlobs* class_blobs,
                                                 const int current_semantic_class,
                                                 const BlobExtractorParams& params) {
#ifdef X_VIEW_DEBUG
  std::cout << "Class " << current_semantic_class << " has no instances"
            << std::endl;
#endif
  // Compute a smoother version of the image by performing dilation
  // followed by erosion.
  cv::Mat dilated, eroded;
  if (params.dilate_and_erode_) {
    cv::dilate(current_class_layer, dilated, cv::Mat(), cv::Point(-1, -1),
               params.num_dilate_reps_);
    cv::erode(dilated, eroded, cv::Mat(), cv::Point(-1, -1),
              params.num_erode_reps_);
  } else
    eroded = current_class_layer;

  // Extract all blobs associated with the current semantic class.
  const int num_threads = params.num_threads_;
  const int minimum_blob_size =
      params.minimumBlobSize(current_class_layer.size());
  CBlobResult res(eroded, cv::Mat(), num_threads);
  for (int b = 0; b < res.GetNumBlobs(); ++b) {
    if (res.GetBlob(b)->Area(AreaMode::PIXELWISE) >= minimum_blob_size) {
      const int instance_value = -1;
      class_blobs->push_back(Blob(current_semantic_class,
                                  instance_value,
                                  *(res.GetBlob(b))));
#ifdef X_VIEW_DEBUG
      std::cout << "Added blob with size "
                << class_blobs->back().num_pixels_
                << " and instance id: "
                << class_blobs->back().instance_
                << std::endl;
#endif
    }
  }
}
}

