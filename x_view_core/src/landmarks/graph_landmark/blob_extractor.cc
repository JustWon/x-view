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

    // If the current semantic class has no distinguishable instances,
    // process the input image only considering the semantic labels.
    if (cv::countNonZero(instance_layer) == 0) {
      BlobExtractor::extractBlobsWithoutInstances(current_class_layer,
                                                  &(image_blobs[c]),
                                                  c, params);
    }
      // Otherwise the pixels associated to the current semantic label have
      // instance information and each instance has to be extracted separately.
    else {
      BlobExtractor::extractBlobsConsideringInstances(instance_layer,
                                                      &(image_blobs[c]),
                                                      c, params);
    }
  }
  return image_blobs;
}

void BlobExtractor::extractBlobsWithoutInstances(cv::Mat& current_class_layer,
                                                 ClassBlobs* class_blobs,
                                                 const int current_semantic_class,
                                                 const BlobExtractorParams& params) {

  LOG(INFO) << "Class " << current_semantic_class << " has no instances.";

  // Since there are no instances, set the instance_value to -1.
  const int instance_value = -1;

  // Compute a smoother version of the image by performing dilation
  // followed by erosion.
  if (params.dilate_and_erode_)
    BlobExtractor::dilateAndErode(&current_class_layer, params);

  BlobExtractor::extractBlobsAndAddToContainer(current_class_layer,
                                               class_blobs,
                                               instance_value,
                                               current_semantic_class,
                                               params);

}

void BlobExtractor::extractBlobsConsideringInstances(cv::Mat& instance_layer,
                                                     ClassBlobs* class_blobs,
                                                     const int current_semantic_class,
                                                     const BlobExtractorParams& params) {

  // Collect all different instances of the current label.
  std::unordered_set<unsigned char> instance_set;
  collectInstancesFromImage(instance_layer, &instance_set);

  // Remove the instance '0' as it is not a real instance, but rather it
  // comes from the masking operation when creating instance_layer.
  const unsigned long removed_zero =
      instance_set.erase(static_cast<unsigned char>(0));
  CHECK(removed_zero > 0) << "instance_layer image should contain zero "
      "elements resulting from the masking with the current_class_layer "
      "image.";

  LOG(INFO) << "Class " << current_semantic_class << " has "
            << instance_set.size() << " instances.";

  // Iterate over all instance found in the current semantic class.
  for (const unsigned char instance_value : instance_set) {
    // Extract only the instance with instance value equal to
    // instance_value.
    cv::Mat current_instance;
    cv::inRange(instance_layer, cv::Scalar(instance_value),
                cv::Scalar(instance_value), current_instance);

    // Compute a smoother version of the image by performing dilation
    // followed by erosion.
    if (params.dilate_and_erode_)
      BlobExtractor::dilateAndErode(&current_instance, params);

    BlobExtractor::extractBlobsAndAddToContainer(current_instance,
                                                 class_blobs,
                                                 static_cast<int>(instance_value),
                                                 current_semantic_class,
                                                 params);
  }
}

void BlobExtractor::extractBlobsAndAddToContainer(cv::Mat& image,
                                                  ClassBlobs* class_blobs,
                                                  const int instance_value,
                                                  const int current_semantic_class,
                                                  const BlobExtractorParams& params) {
  // Extract the blobs associated to the current instance.
  // Usually there is only one blob per instance, but due to occlusions in
  // the scene, a single instance could also be composed by two blobs.
  const int num_threads = params.num_threads_;
  const int minimum_blob_size = params.minimumBlobSize(image.size());
  CBlobResult res(image, cv::Mat(), num_threads);
  for (int b = 0; b < res.GetNumBlobs(); ++b) {
    if (res.GetBlob(b)->Area(AreaMode::PIXELWISE) >= minimum_blob_size) {
      class_blobs->push_back(
          Blob(current_semantic_class, instance_value, *(res.GetBlob(b))));

      LOG(INFO) << "Added blob with size " << class_blobs->back().num_pixels_
                << " and instance id: " << class_blobs->back().instance_ << ".";

    }
  }
}

void BlobExtractor::dilateAndErode(cv::Mat* image,
                                   const BlobExtractorParams& params) {

  CHECK_NOTNULL(image);
  
  cv::dilate(*image, *image, cv::Mat(), cv::Point(-1, -1),
             params.num_dilate_reps_);
  cv::erode(*image, *image, cv::Mat(), cv::Point(-1, -1),
            params.num_erode_reps_);
}

void BlobExtractor::collectInstancesFromImage(const cv::Mat& image,
                                              std::unordered_set<unsigned char>* instance_set) {
  CHECK_NOTNULL(instance_set);

  instance_set->clear();
  for (int i = 0; i < image.rows; ++i) {
    const unsigned char* row_ptr = image.ptr<uchar>(i);
    for (int j = 0; j < image.cols; j++)
      instance_set->insert(row_ptr[j]);
  }

}

}


