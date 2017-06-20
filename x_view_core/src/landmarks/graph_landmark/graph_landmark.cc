#include <x_view_core/landmarks/graph_landmark/graph_landmark.h>

#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>
#include <x_view_core/landmarks/graph_landmark/graph_builder.h>

namespace x_view {

// FIXME: should this parameter be read by the config file?
int GraphLandmark::MINIMUM_BLOB_SIZE = 500;

bool GraphLandmark::DILATE_AND_ERODE = true;

GraphLandmark::GraphLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {

  // Graph descriptor filled up by this function
  Graph descriptor;

  // *********** Blobs extraction ********** //
  BlobExtractorParams blob_extractor_params;
  blob_extractor_params.dilate_and_erode = GraphLandmark::DILATE_AND_ERODE;
  blob_extractor_params.num_erode_reps = 5;
  blob_extractor_params.num_dilate_reps = 5;
  blob_extractor_params.blob_size_filtering.type =
      BlobExtractorParams::MIN_BLOB_SIZE_TYPE::ABSOLUTE;
  blob_extractor_params.blob_size_filtering.num_min_pixels =
      GraphLandmark::MINIMUM_BLOB_SIZE;
  image_blobs_ =
      BlobExtractor::findBlobsWithContour(image, blob_extractor_params);

  // *********** Graph generation ********** //
  GraphBuilderParams graph_builder_params;
  graph_builder_params.max_distance_for_neighborhood = 10;
  descriptor = GraphBuilder::createGraphFromNeighborBlobs(image_blobs_,
                                                          graph_builder_params);
  // create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the histogram data
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

}

