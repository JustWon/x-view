#include <x_view_core/landmarks/graph_landmark/graph_landmark.h>

#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>
#include <x_view_core/landmarks/graph_landmark/graph_builder.h>
#include <x_view_core/x_view_locator.h>

namespace x_view {

GraphLandmark::GraphLandmark(const FrameData& frame_data)
    : AbstractSemanticLandmark(frame_data) {

  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");

  // Graph descriptor filled up by this function
  Graph descriptor;

  // *********** Blobs extraction ********** //

  const bool dilate_and_erode =
      landmark_parameters->getBoolean("dilate_and_erode", true);
  const int num_dilate =
      landmark_parameters->getInteger("num_dilate", 4);
  const int num_erode =
      landmark_parameters->getInteger("num_erode", 4);

  const std::string blob_filter_type =
      landmark_parameters->getString("blob_filter_type", "ABSOLUTE");

  BlobExtractorParams blob_extractor_params;
  blob_extractor_params.dilate_and_erode = dilate_and_erode;
  blob_extractor_params.num_dilate_reps = num_dilate;
  blob_extractor_params.num_erode_reps = num_erode;

  if(blob_filter_type == "ABSOLUTE") {
    blob_extractor_params.blob_size_filtering.type =
        BlobExtractorParams::MIN_BLOB_SIZE_TYPE::ABSOLUTE;
    const int min_blob_size =
        landmark_parameters->getInteger("min_blob_size", 300);
    blob_extractor_params.blob_size_filtering.num_min_pixels = min_blob_size;
  } else if(blob_filter_type == "RELATIVE") {
    blob_extractor_params.blob_size_filtering.type =
    BlobExtractorParams::MIN_BLOB_SIZE_TYPE::RELATIVE;
    const float fraction_min_blob =
        landmark_parameters->getFloat("min_blob_size", 0.05f);
    blob_extractor_params.blob_size_filtering.fraction_min_pixels =
        fraction_min_blob;
  } else {
    LOG(ERROR) << "Graph landmark could not recognize <" << blob_filter_type
               << "> as 'blob_filter_type' parameter.";
  }

  image_blobs_ = BlobExtractor::findBlobsWithContour(semantic_image_,
                                                     blob_extractor_params);

  // *********** Graph generation ********** //

  const int blob_neighbor_distance =
      landmark_parameters->getInteger("blob_neighbor_distance", 10);

  GraphBuilderParams graph_builder_params;
  graph_builder_params.max_distance_for_neighborhood = blob_neighbor_distance;

  descriptor = GraphBuilder::createGraphFromNeighborBlobs(frame_data,
                                                          image_blobs_,
                                                          graph_builder_params);
  // Create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the graph data.
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

}

