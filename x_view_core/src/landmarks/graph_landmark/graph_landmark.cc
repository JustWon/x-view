#include <x_view_core/landmarks/graph_landmark/graph_landmark.h>

#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>
#include <x_view_core/landmarks/graph_landmark/graph_builder.h>
#include <x_view_core/landmarks/graph_landmark/graph_drawer.h>

#include <opencv2/highgui/highgui.hpp>

namespace x_view {

// FIXME: should this parameter be read by the config file?
int GraphLandmark::MINIMUM_BLOB_SIZE = 500;

bool GraphLandmark::DILATE_AND_ERODE = false;

GraphLandmark::GraphLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {

  // Graph descriptor filled up by this function
  Graph descriptor;
  Graph::GraphType& graph = descriptor.graph();

  // *********** Blobs extraction ********** //
  BlobExtractorParams blob_extractor_params;
  blob_extractor_params.dilate_and_erode_ = GraphLandmark::DILATE_AND_ERODE;
  blob_extractor_params.blob_size_filtering_.type_ =
      BlobExtractorParams::MIN_BLOB_SIZE_TYPE::ABSOLUTE;
  blob_extractor_params.blob_size_filtering_.n_min_pixels_ =
      GraphLandmark::MINIMUM_BLOB_SIZE;
  image_blobs_ =
      BlobExtractor::findBlobsWithContour(image.clone(), blob_extractor_params);

  // *********** Graph generation ********** //
  GraphBuilderParams graph_builder_params;
  graph_builder_params.max_distance_for_neighborhood_ = 4;
  graph = GraphBuilder::createGraphFromNeighborBlobs(image_blobs_,
                                                     graph_builder_params);

#ifdef X_VIEW_DEBUG
  // ************ Graph drawing *********** //
  cv::Mat draw_image =
      GraphDrawer::createImageFromBlobs(image_blobs_, image.size());
  GraphDrawer::addGraphEdgesToImage(graph, &draw_image);
  GraphDrawer::addGraphNodesToImage(graph, &draw_image);
  GraphDrawer::addEllipsesToImage(image_blobs_, &draw_image);
  GraphDrawer::addLabelsToImage(image_blobs_, &draw_image);
  cv::imshow("Semantic entities and graph structure", draw_image);
  cv::waitKey();
#endif // X_VIEW_DEBUG

  // create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the histogram data
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

}

