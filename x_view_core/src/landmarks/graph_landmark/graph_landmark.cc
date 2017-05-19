#include <x_view_core/landmarks/graph_landmark/graph_landmark.h>

#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>
#include <x_view_core/landmarks/graph_landmark/graph_builder.h>

#include <opencvblobslib/BlobResult.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <bitset>

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
  blob_extractor_params.blob_size_filtering_.value_ =
      GraphLandmark::MINIMUM_BLOB_SIZE;
  image_blobs_ =
      BlobExtractor::findBlobsWithContour(image, blob_extractor_params);

  // *********** Graph generation ********** //
  GraphBuilderParams graph_builder_params;
  graph_builder_params.max_distance_for_neighborhood_ = 4;
  graph = GraphBuilder::createGraphFromNeighborBlobs(image_blobs_,
                                                     graph_builder_params);

#ifdef X_VIEW_DEBUG
  cv::imshow("Semantic entities and graph structure",
             createImageWithGraphOntop(graph));
  cv::waitKey();
#endif // X_VIEW_DEBUG

  // create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the histogram data
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

#ifdef X_VIEW_DEBUG
void GraphLandmark::printBlobs(std::ostream& out) const {
  for (int c = 0; c < image_blobs_.size(); ++c) {
    out << "Found " << image_blobs_[c].size()
        << " instances of class " << c << ":" << std::endl;
    for (int i = 0; i < image_blobs_[c].size(); ++i) {
      out << "\tInstance " << i << " composed by "
          << image_blobs_[c][i].size_ << " pixels with mean "
              "pixel "
          << image_blobs_[c][i].center_ << std::endl;
    }
    out << std::endl;
  }
}

cv::Mat GraphLandmark::getImageFromBlobs(const std::vector<int>& labels_to_render) {
  cv::Mat resImage(semantic_image_.rows, semantic_image_.cols,
                   CV_8UC3, cv::Scalar::all(0));

  // Draw the blobs onto the image
  for (int c = 0; c < image_blobs_.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : image_blobs_[c]) {
        const int semantic_label = c;
        const uchar intensity =
            static_cast<uchar>(255. / (semantic_label / 8 + 1));

        cv::Scalar color(0, 0, 0);
        std::bitset<3> bits(semantic_label);

        color[0] = bits[0] ? intensity : (uchar) 0;
        color[1] = bits[1] ? intensity : (uchar) 0;
        color[2] = bits[2] ? intensity : (uchar) 0;

        std::vector<std::vector<cv::Point>> v_contours;
        v_contours.push_back(blob.external_contour_pixels_);

        cv::drawContours(resImage, v_contours, 0, color, CV_FILLED);

      }

  }


  // Draw the labels
  const cv::Scalar center_color = cv::Scalar(255, 130, 100);
  const cv::Scalar font_color = cv::Scalar(255, 255, 255);
  for (int l = 0; l < image_blobs_.size(); ++l) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), l) !=
        std::end(labels_to_render))
      for (const Blob& blob : image_blobs_[l]) {

        std::string label = std::to_string(blob.semantic_label_) + ") " +
            global_dataset_ptr->label(blob.semantic_label_);
        cv::putText(resImage, label, blob.center_, cv::FONT_HERSHEY_DUPLEX,
                    0.85, font_color, 1, CV_AA);
      }
  }

  return resImage;
}

void GraphLandmark::addEllipsesOnSemanticImage(cv::Mat& image, const
std::vector<int>& labels_to_render) {
  // Draw the blobs onto the image
  for (int c = 0; c < image_blobs_.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (Blob& blob : image_blobs_[c]) {
        cv::Scalar ellipse_color;
        if (global_dataset_ptr->semanticEntities()[c].is_static_)
          ellipse_color = cv::Scalar(255, 40, 0);
        else
          ellipse_color = cv::Scalar(10, 60, 255);

        cv::Scalar center_color(10, 220, 220);
        int ellipse_thickness = 2;
        int center_radius = 2;
        cv::ellipse(image, blob.ellipse_, ellipse_color, ellipse_thickness);
        cv::circle(image, blob.center_, center_radius, center_color, CV_FILLED);
      }

  }
}

const cv::Mat GraphLandmark::createImageWithGraphOntop(
    const Graph::GraphType& graph,
    const std::vector<int>& labels_to_render) {

  // first get the base image composed by the blobs
  cv::Mat blobImage = getImageFromBlobs(labels_to_render);

  // add ellipses on corresponding blobs.
  addEllipsesOnSemanticImage(blobImage, labels_to_render);

  // second, add the edges between the nodes of the graph
  auto edge_iter = boost::edges(graph);
  for (edge_iter; edge_iter.first != edge_iter.second; ++edge_iter.first) {
    // get the two vertices connected by this edge
    Graph::VertexDescriptor vd_from = boost::source(*edge_iter.first, graph);
    Graph::VertexDescriptor vd_to = boost::target(*edge_iter.first, graph);

    const Graph::VertexProperty v_from = graph[vd_from];
    const Graph::VertexProperty v_to = graph[vd_to];

    const int from_label = v_from.semantic_label_;
    const int to_label = v_to.semantic_label_;

    // if one of the two nodes does not belong to the labels to be rendered,
    // then don't draw the edge connecting them
    if (std::find(labels_to_render.begin(), labels_to_render.end(), from_label)
        == std::end(labels_to_render) ||
        std::find(labels_to_render.begin(), labels_to_render.end(), to_label)
            == std::end(labels_to_render))
      continue;
    else {
      const cv::Point& from_center = v_from.center_;
      const cv::Point& to_center = v_to.center_;

      const cv::Scalar line_color(58, 189, 255);
      int thickness = 2;
      cv::line(blobImage, from_center, to_center, line_color, thickness);
    }
  }

  return blobImage;

}

#endif // X_VIEW_DEBUG

}

