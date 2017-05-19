#include <x_view_core/landmarks/graph_landmark/graph_drawer.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>
#include <x_view_core/datasets/abstract_dataset.h>

#include <iostream>
#include <bitset>

namespace x_view {

void GraphDrawer::printBlobs(const ImageBlobs& blobs, std::ostream& out) {
  for (int c = 0; c < blobs.size(); ++c) {
    out << "Found " << blobs[c].size()
        << " instances of class " << c << ":" << std::endl;
    for (int i = 0; i < blobs[c].size(); ++i) {
      out << "\tInstance " << i << " composed by "
          << blobs[c][i].size_ << " pixels with mean "
              "pixel "
          << blobs[c][i].center_ << std::endl;
    }
    out << std::endl;
  }
}

cv::Mat GraphDrawer::createImageFromBlobs(const ImageBlobs& blobs,
                                          const int rows, const int cols) {
  cv::Mat image(rows, cols, CV_8UC3, cv::Scalar::all(0));

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  // Draw the blobs onto the image
  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        const int semantic_label = blob.semantic_label_;
        const uchar intensity =
            static_cast<uchar>(255. / (semantic_label / 8 + 1));

        cv::Scalar color(0, 0, 0);
        std::bitset<3> bits(semantic_label);

        color[0] = bits[0] ? intensity : static_cast<uchar>(0);
        color[1] = bits[1] ? intensity : static_cast<uchar>(0);
        color[2] = bits[2] ? intensity : static_cast<uchar>(0);

        std::vector<std::vector<cv::Point>> v_contours;
        v_contours.push_back(blob.external_contour_pixels_);

        cv::drawContours(image, v_contours, 0, color, CV_FILLED);

      }

  }

  return image;

}

void GraphDrawer::addLabelsToImage(const ImageBlobs& blobs, cv::Mat& image) {

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  const cv::Scalar font_color = cv::Scalar(255, 255, 255);
  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {

        std::string label = std::to_string(blob.semantic_label_) + ") " +
            global_dataset_ptr->label(blob.semantic_label_);
        cv::putText(image, label, blob.center_, cv::FONT_HERSHEY_DUPLEX,
                    0.85, font_color, 1, CV_AA);
      }
  }
}

void GraphDrawer::addEllipsesToImage(const ImageBlobs& blobs, cv::Mat& image) {

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  // Draw the blobs onto the image
  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        cv::Scalar ellipse_color;
        if (global_dataset_ptr->semanticEntities()[c].is_static_)
          ellipse_color = cv::Scalar(255, 40, 0); // bluish
        else
          ellipse_color = cv::Scalar(10, 60, 255); // reddish

        int ellipse_thickness = 2;
        int center_radius = 2;
        cv::ellipse(image, blob.ellipse_, ellipse_color, ellipse_thickness);
      }
  }
}

void GraphDrawer::addGraphNodesToImage(const Graph::GraphType& graph,
                                       cv::Mat& image) {

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  const int node_radius = 4;
  const cv::Scalar node_color(216, 28, 141);

  auto node_iter = boost::vertices(graph);
  for (node_iter; node_iter.first != node_iter.second; ++node_iter.first) {
    const Graph::VertexDescriptor& node_descriptor = *node_iter.first;
    const Graph::VertexProperty& node = graph[node_descriptor];

    const cv::Point& center = node.center_;
    const int label = node.semantic_label_;

    if (std::find(labels_to_render.begin(), labels_to_render.end(), label) !=
        std::end(labels_to_render)) {
      cv::circle(image, center, node_radius, node_color, CV_FILLED);
    }
  }
}

void GraphDrawer::addGraphEdgesToImage(const Graph::GraphType& graph,
                                       cv::Mat& image) {

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  const int edge_thickness = 2;
  const cv::Scalar edge_color(58, 189, 255);

  // second, add the edges between the nodes of the graph
  auto edge_iter = boost::edges(graph);
  for (edge_iter; edge_iter.first != edge_iter.second; ++edge_iter.first) {
    // get the two vertices connected by this edge
    const Graph::VertexDescriptor& vd_from =
        boost::source(*edge_iter.first, graph);
    const Graph::VertexDescriptor& vd_to =
        boost::target(*edge_iter.first, graph);

    const Graph::VertexProperty& v_from = graph[vd_from];
    const Graph::VertexProperty& v_to = graph[vd_to];

    const int from_label = v_from.semantic_label_;
    const int to_label = v_to.semantic_label_;

    // only draw the edge if both nodes are to render
    if (std::find(labels_to_render.begin(), labels_to_render.end(), from_label)
        != std::end(labels_to_render) &&
        std::find(labels_to_render.begin(), labels_to_render.end(), to_label)
            != std::end(labels_to_render)) {
      const cv::Point& from_center = v_from.center_;
      const cv::Point& to_center = v_to.center_;

      cv::line(image, from_center, to_center, edge_color, edge_thickness);
    }
  }

}

}