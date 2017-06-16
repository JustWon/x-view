#include <x_view_core/landmarks/graph_landmark/graph_drawer.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>

namespace x_view {

cv::Scalar GraphDrawer::vertex_color_ = CV_RGB(141, 28,216);
int GraphDrawer::vertex_radius_ = 4;

cv::Scalar GraphDrawer::edge_color_ = CV_RGB(255, 189, 58);
int GraphDrawer::edge_thickness_ = 2;

cv::Scalar GraphDrawer::ellipse_color_static_ = CV_RGB(0, 40, 255);
cv::Scalar GraphDrawer::ellipse_color_dynamic_ = CV_RGB(255, 60, 10);
int GraphDrawer::ellipse_thickness_ = 2;

cv::Scalar GraphDrawer::label_color_ = CV_RGB(255, 255, 255);
float GraphDrawer::label_scale_ = 0.65f;


void GraphDrawer::resetProperties() {
  GraphDrawer::vertex_color_ = CV_RGB(141, 28,216);
  GraphDrawer::vertex_radius_ = 4;

  GraphDrawer::edge_color_ = CV_RGB(255, 189, 58);
  GraphDrawer::edge_thickness_ = 2;

 GraphDrawer::ellipse_color_static_ = CV_RGB(0, 40, 255);
  GraphDrawer::ellipse_color_dynamic_ = CV_RGB(255, 60, 10);
  GraphDrawer::ellipse_thickness_ = 2;

  GraphDrawer::label_color_ = CV_RGB(255, 255, 255);
  GraphDrawer::label_scale_ = 0.65f;
}

void GraphDrawer::printBlobs(const ImageBlobs& blobs) {
  for (int c = 0; c < blobs.size(); ++c) {
    LOG(INFO) << "Found " << blobs[c].size()
              << " instances of class " << c << ":";
    for (int i = 0; i < blobs[c].size(); ++i) {
      LOG(INFO) << "\tInstance " << i << " composed by "
                << blobs[c][i].num_pixels_ << " pixels with mean "
                    "pixel " << blobs[c][i].center_;
    }
  }
}

cv::Mat GraphDrawer::createImageWithLabels(const ImageBlobs& blobs,
                                           const Graph& graph,
                                          const cv::Size& size) {
  cv::Mat image = GraphDrawer::createImageFromBlobs(blobs, size);
  GraphDrawer::addGraphEdgesToImage(graph, &image);
  GraphDrawer::addGraphNodesToImage(graph, &image);
  GraphDrawer::addEllipsesToImage(blobs, &image);
  GraphDrawer::addLabelsToImage(blobs, &image);

  return image;
}

cv::Mat GraphDrawer::createImageFromBlobs(const ImageBlobs& blobs,
                                          const cv::Size& size) {
  cv::Mat image(size.height, size.width, CV_8UC3, cv::Scalar::all(0));

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

void GraphDrawer::addLabelsToImage(const ImageBlobs& blobs, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        const std::string label = std::to_string(blob.semantic_label_);
        const std::string label_descr =
            global_dataset_ptr->label(blob.semantic_label_);

        const std::string text = label + ") " + label_descr;
        cv::putText(*image, text, blob.center_, cv::FONT_HERSHEY_DUPLEX,
                    GraphDrawer::label_scale_, GraphDrawer::label_color_,
                    1, CV_AA);
        // if the blob has an instance id associated to it, render it on a
        // new line.
        // new line.
        if (blob.instance_ != -1) {
          const std::string instance =
              "id: " + std::to_string(blob.instance_);
          cv::putText(*image, instance, blob.center_ + cv::Point(0, 20),
                      cv::FONT_HERSHEY_DUPLEX, GraphDrawer::label_scale_,
                      GraphDrawer::label_color_, 1, CV_AA);
        }
      }
  }
}

void GraphDrawer::addEllipsesToImage(const ImageBlobs& blobs, cv::Mat* image) {

  CHECK_NOTNULL(image);

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
          ellipse_color = GraphDrawer::ellipse_color_static_;
        else
          ellipse_color = GraphDrawer::ellipse_color_dynamic_;

        cv::ellipse(*image, blob.ellipse_, ellipse_color,
                    GraphDrawer::ellipse_thickness_);
      }
  }
}

void GraphDrawer::addGraphNodesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  auto node_iter = boost::vertices(graph);
  for (; node_iter.first != node_iter.second; ++node_iter.first) {
    const VertexDescriptor& node_descriptor = *node_iter.first;
    const VertexProperty& node = graph[node_descriptor];

    const cv::Point& center = node.center_;
    const int label = node.semantic_label_;

    if (std::find(labels_to_render.begin(), labels_to_render.end(), label) !=
        std::end(labels_to_render)) {
      cv::circle(*image, center, GraphDrawer::vertex_radius_,
                 GraphDrawer::vertex_color_, CV_FILLED);
    }
  }
}

void GraphDrawer::addGraphEdgesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  // second, add the edges between the nodes of the graph
  auto edge_iter = boost::edges(graph);
  for (; edge_iter.first != edge_iter.second; ++edge_iter.first) {
    // get the two vertices connected by this edge
    const VertexDescriptor& vd_from = boost::source(*edge_iter.first, graph);
    const VertexDescriptor& vd_to = boost::target(*edge_iter.first, graph);

    const VertexProperty& v_from = graph[vd_from];
    const VertexProperty& v_to = graph[vd_to];

    const int from_label = v_from.semantic_label_;
    const int to_label = v_to.semantic_label_;

    // only draw the edge if both nodes are to render
    if (std::find(labels_to_render.begin(),
                  labels_to_render.end(),
                  from_label)
        != std::end(labels_to_render) &&
        std::find(labels_to_render.begin(), labels_to_render.end(), to_label)
            != std::end(labels_to_render)) {
      const cv::Point& from_center = v_from.center_;
      const cv::Point& to_center = v_to.center_;

      cv::line(*image, from_center, to_center, GraphDrawer::edge_color_,
               GraphDrawer::edge_thickness_);
    }
  }

}

}