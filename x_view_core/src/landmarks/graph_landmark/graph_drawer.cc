#include <x_view_core/landmarks/graph_landmark/graph_drawer.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>

namespace x_view {

void GraphDrawer::printBlobs(const ImageBlobs& blobs) {
  for (int c = 0; c < blobs.size(); ++c) {
    LOG(INFO) << "Found " << blobs[c].size()
              << " instances of class " << c << ":";
    for (int i = 0; i < blobs[c].size(); ++i) {
      LOG(INFO) << "\tInstance " << i << " composed by "
                << blobs[c][i].num_pixels << " pixels with mean "
                    "pixel " << blobs[c][i].center;
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
        const int semantic_label = blob.semantic_label;
        const uchar intensity =
            static_cast<uchar>(255. / (semantic_label / 8 + 1));

        cv::Scalar color(0, 0, 0);
        std::bitset<3> bits(semantic_label);

        color[0] = bits[0] ? intensity : static_cast<uchar>(0);
        color[1] = bits[1] ? intensity : static_cast<uchar>(0);
        color[2] = bits[2] ? intensity : static_cast<uchar>(0);

        std::vector<std::vector<cv::Point>> v_contours;
        v_contours.push_back(blob.external_contour_pixels);

        cv::drawContours(image, v_contours, 0, color, CV_FILLED);

      }
  }
  return image;
}

void GraphDrawer::addLabelsToImage(const ImageBlobs& blobs, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  const cv::Scalar font_color = cv::Scalar(255, 255, 255);
  for (int c = 0; c < blobs.size(); ++c) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), c) !=
        std::end(labels_to_render))
      for (const Blob& blob : blobs[c]) {
        const std::string label = std::to_string(blob.semantic_label);
        const std::string label_descr =
            global_dataset_ptr->label(blob.semantic_label);

        const std::string text = label + ") " + label_descr;
        cv::putText(*image, text, blob.center, cv::FONT_HERSHEY_DUPLEX,
                    0.65, font_color, 1, CV_AA);
        // if the blob has an instance id associated to it, render it on a
        // new line.
        // new line.
        if (blob.instance != -1) {
          const std::string instance =
              "id: " + std::to_string(blob.instance);
          cv::putText(*image, instance, blob.center + cv::Point(0, 20),
                      cv::FONT_HERSHEY_DUPLEX, 0.65, font_color, 1, CV_AA);
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
        if (global_dataset_ptr->semanticEntities()[c].is_static)
          ellipse_color = cv::Scalar(255, 40, 0); // bluish
        else
          ellipse_color = cv::Scalar(10, 60, 255); // reddish

        int ellipse_thickness = 2;
        cv::ellipse(*image, blob.ellipse, ellipse_color, ellipse_thickness);
      }
  }
}

void GraphDrawer::addGraphNodesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  const int node_radius = 4;
  const cv::Scalar node_color(216, 28, 141);

  auto node_iter = boost::vertices(graph);
  for (; node_iter.first != node_iter.second; ++node_iter.first) {
    const VertexDescriptor& node_descriptor = *node_iter.first;
    const VertexProperty& node = graph[node_descriptor];

    const cv::Point& center = node.center;
    const int label = node.semantic_label;

    if (std::find(labels_to_render.begin(), labels_to_render.end(), label) !=
        std::end(labels_to_render)) {
      cv::circle(*image, center, node_radius, node_color, CV_FILLED);
    }
  }
}

void GraphDrawer::addGraphEdgesToImage(const Graph& graph, cv::Mat* image) {

  CHECK_NOTNULL(image);

  const std::vector<int>& labels_to_render =
      global_dataset_ptr->getLabelsToRender();

  const int edge_thickness = 2;
  const cv::Scalar edge_color(58, 189, 255);

  // second, add the edges between the nodes of the graph
  auto edge_iter = boost::edges(graph);
  for (; edge_iter.first != edge_iter.second; ++edge_iter.first) {
    // get the two vertices connected by this edge
    const VertexDescriptor& vd_from = boost::source(*edge_iter.first, graph);
    const VertexDescriptor& vd_to = boost::target(*edge_iter.first, graph);

    const VertexProperty& v_from = graph[vd_from];
    const VertexProperty& v_to = graph[vd_to];

    const int from_label = v_from.semantic_label;
    const int to_label = v_to.semantic_label;

    // only draw the edge if both nodes are to render
    if (std::find(labels_to_render.begin(),
                  labels_to_render.end(),
                  from_label)
        != std::end(labels_to_render) &&
        std::find(labels_to_render.begin(), labels_to_render.end(), to_label)
            != std::end(labels_to_render)) {
      const cv::Point& from_center = v_from.center;
      const cv::Point& to_center = v_to.center;

      cv::line(*image, from_center, to_center, edge_color, edge_thickness);
    }
  }

}

}