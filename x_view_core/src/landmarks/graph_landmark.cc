#include <bitset>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/features/graph_descriptor.h>

#include <x_view_core/x_view_tools.h>

namespace x_view {

// FIXME: should this parameter be read by the config file?
int GraphLandmark::MINIMUM_BLOB_SIZE = 40;

GraphLandmark::GraphLandmark(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
  // Graph descriptor filled up by this function
  Graph descriptor;
  Graph::GraphType& graph = descriptor.graph();

  // perform image segmentation on the first channel of the image
  findBlobsWithContour();

  createCompleteGraph(graph);
  // generate a complete graph out of the computed blobs
  //createCompleteGraph(graph);

#ifdef X_VIEW_DEBUG
  cv::imshow("Semantic entities", createImageWithGraphOntop(graph));
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

const cv::Mat GraphLandmark::getImageFromBlobs(
    const std::vector<int>& labels_to_render) const {
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
        v_contours.push_back(blob.contour_pixels_);

        cv::drawContours(resImage,
                         std::vector<std::vector<cv::Point>>{
                             blob.contour_pixels_},
                         0, color, CV_FILLED);

      }

  }


  // Draw the labels and the blobs centers
  const int circle_radius = 5;
  const int line_thickness = -1;
  const cv::Scalar center_color = cv::Scalar(255, 130, 100);
  const cv::Scalar font_color = cv::Scalar(255, 255, 255);
  for (int l = 0; l < image_blobs_.size(); ++l) {
    // only add the blob if the label has not to be ignored, thus if it can
    // not be found in the passed parameter
    if (std::find(labels_to_render.begin(), labels_to_render.end(), l) !=
        std::end(labels_to_render))
      for (const Blob& blob : image_blobs_[l]) {
        cv::circle(resImage, blob.center_, circle_radius,
                   center_color, line_thickness);

        std::string label = std::to_string(blob.semantic_label_) + ") " +
            global_dataset_ptr->label(blob.semantic_label_);
        cv::putText(resImage, label, blob.center_, cv::FONT_HERSHEY_DUPLEX,
                    0.85, font_color, 1, CV_AA);
      }
  }

  return resImage;
}

const cv::Mat GraphLandmark::createImageWithGraphOntop(
    const Graph::GraphType& graph,
    const std::vector<int>& labels_to_render) const {

  // first get the base image composed by the blobs
  cv::Mat blobImage = getImageFromBlobs(labels_to_render);

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
      cv::line(blobImage, from_center, to_center, line_color);
    }
  }

  return blobImage;

}

#endif // X_VIEW_DEBUG

void GraphLandmark::findBlobs() {

  const int dataset_size = global_dataset_ptr->numSemanticClasses();
  image_blobs_.resize(dataset_size);

  const cv::Mat label_image = extractChannelFromImage(semantic_image_, 0);

  auto PointComp = [](const cv::Point& p1, const cv::Point& p2) -> bool {
    return std::tie(p1.x, p1.y) < std::tie(p2.x, p2.y);
  };

  // this container contains all pixels that have already been inserted into a
  // blob, it is used to avoid counting the same pixel multiple times
  std::set<cv::Point, decltype(PointComp)> pixels_in_blob(PointComp);

  // variable used for the flood algorithm
  cv::Mat flood_mask =
      cv::Mat::zeros(label_image.rows + 2, label_image.cols + 2, CV_8U);

  // helper function to get the label associated to a pixel
  auto getPixelLabel = [&](const cv::Point& pixel) -> int {
    return static_cast<int>(label_image.at<uchar>(pixel));
  };

  for (int y = 0; y < label_image.rows; y++) {
    for (int x = 0; x < label_image.cols; x++) {
      if (pixels_in_blob.find(cv::Point(x, y)) == pixels_in_blob.end()) {
        const cv::Point seed_pixel(x, y);

        const int seed_pixel_label = getPixelLabel(seed_pixel);

        cv::Rect blob_bounding_box;
        cv::floodFill(label_image, flood_mask, seed_pixel, 255,
                      &blob_bounding_box, 0, 0,
                      4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

        // build a blob around the seed pixel
        std::vector<cv::Point> blob_around_seed_pixel;

        const int y_blob_min = blob_bounding_box.y;
        const int y_blob_max = y_blob_min + blob_bounding_box.height;
        const int x_blob_min = blob_bounding_box.x;
        const int x_blob_max = x_blob_min + blob_bounding_box.width;

        for (int i = y_blob_min; i < y_blob_max; ++i) {
          for (int j = x_blob_min; j < x_blob_max; ++j) {
            const cv::Point query_pixel(j, i);
            if (getPixelLabel(query_pixel) == seed_pixel_label) {
              bool has_query_pixel_been_inserted_now =
                  pixels_in_blob.insert(query_pixel).second;
              if (has_query_pixel_been_inserted_now)
                blob_around_seed_pixel.push_back(query_pixel);
            } else {
            }
          }
        }
        // we are only interested in large blobs!
        if (blob_around_seed_pixel.size() > MINIMUM_BLOB_SIZE)
          image_blobs_[seed_pixel_label].push_back(
              Blob(seed_pixel_label, blob_around_seed_pixel));
      }
    }
  }
}

void GraphLandmark::findBlobsWithContour() {

  const cv::Mat all_labels_image = extractChannelFromImage(semantic_image_, 0);
  image_blobs_.resize(global_dataset_ptr->numSemanticClasses());

  // extract only the pixels associated with each class, and compute their
  // corresponding contours
  for (int c = 0; c < global_dataset_ptr->numSemanticClasses(); ++c) {
    cv::Mat current_class_layer;
    cv::inRange(all_labels_image, cv::Scalar(c), cv::Scalar(c),
                current_class_layer);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::imshow("Binary for class " + global_dataset_ptr->label(c),
               current_class_layer);
    cv::waitKey();

    cv::findContours(current_class_layer, contours, hierarchy,
                     CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); ++i) {
      if (hierarchy[i][3] == -1) { // not hole boundary
        const int contour_area = cv::contourArea(contours[i]);
        if (contour_area >= MINIMUM_BLOB_SIZE && contour_area > 5) {
          Blob blob(c, contours[i]);
          image_blobs_[c].push_back(blob);

          cv::drawContours(current_class_layer, contours, i, cv::Scalar::all
              (125));
          cv::imshow("Contours for " + global_dataset_ptr->label(c),
                     current_class_layer);
          cv::waitKey();
        }
      }
    }
  }
}

void GraphLandmark::createGraph(Graph::GraphType& graph) const {

}

void GraphLandmark::createCompleteGraph(Graph::GraphType& graph) const {

  std::vector<Graph::VertexProperty> vertices;
  std::vector<Graph::VertexDescriptor> vertex_descriptors;

  // push the blobs into the graph
  for (int c = 0; c < image_blobs_.size(); ++c) {
    for (const Blob& blob : image_blobs_[c]) {
      const int semantic_label = blob.semantic_label_;
      const std::string label = global_dataset_ptr->label(semantic_label);
      const int size = blob.size_;
      const cv::Point center = blob.center_;
      vertices.push_back({semantic_label, label, size, center});
      vertex_descriptors.push_back(boost::add_vertex(vertices.back(), graph));
    }
  }

  // create the edges between the nodes
  for (int i = 0; i < vertices.size(); ++i) {
    for (int j = i + 1; j < vertices.size(); ++j) {
      boost::add_edge(vertex_descriptors[i], vertex_descriptors[j],
                      {i, j}, graph);
    }
  }
}

}

