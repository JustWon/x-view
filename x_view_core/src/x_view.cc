#include <x_view_core/x_view.h>

#include <x_view_core/datasets/synthia_dataset.h>
#include <x_view_core/datasets/airsim_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/landmarks/histogram_landmark.h>
#include <x_view_core/landmarks/visual_descriptor_landmark.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/matchers/vector_matcher.h>

namespace x_view {

// Declaration of global dataset pointer.
ConstDatasetPtr global_dataset_ptr;

XView::XView(XViewParams& params) : params_(params), frame_number_(0) {
  // Parse the passed parameters and instantiate concrete classes for all
  // class members.
  parseParameters();

  // Print XView parameters and info.
  printInfo();
}

XView::~XView() {};

void XView::processSemanticImage(const cv::Mat& image, const SE3& pose) {

  LOG(INFO) << "XView starts processing frame " << frame_number_ << ".";

  // Generate a new semantic landmark pointer.
  SemanticLandmarkPtr landmark_ptr;

  // Extract semantics associated to the semantic image and pose.
  extractSemanticsFromImage(image, pose, landmark_ptr);

  // Compute the matches between the new feature and the ones
  // stored in the database.
  if (frame_number_ == 0) {
    // Simply add the landmark to the matcher without matching anything.
    descriptor_matcher_->addDescriptor(landmark_ptr->getDescriptor());
  } else {
    // Perform full matching.
    AbstractMatcher::MatchingResultPtr matching_result_ptr;
    matchSemantics(landmark_ptr, matching_result_ptr);
  }

  // Add the semantic landmark to the database.
  semantics_db_.push_back(landmark_ptr);

  LOG(INFO) << "XView ended processing frame " << frame_number_ << ".";
  ++frame_number_;
}

void XView::printInfo() const {
  LOG(INFO)
      << "\n==========================================================\n"
      << "                  XView"
      #ifdef X_VIEW_DEBUG
      << " (Debug)"
      #else
      << " (Release)"
      #endif
      << "\n\n" << dataset_
      << "\n\tLandmark type:\t<" + params_.semantic_landmark_type_ + ">"
      << "\n\tMatcher type: \t<" + params_.landmark_matching_type_ + ">"
      << "\n==========================================================\n";
}

void XView::parseParameters() {
  // Define the dataset being used.
  parseDatasetType();

  // Define landmark type.
  parseLandmarkType();

  // Define a landmark matcher.
  parseMatcherType();

}

void XView::parseDatasetType() {

  if (params_.semantic_dataset_name_.compare("SYNTHIA") == 0) {
    dataset_ = std::make_shared<const SynthiaDataset>();
  } else if (params_.semantic_dataset_name_.compare("AIRSIM") == 0) {
    dataset_ = std::make_shared<const AirsimDataset>();
  } else {
    CHECK(false) << "Unrecognized dataset type <"
                 << params_.semantic_dataset_name_ << ">" << std::endl;
  }
  global_dataset_ptr = dataset_;
}

void XView::parseLandmarkType() {

  if (params_.semantic_landmark_type_.compare("ORB") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::ORB_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction
        (ORBVisualDescriptorLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("SIFT") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SIFT_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction
        (SIFTVisualDescriptorLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("SURF") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SURF_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction
        (SURFVisualDescriptorLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("HISTOGRAM") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SEMANTIC_HISTOGRAM;
    semantic_landmark_factory_.setCreatorFunction
        (HistogramLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("GRAPH") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SEMANTIC_GRAPH;
    semantic_landmark_factory_.setCreatorFunction
        (GraphLandmark::create);
  } else {
    CHECK(false) << "Unrecognized landmark type <"
                 << params_.semantic_landmark_type_ << ">" << std::endl;
  }

}

void XView::parseMatcherType() {
  if (params_.landmark_matching_type_.compare("VECTOR") == 0) {
    landmarks_matcher_type_ = LandmarksMatcherType::VECTOR_MATCHER;
    descriptor_matcher_ = VectorMatcher::create();
  } else if (params_.landmark_matching_type_.compare("GRAPH") == 0) {
    landmarks_matcher_type_ = LandmarksMatcherType::GRAPH_MATCHER;
    descriptor_matcher_ = GraphMatcher::create();
  } else {
    CHECK(false) << "Unrecognized matcher type <"
                 << params_.landmark_matching_type_ << ">" << std::endl;
  }
}

//==========================================================================//
//         FUNCTIONS CALLED BY 'processSemanticImage' FUNCTION              //
//==========================================================================//

void XView::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                      SemanticLandmarkPtr& semantics_out) {

  // TODO: preprocess image and pose

  // Create the actual landmark representation whose implementation depends
  // on the parameters passed to XView.
  semantics_out =
      semantic_landmark_factory_.createSemanticLandmark(image, pose);

  LOG(INFO) << "XView created semantic landmark.";
}

void XView::matchSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matching_result) {

  const SemanticLandmarkPtr& previous_semantics = semantics_db_.back();

  const cv::Mat& current_semantic_image = semantics_a->getSemanticImage();
  const cv::Mat& previous_semantic_image =
      previous_semantics->getSemanticImage();

  // Extract graph from current and previous landmark.
  const auto current_graph_landmark =
      std::dynamic_pointer_cast<GraphLandmark>(semantics_a);
  CHECK_NOTNULL(current_graph_landmark.get());

  const auto previous_graph_landmark =
      std::dynamic_pointer_cast<GraphLandmark>(previous_semantics);
  CHECK_NOTNULL(previous_graph_landmark.get());

  const auto current_graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>
          (current_graph_landmark->getDescriptor());

  const auto previous_graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>
          (previous_graph_landmark->getDescriptor());

  // Create a new Graph matcher which matches the current landmark with the
  // last one in the database.
  RandomWalkerParams random_walker_params;
  random_walker_params.num_walks = 20;
  random_walker_params.walk_length = 2;
  random_walker_params.random_sampling_type =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;
  VertexSimilarity::SCORE_TYPE score_type = VertexSimilarity::SCORE_TYPE::WEIGHTED;
  GraphMatcher graph_matcher(random_walker_params, score_type);

  // Add the previous graph to the matcher.
  graph_matcher.addDescriptor(semantics_db_.back()->getDescriptor());

  // Compute a match between the current semantic landmark and the ones
  // already visited.
  matching_result =
      graph_matcher.match(current_graph_descriptor->getDescriptor());

  // Compute similarity matrices.
  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);

  // Display the computed similarities.
  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  cv::imshow("Vertex similarity", similarity_image);

  const cv::Mat max_col_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_colwise);
  cv::imshow("Max col similarity", max_col_similarity_image);

  const cv::Mat max_row_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_rowwise);
  cv::imshow("Max row similarity", max_row_similarity_image);

  const cv::Mat max_agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);
  cv::imshow("Max agree similarity", max_agree_similarity_image);

  // Function to create a curved polyline between two points.
  auto createCurveBetween = [&](const cv::Point& start, const cv::Point& end,
                               const int step) -> std::vector<cv::Point> {
    const int dist = std::abs(end.x - start.x) - 1;
    const int num_steps = dist / step;
    std::vector<cv::Point> line(num_steps);
    cv::Point anchor = (start + end) * 0.5;
    anchor.y -= 120;
    float t = 0.f;
    float dt = 1.f / num_steps;
    for (auto& point : line) {
      point = start * ((1. - t) * (1. - t)) + anchor * (2. * t * (1. - t))
          + end * (t * t);
      t += dt;
    }
    line.push_back(end);
    return line;
  };

  // Display the semantic images.
  GraphDrawer::setEdgeThickness(1);
  GraphDrawer::setEllipseThickness(1);
  GraphDrawer::setLabelScale(0.f);
  GraphDrawer::setEdgeColor(cv::Scalar::all(255));

  cv::Mat current_graph_image = GraphDrawer::createImageWithLabels(
      current_graph_landmark->getBlobs(),
      current_graph_descriptor->getDescriptor(),
      current_semantic_image.size()
  );

  cv::Mat previous_graph_image = GraphDrawer::createImageWithLabels(
      previous_graph_landmark->getBlobs(),
      previous_graph_descriptor->getDescriptor(),
      previous_semantic_image.size()
  );

  cv::Mat side_by_side;
  cv::hconcat(current_graph_image, previous_graph_image, side_by_side);
  // Add separator line between the two images.
  cv::line(side_by_side, cv::Point(current_graph_image.cols, 0),
           cv::Point(current_graph_image.cols, current_graph_image.rows),
           cv::Scalar::all(0), 3);
  const cv::Point right_image_shift(current_graph_image.cols, 0);

  std::vector<std::vector<cv::Point> > links;
  // Draw correspondences between the current and the previous image
  for (int i = 0; i < max_similarity_agree.rows(); ++i) {
    for (int j = 0; j < max_similarity_agree.cols(); ++j) {
      if (max_similarity_agree(i, j) == true) {
        const cv::Point center_j =
            current_graph_descriptor->getDescriptor()[j].center;
        const cv::Point center_i =
            previous_graph_descriptor->getDescriptor()[i].center;
        const int link_step = 15;
        links.push_back(createCurveBetween(center_j,
                                           center_i + right_image_shift,
                                           link_step));
      }
    }
  }

  const cv::Scalar link_color = CV_RGB(255, 188, 56);
  const int link_thickness = 1;
  cv::polylines(side_by_side, links, false, link_color, link_thickness);

  cv::imshow("Matches between consecutive frames", side_by_side);

  cv::waitKey();

  /*
  // The code inside these brackets is here only for log purposes
  {

    // FIXME: depending on the feature types, we extract the matches in a different way, how to encapsulate this in the Abstract interface?
    auto matching =
        std::dynamic_pointer_cast<VectorMatcher::VectorMatchingResult>
            (matching_result);

    std::vector<std::vector<cv::DMatch>> matches = matching->matches;

    const int number_of_training_images =
        static_cast<int>(semantics_db_.size());

      std::vector<int> voting_per_image(number_of_training_images, 0);
      for (int feature = 0; feature < matches.size(); ++feature) {
        for (int vote = 0; vote < matches[feature].size(); ++vote) {
          int preference = matches[feature][vote].imgIdx;
          voting_per_image[preference]++;
        }
      }

      auto max_vote =
          std::max_element(voting_per_image.begin(), voting_per_image.end());

      LOG(INFO) << "Current frame (" << number_of_training_images
                << ") voted " << 100 * ((double) *max_vote) / (matches.size())
                << "% for image ("
                << std::distance(voting_per_image.begin(), max_vote) << ")!";
  }
   */
}

void XView::filterMatches(const SemanticLandmarkPtr& semantics_a,
                          AbstractMatcher::MatchingResultPtr& matching_result) {
  // TODO: filter matches, e.g., with geometric verification.
  CHECK(false) << "Not implemented.";
}

void XView::mergeSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matching_result) {
  // TODO: Merge semantics with semantics_db_ if dominant matches,
  // otherwise add semantics as new instance to semantics_db_.
  // TODO: use filterMatches function before merging.
  CHECK(false) << "Not implemented.";
}

void XView::cleanDatabase() {
  // TODO: sweep over semantics_db_ to match and merge unconnected semantics.
  CHECK(false) << "Not implemented.";
}

}
