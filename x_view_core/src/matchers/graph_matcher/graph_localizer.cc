#include <x_view_core/matchers/graph_matcher/graph_localizer.h>
#include <x_view_core/x_view_locator.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace x_view {

GraphLocalizer::GraphLocalizer(const double prior_noise)
    : prior_noise_(prior_noise),
      observations_(0) {
}

bool GraphLocalizer::localize(
    const GraphMatcher::GraphMatchingResult& matching_result,
    const Graph& query_semantic_graph, const Graph& database_semantic_graph,
    SE3* transformation) {

  CHECK_NOTNULL(transformation);

  const auto& parameters = Locator::getParameters();
  const auto& localizer_parameters =
      parameters->getChildPropertyList("localizer");
  const std::string localizer_type = localizer_parameters->getString("type");

  if (localizer_type == "OPTIMIZATION") {
    if(observations_.size() < 4) {
      LOG(ERROR) << "Localization only works correctly with N>=4 "
          "observations! Given observations: " << observations_.size() << ".";
      return false;
    }

    if(observations_.size() < 7) {
      LOG(WARNING) << "Localization result might be inaccurate due to lack of "
          "data: using " << observations_.size() << " observations.";
    }

    LOG(INFO) << "Localizing robot with " << observations_.size() << " semantic"
        " observations.";

    // Create a factor graph container.
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initials;

    // Add all observations to the factor graph adding a prior on their
    // position with a small noise (we want their position to stay fix during
    // optimization).
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3::Ones() * prior_noise_);

    for(int i = 0; i < observations_.size(); ++i) {
      gtsam::Point3 location(observations_[i].vertex_property.location_3d);

      graph.add(gtsam::PriorFactor<gtsam::Point3>(
          gtsam::Symbol('x', i), location, prior_noise));
      initials.insert(gtsam::Symbol('x', i), location);
    }

    for(int i = 0; i < observations_.size(); ++i) {
      // Add observation distance to the factor graph.
      gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
          gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector1::Ones() * (
                  1./(0.001 + observations_[i].evidence) - 1.0));
      graph.add(gtsam::RangeFactor<gtsam::Point3>(
          gtsam::Symbol('r', 0), gtsam::Symbol('x', i),
          observations_[i].distance, observation_noise));
    }

    // Compute initial guess for robot position.
    gtsam::Vector3 initial_robot_position = gtsam::Vector3::Zero();
    for(int i = 0; i < observations_.size(); ++i) {
      initial_robot_position += observations_[i].vertex_property.location_3d;
    }
    initial_robot_position /= observations_.size();
    initials.insert(gtsam::Symbol('r', 0), gtsam::Point3(initial_robot_position));

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials);
    gtsam::Values results = optimizer.optimize();

    const gtsam::Point3 robot_position =
        results.at<gtsam::Point3>(gtsam::Symbol ('r', 0));

    const Eigen::Vector3d computed_robot_pose(robot_position[0],
                                              robot_position[1],
                                              robot_position[2]);
    transformation->setIdentity();
    transformation->getPosition() = computed_robot_pose;
    return true;

  } else if (localizer_type == "ESTIMATION") {

    // Retrieve locations of matching node pairs.
    GraphMatcher::MaxSimilarityMatrixType similarities = matching_result
        .computeMaxSimilarityColwise();

    // Retrieve invalid matches.
    VectorXb invalid_matches = matching_result.getInvalidMatches();
    size_t num_valids = similarities.cols() - invalid_matches.count();

    if (num_valids == 0) {
      LOG(WARNING) << "Unable to estimate transformation, only invalid matches.";
      return false;
    }

    // todo(gawela): Should we generally use PCL types for points /
    // correspondences?
    // Prepare PCL containers with corresponding 3D points.
    pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr database_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    query_cloud->points.resize(num_valids);
    database_cloud->points.resize(num_valids);
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    correspondences->resize(num_valids);

    size_t valid = 0u;
    for (size_t i = 0u; i < similarities.cols(); ++i) {
      // Only add matches if they are valid.
      if (!invalid_matches(i)) {
        GraphMatcher::MaxSimilarityMatrixType::Index maxIndex;
        similarities.col(i).maxCoeff(&maxIndex);
        query_cloud->points[valid].getVector3fMap() = query_semantic_graph[i]
            .location_3d.cast<float>();
        database_cloud->points[valid].getVector3fMap() =
            database_semantic_graph[maxIndex].location_3d.cast<float>();
        (*correspondences)[valid].index_query = valid;
        (*correspondences)[valid].index_match = valid;
        ++valid;
      }
    }

    // Perform transformation estimation based on SVD.
    pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr
    transformation_estimation(
        new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
        pcl::PointXYZ>);

    Eigen::Matrix4f transform;
    transformation_estimation->estimateRigidTransformation(
        *query_cloud, *database_cloud, *correspondences, transform);

    SO3 proper_rotation;
    proper_rotation.fromApproximateRotationMatrix(
        transform.block(0, 0, 3, 3).cast<double>());
    (*transformation) = SE3(transform.block(0, 3, 3, 1).cast<double>(),
                            proper_rotation);
    return true;
  } else {
    CHECK(false) << "Unrecognized localizer type <" << localizer_type << ">"
        << std::endl;
  }
  return false;
}

void GraphLocalizer::addObservation(const VertexProperty& vertex_property,
                                    const double distance,
                                    const double evidence) {
  observations_.push_back(Observation{vertex_property, distance, evidence});
}

}
