#include <x_view_core/matchers/graph_matcher/graph_localizer.h>

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

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    TransformationVector;

GraphLocalizer::GraphLocalizer(const double prior_noise)
    : prior_noise_(prior_noise),
      observations_(0) {
}

const Eigen::Vector3d GraphLocalizer::localize() const {

  if(observations_.size() < 4) {
    LOG(ERROR) << "Localization only works correctly with N>=4 "
        "observations! Given observations: " << observations_.size() << ".";
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
  return computed_robot_pose;
}

void GraphLocalizer::addObservation(const VertexProperty& vertex_property,
                                    const double distance,
                                    const double evidence) {
  observations_.push_back(Observation{vertex_property, distance, evidence});
}

// Function extracted from ethz-asl modelify repository.
void sortCorrespondenceClusters(
    std::vector<pcl::Correspondences>* clustered_correspondences,
    TransformationVector* transforms) {
  CHECK_NOTNULL(clustered_correspondences);
  CHECK_NOTNULL(transforms);
  CHECK_EQ(clustered_correspondences->size(), transforms->size());

  typedef std::pair<pcl::Correspondences, Eigen::Matrix4f> ClusterTransformPair;
  std::vector<ClusterTransformPair> cluster_transform_pairs;
  cluster_transform_pairs.reserve(clustered_correspondences->size());
  std::transform(clustered_correspondences->begin(),
                 clustered_correspondences->end(), transforms->begin(),
                 std::back_inserter(cluster_transform_pairs),
                 [](pcl::Correspondences a, Eigen::Matrix4f b) {
    return std::make_pair(a, b);
  });

  std::sort(cluster_transform_pairs.begin(), cluster_transform_pairs.end(),
            [](const ClusterTransformPair& a, const ClusterTransformPair& b) {
    return a.first.size() > b.first.size();
  });

  // Clean up old vectors.
  transforms->clear();
  clustered_correspondences->clear();

  for (auto it = std::make_move_iterator(cluster_transform_pairs.begin()),
      end = std::make_move_iterator(cluster_transform_pairs.end());
      it != end; ++it) {
    clustered_correspondences->push_back(std::move(it->first));
    transforms->push_back(std::move(it->second));
  }
}

bool GraphLocalizer::estimateTransformation(
    const GraphMatcher::GraphMatchingResult& matching_result,
    const Graph& query_semantic_graph, const Graph& database_semantic_graph,
    SE3* transformation) {
  CHECK_NOTNULL(transformation);

  // Retrieve locations of matching node pairs.
  GraphMatcher::MaxSimilarityMatrixType similarities = matching_result
      .computeMaxSimilarityColwise();

  // todo(gawela): Should we generally use PCL types for points /
  // correspondences?
  pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr database_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  query_cloud->points.resize(similarities.cols());
  database_cloud->points.resize(similarities.cols());
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  correspondences->resize(similarities.cols());

  for (size_t i = 0u; i < similarities.cols(); ++i) {
    GraphMatcher::MaxSimilarityMatrixType::Index maxIndex;
    similarities.col(i).maxCoeff(&maxIndex);
    query_cloud->points[i].getVector3fMap() = query_semantic_graph[i]
        .location_3d.cast<float>();
    database_cloud->points[i].getVector3fMap() =
        database_semantic_graph[maxIndex].location_3d.cast<float>();
    (*correspondences)[i].index_query = i;
    (*correspondences)[i].index_match = i;
  }

  pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr
  transformation_estimation(
      new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
          pcl::PointXYZ>);

  Eigen::Matrix4f transform;
  transformation_estimation->estimateRigidTransformation(
      *query_cloud, *database_cloud, *correspondences, transform);

  (*transformation) = SE3(Eigen::Matrix4d(transform.cast<double>()));
  return true;
}

}
