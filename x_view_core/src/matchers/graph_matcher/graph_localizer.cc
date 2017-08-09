#include <x_view_core/matchers/graph_matcher/graph_localizer.h>
#include <x_view_core/matchers/graph_matcher/RangeFactor.h>
#include <x_view_core/x_view_locator.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

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
      graph.add(
          gtsam::RangeFactor<gtsam::Point3>(
              gtsam::Symbol('r', 0),
              gtsam::Symbol('x', i),
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

    const Eigen::Vector3d computed_robot_pose(robot_position.x(),
                                              robot_position.y(),
                                              robot_position.z());
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

// Create GTSAM Expression for relative pose measurements (e.g., odometry).
gtsam::ExpressionFactor<SE3> GraphLocalizer::relativeFactor(const SE3& relative_pose,
                                            int index_a,
                                            int index_b,
                                            gtsam::noiseModel::Base::shared_ptr noise_model) const {
  gtsam::Expression<SE3> T_w_a(index_a);
  gtsam::Expression<SE3> T_w_b(index_b);
  gtsam::Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
  gtsam::Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
  return gtsam::ExpressionFactor<SE3> (noise_model, relative_pose, relative);
}

// Create GTSAM expression for absolute pose measurements (e.g., priors).
gtsam::ExpressionFactor<SE3> GraphLocalizer::absolutePoseFactor(const SE3& pose_measurement,
                                                int index,
                                                gtsam::noiseModel::Base::shared_ptr noise_model) const {
  gtsam::Expression<SE3> T_w(index);
  return gtsam::ExpressionFactor<SE3>(noise_model, pose_measurement, T_w);
}

bool GraphLocalizer::localize2(
    const GraphMatcher::GraphMatchingResult& matching_result,
    const Graph& query_semantic_graph, const Graph& database_semantic_graph,
    SE3* transformation) {

  CHECK_NOTNULL(transformation);

  const auto& parameters = Locator::getParameters();
  const auto& localizer_parameters =
      parameters->getChildPropertyList("localizer");
  const std::string localizer_type = localizer_parameters->getString("type");

  if (localizer_type == "OPTIMIZATION") {
    if (pose_pose_measurements_.size() + pose_vertex_measurements_.size()
        + vertex_vertex_measurements_.size() < 4) {
      LOG(ERROR) << "Localization only works correctly with N>=4 "
          "observations! Given observations: "
          << pose_pose_measurements_.size() + pose_vertex_measurements_.size()
              + vertex_vertex_measurements_.size() << ".";
      return false;
    }

    if (pose_pose_measurements_.size() + pose_vertex_measurements_.size()
        + vertex_vertex_measurements_.size() < 7) {
      LOG(WARNING) << "Localization result might be inaccurate due to lack of "
          "data: using "
          << pose_pose_measurements_.size() + pose_vertex_measurements_.size()
              + vertex_vertex_measurements_.size() << " observations.";
    }

    LOG(INFO) << "Localizing robot with "
        << pose_pose_measurements_.size() + pose_vertex_measurements_.size()
            + vertex_vertex_measurements_.size() << " semantic"
            " observations.";

    // Create a factor graph container.
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initials;

    // Treat the database graph as static, therefore build factor graph on these
    // vertices without noise, so these vertices don't move during optimization.
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3::Ones() * prior_noise_);
    const uint64_t num_vertices = boost::num_vertices(database_semantic_graph);

    for (size_t i = 0u; i < num_vertices; ++i) {
      const VertexProperty& vertex_property = database_semantic_graph[i];
      gtsam::Point3 location(database_semantic_graph[i].location_3d);

      graph.add(gtsam::PriorFactor <gtsam::Point3>(
          gtsam::Symbol('x', database_semantic_graph[i].index), location, prior_noise));
      initials.insert(gtsam::Symbol('x', database_semantic_graph[i].index), location);
    }

    const uint64_t num_vertices_query = boost::num_vertices(query_semantic_graph);

    // Treat odometry of the localization as perfect, therefore add relative factors
    // without noise to the graph.
    gtsam::noiseModel::Diagonal::shared_ptr relative_noise =
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector6::Ones() * prior_noise_);

    for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
      SE3 T_a_b = pose_pose_measurements_[i].pose_a.pose.inverted()
          * pose_pose_measurements_[i].pose_b.pose;
      graph.push_back(relativeFactor(T_a_b, pose_pose_measurements_[i].pose_a.id,
                               pose_pose_measurements_[i].pose_b.id, relative_noise));
    }

    // Add pose-vertex factors between robot poses and vertices.
    for (size_t i = 0u; i < pose_vertex_measurements_.size(); ++i) {
      gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
                gtsam::noiseModel::Diagonal::Sigmas(
                    gtsam::Vector1::Ones() * 5.0);
      gtsam::noiseModel::Robust::shared_ptr temp_noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(10), observation_noise);
      double distance = (pose_vertex_measurements_[i].observer_pose.pose
          .getPosition()
          - pose_vertex_measurements_[i].vertex_property.location_3d).norm();
      graph.push_back(
          gtsam::RangeFactor<SE3, gtsam::Point3> (gtsam::Symbol(pose_vertex_measurements_[i].observer_pose.id), gtsam::Symbol(
                  'x', pose_vertex_measurements_[i].vertex_property.index), distance, temp_noise));
    }

    // Add vertex-vertex measurements to graph as identity matches.
    for (size_t i = 0u; i < vertex_vertex_measurements_.size(); ++i) {
      gtsam::noiseModel::Robust::shared_ptr observation_noise = gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Cauchy::Create(10),
      gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector1::Ones() * 5.0));
      graph.add(
          gtsam::RangeFactor<gtsam::Point3>(
              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index),
              0.0, observation_noise));

      // Add initial guesses for query_graph at database vertex locations.
      gtsam::Point3 location(vertex_vertex_measurements_[i].vertex_b.location_3d);
      graph.add(gtsam::PriorFactor <gtsam::Point3> (gtsam::Symbol('x',
        vertex_vertex_measurements_[i].vertex_a.index), location, prior_noise));
      initials.insert(
          gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index), location);
    }

    // Compute initial guess for robot and vertex poses / positions.
    gtsam::Vector3 initial_robot_position = gtsam::Vector3::Zero();
    for(int i = 0; i < vertex_vertex_measurements_.size(); ++i) {
      initial_robot_position += vertex_vertex_measurements_[i].vertex_b.location_3d;
    }
    initial_robot_position /= vertex_vertex_measurements_.size();

    x_view::SE3 initial_robot_pose;
    initial_robot_pose.setIdentity();
    initial_robot_pose.getPosition() = initial_robot_position;

    // todo remove temp transform
    SE3 temp_transform;
    temp_transform.setIdentity();
    temp_transform.getPosition() = gtsam::Vector3(2,2,2);

    initials.insert(gtsam::Symbol(pose_pose_measurements_[0].pose_a.id), pose_pose_measurements_[0].pose_a.pose * temp_transform);
    for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
      initials.insert(gtsam::Symbol(pose_pose_measurements_[i].pose_b.id), pose_pose_measurements_[i].pose_b.pose * temp_transform);
    }

    gtsam::LevenbergMarquardtParams params;
    params.setVerbosity("LINEAR");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials, params);

    gtsam::Values results = optimizer.optimize();

    std::cout << "Inital position: " << initial_robot_position << std::endl;
    x_view::SE3 robot_pose =
        results.at<x_view::SE3>(gtsam::Symbol (pose_pose_measurements_[pose_pose_measurements_.size() -1].pose_b.id));

    (*transformation) = robot_pose;
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

void GraphLocalizer::addPosePoseMeasurement(PoseId pose_id_a, PoseId pose_id_b) {
  pose_pose_measurements_.push_back(PosePoseMeasurement {pose_id_a, pose_id_b});
}

void GraphLocalizer::addPoseVertexMeasurement(const VertexProperty& vertex_property,
                                              const PoseId& pose_id,
                                              const double evidence) {
  pose_vertex_measurements_.push_back(PoseVertexMeasurement {vertex_property, pose_id,
    evidence });
}

void GraphLocalizer::addVertexVertexMeasurement(
    const VertexProperty& vertex_property_a,
    const VertexProperty& vertex_property_b, double similarity)
{
  vertex_vertex_measurements_.push_back(VertexVertexMeasurement {
      vertex_property_a, vertex_property_b, similarity});
}

}
