#include <x_view_core/matchers/graph_matcher/graph_localizer.h>
#include <x_view_core/x_view_locator.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace x_view {

GraphLocalizer::GraphLocalizer(const real_t prior_noise)
    : prior_noise_(prior_noise),
      pose_pose_measurements_(0),
      pose_vertex_measurements_(0),
      vertex_vertex_measurements_(0) {}

// Create GTSAM Expression for relative pose measurements (e.g., odometry).
gtsam::ExpressionFactor<SE3> GraphLocalizer::relativeFactor(
    const SE3& relative_pose, gtsam::Symbol index_a, gtsam::Symbol index_b,
    gtsam::noiseModel::Base::shared_ptr noise_model) const {
  gtsam::Expression<SE3> T_w_a(index_a);
  gtsam::Expression<SE3> T_w_b(index_b);
  gtsam::Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
  gtsam::Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
  return gtsam::ExpressionFactor<SE3> (noise_model, relative_pose, relative);
}

// Create GTSAM expression for absolute pose measurements (e.g., priors).
gtsam::ExpressionFactor<SE3> GraphLocalizer::absolutePoseFactor(
    const SE3& pose_measurement, gtsam::Symbol index,
    gtsam::noiseModel::Base::shared_ptr noise_model) const {
  gtsam::Expression<SE3> T_w(index);
  return gtsam::ExpressionFactor<SE3> (noise_model, pose_measurement, T_w);
}

// Create GTSAM expression for relative point measurements (e.g., translation).
gtsam::ExpressionFactor<gtsam::Point3> GraphLocalizer::relativePointFactor(
    const gtsam::Point3& translation, gtsam::Symbol index_a,
    gtsam::Symbol index_b,
    gtsam::noiseModel::Base::shared_ptr noise_model) const {
  gtsam::Expression <gtsam::Point3> t_w_a(index_a);
  gtsam::Expression <gtsam::Point3> t_w_b(index_b);
  gtsam::Expression <gtsam::Point3> t_a_b(gtsam::between(t_w_a, t_w_b));
  return gtsam::ExpressionFactor < gtsam::Point3
      > (noise_model, translation, t_a_b);
}

real_t GraphLocalizer::localize(
    const GraphMatcher::GraphMatchingResult& matching_result,
    const Graph& query_semantic_graph, const Graph& database_semantic_graph,
    SE3* transformation) {

  CHECK_NOTNULL(transformation);
  CHECK_GT(pose_vertex_measurements_.size(), 0)
      << "No pose vertex measurements given.";

  const auto& parameters = Locator::getParameters();
  const auto& localizer_parameters =
      parameters->getChildPropertyList("localizer");
  const std::string localizer_type = localizer_parameters->getString("type");

  if (localizer_type == "OPTIMIZATION") {
    if (pose_vertex_measurements_.size() < 4) {
      LOG(ERROR) << "Localization only works correctly with N>=4 "
          "observations! Given observations: "
          << pose_pose_measurements_.size() + pose_vertex_measurements_.size()
          + vertex_vertex_measurements_.size() << ".";
      // Localization failed, return maximal residual.
      return std::numeric_limits<real_t>::max();
    }

    if (pose_vertex_measurements_.size() < 7) {
      LOG(WARNING) << "Localization result might be inaccurate due to lack of "
          "data: using "
          << pose_pose_measurements_.size() + pose_vertex_measurements_.size()
          + vertex_vertex_measurements_.size() << " observations.";
    }

    LOG(INFO) << "Localizing robot with "
        << pose_vertex_measurements_.size() << " semantic"
        " observations.";

    // Create a factor graph container.
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initials;

    // Treat the database graph as static, therefore build factor graph on these
    // vertices only with prior noise, so these vertices don't move during optimization.
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector6::Ones() * prior_noise_);
    const uint64_t num_vertices = boost::num_vertices(database_semantic_graph);

    // Treat odometry of the localization as almost perfect, therefore add relative factors
    // with only prior noise to the graph.
    gtsam::noiseModel::Diagonal::shared_ptr relative_noise_trans =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector6::Ones() * prior_noise_);

    for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
      SE3 T_a_b = pose_pose_measurements_[i].pose_a.pose.inverted()
          * pose_pose_measurements_[i].pose_b.pose;
      // Add odometry as relative movement between poses.
      graph.push_back(
          relativeFactor (T_a_b, gtsam::Symbol('p', pose_pose_measurements_[i].pose_a.id), gtsam::Symbol(
              'p', pose_pose_measurements_[i].pose_b.id), relative_noise_trans));
    }

    // Create noise model for position-vertex measurements.
    gtsam::Vector6 pv_noise;
    pv_noise << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
    gtsam::noiseModel::Diagonal::shared_ptr pv_observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(pv_noise);
    // Make noise model robust by adding m-estimation.
    gtsam::noiseModel::Robust::shared_ptr temp_noise =
        gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            pv_observation_noise);
    // Add all pose-vertex observations to the factor graph.
    for (size_t i = 0u; i < pose_vertex_measurements_.size(); ++i) {
      // Calculate translation measurement between robot position and vertex.
      SE3 inverse = pose_vertex_measurements_[i].observer_pose.pose.inverse();
      SE3 vertex_pose(
          pose_vertex_measurements_[i].vertex_property.location_3d.cast<double>(),
          SO3(1, 0, 0, 0));
      SE3 transformation = inverse * vertex_pose;
      graph.push_back(
          relativeFactor(
              transformation,
              gtsam::Symbol('p', pose_vertex_measurements_[i].observer_pose.id),
              gtsam::Symbol('x', pose_vertex_measurements_[i].vertex_property.index),
              temp_noise));
    }

    // Create noise model for vertex-vertex identity matches.
    gtsam::Vector6 vv_noise;
    vv_noise << 5.0, 5.0, 5.0, 100.0, 100.0, 100.0;
    gtsam::noiseModel::Diagonal::shared_ptr vv_observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            vv_noise);
    // Make noise model robust by adding m-estimation.
    gtsam::noiseModel::Robust::shared_ptr vv_robust_noise =
        gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            vv_observation_noise);
    // Add vertex-vertex measurements to graph as identity matches.
    SE3 zero_transform(Eigen::Vector3d(0, 0, 0), SO3(1, 0, 0, 0));
    for (size_t i = 0u; i < vertex_vertex_measurements_.size(); ++i) {
      // Add identity matches to factor graph.
      graph.push_back(
          relativeFactor(
              zero_transform,
              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index),
              vv_robust_noise));

      // Add database vertex factors.
      if (!initials.exists(
          gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index))) {
        SE3 location_db(
            vertex_vertex_measurements_[i].vertex_b.location_3d.cast<double>(),
            SO3(1, 0, 0, 0));
        // Add Prior factors on the database vertices.
        graph.add(
            absolutePoseFactor(
                location_db,
                gtsam::Symbol('x',
                              vertex_vertex_measurements_[i].vertex_b.index),
                prior_noise));
        initials.insert(
            gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index),
            location_db);
      }
      // Add initial guesses for query_graph at database vertex locations.
      SE3 location(
          vertex_vertex_measurements_[i].vertex_b.location_3d.cast<double>(),
          SO3(1, 0, 0, 0));
      initials.insert(
          gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
          location);
    }

    // Compute initial guess for robot and vertex positions as the
    // mean location of the matched vertices.
    gtsam::Vector3 initial_robot_position = gtsam::Vector3::Zero();
    for (int i = 0; i < vertex_vertex_measurements_.size(); ++i) {
      initial_robot_position += vertex_vertex_measurements_[i].vertex_b
          .location_3d.cast<double>();
    }
    initial_robot_position /= vertex_vertex_measurements_.size();

    // Add robot pose initials to GTSAM.
    if (pose_pose_measurements_.size() > 0) {
      initials.insert(gtsam::Symbol('p', pose_pose_measurements_[0].pose_a.id),
                      SE3(initial_robot_position, SO3(1 ,0, 0, 0)));
      for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
        initials.insert(gtsam::Symbol('p', pose_pose_measurements_[i].pose_b.id),
                        SE3(initial_robot_position, SO3(1, 0, 0, 0)));
      }
      // If no intra-pose measurements, take the observer of the first vertex.
    } else {
      initials.insert(
          gtsam::Symbol('p', pose_vertex_measurements_[0].observer_pose.id),
          SE3(initial_robot_position, SO3(1, 0, 0, 0)));
    }

    // Add graph to optimizer object and estimate transformation.
    gtsam::LevenbergMarquardtParams params;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials, params);
    gtsam::Values results = optimizer.optimize();

    LOG(INFO) << "Inital position: " << initial_robot_position.transpose() << std::endl;

    SE3 robot_position;

    // Estimate last position of pose-pose measurements.
    if (pose_pose_measurements_.size() > 0) {
      robot_position = results.at < SE3
          > (gtsam::Symbol(
              'p',
              pose_pose_measurements_[pose_pose_measurements_.size() - 1].pose_b
                  .id));
      // Otherwise estimate the observer pose.
    } else {
      robot_position = results.at < SE3
          > (gtsam::Symbol('p', pose_vertex_measurements_[0].observer_pose.id));
    }

    (*transformation) = robot_position;

    // Compute the residual of the optimization (unnormalized).
    const real_t unnormalized_residual = graph.error(results);
    const uint64_t num_factors = graph.nrFactors();
    return unnormalized_residual / num_factors;

  } else if (localizer_type == "ESTIMATION") {

    // Retrieve locations of matching node pairs.
    GraphMatcher::MaxSimilarityMatrixType similarities = matching_result
        .computeMaxSimilarityColwise();

    // Retrieve invalid matches.
    VectorXb invalid_matches = matching_result.getInvalidMatches();
    size_t num_valids = similarities.cols() - invalid_matches.count();

    if (num_valids == 0) {
      LOG(WARNING) << "Unable to estimate transformation, only invalid matches.";
      return std::numeric_limits<real_t>::max();
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
    // FIXME what is the residual here?
    return static_cast<real_t>(0);
  } else {
    CHECK(false) << "Unrecognized localizer type <" << localizer_type << ">"
        << std::endl;
  }
  return std::numeric_limits<real_t>::max();
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
