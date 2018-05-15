#include <x_view_core_os/matchers/graph_matcher/graph_localizer.h>
#include <x_view_core_os/x_view_locator.h>

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

  if(pose_vertex_measurements_.size() == 0) {
    LOG(WARNING) << "No pose-vertex measurements found.";
    transformation->setIdentity();
    return std::numeric_limits<real_t>::max();
  }

  const auto& parameters = Locator::getParameters();
  const auto& localizer_parameters =
      parameters->getChildPropertyList("localizer");
  const std::string localizer_type = localizer_parameters->getString("type");

  const bool use_robust_noise_model_default = false;
  const bool use_robust_noise_model =
      localizer_parameters->getBoolean("use_robust_noise",
                                       use_robust_noise_model_default);

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

    // Add simple prior factor on the pose of the robot to fix the height.
    gtsam::Vector6 pose_prior_noise;
//    Eigen::Vector3d t_a = pose_pose_measurements_[0].pose_a.pose.getPosition();
//    t_a(2) = 50.0;
//    gtsam::Point3 t_w_a(t_a);
    pose_prior_noise << 10000.0, 10000.0, 0.0, 0.0, 0.0, 100000.0;
    gtsam::noiseModel::Diagonal::shared_ptr pose_prior_observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(pose_prior_noise);
    graph.push_back(
        absolutePoseFactor( pose_pose_measurements_[0].pose_a.pose,
            gtsam::Symbol('p', pose_pose_measurements_[0].pose_a.id),
            pose_prior_observation_noise));

    // Treat odometry of the localization as almost perfect, therefore add relative factors
    // with only prior noise to the graph.
//    gtsam::noiseModel::Diagonal::shared_ptr relative_noise_trans =
//        gtsam::noiseModel::Diagonal::Sigmas(
//            gtsam::Vector3::Ones() * prior_noise_);
//
//    for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
//      gtsam::Point3 t_a_b(
//          pose_pose_measurements_[i].pose_b.pose.getPosition()
//              - pose_pose_measurements_[i].pose_a.pose.getPosition());
//      // Add odometry as relative movement between poses.
//      graph.push_back(
//          relativePointFactor(
//              t_a_b, gtsam::Symbol('p', pose_pose_measurements_[i].pose_a.id),
//              gtsam::Symbol('p', pose_pose_measurements_[i].pose_b.id), relative_noise_trans));
//    }
//
//    gtsam::noiseModel::Diagonal::shared_ptr relative_noise_trans =
//        gtsam::noiseModel::Diagonal::Sigmas(
//            gtsam::Vector6::Ones() * prior_noise_);

    gtsam::noiseModel::Diagonal::shared_ptr relative_noise_trans =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector6::Ones() * prior_noise_);

    for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
      SE3 T_a_b = pose_pose_measurements_[i].pose_a.pose.inverse()
          * pose_pose_measurements_[i].pose_b.pose;
      // Add odometry as relative movement between poses.
      graph.push_back(
          relativeFactor (T_a_b, gtsam::Symbol('p', pose_pose_measurements_[i].pose_a.id), gtsam::Symbol(
              'p', pose_pose_measurements_[i].pose_b.id), relative_noise_trans));
    }

    // Create noise model for position-vertex measurements.
//    gtsam::noiseModel::Diagonal::shared_ptr pv_observation_noise_ =
//        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Ones() * 10.0);
//
//    gtsam::noiseModel::Base::shared_ptr pv_observation_noise =
//        pv_observation_noise_;
//    if(use_robust_noise_model) {
//      // Make noise model robust by adding m-estimation.
//      gtsam::noiseModel::Robust::shared_ptr temp_noise =
//          gtsam::noiseModel::Robust::Create(
//              gtsam::noiseModel::mEstimator::Cauchy::Create(1),
//              pv_observation_noise);
//      pv_observation_noise = temp_noise;
//    }
//    // Add all pose-vertex observations to the factor graph.
//    for (size_t i = 0u; i < pose_vertex_measurements_.size(); ++i) {
//      // Calculate translation measurement between robot position and vertex.
//      gtsam::Point3 t_a_v(
//          pose_vertex_measurements_[i].vertex_property.location_3d.cast<double>()
//              - pose_vertex_measurements_[i].observer_pose.pose.getPosition());
//      graph.push_back(
//          relativePointFactor(
//              t_a_v,
//              gtsam::Symbol('p', pose_vertex_measurements_[i].observer_pose.id),
//              gtsam::Symbol('x', pose_vertex_measurements_[i].vertex_property.index),
//              pv_observation_noise));
//    }
    gtsam::Vector6 pv_noise;
    pv_noise << 10.0, 10.0, 10.0, 0.0, 0.0, 5.0;
    gtsam::noiseModel::Diagonal::shared_ptr pv_observation_noise_ =
        gtsam::noiseModel::Diagonal::Sigmas(pv_noise);

    gtsam::noiseModel::Base::shared_ptr pv_observation_noise =
        pv_observation_noise_;
    if(use_robust_noise_model) {
      // Make noise model robust by adding m-estimation.
      gtsam::noiseModel::Robust::shared_ptr temp_noise =
          gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Cauchy::Create(1),
              pv_observation_noise);
      pv_observation_noise = temp_noise;
    }
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
              pv_observation_noise));
    }

//    // Create noise model for vertex-vertex identity matches.
//    gtsam::noiseModel::Diagonal::shared_ptr vv_observation_noise_ =
//        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Ones() * 10.0);
//    gtsam::noiseModel::Base::shared_ptr vv_observation_noise =
//        vv_observation_noise_;
//    if(use_robust_noise_model) {
//      // Make noise model robust by adding m-estimation.
//      gtsam::noiseModel::Robust::shared_ptr vv_robust_noise =
//          gtsam::noiseModel::Robust::Create(
//              gtsam::noiseModel::mEstimator::Cauchy::Create(1),
//              vv_observation_noise);
//      vv_observation_noise = vv_robust_noise;
//    }
//    // Add vertex-vertex measurements to graph as identity matches.
//    gtsam::Point3 zero_transform(Eigen::Vector3d(0, 0, 0));
//    for (size_t i = 0u; i < vertex_vertex_measurements_.size(); ++i) {
//      // Add identity matches to factor graph.
//      graph.push_back(
//          relativePointFactor(
//              zero_transform,
//              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
//              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index),
//              vv_observation_noise));
//
//      // Add database vertex factors.
//      if (!initials.exists(
//          gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index))) {
//        gtsam::Point3 location_db(
//            vertex_vertex_measurements_[i].vertex_b.location_3d.cast<double>());
//        // Add Prior factors on the database vertices.
//        graph.add(
//            gtsam::PriorFactor < gtsam::Point3
//                > (gtsam::Symbol('x',
//                                 vertex_vertex_measurements_[i].vertex_b.index), location_db, prior_noise));
//        initials.insert(
//            gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index),
//            location_db);
//      }
//      // Add initial guesses for query_graph at database vertex locations.
//      if (!initials.exists(
//          gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index))) {
//        gtsam::Point3 location(
//            vertex_vertex_measurements_[i].vertex_b.location_3d.cast<double>());
//        initials.insert(
//            gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
//            location);
//      }
//    }

    // Create noise model for vertex-vertex identity matches.
    gtsam::Vector6 vv_noise;
    vv_noise << 10.0, 10.0, 10.0, 0.0, 0.0, 100.0;
    gtsam::noiseModel::Diagonal::shared_ptr vv_observation_noise_ =
        gtsam::noiseModel::Diagonal::Sigmas(
            vv_noise);
    gtsam::noiseModel::Base::shared_ptr vv_observation_noise =
        vv_observation_noise_;
    if(use_robust_noise_model) {
      // Make noise model robust by adding m-estimation.
      gtsam::noiseModel::Robust::shared_ptr vv_robust_noise =
          gtsam::noiseModel::Robust::Create(
              gtsam::noiseModel::mEstimator::Cauchy::Create(1),
              vv_observation_noise);
      vv_observation_noise = vv_robust_noise;
    }
    // Add vertex-vertex measurements to graph as identity matches.
    SE3 zero_transform(Eigen::Vector3d(0, 0, 0), SO3(1, 0, 0, 0));
    for (size_t i = 0u; i < vertex_vertex_measurements_.size(); ++i) {
      // Add identity matches to factor graph.
      graph.push_back(
          relativeFactor(
              zero_transform,
              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
              gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_b.index),
              vv_observation_noise));

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
      if (!initials.exists(
          gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index))) {
        SE3 location(
            vertex_vertex_measurements_[i].vertex_b.location_3d.cast<double>(),
            SO3(1, 0, 0, 0));
        initials.insert(
            gtsam::Symbol('x', vertex_vertex_measurements_[i].vertex_a.index),
            location);
      }
    }

    // Compute initial guess for robot and vertex positions as the
    // mean location of the matched vertices.
    gtsam::Vector3 initial_robot_position = gtsam::Vector3::Zero();
    for (int i = 0; i < vertex_vertex_measurements_.size(); ++i) {
      initial_robot_position += vertex_vertex_measurements_[i].vertex_b
          .location_3d.cast<double>();
    }
    initial_robot_position /= vertex_vertex_measurements_.size();
    // Fix robot position to certain height.
    initial_robot_position(2) = 50.0;

//    // todo(gawela) better initialization)
//    // Add robot pose initials to GTSAM.
//    if (pose_pose_measurements_.size() > 0) {
//      initials.insert(gtsam::Symbol('p', pose_pose_measurements_[0].pose_a.id),
//                      gtsam::Point3(initial_robot_position));
//      for (size_t i = 0u; i < pose_pose_measurements_.size(); ++i) {
//        initials.insert(gtsam::Symbol('p', pose_pose_measurements_[i].pose_b.id),
//                        gtsam::Point3(initial_robot_position));
//      }
//      // If no intra-pose measurements, take the observer of the first vertex.
//    } else {
//      initials.insert(
//          gtsam::Symbol('p', pose_vertex_measurements_[0].observer_pose.id),
//          gtsam::Point3(initial_robot_position));
//    }
//
//    // Add graph to optimizer object and estimate transformation.
//    gtsam::LevenbergMarquardtParams params;
//    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials, params);
//    gtsam::Values results = optimizer.optimize();
//
//    LOG(INFO) << "Inital position: " << initial_robot_position.transpose() << std::endl;
//
//    SE3 robot_pose;
//    robot_pose.setIdentity();
//    gtsam::Point3 robot_position;
//
//    // Estimate last position of pose-pose measurements.
//    if (pose_pose_measurements_.size() > 0) {
//      robot_position = results.at < gtsam::Point3
//          > (gtsam::Symbol(
//              'p',
//              pose_pose_measurements_[pose_pose_measurements_.size() - 1].pose_b
//              .id));
//      // Otherwise estimate the observer pose.
//    } else {
//      robot_position = results.at < gtsam::Point3
//          > (gtsam::Symbol('p', pose_vertex_measurements_[0].observer_pose.id));
//    }
//    robot_pose.getPosition() = gtsam::Vector3(robot_position.x(),
//                                              robot_position.y(),
//                                              robot_position.z());

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
//    gtsam::LevenbergMarquardtParams params;
//    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials, params);
//    gtsam::Values results = optimizer.optimize();

    LOG(INFO) << "Inital position: " << initial_robot_position.transpose() << std::endl;

    SE3 robot_pose;

//    // Estimate last position of pose-pose measurements.
//    if (pose_pose_measurements_.size() > 0) {
//      robot_pose = results.at < SE3
//          > (gtsam::Symbol(
//              'p',
//              pose_pose_measurements_[pose_pose_measurements_.size() - 1].pose_b
//                  .id));
//      // Otherwise estimate the observer pose.
//    } else {
//      robot_pose = results.at < SE3
//          > (gtsam::Symbol('p', pose_vertex_measurements_[0].observer_pose.id));
//    }

    // HACK!!!! Overwriting the estimation!!!!
    robot_pose.setIdentity();
    robot_pose.getPosition() = initial_robot_position;
    /////////////////////////////////////////////
    (*transformation) = robot_pose;

    // Compute the residual of the optimization (unnormalized).
//    const real_t unnormalized_residual = graph.error(results);
//    const uint64_t num_factors = graph.nrFactors();
//    return unnormalized_residual / num_factors;
    return 0.0;

  } else if (localizer_type == "ESTIMATION") {

    // FIXME: this technique appears not to work correctly
    LOG(ERROR) << "ESTIMATION is not fully supported.";

    // Retrieve locations of matching node pairs.
    const GraphMatcher::MaxSimilarityMatrixType similarity_matrix =
        matching_result.computeMaxSimilarityColwise();

    // Retrieve invalid matches.
    const GraphMatcher::IndexMatrixType& candidate_matches =
        matching_result.getCandidateMatches();

    uint64_t num_valid_query = 0;
    for(int j = 0; j < candidate_matches.cols(); ++j){
      for(int i = 0; i < candidate_matches.rows(); ++i) {
        if(candidate_matches(i, j) != GraphMatcher::INVALID_MATCH_INDEX) {
          ++num_valid_query;
        }
      }
    }

    if (num_valid_query == 0) {
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

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

    // Keep track of which vertex of the global semantic graph has which index
    // in the database_cloud.
    std::unordered_map<uint64_t, uint64_t>
        vertex_index_to_database_pointcloud;

    for (size_t i = 0u; i < similarity_matrix.cols(); ++i) {
      // Iterate over the candidate matches.
      for(uint64_t j_id = 0; j_id < candidate_matches.rows(); ++j_id) {
        const int candidate_match_in_global_graph =
            candidate_matches(j_id, i);
        // Ignore the match in case it was marked as invalid.
        if(candidate_match_in_global_graph == GraphMatcher::INVALID_MATCH_INDEX)
          continue;
        const Vector3r& query_position = query_semantic_graph[i].location_3d;
        query_cloud->points.push_back(pcl::PointXYZ(query_position[0],
                                                    query_position[1],
                                                    query_position[2]));

        // Determine if the database_vertex has already been seen.
        uint64_t index_match;
        // If that database vertex has already been added to the
        // database_pointcloud, we don't need to add that vertex to the
        // pointcloud, but only need to refer to it through its index in the
        // database_cloud.
        if(vertex_index_to_database_pointcloud.count
            (candidate_match_in_global_graph) > 0)
          index_match = vertex_index_to_database_pointcloud
          [candidate_match_in_global_graph];
        else {
          // Add the new vertex to the database_cloud and use its new index.
          const Vector3r& vertex_position =
              database_semantic_graph[candidate_match_in_global_graph].location_3d;
          database_cloud->points.push_back(pcl::PointXYZ(vertex_position[0],
                                                         vertex_position[1],
                                                         vertex_position[2]));

          // The index of the newly added vertex is the one corresponding to
          // the last inserted element in the pointcoloud.
          index_match = database_cloud->points.size() - 1;

          // Store the correspondences in indices.
          vertex_index_to_database_pointcloud[candidate_match_in_global_graph] =
              index_match;
        }

        // Create a correspondence between the query vertex and the associated
        // database vertex.
        pcl::Correspondence correspondence;
        // The index query corresponds to the index of the point in the
        // query_cloud of th vertex of the query_graph.
        correspondence.index_query = i;
        // The index match corresponds to the index of the last added point in
        // the database_cloud.
        correspondence.index_match = index_match;
        correspondences->push_back(correspondence);
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
