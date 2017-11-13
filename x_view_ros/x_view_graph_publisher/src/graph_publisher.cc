#include <x_view_graph_publisher/graph_publisher.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

namespace x_view_ros {

GraphPublisher::GraphPublisher(ros::NodeHandle& nh,
                               const std::string& vertex_topic,
                               const std::string& edge_topic,
                               const std::string& matches_topic,
                               const std::string& position_topic)
    : nh_(nh),
      vertex_topic_(vertex_topic),
      edge_topic_(edge_topic),
      matches_topic_(matches_topic),
      position_topic_(position_topic),
      id_(0) {

  // Create the publisher objects.
  vertex_publisher_ =
      nh_.advertise<visualization_msgs::Marker>(vertex_topic_, 10000);

  edge_publisher_ =
      nh_.advertise<visualization_msgs::Marker>(edge_topic_, 100000);

  matches_publisher_ =
      nh_.advertise<visualization_msgs::Marker>(matches_topic_, 50);

  position_publisher_ =
      nh_.advertise<visualization_msgs::Marker>(position_topic_, 50);
}

void GraphPublisher::clean() const {

  visualization_msgs::Marker reset_marker;
  reset_marker.action = 3;
  vertex_publisher_.publish(reset_marker);
  edge_publisher_.publish(reset_marker);
  matches_publisher_.publish(reset_marker);
  id_ = 0;
}

void GraphPublisher::publish(const x_view::Graph& graph,
                             const ros::Time& time,
                             double z_offset) const {

  const uint64_t num_vertices = boost::num_vertices(graph);
  const uint64_t num_edges = boost::num_edges(graph);

  LOG(INFO) << "Publishing " << num_vertices << " vertices and "
            << num_edges << " edges.";

  // Publish all new vertices and edges.
  publishEdges(graph, time, z_offset);
  publishVertices(graph, time, z_offset);
}

void GraphPublisher::matchesToRosMsg(
    const x_view::Graph& query_graph, const x_view::Graph& database_graph,
    const x_view::GraphMatcher::IndexMatrixType& candidate_matches,
    const ros::Time& time, double z_offset,
    visualization_msgs::Marker* marker) const {

  // Create marker properties which are the same for all lines.
  marker->header.frame_id = "/world";
  marker->header.stamp = time;
  marker->ns = "semantic_graph_matches";
  marker->type = visualization_msgs::Marker::LINE_LIST;
  marker->action = visualization_msgs::Marker::ADD;
  marker->id = id_++;

  marker->scale.x = 0.2f;

  marker->color.r = 0.0f;
  marker->color.g = 1.0f;
  marker->color.b = 0.0f;
  marker->color.a = 0.7f;
  marker->lifetime = ros::Duration(1);

  // Iterate over all matches and publish edge between local graph and database
  // graph if there is a valid match.
  for (size_t row = 0u; row < candidate_matches.rows(); ++row) {
    for (size_t col = 0; col < candidate_matches.cols(); ++col) {
      if (candidate_matches(row, col)
          != x_view::GraphMatcher::INVALID_MATCH_INDEX) {

        const auto& v_p_from = query_graph[col];
        const auto& v_p_to = database_graph[candidate_matches(row, col)];

        geometry_msgs::Point p_from;
        p_from.x = v_p_from.location_3d[0];
        p_from.y = v_p_from.location_3d[1];
        p_from.z = v_p_from.location_3d[2] + z_offset;

        geometry_msgs::Point p_to;
        p_to.x = v_p_to.location_3d[0];
        p_to.y = v_p_to.location_3d[1];
        p_to.z = v_p_to.location_3d[2];

        marker->points.push_back(p_from);
        marker->points.push_back(p_to);
      }
    }
  }
}


void GraphPublisher::publishMatches(
    const x_view::Graph& query_graph, const x_view::Graph& database_graph,
    const x_view::GraphMatcher::IndexMatrixType& candidate_matches,
    const ros::Time& time, double z_offset) const {

  visualization_msgs::Marker marker;
  matchesToRosMsg(query_graph, database_graph, candidate_matches, time,
                  z_offset, &marker);
  matches_publisher_.publish(marker);
}

void GraphPublisher::robotPositionToRosMsg(const x_view::Vector3r& pos,
                                           const x_view::Vector3r& color,
                                           const ros::Time& time,
                                           const std::string ns,
                                           visualization_msgs::Marker* marker) const {

  marker->header.frame_id = "/world";
  marker->header.stamp = time;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker->ns = ns;
  marker->id = 0;

  marker->type = visualization_msgs::Marker::CYLINDER;
  marker->action = visualization_msgs::Marker::ADD;

  marker->pose.position.x = pos[0];
  marker->pose.position.y = pos[1];
  marker->pose.position.z = pos[2];
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 4.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker->color.r = static_cast<float>(color[0]);
  marker->color.g = static_cast<float>(color[1]);
  marker->color.b = static_cast<float>(color[2]);
  marker->color.a = 1.0;

  marker->lifetime = ros::Duration();
}

void GraphPublisher::publishRobotPosition(const x_view::Vector3r& pos,
                                          const x_view::Vector3r& color,
                                          const ros::Time& time,
                                          const std::string ns) const {

  visualization_msgs::Marker marker;
  robotPositionToRosMsg(pos, color, time, ns, &marker);
  position_publisher_.publish(marker);
}


void GraphPublisher::positionBlobToRosMsg(const x_view::Vector3r& gt_pos,
                                          const x_view::Vector3r& estimation_pos,
                                          const ros::Time& time,
                                          const std::string ns,
                                          visualization_msgs::Marker* marker) const {

  // Evaluate the estimation against the ground truth.
  double distance_xy = sqrt(
      (gt_pos(0) - estimation_pos(0)) * (gt_pos(0) - estimation_pos(0))
      + (gt_pos(1) - estimation_pos(1)) * (gt_pos(1) - estimation_pos(1)));

  marker->header.frame_id = "/world";
  marker->header.stamp = time;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker->ns = ns;
  marker->id = id_;

  marker->type = visualization_msgs::Marker::CYLINDER;
  marker->action = visualization_msgs::Marker::ADD;

  marker->pose.position.x = gt_pos[0];
  marker->pose.position.y = gt_pos[1];
  marker->pose.position.z = gt_pos[2];
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  // Set the color and marker size according to estimation result.
  if (estimation_pos == x_view::Vector3r::Zero()) {
    marker->color.r = 1;
    marker->color.g = 1;
    marker->color.b = 0;
    marker->scale.x = 3.0;
    marker->scale.y = 3.0;
    marker->scale.z = 8.0;
  } else if (distance_xy < 30.0) {
    marker->color.r = 0;
    marker->color.g = 1;
    marker->color.b = 0;
    marker->scale.x = 10.0;
    marker->scale.y = 10.0;
    marker->scale.z = 6.0;
  } else {
    marker->color.r = 1;
    marker->color.g = 0;
    marker->color.b = 0;
    marker->scale.x = 3.0;
    marker->scale.y = 3.0;
    marker->scale.z = 8.0;
  }
  marker->color.a = 0.5;

  marker->lifetime = ros::Duration(10000);
}

void GraphPublisher::publishPositionBlob(const x_view::Vector3r& gt_pos,
                                         const x_view::Vector3r& estimation_pos,
                                         const ros::Time& time,
                                         const std::string ns) const {

  visualization_msgs::Marker marker;
  positionBlobToRosMsg(gt_pos, estimation_pos, time, ns, &marker);
  position_publisher_.publish(marker);
}

void GraphPublisher::verticesToRosMsg(
    const x_view::Graph& graph, const ros::Time& time, double z_offset,
    std::vector<visualization_msgs::Marker>* markers) const {

  const auto vertices = boost::vertices(graph);
  // Get the last time index.
  uint64_t last_time_index = 0;
  for (auto iter = vertices.first; iter != vertices.second; ++iter) {
    last_time_index = std::max(last_time_index, graph[*iter].last_time_seen_);
  }

  for (auto iter = vertices.first; iter != vertices.second; ++iter) {

    visualization_msgs::Marker marker;
    const x_view::VertexProperty& v_p = graph[*iter];
    const int semantic_label = v_p.semantic_label;
    const cv::Scalar color = x_view::getColorFromSemanticLabel(semantic_label);
    const std::string semantic_name =
        x_view::Locator::getDataset()->label(semantic_label);
    const x_view::Vector3r& position = v_p.location_3d;

    marker.header.frame_id = "/world";
    marker.header.stamp = time;
    marker.ns = semantic_name;
    marker.id = id_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(5);

    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2] + z_offset;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;
    marker.color.a = 1.0;
    marker.color.r = static_cast<float>(color[2] / 255);
    marker.color.g = static_cast<float>(color[1] / 255);
    marker.color.b = static_cast<float>(color[0] / 255);

    if (v_p.last_time_seen_ == last_time_index) {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x *= 2;
      marker.scale.y *= 2;
      marker.scale.z *= 2;
    }
    markers->push_back(marker);
  }
}

void GraphPublisher::publishVertices(const x_view::Graph& graph,
                                     const ros::Time& time,
                                     double z_offset) const {

  std::vector<visualization_msgs::Marker> markers;
  verticesToRosMsg(graph, time, z_offset, &markers);

  for (size_t i = 0u; i < markers.size(); ++i) {
    vertex_publisher_.publish(markers[i]);
  }
}

void GraphPublisher::edgesToRosMsg(const x_view::Graph& graph,
                                  const ros::Time& time,
                                  double z_offset,
                                  std::vector<visualization_msgs::Marker>* markers) const {

  const auto edges = boost::edges(graph);

  // Compute the maximal number of time an edge has been seen.
  uint64_t max_times_seen = 1;
  for (auto iter = edges.first; iter != edges.second; ++iter) {
    max_times_seen = std::max(max_times_seen, graph[*iter].num_times_seen);
  }


  const float normalization = 1.0f / max_times_seen;

  for (auto iter = edges.first; iter != edges.second; ++iter) {

    visualization_msgs::Marker marker;
    // All coordinates are expressed in the world frame.
    marker.header.frame_id = "/world";
    marker.header.stamp = time;

    marker.ns = "semantic_graph_edges";
    marker.id = id_++;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    const float width = graph[*iter].num_times_seen * normalization;
    marker.scale.x = std::max(0.2f, width);

    marker.color.r = std::pow(0.5f, 1.f / width) + 0.5f;
    marker.color.g = std::pow(95.f / 255.f, 1.f / width) + 95.f / 255.f;
    marker.color.b = std::pow(25.f / 255.f, 1.f / width) + 25.f / 255.f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    const auto& v_d_from = boost::source(*iter, graph);
    const auto& v_d_to = boost::target(*iter, graph);

    const auto& v_p_from = graph[v_d_from];
    const auto& v_p_to = graph[v_d_to];

    geometry_msgs::Point p_from;
    p_from.x = v_p_from.location_3d[0];
    p_from.y = v_p_from.location_3d[1];
    p_from.z = v_p_from.location_3d[2] + z_offset;

    geometry_msgs::Point p_to;
    p_to.x = v_p_to.location_3d[0];
    p_to.y = v_p_to.location_3d[1];
    p_to.z = v_p_to.location_3d[2] + z_offset;

    marker.points.push_back(p_from);
    marker.points.push_back(p_to);

    markers->push_back(marker);
  }

}

void GraphPublisher::publishEdges(const x_view::Graph& graph,
                                  const ros::Time& time,
                                  double z_offset) const {

  std::vector<visualization_msgs::Marker> markers;
  edgesToRosMsg(graph, time, z_offset, &markers);
  for (size_t i = 0u; i < markers.size(); ++i) {
    edge_publisher_.publish(markers[i]);
  }
}
}

