#include <x_view_graph_publisher/graph_publisher.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

namespace x_view_ros {

GraphPublisher::GraphPublisher(ros::NodeHandle& nh,
                               const std::string& vertex_topic,
                               const std::string& edge_topic)
    : nh_(nh),
      vertex_topic_(vertex_topic),
      edge_topic_(edge_topic) {

  // Create the publisher objects.
  vertex_publisher_ =
      nh_.advertise<visualization_msgs::Marker>(vertex_topic_, 10000);

  edge_publisher_ =
      nh_.advertise<visualization_msgs::Marker>(edge_topic_, 10000);
}

void GraphPublisher::publish(const x_view::Graph& graph,
                             const ros::Time& time) const {

  const uint64_t num_vertices = boost::num_vertices(graph);
  const uint64_t num_edges = boost::num_edges(graph);

  LOG(INFO) << "Publishing " << num_vertices << " vertices and "
            << num_edges << " edges.";

  // Delete all markers of the previous frame.
  visualization_msgs::Marker reset_marker;
  reset_marker.action = 3;
  vertex_publisher_.publish(reset_marker);

  // Publish all new vertices and edges.
  publishVertices(graph, time);
  publishEdges(graph, time);
}

void GraphPublisher::publishVertices(const x_view::Graph& graph,
                                     const ros::Time& time) const {

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
    marker.id = v_p.index;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
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
    vertex_publisher_.publish(marker);
  }

}

void GraphPublisher::publishEdges(const x_view::Graph& graph,
                                  const ros::Time& time) const {

  const auto edges = boost::edges(graph);

  // Compute the maximal number of time an edge has been seen.
  uint64_t max_times_seen = 0;
  for (auto iter = edges.first; iter != edges.second; ++iter) {
    max_times_seen = std::max(max_times_seen, graph[*iter].num_times_seen);
  }

  const float normalization = 1.0f / max_times_seen;

  int edge_index = 0;
  for (auto iter = edges.first; iter != edges.second; ++iter) {

    visualization_msgs::Marker marker;
    // All coordinates are expressed in the world frame.
    marker.header.frame_id = "/world";
    marker.header.stamp = time;

    marker.ns = "semantic_graph_edges";
    marker.id = edge_index++;

    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;

    const float width = graph[*iter].num_times_seen * normalization;
    marker.scale.x = std::max(0.2f, width);

    marker.color.r = std::pow(0.5f, 1.f / width) + 0.5f;
    marker.color.g = std::pow(95.f / 255.f, 1.f / width) + 95.f / 255.f;
    marker.color.b = std::pow(25.f / 255.f, 1.f/ width) + 25.f / 255.f;
    marker.color.a = 0.7f;

    marker.lifetime = ros::Duration();

    const auto& v_d_from = boost::source(*iter, graph);
    const auto& v_d_to = boost::target(*iter, graph);

    const auto& v_p_from = graph[v_d_from];
    const auto& v_p_to = graph[v_d_to];

    geometry_msgs::Point p_from;
    p_from.x = v_p_from.location_3d[0];
    p_from.y = v_p_from.location_3d[1];
    p_from.z = v_p_from.location_3d[2];

    geometry_msgs::Point p_to;
    p_to.x = v_p_to.location_3d[0];
    p_to.y = v_p_to.location_3d[1];
    p_to.z = v_p_to.location_3d[2];

    marker.points.push_back(p_from);
    marker.points.push_back(p_to);

    edge_publisher_.publish(marker);
  }

}

}

