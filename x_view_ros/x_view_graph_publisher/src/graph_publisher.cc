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

  const unsigned long num_vertices = boost::num_vertices(graph);
  const unsigned long num_edges = boost::num_edges(graph);

  LOG(INFO) << "Publishing " << num_vertices << " vertices and "
            << num_edges << " edges.";

  publishVertices(graph, time);
  publishEdges(graph, time);
}

void GraphPublisher::publishVertices(const x_view::Graph& graph,
                                     const ros::Time& time) const {

  // A container keyed by the semantic label referencing a list of all
  // vertices with the given semantic label. This separation is useful as it
  // allows to show only vertices of a given type in RViz.
  std::map<int, visualization_msgs::Marker> vertices_of_semantic_label;
  for(int i = 0; i < x_view::Locator::getDataset()->numSemanticClasses(); ++i) {
    visualization_msgs::Marker point_list;
    // All coordinates are expressed in the world frame.
    point_list.header.frame_id = "/world";
    point_list.header.stamp = time;

    point_list.ns = x_view::Locator::getDataset()->label(i);
    // Since the ID is the same over multiple frames, RViz visualizes only
    // the last received message, which in case of global semantic graph
    // results in a correct behaviour as the global graph needs to be updated
    // in each frame and overwrites previous graphs.
    point_list.id = i;

    point_list.type = visualization_msgs::Marker::POINTS;

    point_list.action = visualization_msgs::Marker::ADD;

    point_list.scale.x = 0.8;
    point_list.scale.y = 0.8;

    // Each semantic label is colored differently.
    cv::Scalar color = x_view::getColorFromSemanticLabel(i);
    point_list.color.r = static_cast<float>(color[2] / 255);
    point_list.color.g = static_cast<float>(color[1] / 255);
    point_list.color.b = static_cast<float>(color[0] / 255);
    point_list.color.a = 1.0;

    point_list.lifetime = ros::Duration();

    vertices_of_semantic_label[i] = point_list;
  }


  const auto vertices = boost::vertices(graph);
  for (auto iter = vertices.first; iter != vertices.second; ++iter) {
    const x_view::VertexProperty& v_p = graph[*iter];
    const int semantic_label = v_p.semantic_label;

    // Set the 3D location of the semantic entity.
    geometry_msgs::Point vertex;

    vertex.x = v_p.location_3d[0];
    vertex.y = v_p.location_3d[1];
    vertex.z = v_p.location_3d[2];

    // Store the vertex in the corresponding point list.
    vertices_of_semantic_label[semantic_label].points.push_back(vertex);
  }

  // Publish all lists separately.
  for(const auto& v : vertices_of_semantic_label)
    vertex_publisher_.publish(v.second);

}

void GraphPublisher::publishEdges(const x_view::Graph& graph,
                                  const ros::Time& time) const {

  visualization_msgs::Marker line_list;
  // All coordinates are expressed in the world frame.
  line_list.header.frame_id = "/world";
  line_list.header.stamp = time;

  line_list.ns = "semantic_graph_edges";
  line_list.id = 0;

  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.action = visualization_msgs::Marker::ADD;

  line_list.scale.x = 0.1;

  line_list.color.r = 1.0;
  line_list.color.g = 189.f / 255.f;
  line_list.color.b = 50.f / 255.f;
  line_list.color.a = 0.7f;

  line_list.lifetime = ros::Duration();

  const auto edges = boost::edges(graph);
  for (auto iter = edges.first; iter != edges.second; ++iter) {
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

    line_list.points.push_back(p_from);
    line_list.points.push_back(p_to);
  }

  edge_publisher_.publish(line_list);

}

}

