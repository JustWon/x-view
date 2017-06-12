#include "test_common.h"

#include <boost/graph/connected_components.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/linear_congruential.hpp>

#include <glog/logging.h>

namespace x_view_test {

x_view::Graph generateRandomGraph(const GraphConstructionParams& params) {

  typedef boost::erdos_renyi_iterator<boost::minstd_rand,
                                      x_view::Graph> ERGen;

  boost::minstd_rand gen(params.seed_);
  std::mt19937 rng(params.seed_);
  std::uniform_int_distribution<int> dist(0, params.num_semantic_classes_ - 1);
  // Create random graph.
  x_view::Graph graph(ERGen(gen, params.num_vertices_, params.edge_probability_),
                      ERGen(), params.num_vertices_);

  // Add properties to the vertices.
  auto vertex_iter = boost::vertices(graph);
  int vertex_index = 0;
  // Iterate over all vertices.
  for (; vertex_iter.first != vertex_iter.second; ++vertex_iter.first) {
    auto& vertex = graph[*vertex_iter.first];
    vertex.index_ = vertex_index++;
    vertex.size_ = 1;
    vertex.center_ = cv::Point(0, 0);
    // Set a random semantic label to the vertex.
    vertex.semantic_label_ = dist(rng);
    vertex.semantic_entity_name_ = std::to_string(vertex.semantic_label_);
  }

  // Add edge properties.
  auto edges_iter = boost::edges(graph);
  for (; edges_iter.first != edges_iter.second; ++edges_iter.first) {
    // Get the vertex descriptors defining the current edge.
    const auto& from_v = graph[boost::source(*edges_iter.first, graph)];
    const auto& to_v = graph[boost::target(*edges_iter.first, graph)];
    // Set the edge properties.
    graph[*edges_iter.first].from_ = from_v.index_;
    graph[*edges_iter.first].to_ = to_v.index_;
  }

  std::vector<int> component(boost::num_vertices(graph));
  int num_connected_components =
      boost::connected_components(graph, &component[0]);

  if (num_connected_components == 1) {
    LOG(INFO) << "Random graph generated with parameters:"
              << "\n\tnum_vertices         : " << params.num_vertices_
              << "\n\tedge_probability     : " << params.edge_probability_
              << "\n\tnum_semantic_classes : " << params.num_semantic_classes_
              << "\n\tseed                 : " << params.seed_;
    return graph;
  } else {
    LOG(WARNING) << "Random graph generated with parameters:"
                 << "\n\tnum_vertices         : " << params.num_vertices_
                 << "\n\tedge_probability     : " << params.edge_probability_
                 << "\n\tnum_semantic_classes : "
                 << params.num_semantic_classes_
                 << "\n\tseed                 : " << params.seed_
                 << "\nis not a single connected component."
                 << "\nGenerating new graph with modified parameters.";

    // Generate new parameters for graph generation.
    GraphConstructionParams new_params;
    new_params.num_vertices_ = params.num_vertices_;
    new_params.edge_probability_ =
        std::min(1.1f, params.edge_probability_ * 1.1f);
    new_params.num_semantic_classes_ = params.num_semantic_classes_;
    new_params.seed_ = params.seed_;
    return generateRandomGraph(new_params);
  }
}

x_view::Graph generateChainGraph(const GraphConstructionParams& params) {

  std::mt19937 rng(params.seed_);
  std::uniform_int_distribution<int> dist(0, params.num_semantic_classes_ - 1);

  x_view::Graph graph;
  std::vector<x_view::VertexDescriptor> vertex_descriptors;
  // Create the vertices of the graph in sequence.
  for (int i = 0; i < params.num_vertices_; ++i) {
    x_view::VertexProperty v_p;
    v_p.index_ = i;
    v_p.center_ = cv::Point(0, 0);
    v_p.semantic_label_ = dist(rng);
    v_p.semantic_entity_name_ = std::to_string(v_p.semantic_label_);
    v_p.size_ = 0;

    vertex_descriptors.push_back(boost::add_vertex(v_p, graph));
  }
  // Add edges between each pair of consequent vertices.
  for (int i = 0; i < params.num_vertices_ - 1; ++i) {
    boost::add_edge(vertex_descriptors[i], vertex_descriptors[i + 1],
                    {i, i + 1}, graph);
  }
  // Close the loop.
  boost::add_edge(vertex_descriptors.back(), vertex_descriptors.front(),
                  {params.num_vertices_ - 1, 0}, graph);

  return graph;
}

x_view::Graph extractSubgraphAroundVertex(const x_view::Graph& original,
                                          const x_view::VertexDescriptor& source,
                                          const int radius) {

  // An object mapping each VertexDescriptor to the corresponding integer
  // distance to the source VertexDescriptor. This object is filled up by the
  // KNNBSVisitor class.
  KhopVisitor::DistanceMap dist;

  // Perform BFS in order to compute the integer distance from each vertex of
  // the original graph from the source VertexDescriptor.
  KhopVisitor vis(dist);
  boost::breadth_first_search(original, source, boost::visitor(vis));

  // Map which assigns two VertexDescriptors to each other. This is
  // used to keep track of which vertex is assigned to which vertex between
  // the original and the newly generated graph.
  typedef std::map<const x_view::VertexDescriptor,
                   const x_view::VertexDescriptor> VertexToVertexMap;
  VertexToVertexMap old_vertex_to_new;

  // Extracted graph.
  x_view::Graph extracted_graph;

  // Add the selected vertices to the new graph.
  for (const auto& p : dist) {
    const int distance = p.second;
    // Only add the vertex if its distance to the source is smaller or equal
    // to the radius passed as argument.
    if (distance <= radius) {
      const x_view::VertexDescriptor& v_old_d = p.first;
      // Deep copy of the VertexProperty.
      const x_view::VertexProperty v_p = original[v_old_d];
      const x_view::VertexDescriptor& v_new_d =
          boost::add_vertex(v_p, extracted_graph);
      // Keep track of the newly added vertices by mapping the old ones to
      // the newly inserted.
      old_vertex_to_new.insert({v_old_d, v_new_d});
    }
  }

  // Add the edges between the selected vertices if an edge was present in
  // the original graph.
  for (auto it = old_vertex_to_new.begin(); it != old_vertex_to_new.end(); ++it)
    for (auto jt = std::next(it); jt != old_vertex_to_new.end(); ++jt) {
      const x_view::VertexDescriptor v_old_i_d = it->first;
      const x_view::VertexDescriptor v_old_j_d = jt->first;
      const x_view::VertexDescriptor v_new_i_d = it->second;
      const x_view::VertexDescriptor v_new_j_d = jt->second;

      const auto edge = boost::edge(v_old_i_d, v_old_j_d, original);
      if (edge.second) { // If the edge was present in the original graph.
        const x_view::EdgeProperty e_p = original[edge.first];
        boost::add_edge(v_new_i_d, v_new_j_d, e_p, extracted_graph);
      }
    }

  LOG(INFO) << "Created subgraph centered on vertex " << original[source]
            << " with 'radius' of " << radius << ".\nIt has "
            << boost::num_vertices(extracted_graph) << " vertices and "
            << boost::num_edges(extracted_graph) << " edges.";

  return extracted_graph;
}

}

