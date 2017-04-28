#include <gtest/gtest.h>

#include <glog/logging.h>

#include <x_view_core/features/graph.h>
#include <x_view_core/datasets/synthia_dataset.h>

#include <vector>

using namespace x_view;

TEST(XViewSlamTestSuite, test_graph) {

  global_dataset_ptr = std::make_shared<SynthiaDataset>();

  typedef Graph::VertexProperty Vertex;
  typedef Graph::VertexDescriptor VertexDescriptor;

  Graph g;
  auto& graph = g.graph();
  const int num_desired_vertices = 5;
  std::vector<Vertex> vertices;
  std::vector<VertexDescriptor> vertex_descriptors;

  for (int i = 0; i < num_desired_vertices; ++i) {
    vertices.push_back({i, global_dataset_ptr->label(i), 0, cv::Point(1, 2)});
    vertex_descriptors.push_back(boost::add_vertex(vertices.back(), graph));
  }

  // print the vertices
  g.printVertices();

  std::vector<std::pair<int, int> > edgeList = {
      {0, 1},
      {1, 2},
      {2, 0},
      {2, 3},
      {3, 4},
      {4, 0}
  };

  int edgeIndex = 10;
  for (auto edge : edgeList) {
    boost::add_edge(vertex_descriptors[edge.first],
                    vertex_descriptors[edge.second],
                    {edge.first, edge.second},
                    graph);
  }

  g.printEdges();

  g.print(std::cout);

}

