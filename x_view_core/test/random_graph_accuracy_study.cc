/**
 * \brief This function generates multiple random graphs with random topology
 * and studies the
 * \param seed
 */
void randomGraphAccuracyStudy(const unsigned long seed) {

  std::vector<int> extraction_radius_vec{1, 2, 3, 4};
  // Add vert, connect, rem. vert., add edge, rem. edge
  std::vector<std::tuple<int, int, int, int, int>> modifier_params
      {std::make_tuple(0, 0, 0, 0, 0),
       std::make_tuple(10, 2, 0, 0, 0),
       std::make_tuple(50, 2, 0, 0, 0),
       std::make_tuple(10, 2, 10, 0, 0),
       std::make_tuple(50, 2, 50, 0, 0),
       std::make_tuple(10, 2, 0, 10, 0),
       std::make_tuple(10, 2, 0, 50, 0),
       std::make_tuple(50, 2, 0, 10, 0),
       std::make_tuple(0, 0, 0, 10, 0),
       std::make_tuple(0, 0, 0, 10, 10),
       std::make_tuple(0, 0, 0, 50, 50),
       std::make_tuple(50, 2, 50, 50, 50)
      };

  std::vector<int> num_walks = {10, 100, 200, 500};
  std::vector<int> walk_lengths = {1, 2, 3, 4};

  std::cout << std::setfill(' ');
  std::cout << std::right << std::setw(12) << "radius"
            << std::right << std::setw(12) << "add vert."
            << std::right << std::setw(12) << "connects."
            << std::right << std::setw(12) << "rem. vert."
            << std::right << std::setw(12) << "add edges"
            << std::right << std::setw(12) << "rem. edges"
            << std::right << std::setw(12) << "num. walks"
            << std::right << std::setw(12) << "walk len."
            << std::left << std::setw(12) << "  accuracy" << std::endl;

  for (auto extraction_radius : extraction_radius_vec)
    for (auto modifier_param : modifier_params)
      for (auto num_walk : num_walks)
        for (auto walk_lengh : walk_lengths) {
          // Define parameter for generating the graphs.
          GraphConstructionParams construction_params;
          construction_params.num_vertices_ = 500;
          construction_params.edge_probability_ = 0.001;
          construction_params.num_semantic_classes_
              = global_dataset_ptr->numSemanticClasses();
          construction_params.seed_ = seed;

          // Define parameters for modifying the graphs.
          GraphModifierParams modifier_params;
          modifier_params.num_vertices_to_add_ = std::get<0>(modifier_param);
          modifier_params.num_links_for_new_vertices_ =
              std::get<1>(modifier_param);
          modifier_params.num_vertices_to_remove_ = std::get<2>(modifier_param);
          modifier_params.num_edges_to_add_ = std::get<3>(modifier_param);
          modifier_params.num_edges_to_remove_ = std::get<4>(modifier_param);

          GraphPair
              graph_pair_random = generateRandomGraphPair(construction_params,
                                                          modifier_params,
                                                          extraction_radius);

          // Define parameters for random walk extraction.
          RandomWalkerParams random_walker_params;
          random_walker_params.walk_length_ = walk_lengh;
          random_walker_params.num_walks_ = num_walk;
          random_walker_params.random_sampling_type_ =
              RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;

          GraphMatcherPtr graph_matcher_ptr =
              CAST(GraphMatcher::create(random_walker_params,
                                        VertexSimilarity::SCORE_TYPE::HARD),
                   GraphMatcher);

          CHECK_NOTNULL(graph_matcher_ptr.get());

          // Add the base graph to the matcher.
          auto ignore_result =
              graph_matcher_ptr->match(graph_pair_random.base_graph_);

          // Match the subgraph to the entire graph.
          auto matching_result =
              graph_matcher_ptr->match(graph_pair_random.sub_graph_);

          // Retrieve the similarity matrix.
          Eigen::MatrixXf random_similarity =
              CAST(matching_result,
                   GraphMatcher::GraphMatchingResult)->getSimilarityMatrix();

          const float accuracy =
              similarityAccuracy(graph_pair_random, random_similarity);

          std::cout << std::right << std::setw(12) << extraction_radius
                    << std::right << std::setw(12)
                    << modifier_params.num_vertices_to_add_
                    << std::right << std::setw(12)
                    << modifier_params.num_links_for_new_vertices_
                    << std::right << std::setw(12)
                    << modifier_params.num_vertices_to_remove_
                    << std::right << std::setw(12)
                    << modifier_params.num_edges_to_add_
                    << std::right << std::setw(12)
                    << modifier_params.num_edges_to_remove_
                    << std::right << std::setw(12) << num_walk
                    << std::right << std::setw(12) << walk_lengh
                    <<"  "<< std::left << std::setw(12) << accuracy
                    << std::endl;
        }
}