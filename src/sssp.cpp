/**
 * @file sssp.cpp
 * @author Muhammad Osama (mosama@ucdavis.edu)
 * @brief
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <vector>
#include <string>
#include <iostream>

#include <load.hxx>
#include <validate.hxx>
#include <sssp.hxx>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: ./bin/<program-name> filename.mtx" << std::endl;
    exit(1);
  }

  // --
  // Define types

  using vertex_t = int;
  using edge_t = int;
  using weight_t = float;

  // --
  // Build graph

  std::string filename = argv[1];
  sssp::matrix_market_t<vertex_t, edge_t, weight_t> mm(filename);
  sssp::graph_t<vertex_t, edge_t, weight_t> graph(
      mm.num_rows, mm.num_columns, mm.num_nonzeros, mm.Ap, mm.Aj, mm.Ax);

  // --
  // Params and memory allocation

  vertex_t n_vertices = graph.get_num_vertices();
  vertex_t single_source = 0;

  std::vector<weight_t> distances = sssp::sssp(graph, single_source);

  // --
  // Validate

  int n_errors =
      sssp::validate(n_vertices, graph.row_offsets, graph.column_indices,
                     graph.values, single_source, distances);

  std::cout << "Distances : ";
  for (const auto& d : distances)
    std::cout << d << ' ';
  std::cout << std::endl;

  std::cout << "Number of errors : " << n_errors << std::endl;
}
