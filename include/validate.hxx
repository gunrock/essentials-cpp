/**
 * @file validate.hxx
 * @author Muhammad Osama (mosama@ucdavis.edu)
 * @brief Validation code for SSSP.
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <chrono>
#include <queue>
#include <vector>

namespace sssp {

template <typename vertex_t, typename weight_t>
class prioritize {
 public:
  bool operator()(std::pair<vertex_t, weight_t>& p1,
                  std::pair<vertex_t, weight_t>& p2) {
    return p1.second > p2.second;
  }
};

template <typename vertex_t, typename edge_t, typename weight_t>
int validate(vertex_t& n_vertices,
             std::vector<edge_t>& row_offsets,
             std::vector<vertex_t>& column_indices,
             std::vector<weight_t>& values,
             vertex_t& single_source,
             std::vector<weight_t>& distances) {
  std::vector<weight_t> ref_distances(n_vertices);
  for (vertex_t i = 0; i < n_vertices; i++)
    ref_distances[i] = std::numeric_limits<weight_t>::max();

  ref_distances[single_source] = 0;

  std::priority_queue<std::pair<vertex_t, weight_t>,
                      std::vector<std::pair<vertex_t, weight_t>>,
                      prioritize<vertex_t, weight_t>>
      pq;
  pq.push(std::make_pair(single_source, 0.0));

  while (!pq.empty()) {
    std::pair<vertex_t, weight_t> curr = pq.top();
    pq.pop();

    vertex_t curr_node = curr.first;
    weight_t curr_dist = curr.second;

    vertex_t start = row_offsets[curr_node];
    vertex_t end = row_offsets[curr_node + 1];

    for (vertex_t offset = start; offset < end; offset++) {
      vertex_t neib = column_indices[offset];
      weight_t new_dist = curr_dist + values[offset];
      if (new_dist < ref_distances[neib]) {
        ref_distances[neib] = new_dist;
        pq.push(std::make_pair(neib, new_dist));
      }
    }
  }

  int errors = 0;
  for (vertex_t i = 0; i < n_vertices; i++) {
    if (distances[i] != ref_distances[i]) {
      errors++;
      std::cout << "Error at " << i << " : " << distances[i]
                << " != " << ref_distances[i] << std::endl;
    }
  }

  return errors;
}

}  // namespace sssp