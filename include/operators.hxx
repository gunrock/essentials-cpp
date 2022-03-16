/**
 * @file operators.hxx
 * @author Muhammad Osama (mosama@ucdavis.edu)
 * @brief Operators for graph analytics.
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <vector>
#include <algorithm>
#include <execution>
#include <mutex>
#include <utility>
#include <ranges>

/**
 * @brief Neighbors expand operator for graph traversal.
 *
 * @par Overview
 * Takes an input frontier and returns the neighbors of the active vertices
 * within the frontier as a new frontier. During the traversal to neighbors,
 * also applies an expand condition on the tuple of source vertex, neighbor
 * vertex, edge and edge weight.
 *
 * @tparam my_graph_t Graph type.
 * @tparam my_frontier_t Frontier type.
 * @tparam expand_cond_t Condition type.
 * @param g Graph.
 * @param f Frontier.
 * @param condition User-defined condition.
 * @return my_frontier_t New frontier.
 */
template <typename my_graph_t, typename my_frontier_t, typename expand_cond_t>
my_frontier_t neighbors_expand(my_graph_t& g,
                               my_frontier_t& f,
                               expand_cond_t condition) {
  std::mutex m;
  my_frontier_t output;
  auto expand = [&](auto const& v) {
    // For all edges of vertex v.
    for (auto e : g.get_edges(v)) {
      auto n = g.get_dest_vertex(e);
      auto w = g.get_edge_weight(e);
      // If expand condition is
      // true, add the neighbor into
      // the output frontier.
      if (condition(v, n, e, w)) {
        std::lock_guard<std::mutex> guard(m);
        output.add_vertex(n);
      }
    }
  };

  // For all active vertices in the
  // frontier, process in parallel.
  std::for_each(std::execution::par, f.active_vertices.begin(),
                f.active_vertices.end(), expand);

  // Return the new output frontier.
  return output;
}