/**
 * @file datastructs.hxx
 * @author Muhammad Osama (mosama@ucdavis.edu)
 * @brief
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

namespace essentials {

/**
 * @brief Example frontier data structure.
 *
 * @tparam type_t
 */
template <typename type_t>
struct frontier_t {
  // Underlying representation of frontier.
  std::vector<type_t> active_vertices;

  // Get the number of active vertices.
  int size() { return active_vertices.size(); }
  // Get the active vertex at a given index.
  type_t get_active_vertex(type_t const& i) { return active_vertices[i]; }
  // Add a vertex to the frontier.
  void add_vertex(type_t const& v) { active_vertices.push_back(v); }
};

/**
 * @brief Compressed-Sparse Row (CSR) matrix.
 *
 * @tparam index_t Type of index.
 * @tparam offset_t Type of offset.
 * @tparam value_t Type of value.
 */
template <typename index_t, typename offset_t, typename value_t>
struct csr_t {
  index_t rows;
  index_t cols;
  offset_t nnzs;
  std::vector<offset_t>& row_offsets;
  std::vector<index_t>& column_indices;
  std::vector<value_t>& values;

  /**
   * @brief Construct a new csr_t object.
   *
   * @param _rows Number of rows.
   * @param _cols Number of columns.
   * @param _nnzs Number of non-zeros.
   * @param _row_offsets Row offsets.
   * @param _column_indices Column indices.
   * @param _values Values of non-zeros.
   */
  csr_t(index_t& _rows,
        index_t& _cols,
        offset_t& _nnzs,
        std::vector<offset_t>& _row_offsets,
        std::vector<index_t>& _column_indices,
        std::vector<value_t>& _values)
      : rows(_rows),
        cols(_cols),
        nnzs(_nnzs),
        row_offsets(_row_offsets),
        column_indices(_column_indices),
        values(_values) {}
};

/**
 * @brief Graph data structure based on CSR format.
 *
 * @tparam vertex_t Type of vertex.
 * @tparam edge_t Type of edge.
 * @tparam weight_t Type of weight.
 */
template <typename vertex_t, typename edge_t, typename weight_t>
struct graph_t : public csr_t<vertex_t, edge_t, weight_t> {
  using csr_type = csr_t<vertex_t, edge_t, weight_t>;
  using vertex_type = vertex_t;
  using edge_type = edge_t;
  using weight_type = weight_t;

  /**
   * @brief Construct a new graph_t object.
   *
   * @param rows Number of rows.
   * @param cols Number of columns.
   * @param nnzs Number of non-zeros.
   * @param row_offsets Row offsets.
   * @param column_indices Column indices.
   * @param values Values of non-zeros.
   */
  graph_t(vertex_t& rows,
          vertex_t& cols,
          edge_t& nnzs,
          std::vector<edge_t>& row_offsets,
          std::vector<vertex_t>& column_indices,
          std::vector<weight_t>& values)
      : csr_type(rows, cols, nnzs, row_offsets, column_indices, values) {}

  weight_t get_edge_weight(edge_t const& e) { return csr_type::values[e]; }
  vertex_t get_num_vertices() { return csr_type::rows; }
  vertex_t get_dest_vertex(edge_t const& e) {
    return csr_type::column_indices[e];
  }
  auto get_edges(const vertex_t& v) {
    return std::ranges::iota_view{csr_type::row_offsets[v],
                                  csr_type::row_offsets[v + 1]};
  }
  auto get_vertices() { return std::ranges::iota_view{0, get_num_vertices()}; }
};

}  // namespace essentials