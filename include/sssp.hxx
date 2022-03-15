/**
 * @file sssp.hxx
 * @author Muhammad Osama (mosama@ucdavis.edu)
 * @brief Single-Source Shortest Path (SSSP) implementation in modern C++ for
 * 2022 IPDPS workshop on Graphs, Architectures, Programming, and Learning
 * (GrAPL 2022) submission.
 *
 * @par Overview
 * Goal of this code is to implement a single-source shortest path (SSSP)
 * algorithm presented in the paper using modern C++ and show-case the use of
 * the abstraction and essential elements proposed within the paper for parallel
 * graph analytics. You can find the paper at IPDPS
 * https://ieeexplore.ieee.org/.
 *
 * GodBolt full link:
 * https://godbolt.org/#z:OYLghAFBqd5QCxAYwPYBMCmBRdBLAF1QCcAaPECAMzwBtMA7AQwFtMQByARg9KtQYEAysib0QXACx8BBAKoBnTAAUAHpwAMvAFYTStJg1DIApACYAQuYukl9ZATwDKjdAGFUtAK4sGIAMwAnKSuADJ4DJgAcj4ARpjEEtIADqgKhE4MHt6%2BAcGp6Y4C4ZExLPGJUraY9kUMQgRMxATZPn5B1bWZDU0EJdFxCUm2jc2tuR0Ko30RA%2BVDUgCUtqhexMjsHOb%2BEcjeWADUJv5uAG6YDiTH2CYaAILbu/uYRydiwCSECCzXtw9mOwYey8h2ObkwqguXjqv3ujyBz1ebhY0IhsP%2BgOBoJO0LohAAnuj4ViXmDiIZgJgFET7lNiF4HAcqMRZHgEgB9AhHADsVnuBwFBwA9EKDnIGFhiLR8RFgAdiJhkgqlIImHUDqgqEyWYI2cQAHR/QUHKboEAgc6XYhgiIEa4HJgOPDndnnZp4DbU/x8u7Go2CkUHADimC5BAQLwYg2IGq1jsc5wObscnsN/MFtpNeAAXpgIIseT7jcaFQQ1gwHU6XcmPVT9elc/njkXBSZuQARf0CwMhsMRysJl7JiEOrlMA7AZ2MA4RLCqNO%2BjOCCeh9nxqeuhIECEQTNoBhTcwANhnBbbLeL8tD5YHG5rnpMAFYLHgn53vV2eZ3093RXd0OgDpJluI5EAc4YvMyrIJAuxqnKgeCAUwAGbs0O57gIh5mCepxnryn7Guu1ZbrWCj6skXgKAg7KxI6ADWEC4c2n5tt%2BDwdsxcL3IGHgsEqVJKOgAC0QjJE0SgHAASqgADuBwQG4QiSQWLBqsQeDzn8dIMlyyAKMQnKFp%2BmYsjJXoXjOy5oLQ5mfqa5qWkQ1onLa9qmeympUEoBC2T%2BJoEGaFoXE5NqCPa1ltOys6kZxi4CvZQVWmCVC0Kgar2qcYheFSsWsbltIEPSjLAOSyTUVyIAHBRsS0B6Bx6QZXLnp%2BPahgcmDoJSBwyZgeDAAgXL8DG46TucFYdZSsGCilaVcpSBDshNmDsj1fUDbuVmYXa2HtXhFkltexAVpl3g5c%2BmBvrFfocVxcWWXNq5RiwqEplS%2BaFleZZHfKsnmV%2BxnLvN7JYFML07vuUz3ceu0faWN4Rb4UUSjF52Xd6/1%2BUw0KoCuC1LQoEAQ1yrk7bhsOHRWCXkkYVLmk4jSumyMnnu5nnedSz6nG%2BpA/TJHlUF5oYcxYXOWFwl0cejeWY9juMvaR73np9N5UxStMgPTTCM5gzO8hoPNA098uek2ksWBj7Hvj6fyBlEvX9bEJAKO1qhiRKM58fQbCCB1lkHG41jWAcZgaAu26e2qF0nAQ%2BLJIwrAvBCbvoOy%2B4p3a/g3PcUG6hyXKRGtjvEAoi2u4Y6AQJ%2BJVMGVnLQ8APOfjnjh59DVCN35Sfl6nAjp/VvcZAIfx7XZAXmii26qAcPwfn5zd6oZqwEBRGcWVjYFd%2B7xztkcz7Hm%2B8kYQe204SPfnGoGABiJAOrQtDtZ1VKxsBaFT6cU2XkN8nrzjLyVcA%2Bogb40YosM%2Bd1LwCh/gcCs28JyANXCDBaw5VAQEwIsK6EDCKyzkrAgBQDH4rXtutNBGDMG/gOAASS1JvQCadB4VjwNSc%2BEDAyFWyjzZCgEILQKIUXe6qACIsNFNwpeK9tTQQNIIy8eAtSEwHnURiPMGA80wDzGSoCjLMLIf5QKqVkB0XZMALwTR0BggShPNEmcjhaO0YKIxJiIAsHQbPcBtjRHQn1JwsGKCGDOP2hA6WriBSBOulbP4LVRTX2GnfW8iZ7xPwiOBCMETxG5zIFVFknpnaJLEuSO%2BNQP46PNENRajoEAQAShCKEdRzS5I7kEpkniqzLXiWReIk4GD5nqZgqgTTBzGzrK4Lp1iGk0L8SxPygZJIUySZGHWGpoRiPnjBT8cNvruNXuEm6Dx7jMDYAoMSGxRyoBYHVZqflw7JAMNuMEMc457JeAAFXRMaR508IgQEeQAKgdDzN5sQwGYLeZ4QCsCvlMFIcWN5hhnbb3MR8kFPMAWQuNOC14O8YUosFGsisILIWBIJQVQKjkrgnBmulKxCgqXJErjY401da4nwnKQKRxYj5YRPAoVY6wLr3EBQGUUFCGAZDEDmF46A1RMEKQlElzk3DkoztgA4%2BApiGBNngx6PgBkE1AZCr%2BEAoGnAOP/eBSCSIm11TYlVjQgRnRFgfWBCUnoJA9OyWqpyfLJVShS7A48mAoPGX5a1aq7VcrWBsB1/gd4aD1TqFujUmR6s8ShZBFTuUbEDTsu6MrgqkrcOY1Eqh7TPT0XRAmGqFpG1afmTNn4ZIIDoHmXpDY8wFjAGAWBGh%2BWXkDNgMu7tuHLMkTYrUsCC4OydqXZO0Aebt1ZYGRQCQhJYBoJEWh8jMhMhvkIHdyh9QjMwU%2BKw2E3wbV0ltaG%2BlkA8wvqKMNPLWXGnZUy9AUwb0CuVVSRwzB1SPqXOe4%2B0MlrvuFKKJaf6BQKv7oBnaq1%2BpckDHBgarLu1kKg5EPmoKo3KsYTah8z4r0OssN1Ihmy6UfrVCcj049EmUdOaIO%2B%2BIDheGSBK7cztuHBttc7Jo5ImNqmVZDGSXxZnvOFSiFgz8MPAw1DGQgztkBrAVMuE62V913Hdjijj/YQVJiypgfdqCACOlULFTxY2xqk6DyOQe9bpJTMnYF0eoyAU5nTjxcfwxYV9dpHztiUTrYGmbbHK2%2BtJrDbh6oOdMS4w9HZgutm2QdL6FZPM5RcaxDgyxaCcEfLwPwHAtCkFQJwAOlgg73qOdsHgpACCaCy8sOiIBHz6xyxwSQ%2BX6vFc4LwBQIB9Z1cK1l0gcBYAwEQCgE5yRG1kAoHIviM2QDAC4FwMwfA6DbmLpQWIXXYgRCaPiTgNW0AsG9gQAA8gwaUXWsCqSMOIIbpB8AKmaX1x7VTFPbiO7wW0NQuu1ViOSYg%2BIPBYC64VPALBvvLBSkwYACgABqTNzv3O%2BzIQQIgxDsCqPwQQi61Bdd0FwfQFIUCB0sPoPAsQ%2BuQGWKgZIdQ3tCVNNvUw5XLAh16zUYKmQXASnGH4YnYRZhlAqHoAo9CBfi7SPQ/oouFidB5wIHoYxPBtD0HYJX9Rphy%2BjBr6YUviequaLr%2BYlRliVex7VhUmweDZdy51x7JWOCqAABxHiEkeSQE5kDIAOCt/UZh5JlePTYA4uBCA32qzzXi036Axmq4sXgg2tCgNIE1lr%2BhOAddIFDzPBWivO96/12r9XlijYmxs8glATtx4Vx1SPlR0fCAY9j6QuP5BKAJ49wqmBbe8BkqVaHWeOB5dIAX3gzvzuLOhM/N3Huvc%2B79wHoPCkpszaOACLgSfS9DbTxGZCQwultZz3n/WE/uscGLwNsvjXmutc4P4R3heeu79T/bjgZhn%2BT9fynhrpA3R0hnBJAgA%3D%3D%3D
 *
 * GodBolt (overloaded) full link:
 * https://godbolt.org/#z:OYLghAFBqd5QCxAYwPYBMCmBRdBLAF1QCcAaPECAMzwBtMA7AQwFtMQByARg9KtQYEAysib0QXACx8BBAKoBnTAAUAHpwAMvAFYTStJg1DIApACYAQuYukl9ZATwDKjdAGFUtAK4sGIaa4AMngMmAByPgBGmMQgAJxcpAAOqAqETgwe3r7%2ByanpAsGhESzRsQm2mPaOAkIETMQEWT5%2B0naYDhl1DQRF4VEx8YkK9Y3NOW2jvSH9pYMJAJS2qF7EyOwc5gDMIcjeWADUJltuAG4dRMTH2CYaAILbu/uYRydiwCSECCzXtw9mOwYey8h2Obkwqg6XhqDF%2B90eQOerzcLGhELh/0BwNBJ2hdEIAE8MQjsS8wcRDMBMApiQCniCyScCASkpgAPoECmEGlbG7w%2B4jYheBwHKjEWR4GIco4Adis9wOioOAHplQc5AwsMRaASQsADsRMElDUpBEwYQdUFRReLBJLiAA6P5Kg4jdAgEDnBwkMEhAjXA5MTrnNnnRp4dY8%2BV3F3OpWqg4AcUwBAOBAQLwYA2IlutQcc5wOYcckadCqVftdeAAXpgIAtZdGXS7DQRVgxA8H2cWI9SHWla/Xjk2lSYZQAROOKhPJ1Ppl75vCF4sQwOppgHYBLxgHEJYVRlmMVwSblNsxchleqCCVtAMEbmABsu4bY5HzYNKfbnYL3ZiJepEwAFYLDwYDJy2d9ZUnctpzVO50HQQMi3/VciDTDMbQlGJDxdU5UDwJCmEQ0NUOvW8BAfMxn1OV85SnZsLz/cNSySLwFAQNlIiDABrCBaOHBixxgh4J0E/k7gTDwWGNaklHQABaIQkgaJQDgAJVQAB3A4IDcIR1IbFhzWIPADz%2BQVhVTZAFGIaU3wYytxS0qNHJPNBaFc2DXQId1PQuH0Tj9ANnLZK0qCUAgvKPRU3Q9L1Ll9QQAw8lo2T3XtopdOL/O9K4TioWhUHNANTjELxAMgv5hPEh4BU5KzNwpJJONTEADjYyJaAjA4bLs1MHO8mcUwOTB0CpA4tMwPBgAQVN%2BBzDct3ODsxqpXClUK4rUypAg2TW9kppmuab3cyj/Wo0a6Kglsv2IDsyu8QCQMwcDatjMSJJdStdrZLMWFIljqXrRtPzbe6DW0qNoLcnazywEZAYICEIDvEZd2Sy7MGusHv1S3x0s1TLgIsV6gIgiwYe8phoVQU89oOhRUfOjGLponHW2/HKKSMakPScepQ0lLS31C8LIppEDTnA0hIa0sKqAilNJYsaXLC4N6xMgqmYsDWn6aRzKQbfXGIe5yk%2BZAAWmCFzARblDRZd%2B/7DcjIctcpmr4Q96r7gTMJptmyISAUUbVBUzVdxk%2Bg2EEMaMYONxrGsA4zA0Q9kej81XqZFlGFYF4IQj9A2TvEuCFl5lWWYNgOs8CMCQ5WWcvzrr2TwKgOTBHK8AUNkFAL0MwRSbrkEbiuDiwPYq7rFvIWQPFnBAFTiAWa5ZeC3lXnHA4NAxMVsP6g5QmO4PiD7ovDHQCAGJHhvpSSUgGOAZrWqfTdZYYg%2B7SlNnRU/7yl9NSlwEOXXqoCCiwnuNdBiOVUTI1UAcH4VVvLf0cL/S00I2L%2BhQbrGm6EgFIWODvEmT5wK6QoveP%2BtFGwMRdAmAAYiQQMtBaCjXGtSXMKFGirlOBtD8C1dL4Lpi8dqwAHS/UZvxBYMDvIfkVMI4%2B29NwSPhtSPaV4IDY3evIl0iidLEJUZIjhbIjqzQIFoteuDdH0LVAASWtIQ8BRMLS9zofIhMDVMCy2Ikhecx9A4IDPqzVA7iPyeMwisAg2CsI/0dGE5sHddJl0gfxWWDBZbeMmjI2hcibGxV8h6IqyAeJsmAF4Bo6Bu6FJAPA9EW8En5MVOUypEAWBWJuk0qJ2CHS%2BKRijBgHTGmjk%2BrrD6Ilxm1QYow5hYg2FMW4QBUOIQMKvSGmqNB9pZbGlQJGZZHYV5zKqPwnyfkFr7SDAgCAj8jh5MVFQXpXZXZ9miFuBg9YAFjM2o838zyFAOlcB825XzFSEKGd5aZap1J3Q7P40IOlunQlieg%2BJ3lOYQ0RTg6MXs6p3EzkkAwyMwQzxroXcOV8QHAIniSguddR7j2bjU1u9B0qdxwW4MAYAe59wHmwIeJw75jybpPDotAZ4QDnlCGEHoV5WOwBvZKDStg7z3ryP4myMEnyDiHfa5LNQ328oK8eHUn7eRfkwFqXdLrAE%2BS6DV/V35UFtUqQhlKwEpJhH8GBoy/ikoUCpdYa5UAsB6oNXW%2BLCU5zcDS2uAAVDELpY1IJCBAWNAAqQMssk2RFkSCg4SbPBEOVQcNNTAdEfiTYYUOxC4EpsLbLHN5bmyluUVWptt1wYdkLU2nFvb6p%2BQSoFNwW0SpbwUOOpIBq80unNZav%2BNrhmKkoVRZ8CgVhrDWXcXNtiDh2IYOkMQNYXjoHNEwE5OVB35WHUVUd2BJ693qECYG4jnY%2BD%2BfWcFutBEQEUacA4YjVEaP/EbGRCT8AjEMJGEm0tybKJyv9GIEY2TdRDVFMEI6cHYA9MZa8n6XTgcfVBkCa7VjrHIYY1VUF7XSioE2h5fTNEkY3Z%2B2BNTL3VL8nU1QAYAbFJ4kzF9Z4XY9jdqB3BDEtIIDoHWB5A46wNk5RR7dHi1TYD1X4zC9qTl2uUVqoJOrCESqZfPRefhl4NCdrLJ1CSEyKBiApLANBQhIQ9RkUUzChCeeUA6YFujSHUXAqday51362WQLLHdTGyN3MSWdKh790AjAi/GNUCNHDMAtIu1mziV3sKpMllUaoDpZYwzlv%2BZi5qFcmoE/0MXlP5NK/CtkRad4Ecg89CwYXyOWGq8dLFWWEzmmDRGbDKyhshtEKwgkBwvBJBPcjUO/i2tPtDg0Ck03zST3RlpL4qzk37tRCwLhTWkLMO5L1VYhoTyPQqj5u4kd0X3j24Wos5VMA%2Ba0QAR3alxmbc3s4KDXjFu1N7rKXea62ogE3RvvKfMtojFhEv%2BnJuku2zW8NNMewE%2BWRa3AXeIHZKp1j5HCQx4qHFHbvzw8qtiicHAli0E4EBXgfgOBaFIKgTgSdLApyi2SAEPBSAEE0PTpYPEQCPnTlwMwZguCPkkBoSQAAOJXj4gKPjMPoTgkgWci455wXgCgQCO2F2z%2BnpA4CwBgIgFAwakjSbIBQZmMkHcgGAFwaXfA6DI3PpQSIevIghAaASTggu0AsFjgQAA8gwHUeusDGSMOIM3pB8CGi7EblPEIpUbEF36KoevuqRApMQAkHgsB685HgFgofzeFSYMABQAA1YWUfq615kIIEQYh2BSE7/IJQag9e6ESAYIwKBk6WH0HgSIRvIBLFQEkGEmeFJumIaYHnlg06G6qBcDILhNTjD8IkIIMwShlD0CkNIMIj%2BX/yDCPo5/BjDF350WoUxb8v%2BqF0KYj/sx6AgzGE8BaAAN/zP3/y4CWD5z0E5EwFz3N0Zw4GZ1IFZ3Z05w4FUFVwUnl03GQGQAOA9wdDMF0m5ysCnwOFwEIGYW2ESETjtwdyOAFwWF4FNy0BkVIHFyAkdkQJ11IBry4JQL13QMN2NyFxFyWEtxt0xXIEoHD3t3oFiBP3wEuD0H4C70m172kDUIHxUHUBT1gPgNIC0malrwZyZ11xT3QKjywSRStAOEwMfGwMkFwPwMIOIL0noIUMYK2EgJYPEKWAzGIkGA%2BR4N4H4MdlQN4GENsFENYNFw4JAC2DiAdCVxlBlCVy2CAjiCAg9y2EkC2CVy1w4C2AsLQINzELN3YMQLMFKKiPKLiPYLDDSGcEkCAA%3D
 *
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

namespace sssp {

template <typename type_t>
struct frontier_t {
  // Underlying representation of frontier.
  std::vector<type_t> active_vertices;

  // Get the number of active vertices.
  int size() { return active_vertices.size(); }
  // Get the active vertex at a given index.
  type_t get_active_vertex(int const& i) { return active_vertices[i]; }
  // Add a vertex to the frontier.
  void add_vertex(int const& v) { active_vertices.push_back(v); }
};

// Compressed-Sparse Row (CSR) matrix.
template <typename index_t, typename offset_t, typename value_t>
struct csr_t {
  index_t rows;
  index_t cols;
  offset_t nnzs;
  std::vector<offset_t>& row_offsets;
  std::vector<index_t>& column_indices;
  std::vector<value_t>& values;

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

template <typename vertex_t, typename edge_t, typename weight_t>
struct graph_t : public csr_t<vertex_t, edge_t, weight_t> {
  using csr_type = csr_t<vertex_t, edge_t, weight_t>;
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

// Neighbors expand implemented in C++ 20.
template <typename my_graph_t, typename my_frontier_t, typename expand_cond_t>
my_frontier_t neighbors_expand(my_graph_t& g,
                               my_frontier_t& f,
                               expand_cond_t condition) {
  std::mutex m;
  my_frontier_t output;
  auto expand = [&](int const& v) {
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

namespace atomic {
template <typename T, typename lock_t>
T min(T* a, T b, lock_t& m) {
  std::lock_guard<std::mutex> guard(m);
  T old = *a;
  T ans = std::min(old, b);
  *a = ans;
  return old;
}
}  // namespace atomic

template <typename my_graph_t, typename vertex_t>
std::vector<float> sssp(my_graph_t& g, vertex_t const& source) {
  // Initialize data.
  std::vector<float> distances(g.get_num_vertices());
  for (auto v : g.get_vertices())
    distances[v] = std::numeric_limits<float>::max();
  distances[source] = 0;
  frontier_t<vertex_t> f;
  f.add_vertex(source);

  std::vector<std::mutex> m_locks(g.get_num_vertices());

  while (f.size() != 0) {
    // Expand the frontier.
    f = neighbors_expand(g, f,
                         // User-defined condition for SSSP.
                         [&](int const& src,      // source
                             int const& dst,      // destination
                             int const& edge,     // edge
                             float const& weight  // weight
                         ) {
                           float new_d = distances[src] + weight;
                           // atomic::min atomically updates the distances array
                           // at dst with the minimum of new_d or its current
                           // value. And returns the old value. (eq: mutex
                           // updates)
                           float curr_d = atomic::min(&distances[dst], new_d,
                                                      m_locks[dst]);
                           return new_d < curr_d;
                         });
  }
  return distances;
}

}  // namespace sssp