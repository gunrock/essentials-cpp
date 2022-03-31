# Essentials of Parallel Graph Analytics
Single-Source Shortest Path (SSSP) implementation in modern C++ for 2022 IPDPS workshop on Graphs, Architectures, Programming, and Learning (GrAPL 2022) submission. For a more complete implementation of the ideas presented in the paper, please refer to the on-going work of graph analytics on GPUs at [gunrock/essentials](https://github.com/gunrock/essentials).

| System  | Version                                                                                                                                                       | Status                                                                                                                                                   |
|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| Ubuntu  | [Ubuntu 20.04](https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners#supported-runners-and-hardware-resources)        | [![Ubuntu](https://github.com/neoblizz/sssp/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/neoblizz/sssp/actions/workflows/ubuntu.yml)    |
| Windows | [Windows Server 2019](https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners#supported-runners-and-hardware-resources) | [![Windows](https://github.com/neoblizz/sssp/actions/workflows/windows.yml/badge.svg)](https://github.com/neoblizz/sssp/actions/workflows/windows.yml) |

## Dependencies
- `C++20` for linux (requires `gcc/g++-11` or higher), `C++23` for windows.
- `cmake` version `3.22.2`.
- `tbb` library for execution policies (automatically fetched using `cmake`).

## Implementation Detail
This code base makes use of modern C++ features such as `ranges`, `execution_policy`, and lambda expressions to implement the essential components for parallel graph analytics. We focus on a simple implementation of Single-Source Shortest Path (SSSP), but the concepts can easily be extended to support other graph algorithms such as Breadth-First Search with minor changes to the lambda expression during traversal.

### SSSP Traversal Condition
```cpp
[&](vertex_t const& src,    // source
    vertex_t const& dst,    // destination
    edge_t const& edge,     // edge
    weight_t const& weight  // weight
   ) {
     weight_t new_d = distances[src] + weight;
     weight_t curr_d = atomic::min(&distances[dst], new_d, m_locks[dst]);
     return new_d < curr_d;
};
```

### BFS Traversal Condition
```cpp
[&](vertex_t const& src,    // source
    vertex_t const& dst,    // destination
    edge_t const& edge,     // edge
    weight_t const& weight  // weight
   ) {
     // If the neighbor is not visited, update the distance. Returning false
     // here means that the neighbor is not added to the output frontier, and
     // instead an invalid vertex is added in its place. These invalides (-1 in
     // most cases) can be removed using a filter operator or uniquify.
     if (distances[dst] != std::numeric_limits<vertex_t>::max())
       return false;
     else
       return (atomic::cas(
                   &distances[dst], std::numeric_limits<vertex_t>::max(),
                   iteration + 1) == std::numeric_limits<vertex_t>::max(), m_locks[dst]);
};
```

## Quick Start Guide
Before building this project, make sure your system/compiler supports **C++20** and **cmake**.

```bash
git clone https://github.com/gunrock/essentials-cpp.git
cd essentials-cpp
mkdir build && cd build
cmake .. 
make
bin/sssp ../datasets/chesapeake/chesapeake.mtx
```

## How to Cite
Thank you for citing our work.
```tex
@InProceedings{   Osama:2022:EOP,
  author        = {Muhammad Osama and Serban D. Porumbescu and John D.
                  Owens},
  title         = {Essentials of Parallel Graph Analytics},
  booktitle     = {Proceedings of the Workshop on Graphs, Architectures,
                  Programming, and Learning},
  year          = 2022,
  series        = {GrAPL 2022},
  month         = may
}
```
