# Essentials of Parallel Graph Analytics
Single-Source Shortest Path (SSSP) implementation in modern C++ for 2022 IPDPS workshop on Graphs, Architectures, Programming, and Learning (GrAPL 2022) submission.

| System  | Version                                                                                                                                                       | Status                                                                                                                                                   |
|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| Ubuntu  | [Ubuntu 20.04](https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners#supported-runners-and-hardware-resources)        | [![Ubuntu](https://github.com/neoblizz/sssp/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/neoblizz/sssp/actions/workflows/ubuntu.yml)    |
| Windows | [Windows Server 2019](https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners#supported-runners-and-hardware-resources) | [![Windows](https://github.com/neoblizz/sssp/actions/workflows/windows.yml/badge.svg)](https://github.com/neoblizz/sssp/actions/workflows/windows.yml) |

## Requirements
- `C++20` for linux, `C++23` for windows.
- `cmake` version 3.22.2.
- `tbb` library for execution policies (automatically fetched using cmake).

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
