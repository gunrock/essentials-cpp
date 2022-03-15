# Essentials of Parallel Graph Analytics
Single-Source Shortest Path (SSSP) implementation in modern C++ for 2022 IPDPS workshop on Graphs, Architectures, Programming, and Learning (GrAPL 2022) submission.

## Quick Start Guide
Before building this project, make sure your system/compiler supports **C++20** and **cmake** (see `CMakeLists.txt` for the version).

```bash
git clone https://github.com/owensgroup/sssp.git
cd sssp
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