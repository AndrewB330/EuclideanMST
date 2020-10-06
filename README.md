# EuclideanMST

[![Generic badge](https://img.shields.io/badge/C++-Brightgreen.svg)](https://github.com/AndrewB330/)
![XLOCC](https://europe-west6-xlocc-badge.cloudfunctions.net/XLOCC/AndrewB330/EuclideanMST?caption=Lines&color=blue&ifiles=standalone_header&kill_cache=1)

Implementations of different algorithms for building [Euclidean minimum spanning tree](https://en.wikipedia.org/wiki/Euclidean_minimum_spanning_tree) in k-dimensional space.
### Algorithms:
  - EMST using Kd-tree __O(NlogN)__*
  	- Implementation of algorithm described in "Fast Euclidean Minimum Spanning Tree: Algorithm, Analysis, 	and Applications. William B. March, Parikshit Ram, Alexander G. Gray"*
    - For higher dimensions (5D and more) and small number of points it can work even slower than Prim's algorithm*
  - Prim's algorithm __O(N^2)__
    - Straightforward MST on fully connected Euclidean graph

### How to add to your project
You can use standalone header file from `standalone_header/` folder, or you can
add this project as a CMake subdirectory:
```cmake
add_subdirectory(EuclideanMST)
target_link_libraries(<TARGET_NAME> EuclideanMST)
```

### How to use

```cpp
    std::vector<Point<3>> points(n); // your data
    // read data ...
    // you can acces any Point<> dimension by index
    // e.g. cin >> points[i][0] >> points[i][1] >> points[i][2]; 
    
    KdTreeSolver<3> solver(points);
    
    double total_length = solver.get_total_length();
    std::vector<Edge> edges = solver.get_solution(); 
    // Edge is std::pair<size_t, size_t> - describes connected pair
```

### Benchmarks:

| Dimensions | Number of points | Kd-tree | Prim |
| -----: |-----------:| -----------:| -------------:|
| 2      | 50'000 	  | 0.24 sec 	| 29.0 sec 		|
| 3      | 50'000 	  | 0.67 sec 	| 32.0 sec 		|
| 4      | 50'000 	  | 1.59 sec 	| 36.0 sec 		|
| 2      | 10'000'000 | 69.0 sec 	| ~10+ days 	|
| 3      | 10'000'000 | 186.0 sec   | ~13+ days 	|
| 4      | 10'000'000 | 673.9 sec   | ~15+ days 	|
| 5      | 180'000 	  | 15.3 sec 	| ~300+ sec     |

### Contribution
__Very appreciated__

### TODO:
- Implement EMST using Cover-tree
- More use-cases
- Online-solver
- Parallel implementation using OMP (actually had some experiments here, this algorithm can be parallelized quite well)
- Other...
