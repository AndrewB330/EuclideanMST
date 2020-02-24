# EuclideanMST
Implementations of different algorithms for building Euclidean minimum spanning tree in k-dimensional space.
### Algorithms:
  - EMST using Kd-tree __O(NlogN)__*
  	- Implementation of algorithm described in "Fast Euclidean Minimum Spanning Tree: Algorithm, Analysis, 	and Applications. William B. March, Parikshit Ram, Alexander G. Gray"*
  - Prim's algorithm __O(N^2)__
    - Straightforward MST on fully connected Eclidean graph

### Build

```>>> g++ main.cpp -o main```

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

| Dimensions        | Number of points | Kd-tree (sec)  | Prim (sec) |
| -------------: |-------------:| -----:| ----:|
| 2      | 50000 	| 0.24 	| 29.0 		|
| 3      | 50000 	| 0.67 	| 32.0 		|
| 4      | 50000 	| 1.59 	| 36.0 		|
| 2      | 10000000 | 69.0 	| ~1000000 	|
| 3      | 10000000 | 186.0 | ~1300000 	|
| 4      | 10000000 | 673.9 | ~1500000 	|

### Contribution
__Very appreciated__

### TODO:
- Implement EMST using Cover-tree
- More use-cases
- Online-solver
- \dots