# EuclideanMST
Implementations of different algorithms for building Euclidean minimum spanning tree in k-dimensional space.
### Algorithms:
  - EMST using Kd-tree O(NlogN)
  
  	Implementation of algorithm described in "Fast Euclidean Minimum Spanning Tree: Algorithm, Analysis, 	and Applications. William B. March, Parikshit Ram, Alexander G. Gray"
  - Prim's algorithm O(N^2)
    
    Straightforward MST on fully connected Eclidean graph

### Benchmarks:

| Dimensions        | Number of points | Kd-tree (sec)  | Prim (sec) |
| -------------: |-------------:| -----:| ----:|
| 2      | 50000 	| 0.24 	| 29.0 		|
| 3      | 50000 	| 0.67 	| 32.0 		|
| 4      | 50000 	| 1.59 	| 36.0 		|
| 2      | 10000000 | 69.0 	| ~1000000 	|
| 3      | 10000000 | 186.0 | ~1300000 	|
| 4      | 10000000 | 673.9 | ~1500000 	|

### TODO:
- Implement EMST using Cover-tree
- \dots