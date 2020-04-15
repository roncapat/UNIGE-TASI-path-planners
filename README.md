# UNIGE-TASI Path Planners

This repository contains a generic implementation of the Field D* algorithm,
[originally proposed by Dave Ferguson and Anthony Stentz](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_3/ferguson_david_2005_3.pdf).

In the first run it employs the "rhs-trick" suggested in ([Changwen Zheng, Jiawei Cai, Huafei Yin](https://www.scirp.org/pdf/ALAMT20120200001_65663100.pdf)).<br>
<strong>  However, it seems ineffective when using the optimized version of Field D*. This should be addressed.</strong>

The algorithm keeps backpointers from each node to the edge minimizing the cost, and use them to reduce the replanning load.
Refer to "initial optimizations" version of the original Field D* paper. The implementation caches values whenever possible.

When the robot moves and updated the map, it uses the "key modifier" technique described in ([Koenig and Likhachev](https://aaai.org/Papers/AAAI/2002/AAAI02-072.pdf)).

The algorithm features a custom path extraction technique, inspired by ([Michael W. Otte and Greg Grudic](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5354775)).

To build the demo and run tests:
```
cd Tests
chmod +x run_test__field_d_star.sh
./run_test__field_d_star.sh
```
The test files can be found in [Tests/Tests](Tests/Tests).

The logs and the results can be found in [Tests/Results](Tests/Results).

The repository should always contain the results of all tests executed with the last version of the software.