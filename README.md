# UNIGE-TASI Path Planners

This repository contains a generic implementation of the Field D* algorithm,
[originally proposed by Dave Ferguson and Anthony Stentz](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_3/ferguson_david_2005_3.pdf).

In the first step run it employs the "rhs-trick" suggested in ([Changwen Zheng, Jiawei Cai, Huafei Yin](https://www.scirp.org/pdf/ALAMT20120200001_65663100.pdf)).<br>
<strong>. However, it seems ineffective when using the optimized version of Field D*. This should be addressed.</strong>

The "Level 0" algorithm implements the basic D* Lite implementation of the original paper.

The "Level 1" algorithm keeps backpointers from each node to the edge minimizing the cost, and use them to reduce the replanning load.
Refer to "initial optimizations" version of the original Field D* paper.

When the robot moves and updates the map, keys are recalculated w.r.t. the new position and the minimum traversability of the map at the current state.
This is a novel approach, different to heuristics suggested in various other works.

The algorithm features a custom path extraction technique, inspired by ([Michael W. Otte and Greg Grudic](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5354775)).

The interpolation algorithm has been redesigned to fix a suboptimal minimization of the traversal cost by the origianl authoers. More detail in <THESIS CHAP 2>.

To build the demo and run tests:
```
cd Tests
chmod +x run_test__field_d_star.sh
./run_test__field_d_star.sh
```
The test files can be found in [Tests/Tests](Tests/Tests).

The logs and the results can be found in [Tests/Results](Tests/Results) after a successful run of the above commands.