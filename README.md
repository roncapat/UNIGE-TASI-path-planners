# UNIGE-TASI Path Planners
![C/C++ CI](https://github.com/roncapat/UNIGE-TASI-path-planners/workflows/C/C++%20CI/badge.svg)

## Credits
If you use this software, please cite the associated [Master Thesis on ResearchGate](https://www.researchgate.net/publication/348732289_Evaluation_and_Implementation_of_Modern_Path_Planning_Algorithms_for_Planetary_Exploration_Rovers).

It is also a good reference on the design choices of this software package.

## Content
This repository contains three dynamic path planners (replanners):
* FD* - Field D*
* MS-DFM - Multistencil Dynamic Fast Marching
* MFD* - Marching Field D* (AKA Shifted-Grid 8-connected Fast Marching)

## Field D*
This repository contains a generic implementation of the Field D* algorithm,
[originally proposed by Dave Ferguson and Anthony Stentz](
https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2005_3/ferguson_david_2005_3.pdf).

The "Level 0" algorithm implements the basic D* Lite implementation of the original paper.

The "Level 1" algorithm keeps backpointers from each node to the edge minimizing the cost,
and use them to reduce the replanning load. Whenever a node is overconsistent, it recomputes
only the quadrants of the neighbors affected by lowering G to RHS.
Refer to "initial optimizations" version of the original Field D* paper for more details.

When the robot moves and updates the map, keys are recalculated w.r.t. the new position
(if heuristic is used) and the minimum traversability of the map at the current state.
This is a novel approach, different to heuristics suggested in various other works.

The interpolation algorithm has been redesigned to fix a suboptimal minimization
of the traversal cost by the origianl authors. More detail in <THESIS CHAP 2>.


## Multistencil Dynamic Fast Marching
This repository contains a generic implementation of Dynamic Fast Marching,
[originally proposed by Clement Petres et al.](
https://ieeexplore.ieee.org/document/4154833).

The algorithm has been tuned to leverage both orthogonal and diagonal neighbors
of a cell, adding a "diagonal stencil". This technique was inspired by this
[paper](https://ieeexplore.ieee.org/document/6970475).

## Marching Field D*
This repository contains a variant of Field D* with pruned codepaths to comply with
[the general solution of the Eikonal Equation on a mesh](https://www.pnas.org/content/97/11/5699).

The idea comes from the observation of the conceptual similarity of [Shifted-Grid Fast Marching](
https://link.springer.com/content/pdf/10.1007/3-540-45103-X_151.pdf) with Field D*. This change of
perspective allows to reinterpret it as a simpler _subset_ of Field D*.


## Path extraction
The path extractors feature a custom path extraction technique, inspired by
([Michael W. Otte and Greg Grudic](https://ieeexplore.ieee.org/document/5354775)).

## Repository content
The repository holds implementations for the 3 planners, as long as a support library and a path extraction algorithm.

Relevant files are in folders [FieldDStar](FieldDStar), [DynamicFastMarching](DynamicFastMarching),
[ShiftedGridFastMarching](ShiftedGridFastMarching), [PathExtraction](PathExtraction) and [ProjectToolkit](ProjectToolkit).

The [Tests/Planners](Tests/Planners) folder also host demo applications that work in conjunction with 
the [run_test.py](Tests/run_test.py) comparative benchmark.

Alternative sources and CMakeLists for a basic RTEMS build have an extension prefix \*.rtems.\*

## Build and run
To build the demo and run tests:
```
cd Tests
chmod +x run_test.sh
./run_test.sh
```
The test files can be found in [Tests/Tests](Tests/Tests).

The logs and the results can be found in [Tests/Results](Tests/Results) after a successful run of the above commands.

## Python 3 requirements
* opencv-python
* Pillow
* setuptools
