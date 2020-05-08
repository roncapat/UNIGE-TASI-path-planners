#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

#include <cassert>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>
#include <utility>

#include "Macros.h"
#include "Graph.h"
#include "ExpandedMap.h"
#include "PriorityQueue.h"
#include "LinearTraversalCostInterpolation.h"

#define LOOP_OK 0
#define LOOP_FAILURE_NO_GRAPH -1
#define LOOP_FAILURE_NO_GOAL -2
class DFMPlanner {
  typedef PriorityQueue<Cell> Queue;
  typedef ExpandedMap<Cell, std::pair<Cell, Cell>> Map;
 public:
  DFMPlanner();
  void init();
  int step();
  float e_time, u_time, p_time;

  void set_optimization_lvl(int lvl);
  void set_first_run_trick(bool enable);
  void set_occupancy_threshold(float threshold);
  void set_heuristic_multiplier(float mult);

  void set_map(const std::shared_ptr<uint8_t[]> &m, int w, int l);
  void patch_map(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h);;
  void set_start(const Position &pos);
  void set_goal(const Position &pos);

  std::vector<Position> path_;
  std::vector<float> cost_;
  float total_cost = 0;
  float total_dist = 0;

  bool goalReached(const Position &p);

  Map map;

 private:
  unsigned long num_cells_updated = 0;
  unsigned long num_nodes_expanded = 0;

  int optimization_lvl = 1;
  bool first_run_trick = true;        // Zheng trick from Update-Reducing Field-D*
  float heuristic_multiplier = 1;
  bool initialize_graph_ = true;  // set to true if the graph must be initialized
  bool goal_set_ = false;         // true if the goal has been set
  bool new_goal_ = false;     // true if the goal changed and the graph must be re-initialized
  std::vector<Cell> start_nodes;
  Queue priority_queue;
  Graph grid;
  bool initialize_search = true;  // set to true if the search problem must be initialized

  float computeOptimalCost(const Cell &p);
  bool end_condition();
  Queue::Key calculateKey(const Cell &s);
  Queue::Key calculateKey(const Cell &s, float cost_so_far);
  Queue::Key calculateKey(const Cell &s, float g, float rhs);
  void enqueueIfInconsistent(Map::iterator it);
  bool consistent(const Cell &s);
  static bool consistent(const Map::iterator &it);
  void initializeSearch();
  bool isVertex(const Position &p);
  void updateCell(const Cell &cell);
  unsigned long computeShortestPath();
  unsigned long updateCells();
  void constructOptimalPath();
  bool new_start;
  std::pair<Cell, float> minCost(const Cell &a, const Cell &b);
  std::tuple<float, float> gradientAtCell(const Cell __c);
  std::pair<std::shared_ptr<float[]>, std::shared_ptr<float[]>> costMapGradient();
  std::tuple<float, float> interpolateGradient(const Position &c);
  float computePathAdditionsCost(const std::vector<Position> &p);
  void computeRoughtPath(bool eight_if_true = false);
  float getInterpRHS(const Node &node);
  float getInterpG(const Node &node);
  void computeInterpolatedPath();
  void getBC(TraversalParams &t);
  path_additions traversalFromCorner(const Position &p,
                                     const Node &p_a,
                                     const Node &p_b,
                                     float &step_cost);
  path_additions traversalFromEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);
  path_additions getPathAdditions(const Position &p, const bool &do_lookahead, float &step_cost);
  bool lookahead = false;
  path_additions traversalFromOppositeEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);
  path_additions traversalFromContiguousEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);
};

#endif
