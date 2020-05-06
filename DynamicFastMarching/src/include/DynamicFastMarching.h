#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

#include <cassert>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>
#include <interpolation.h>

#include "Graph.h"
#include "PriorityQueue.h"
#include "Macros.h"

#define LOOP_OK 0
#define LOOP_FAILURE_NO_GRAPH -1
#define LOOP_FAILURE_NO_GOAL -2
class DFMPlanner {
  typedef PriorityQueue<Cell> Queue;
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

  class ExpandedMap : public std::unordered_map<Cell, std::tuple<float, float, Cell>> {
   public:
    const Cell NULLCELL = Cell(-1, -1);
    iterator find_or_init(const Cell &n);
    iterator insert_or_assign(const Cell &s, float g, float rhs);
    float getG(const Cell &s);
    float getRHS(const Cell &s);
    std::pair<float, float> getGandRHS(const Cell &s);
  };

  static inline const Cell &CELL(const ExpandedMap::iterator &map_it) { return (map_it)->first; }
  static inline float &G(const ExpandedMap::iterator &map_it) { return std::get<0>((map_it)->second); }
  static inline float &RHS(const ExpandedMap::iterator &map_it) { return std::get<1>((map_it)->second); }
  static inline Cell &BPTR(const ExpandedMap::iterator &map_it) { return std::get<2>((map_it)->second); }

  ExpandedMap map;

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
  void enqueueIfInconsistent(ExpandedMap::iterator it);
  bool consistent(const Cell &s);
  static bool consistent(const ExpandedMap::iterator &it);
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
  void computeRoughtPath();
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
  bool lookahead = true;
  path_additions traversalFromOppositeEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);
  path_additions traversalFromContiguousEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);
};

#endif
