#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

#include <cassert>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Graph.h"
#include "PriorityQueue.h"
#include "Map.h"
#include "Pose.h"
#include "interpolation.h"

#define LOOP_OK 0
#define LOOP_FAILURE_NO_GRAPH -1
#define LOOP_FAILURE_NO_GOAL -2
class FieldDPlanner {
 public:
  FieldDPlanner();
  void init();
  int step();

  void set_optimization_lvl(int lvl);
  void set_first_run_trick(bool enable);;
  void set_occupancy_threshold(float threshold);
  void set_heuristic_multiplier(float mult);
  void set_lookahead(bool use_lookahead);

  void set_map(const MapPtr &msg);
  void patch_map(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h);;
  void set_start(const Position &pos);
  void set_goal(const Position &pos);

  std::vector<Position> path_;
  std::vector<float> cost_;
  float total_cost = 0;
  float total_dist = 0;

  bool goalReached(const Position &p);

  class ExpandedMap : public std::unordered_map<Node, std::tuple<float, float, Node>> {
   public:
    const Node NULLNODE = Node(-1, -1);
    iterator find_or_init(const Node &n);
    iterator insert_or_assign(const Node &s, float g, float rhs);
  };

  static inline const Node &NODE(const ExpandedMap::iterator &map_it) { return (map_it)->first; }
  static inline float &G(const ExpandedMap::iterator &map_it) { return std::get<0>((map_it)->second); }
  static inline float &RHS(const ExpandedMap::iterator &map_it) { return std::get<1>((map_it)->second); }
  static inline Node &BPTR(const ExpandedMap::iterator &map_it) { return std::get<2>((map_it)->second); }

  ExpandedMap expanded_map;

 private:
  class path_additions {
   public:
    std::vector<Position> steps;
    std::vector<float> stepcosts;
    float cost_to_goal;
  };

  unsigned long num_nodes_updated = 0;
  unsigned long num_nodes_expanded = 0;

  int optimization_lvl = 1;
  bool first_run_trick = true;        // Zheng trick from Update-Reducing Field-D*
  bool lookahead = true;
  float heuristic_multiplier = 1;
  bool initialize_graph_ = true;  // set to true if the graph must be initialized
  bool goal_set_ = false;         // true if the goal has been set
  bool goal_changed_ = false;     // true if the goal changed and the graph must be re-initialized
  Position start_pos;
  Cell start_cell = Cell(0, 0);
  std::vector<Node> start_nodes;
  PriorityQueue priority_queue;
  Graph grid;
  bool initialize_search = true;  // set to true if the search problem must be initialized


  void getBC(TraversalParams &t);
  float getG(const Node &s);
  float getRHS(const Node &s);
  PriorityQueue::Key getKey(const Node &s);
  float computeOptimalCost(const Node &p, const Node &p_a, const Node &p_b);
  bool end_condition();
  PriorityQueue::Key calculateKey(const Node &s);
  PriorityQueue::Key calculateKey(const Node &s, float cost_so_far);
  PriorityQueue::Key calculateKey(const Node &s, float g, float rhs);
  void enqueueIfInconsistent(ExpandedMap::iterator it);
  float minRHS_0(const Node &s);
  float minRHS_1(const Node &s, Node &bptr_idx);
  bool consistent(const Node &s);
  static bool consistent(const ExpandedMap::iterator &it);
  void initializeSearch();
  bool isVertex(const Position &p);
  void updateNode_0(const Node &s);
  unsigned long computeShortestPath_0();
  unsigned long computeShortestPath_1();
  unsigned long updateNodesAroundUpdatedCells();
  void constructOptimalPath();

  path_additions computeOptimalCellTraversalFromEdge(const Position &p,
                                                     const Node &p_a,
                                                     const Node &p_b,
                                                     float &step_cost);
  path_additions computeOptimalCellTraversalFromContiguousEdge(const Position &p,
                                                               const Node &p_a,
                                                               const Node &p_b,
                                                               float &step_cost);
  path_additions computeOptimalCellTraversalFromOppositeEdge(const Position &p,
                                                             const Node &p_a,
                                                             const Node &p_b,
                                                             float &step_cost);
  path_additions computeOptimalCellTraversalFromCorner(const Position &p,
                                                       const Node &p_a,
                                                       const Node &p_b,
                                                       float &step_cost);
  path_additions getPathAdditions(const Position &p,
                                  const bool &do_lookahead,
                                  float &step_cost);
};

#endif
