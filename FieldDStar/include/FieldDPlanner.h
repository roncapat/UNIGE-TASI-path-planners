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

class FieldDPlanner {
 public:
  FieldDPlanner();
  void init();
#define LOOP_OK 0
#define LOOP_FAILURE_NO_GRAPH -1
#define LOOP_FAILURE_NO_GOAL -2
  int step();

  // launch parameters
  bool lookahead = true;
  int optimization_lvl = 1;
  bool first_run_trick = true;        // Zheng trick from Update-Reducing Field-D*
  float occupancy_threshold_ = 0.5;   // maximum occupancy probability before a cell has infinite traversal cost
  float heuristic_multiplier = 1;

  bool initialize_graph_ = true;  // set to true if the graph must be initialized
  bool goal_set_ = false;         // true if the goal has been set
  bool goal_changed_ = false;     // true if the goal changed and the graph must be re-initialized

  Position start_pos;
  Cell start_cell = Cell(0, 0);
  std::vector<Node> start_nodes;

  void set_heuristic_multiplier(float mult) { heuristic_multiplier = mult; }
  void set_lookahead(bool use_lookahead) { lookahead = use_lookahead; }

  void set_map(const MapPtr &msg);
  void set_start(const Position &pos);
  void set_goal(std::pair<float, float> point);

  Graph grid;
  std::vector<Position> path_;
  std::vector<float> cost_;
  float total_cost = 0;
  float total_dist = 0;

  class path_additions {
   public:
    std::vector<Position> steps;
    std::vector<float> stepcosts;
    float cost_to_goal;
  };

  bool isVertex(const Position &p);

  void initializeSearch();

  void updateNode_0(const Node &s);

  unsigned long computeShortestPath_0();
  unsigned long computeShortestPath_1();

  unsigned long updateNodesAroundUpdatedCells();

  void constructOptimalPath();

  FieldDPlanner::path_additions computeOptimalCellTraversalFromEdge(const Position &p,
                                                                    const Node &p_a,
                                                                    const Node &p_b,
                                                                    float &step_cost);
  FieldDPlanner::path_additions computeOptimalCellTraversalFromContiguousEdge(const Position &p,
                                                                              const Node &p_a,
                                                                              const Node &p_b,
                                                                              float &step_cost);
  FieldDPlanner::path_additions computeOptimalCellTraversalFromOppositeEdge(const Position &p,
                                                                            const Node &p_a,
                                                                            const Node &p_b,
                                                                            float &step_cost);
  FieldDPlanner::path_additions computeOptimalCellTraversalFromCorner(const Position &p,
                                                                      const Node &p_a,
                                                                      const Node &p_b,
                                                                      float &step_cost);
  FieldDPlanner::path_additions getPathAdditions(const Position &p,
                                                 const bool &do_lookahead,
                                                 float &step_cost);
  bool goalReached(const Position &p);

  float getG(const Node &s);
  float getRHS(const Node &s);
  PriorityQueue::Key getKey(const Node &s);

  #define NODE(map_it) (map_it)->first
  #define G(map_it) std::get<0>((map_it)->second)
  #define RHS(map_it) std::get<1>((map_it)->second)
  #define BPTR(map_it) std::get<2>((map_it)->second)

  class ExpandedMap : public std::unordered_map<Node, std::tuple<float, float, Node>> {
   public:
    const Node NULLNODE = Node(-1, -1);
    iterator find_or_init(const Node &n);
    iterator insert_or_assign(const Node &s, float g, float rhs);
  };

  ExpandedMap expanded_map;

 private:
  unsigned long num_nodes_updated = 0;
  unsigned long num_nodes_expanded = 0;

  PriorityQueue priority_queue;
  bool initialize_search = true;  // set to true if the search problem must be initialized
  void getBC(TraversalParams &t);
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
};

#endif
