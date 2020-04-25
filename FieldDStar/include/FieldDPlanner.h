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
#include "Node.h"
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
  double maximum_distance_ = 100000;  // maximum distance to goal node before warning messages spit out
  double goal_range_ = 0;             // distance from goal at which a node is considered the goal
  bool follow_old_path_ = true;       // follow the previously generated path if no optimal path currently exists
  bool lookahead = true;
  int optimization_lvl = 1;
  bool first_run_trick = true;        // Zheng trick from Update-Reducing Field-D*
  float occupancy_threshold_ = 0.5;   // maximum occupancy probability before a cell has infinite traversal cost
  float heuristic_multiplier = 1;
  unsigned long num_nodes_updated = 0;
  unsigned long num_nodes_expanded = 0;

  MapPtr map_;  // Most up-to-date map
  int x_initial_, y_initial_;   // Index for initial x and y location in search space

  bool initialize_graph_ = true;  // set to true if the graph must be initialized
  bool goal_set_ = false;         // true if the goal has been set
  bool goal_changed_ = false;     // true if the goal changed and the graph must be re-initialized

  Position start_pos;
  Cell start_cell = Cell(0, 0);
  std::vector<Node> start_nodes;
  void set_start_position(const Position &pos) {
      start_pos = pos;
      grid.start_ = Node(std::round(pos.x), std::round(pos.y)); //TODO remove usage
      start_cell = Cell(std::floor(start_pos.x), std::floor(start_pos.y));
      start_nodes = grid.getNodesAroundCell(start_cell);
      PriorityQueue new_queue;
      for (const auto &elem: priority_queue)
          // Only heuristic changes, so either G or RHS is kept the same
          new_queue.insert(elem.first, calculateKey(elem.first, elem.second.second));
      priority_queue.swap(new_queue);
  }

  void set_map(const MapPtr &msg);
  void set_goal(std::pair<float, float> point);
  void set_heuristic_multiplier(float mult) { heuristic_multiplier = mult; }
  void set_lookahead(bool use_lookahead) { lookahead = use_lookahead; }


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

  float goal_dist_;

  void setGoalDistance(float goal_dist);

  bool isVertex(const Position &p);

  void initializeSearch();

  void updateNode_0(const Node &s);

  int computeShortestPath_0();
  int computeShortestPath_1();

  int updateNodesAroundUpdatedCells();

  void constructOptimalPath();

  FieldDPlanner::path_additions computeOptimalCellTraversalFromEdge(const Position &p,
                                                                    const Position &p_a,
                                                                    const Position &p_b,
                                                                    float &step_cost);
  FieldDPlanner::path_additions computeOptimalCellTraversalFromContiguousEdge(const Position &p,
                                                                              const Position &p_a,
                                                                              const Position &p_b,
                                                                              float &step_cost);
  FieldDPlanner::path_additions computeOptimalCellTraversalFromOppositeEdge(const Position &p,
                                                                            const Position &p_a,
                                                                            const Position &p_b,
                                                                            float &step_cost);
  FieldDPlanner::path_additions computeOptimalCellTraversalFromCorner(const Position &p,
                                                                      const Position &p_a,
                                                                      const Position &p_b,
                                                                      float &step_cost);
  FieldDPlanner::path_additions getPathAdditions(const Position &p,
                                                 const bool &do_lookahead,
                                                 float &step_cost);
  bool isWithinRangeOfGoal(const Position &p);

  float getG(const Node &s);
  float getRHS(const Node &s);
  std::pair<float, float> getGandRHS(const Node &s);

  #define G(map_it) std::get<0>(map_it->second)
  #define RHS(map_it) std::get<1>(map_it->second)
  #define BPTR(map_it) std::get<2>(map_it->second)

  class ExpandedMap : public std::unordered_map<Node, std::tuple<float, float, Node>> {
   public:
    const Node NULLNODE = Node(-1,-1);
    iterator find_or_init(const Node &n);
    iterator insert_or_assign(const Node &s, float g, float rhs);
  } expanded_map;


 private:
  PriorityQueue priority_queue;
  bool initialize_search = true;  // set to true if the search problem must be initialized
  std::pair<float, float> getBC(TraversalParams &t);
  float computeOptimalCost(const Position &p, const Position &p_a, const Position &p_b);
  bool end_condition();
  PriorityQueue::Key calculateKey(const Node &s);
  PriorityQueue::Key calculateKey(const Node &s, const float cost_so_far);
  PriorityQueue::Key calculateKey(const Node &s, const float g, const float rhs);
  void enqueueIfInconsistent(ExpandedMap::iterator it);
  float minRHS_0(const Node &s);
  float minRHS_1(const Node &s, Node &bptr_idx);
  bool consistent(const Node &s);
  bool consistent(const ExpandedMap::iterator &it);
};

#endif
