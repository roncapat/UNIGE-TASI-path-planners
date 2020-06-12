//
// Created by patrick on 17/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_REPLANNERBASE_H
#define RONCAPAT_GLOBAL_PLANNERS_REPLANNERBASE_H

#include <utility>
#include <vector>
#include <memory>
#include <chrono>
#include "Node.h"
#include "PriorityQueue.h"
#include "ExpandedMap.h"

template<class...> constexpr std::false_type always_false{};

#define LOOP_OK 0
#define LOOP_FAILURE_NO_GRAPH -1
#define LOOP_FAILURE_NO_GOAL -2

/* Using CRTP (Curiously Recursive Template Pattern) for static polymorphism
 * (abstract class with no virtual table)
 */
template<typename Derived, typename MapElem_, typename MapInfo_, typename QueueKey_>
class ReplannerBase {
public:
    typedef QueueKey_ Key;
    typedef PriorityQueue<Key, MapElem_> Queue;
    typedef ExpandedMap<MapElem_, MapInfo_> Map;
public:
    float u_time = 0, p_time = 0;

    void reset() {
        initialize_search = true;
    }

    int step() {
        if (initialize_graph) return LOOP_FAILURE_NO_GRAPH;
        if (not goal_set) return LOOP_FAILURE_NO_GOAL;

        auto begin = std::chrono::steady_clock::now();
        if (initialize_search or new_goal) {
            num_nodes_updated = 0;
            num_nodes_expanded = 0;
            priority_queue.clear();
            grid.updated_cells_.clear();
            map.clear();
            map.init(grid.width_, grid.length_, 8);
            init();
        } else if (new_start) {
            new_start = false;
            update();
        }
        auto end = std::chrono::steady_clock::now();
        u_time = std::chrono::duration<float, std::milli>(end - begin).count();

        // only update the graph if nodes have been updated
        begin = std::chrono::steady_clock::now();
        if (new_goal or initialize_search or num_nodes_updated > 0) {
            plan();
        } else {
            num_nodes_expanded = 0;
        }
        new_goal = initialize_search = false;

        end = std::chrono::steady_clock::now();
        p_time = std::chrono::duration<float, std::milli>(end - begin).count();
        return LOOP_OK;
    }

    void set_occupancy_threshold(float threshold) {
        grid.set_occupancy_threshold(threshold);
    }

    void set_heuristic_multiplier(float mult) {
        heuristic_multiplier = mult;
    }

    void set_map(const std::shared_ptr<uint8_t[]> &new_map, int w, int h) {
        grid.init(new_map, w, h);
        initialize_graph = false;
    }

    void patch_map(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h) {
        grid.update(patch, x, y, w, h);
    }

    void set_start(const Position &pos) {
        grid.set_start(pos);
        new_start = true;
    }

    void set_goal(const Position &point) {
        if constexpr(std::is_same_v<MapElem_, Node>)
            new_goal = grid.goal_node_ != Node(point);
        else if constexpr(std::is_same_v<MapElem_, Cell>)
            new_goal = grid.goal_cell_ != Cell(point);
        else static_assert(always_false<Derived>, "A MapElement can only be of type Node or Cell");
        grid.set_goal(point);
        goal_set = true;
    }

    void enqueue_if_inconsistent(typename Map::iterator it) {
        if (G(it) != RHS(it))
            priority_queue.insert_or_update(ELEM(it), calculate_key(ELEM(it), G(it), RHS(it)));
        else
            priority_queue.remove_if_present(ELEM(it));
    }

    const Map& get_expanded_map(){
        return map;
    }

    const Graph& get_grid(){
        return grid;
    }

protected:
    //REPLANNER API
    void init() {
        static_cast<Derived *>(this)->init();
    }

    void update() {
        static_cast<Derived *>(this)->update();
    }

    void plan() {
        static_cast<Derived *>(this)->plan();
    }

    Key calculate_key(const MapElem_ &s, float g, float rhs) {
        return static_cast<Derived *>(this)->calculate_key(s, g, rhs);
    }

public:
    unsigned long num_nodes_updated = 0;
    unsigned long num_nodes_expanded = 0;

    float heuristic_multiplier = 1;
    bool initialize_graph = true;  // set to true if the graph must be initialized
    bool initialize_search = true;  // set to true if the search problem must be initialized
    bool goal_set = false;         // true if the goal has been set
    bool new_goal = false;     // true if the goal changed and the graph must be re-initialized
    bool new_start = false;

    Queue priority_queue;
    Graph grid;
    Map map;

protected:
    ReplannerBase() = default;
    ~ReplannerBase() = default;
};

#endif //RONCAPAT_GLOBAL_PLANNERS_REPLANNERBASE_H
