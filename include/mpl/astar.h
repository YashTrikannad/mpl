//
// Created by yash on 9/14/19.
//

#pragma once

#include "planner.h"

#include <iostream>
#include <queue>
#include <unordered_set>

namespace mpl
{

/// Astar Planner
class astar : public planner<astar>
{
public:
    std::optional<std::vector<location_2d>> get_plan(const location_2d& start, const location_2d& goal) const
    {
        if(graph_(start.row, start.col) != 0)
        {
            std::cout << "Cannot Find Path. Start Position is Occupied. \n";
            return std::nullopt;
        }
        const auto x = graph_(goal.row, goal.col);
        if(graph_(goal.row, goal.col) != 0)
        {
            std::cout << "Cannot Find Path. End Position is Occupied. \n";
            return std::nullopt;
        }

        std::vector<location_2d> path;
        std::unordered_set<location_2d> closed_set;

        std::unordered_map<location_2d, double> f_costs;
        std::unordered_map<location_2d, double> g_costs;

        auto less = [&](const location_2d& left, const location_2d& right) {
            return f_costs[left] > f_costs[right];
        };

        std::priority_queue<location_2d, std::vector<location_2d>, decltype(less)> open_queue(less);
        std::unordered_set<location_2d> open_set;

        std::unordered_map<location_2d, location_2d> parent_from_node;

        open_queue.push(start);
        open_set.insert(start);

        while(!open_queue.empty())
        {
            const auto current_location = open_queue.top();
            open_queue.pop();

            if(current_location == goal)
            {
                auto path_node = goal;
                while (path_node != start)
                {
                    path.emplace_back(path_node);
                    if(parent_from_node.find(path_node) == parent_from_node.end())
                    {
                        std::__throw_logic_error("No Parent for Discovered Node! Check Logic");
                    }
                    path_node = parent_from_node[path_node];
                }
                path.emplace_back(start);
                return path;
            }

            for_all_adjacent_nodes(current_location, graph_, [&](location_2d&& adjacent_loc){

                if(graph_(adjacent_loc.row, adjacent_loc.col) == 0 &&closed_set.find(adjacent_loc) == closed_set.end())
                {

                    const auto node_transition_cost = mpl::get_distance(&current_location, &adjacent_loc);

                    // If child not in open list
                    if(open_set.find(adjacent_loc) == open_set.end())
                    {
                        g_costs[adjacent_loc] = g_costs[current_location] + node_transition_cost;

                        const auto heuristic_cost = mpl::get_distance(&adjacent_loc, &goal);

                        f_costs[adjacent_loc] = g_costs[adjacent_loc] + heuristic_cost;

                        parent_from_node[adjacent_loc] = current_location;

                        open_set.insert(adjacent_loc);
                        open_queue.push(adjacent_loc);
                    }
                    // Else if child in open list
                    else
                    {
                        // cost of current child less than the cost of same child in open list
                        if(const auto g_cost_adjacent = g_costs[current_location] + node_transition_cost;
                        g_costs[adjacent_loc] > g_cost_adjacent)
                        {
                            g_costs[adjacent_loc] = g_cost_adjacent;

                            // replace the parent as current and cost with current cost
                            parent_from_node[adjacent_loc] = current_location;
                        }
                    }
                }
            });
        }

        std::cout << "Path Not Found! \n";
        return std::nullopt;
    }

    std::optional<std::vector<location_2d>> get_plan_with_visualization(const location_2d& start, const location_2d& goal)
    {
        if(graph_(start.row, start.col) != 0)
        {
            std::cout << "Cannot Find Path. Start Position is Occupied. \n";
            return std::nullopt;
        }
        const auto x = graph_(goal.row, goal.col);
        if(graph_(goal.row, goal.col) != 0)
        {
            std::cout << "Cannot Find Path. End Position is Occupied. \n";
            return std::nullopt;
        }

        std::vector<location_2d> path;
        std::unordered_set<location_2d> closed_set;

        std::unordered_map<location_2d, double> f_costs;
        std::unordered_map<location_2d, double> g_costs;

        auto less = [&](const location_2d& left, const location_2d& right) {
            return f_costs[left] > f_costs[right];
        };

        std::priority_queue<location_2d, std::vector<location_2d>, decltype(less)> open_queue(less);
        std::unordered_set<location_2d> open_set;

        std::unordered_map<location_2d, location_2d> parent_from_node;

        open_queue.push(start);
        open_set.insert(start);

        while(!open_queue.empty())
        {
            const auto current_location = open_queue.top();
            open_queue.pop();

            if(current_location == goal)
            {
                auto path_node = goal;
                while (path_node != start)
                {
                    explored_locations_.emplace_back(path_node, "yellow");
                    path.emplace_back(path_node);
                    if(parent_from_node.find(path_node) == parent_from_node.end())
                    {
                        std::__throw_logic_error("No Parent for Discovered Node! Check Logic");
                    }
                    path_node = parent_from_node[path_node];
                }
                path.emplace_back(start);
                explored_locations_.emplace_back(start, "red");
                explored_locations_.emplace_back(goal, "green");
                return path;
            }

            for_all_adjacent_nodes(current_location, graph_, [&](location_2d&& adjacent_loc){

                if(graph_(adjacent_loc.row, adjacent_loc.col) == 0 &&closed_set.find(adjacent_loc) == closed_set.end())
                {

                    const auto node_transition_cost = mpl::get_distance(&current_location, &adjacent_loc);

                    // If child not in open list
                    if(open_set.find(adjacent_loc) == open_set.end())
                    {
                        g_costs[adjacent_loc] = g_costs[current_location] + node_transition_cost;

                        const auto heuristic_cost = mpl::get_distance(&adjacent_loc, &goal);

                        f_costs[adjacent_loc] = g_costs[adjacent_loc] + heuristic_cost;

                        parent_from_node[adjacent_loc] = current_location;

                        open_set.insert(adjacent_loc);
                        open_queue.push(adjacent_loc);

                        explored_locations_.emplace_back(adjacent_loc, "blue");
                    }
                        // Else if child in open list
                    else
                    {
                        // cost of current child less than the cost of same child in open list
                        if(const auto g_cost_adjacent = g_costs[current_location] + node_transition_cost;
                                g_costs[adjacent_loc] > g_cost_adjacent)
                        {
                            g_costs[adjacent_loc] = g_cost_adjacent;

                            // replace the parent as current and cost with current cost
                            parent_from_node[adjacent_loc] = current_location;
                        }
                    }
                }
            });
        }

        std::cout << "Path Not Found! \n";
        return std::nullopt;
    }
};

}
