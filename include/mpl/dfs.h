//
// Created by yash on 9/14/19.
//

#pragma once

#include "mpl/eigen_utility.h"
#include "planner.h"

#include <unordered_set>
#include <stack>
#include <iostream>

namespace mpl
{

/// DFS Planner - Not a Good Planner for Grid Space! (Not Optimal)
class dfs : public planner<dfs>
{
public:
    std::optional<std::vector<location_2d>> get_plan(const location_2d& start, const location_2d& goal) const
    {
        if(graph_(start.row, start.col) == 1 )
        {
            std::cout << "Cannot Find Path. Start Position is Occupied.";
            return std::nullopt;
        }
        if(graph_(goal.row, goal.col) == 1)
        {
            std::cout << "Cannot Find Path. End Position is Occupied.";
            return std::nullopt;
        }

        std::vector<location_2d> path;
        std::unordered_set<location_2d> closed_set;
        std::stack<location_2d> open_stack;
        std::unordered_map<location_2d, location_2d> parent_from_node;

        open_stack.push(start);

        while(!open_stack.empty())
        {
            const auto current_location = open_stack.top();
            open_stack.pop();

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

            for_all_adjacent_nodes(current_location, &graph_, [&](location_2d&& adjacent_loc){
                if(graph_(adjacent_loc.row, adjacent_loc.col) == 0 &&closed_set.find(adjacent_loc) == closed_set.end())
                {
                    parent_from_node[adjacent_loc] = current_location;
                    open_stack.push(adjacent_loc);
                }
            });

            closed_set.insert(current_location);
        }
    }
};

}
