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

/// BFS Planner
class bfs : public planner<bfs>
{
public:
    std::optional<std::vector<location_2d>> get_plan(const location_2d& start, const location_2d& goal) const
    {
        if(graph_(start.row, start.col) == 1 )
        {
            std::cout << "Cannot Find Path. Start Position is Occupied. \n";
            return std::nullopt;
        }
        if(graph_(goal.row, goal.col) == 1)
        {
            std::cout << "Cannot Find Path. End Position is Occupied. \n";
            return std::nullopt;
        }

        std::vector<location_2d> path;
        std::unordered_set<location_2d> closed_set;

        std::queue<location_2d> open_queue;
        std::unordered_set<location_2d> open_set;

        std::unordered_map<location_2d, location_2d> parent_from_node;

        open_queue.push(start);
        open_set.insert(start);

        while(!open_queue.empty())
        {
            const auto current_location = open_queue.front();
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

            for_all_adjacent_nodes(current_location, &graph_, [&](location_2d&& adjacent_loc){

                if(graph_(adjacent_loc.row, adjacent_loc.col) == 0 && closed_set.find(adjacent_loc) == closed_set.end())
                {
                    // If child not in open list
                    if(open_set.find(adjacent_loc) == open_set.end())
                    {
                        parent_from_node[adjacent_loc] = current_location;

                        open_set.insert(adjacent_loc);
                        open_queue.push(adjacent_loc);
                    }
                }
            });

            closed_set.insert(current_location);
        }

        std::cout << "Path Not Found! \n";
        return std::nullopt;
    }
};

}