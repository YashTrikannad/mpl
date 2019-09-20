//
// Created by yash on 9/14/19.
//

#pragma once

#include <iostream>
#include <unordered_set>
#include <queue>
#include "planner.h"

namespace mpl
{

/// JPS Planner
class jps : public planner<jps>
{
public:
    std::optional<std::vector<location_2d>> get_plan(const location_2d &start, const location_2d &goal) const
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
                    assert(parent_from_node.find(path_node) != parent_from_node.end() && "No Parent for Discovered Node! Check Logic");
                    linear_backtrace(path_node, parent_from_node[path_node], &path);
                    path_node = parent_from_node[path_node];
                }
                return path;
            }

            int index = 0;

            // do a-star using jps functions
            for_all_adjacent_nodes(current_location, &graph_, [&](location_2d&& adjacent_loc){

                const auto successors = get_successors(adjacent_loc,
                        location_2d( adjacent_loc.row - current_location.row,
                                     adjacent_loc.col - current_location.col),
                        start,
                        goal);

                for(const auto& successor: successors)
                {
                    if(open_set.find(successor) == open_set.end())
                    {
                        parent_from_node[successor] = current_location;
                        open_set.insert(successor);
                        open_queue.push(successor);
                    }
                }

            });

        }

        std::cout << "Path Not Found! \n";
        return std::nullopt;

    }

private:

    /// Get the JPS successors of a current node in a particular direction (Refer to JPS Paper for more info)
    /// @param current_location
    /// @param previous_step_direction
    /// @param start
    /// @param goal
    /// @return
    std::vector<mpl::location_2d> get_successors(const location_2d& current_location,
                                                   const location_2d& previous_step_direction,
                                                   const location_2d &start,
                                                   const location_2d &goal) const
    {
        assert(is_within_boundary(current_location, n_rows, n_cols));

        std::vector<location_2d> successors;

        const auto pruned_neighbors = get_pruned_neighbors(current_location, previous_step_direction);
        for(const auto& neighbor: pruned_neighbors)
        {
            if(const auto successor = jump(neighbor.first, neighbor.second, start, goal))
            {
                assert(is_within_boundary(*successor, n_rows, n_cols));
                successors.emplace_back(*successor);
            }
        }
        return successors;
    }

    std::optional<location_2d> jump(const location_2d& cur_loc,
                                        const location_2d& step,
                                        const location_2d &start,
                                        const location_2d &goal) const
    {
        assert(cur_loc.is_within_boundary(n_rows, n_cols));
        // take a step in the input direction
        auto next_location = cur_loc + step;

        if(!is_within_boundary(next_location, n_rows, n_cols) || graph_(next_location.row, next_location.col) == 1)
        {
            return std::nullopt;
        }
        else if(next_location == goal)
        {
            return next_location;
        }
        else if(has_forced_neighbors(next_location, step))
        {
            return next_location;
        }
        else if(step.row !=0 && step.col !=0)
        {
            if(jump(next_location, location_2d(step.row, 0), start, goal) ||
            jump(next_location, location_2d(0, step.col), start, goal))
                return next_location;
        }
        return jump(next_location, step, start, goal);
    }

    /// Checks if the current location has forced neighbors (Refer to JPS Paper for more info)
    /// @param cur_loc
    /// @param step
    /// @return
    bool has_forced_neighbors(const mpl::location_2d& cur_loc, const mpl::location_2d& step) const
    {
        assert(cur_loc.is_within_boundary(n_rows, n_cols));

        auto is_within_boundary_after_offset = [&](int step_row, int step_col){
            return is_within_boundary(location_2d(cur_loc.row + step_row, cur_loc.col + step_col), n_rows, n_cols);
        };

        if(step.row == 0 && step.col != 0) // Condition for Horizontal Direction
        {
            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1) return true;
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1) return true;
            }
            return false;
        }
        else if(step.row != 0 && step.col == 0) // Condition for Vertical Direction
        {
            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row, cur_loc.col - 1) == 1) return true;
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row, cur_loc.col + 1) == 1) return true;
            }
            return false;
        }
        else if(step.row == 1 && step.col == 1) // Condition for Diagonal 1
        {
            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row, cur_loc.col - 1) == 1) return true;
            }
            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1) return true;
            }
            return false;
        }
        else if(step.row == -1 && step.col == -1) // Condition for Diagonal 2
        {
            if(is_within_boundary_after_offset(1, 0))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1) return true;
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row, cur_loc.col + 1) == 1) return true;
            }
            return false;
        }
        else if(step.row == 1 && step.col == -1) // Condition for Diagonal 3
        {
            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1) return true;
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row, cur_loc.col + 1) == 1) return true;
            }
            return false;
        }
        else if(step.row == -1 && step.col == 1) // Condition for Diagonal 4
        {
            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row, cur_loc.col - 1) == 1) return true;
            }
            if(is_within_boundary_after_offset(1, 0))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1) return true;
            }
            return false;
        }
    }

    /// Get pruned neighbors of a node with a particular direction (Refer to JPS Paper for more info)
    /// @param cur_loc
    /// @param step
    /// @return Vector of Pair of Neighbor Locations and their direction with respect to the current location
    std::vector<std::pair<mpl::location_2d, mpl::location_2d>> get_pruned_neighbors(
            const mpl::location_2d& cur_loc, const mpl::location_2d& step) const
    {
        assert(cur_loc.is_within_boundary(n_rows, n_cols));

        // Pair of Neighbor Location and Direction
        std::vector<std::pair<mpl::location_2d, mpl::location_2d>> pruned_neighbors;

        auto add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary = [&](int step_row, int step_col){
            const auto neighbor = location_2d(cur_loc.row + step_row, cur_loc.col + step_col);
            if(is_within_boundary(neighbor, n_rows, n_cols))
            {
                pruned_neighbors.emplace_back(std::pair{neighbor, location_2d(step_row, step_col)});
            }
        };

        auto is_within_boundary_after_offset = [&](int step_row, int step_col){
            return is_within_boundary(location_2d(cur_loc.row + step_row, cur_loc.col + step_col), n_rows, n_cols);
        };

        if(step.row == 0 && step.col == 1) // Condition for Horizontal Direction 1
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(0, 1);
            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 1);
                }
            }
            if(is_within_boundary_after_offset(1, 0))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 1);
                }
            }
        }
        else if(step.row == 0 && step.col == -1) // Condition for Horizontal Direction 2
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(0, -1);
            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, -1);
                }
            }
            if(is_within_boundary_after_offset(1, 0))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, -1);
                }
            }
        }
        else if(step.row == 1 && step.col == 0) // Condition for Vertical Direction 1
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 0);
            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row, cur_loc.col - 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, -1);
                }
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row, cur_loc.col + 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 1);
                }
            }
        }
        else if(step.row == -1 && step.col == 0) // Condition for Vertical Direction 2
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 0);
            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row , cur_loc.col - 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, -1);
                }
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row , cur_loc.col + 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 1);
                }
            }
        }
        else if(step.row == 1 && step.col == 1) // Condition for Diagonal 1
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 0);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(0, 1);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 1);

            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row, cur_loc.col - 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, -1);
                }
            }
            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 1);
                }
            }
        }
        else if(step.row == -1 && step.col == -1) // Condition for Diagonal 2
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 0);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(0, -1);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, -1);

            if(is_within_boundary_after_offset(1, 0))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, -1);
                }
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row, cur_loc.col + 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 1);
                }
            }
        }
        else if(step.row == 1 && step.col == -1) // Condition for Diagonal 3
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 0);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(0, -1);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, -1);

            if(is_within_boundary_after_offset(-1, 0))
            {
                if(graph_(cur_loc.row - 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, -1);
                }
            }
            if(is_within_boundary_after_offset(0, 1))
            {
                if(graph_(cur_loc.row, cur_loc.col + 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 1);
                }
            }
        }
        else if(step.row == -1 && step.col == 1) // Condition for Diagonal 4
        {
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 0);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(0, 1);
            add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, 1);

            if(is_within_boundary_after_offset(0, -1))
            {
                if(graph_(cur_loc.row, cur_loc.col - 1) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(-1, -1);
                }
            }
            if(is_within_boundary_after_offset(1, 0))
            {
                if(graph_(cur_loc.row + 1, cur_loc.col) == 1)
                {
                    add_neighbor_direction_pair_to_pruned_neighbors_if_within_boundary(1, 1);
                }
            }
        }
        return pruned_neighbors;
    }

    /// Backtrack from the start to the end and add the intermediate points to the path vector
    /// @param backtrace_start
    /// @param backtrace_end
    /// @param path
    void linear_backtrace(const location_2d& backtrace_start,
            const location_2d& backtrace_end, std::vector<location_2d>* path) const
    {
        assert(is_linear_backtrace_possible(backtrace_start, backtrace_end));
        const location_2d step = [&](){
            const auto diff = backtrace_start - backtrace_end;
            auto get_single_dimension_direction = [&](int x){
                if(x < 0) return -1;
                else if(x > 0) return 1;
                else return 0;
            };
            return location_2d(get_single_dimension_direction(diff.row),
                               get_single_dimension_direction(diff.col));
        }();
        location_2d current_node = backtrace_start;
        while(current_node != backtrace_end)
        {
            current_node = current_node - step;
            assert(is_within_boundary(current_node, n_rows, n_cols) &&
            "Adding the new step puts the location out of range");
            path->emplace_back(current_node);
        }
    }

    /// Debug function to check if linear backtracking is possible between two locations on the grid map
    /// @param backtrace_start
    /// @param backtrace_end
    /// @return
    bool is_linear_backtrace_possible(const location_2d& backtrace_start,
                                      const location_2d& backtrace_end) const
    {
        const auto diff = backtrace_start - backtrace_end;
        return !(diff.row != 0 && diff.col != 0 && diff.row != diff.col && diff.row != -diff.col);
    }
};

} // namespace mpl