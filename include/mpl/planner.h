//
// Created by yash on 9/14/19.
//

#pragma once

#include "mpl/common_utility.h"
#include "mpl/eigen_utility.h"
#include "mpl/visualization_utility.h"

#include <Eigen/Dense>
#include <iostream>

namespace mpl
{

/// Use this flag to show visualization info
/// You need to call
constexpr bool mpl_visualization = false;

/// Planner Interface
/// @tparam PlannerType
template <typename PlannerType>
class planner : public crtp<PlannerType>
{
public:

    explicit planner(Eigen::MatrixXd graph) : graph_(std::move(graph)),
                                              n_rows(graph_.rows()),
                                              n_cols(graph_.cols())
    {
    }

    /// Get the path from start to goal (Optimality depends on the PlannerType)
    /// @param start
    /// @param goal
    /// @return std::vector<location_2d> if a path is found or std::null_opt
    std::optional<std::vector<location_2d>> get_plan(const location_2d& start, const location_2d& goal)
    {
        if constexpr (!mpl_visualization)
        {
            return this->underlying().get_plan(start, goal);
        }
        else
        {
            return this->underlying().get_plan_with_visualization(start, goal);
        }
    }

    std::vector<viz_location_2d> get_visualization_info()
    {
        if constexpr (!mpl_visualization)
        {
            std::__throw_logic_error("Set mpl_visualization to true to call this function.");
        }
        else
        {
            if(explored_locations_.empty())
            {
                std::cout << "No Path available yet. Make sure you call get_plan.";
            }
            return explored_locations_;
        }
    }

    const Eigen::MatrixXd graph_;
    const int n_rows;
    const int n_cols;

    std::vector<viz_location_2d> explored_locations_;
};

} // namespace mpl
