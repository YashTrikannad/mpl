//
// Created by yash on 9/14/19.
//

#include <mpl/visualization_utility.h>
#include "mpl/dfs.h"
#include "mpl/planner.h"
#include "mpl/grid_map_generator/random_grid_map_generator.h"

int main()
{
    auto grid_map = mpl::generate_map(400, 400, 10);

    const auto planner = mpl::planner<mpl::dfs>(grid_map);

    const mpl::location_2d start(0, 0);
    const mpl::location_2d goal(100, 300);

    const auto plan = planner.get_plan(start, goal);

    if(plan)
    {
        mpl::display(grid_map, *plan);
    }
}


