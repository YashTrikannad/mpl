//
// Created by yash on 9/19/19.
//

#include "mpl/jps.h"
#include "mpl/grid_map_generator/random_grid_map_generator.h"
#include "mpl/planner.h"
#include "mpl/visualization_utility.h"

static constexpr bool visualization = false;

int main()
{
    auto grid_map = mpl::generate_map(510, 500, 2);

    auto planner = mpl::planner<mpl::jps>(grid_map);

    const mpl::location_2d start(2, 2);
    const mpl::location_2d goal(500, 499);

    const auto plan = planner.get_plan(start, goal);

    if constexpr (visualization)
    {
        if(plan)
        {
            const auto visualization_info = planner.get_visualization_info();
            std::cout << "Path exist between Start and Goal";
            mpl::display(grid_map, visualization_info);
            return 0;
        }
    }

    if(plan)
    {
        std::cout << "Path exist between Start and Goal";
        mpl::display(grid_map, *plan);
    }
}