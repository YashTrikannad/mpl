//
// Created by yash on 9/14/19.
//

#include "mpl/grid_map_generator/random_grid_map_generator.h"
#include "mpl/visualization_utility.h"

int main()
{
    const auto grid_map = mpl::generate_map(400, 400, 10);

    mpl::display(grid_map);
}

