//
// Created by yash on 9/14/19.
//

#pragma once

#include <Eigen/Dense>
#include <random>
#include <iostream>

namespace mpl
{

Eigen::MatrixXd generate_map(size_t rows, size_t cols, size_t n_obstacles)
{
    Eigen::MatrixXd grid_map{Eigen::MatrixXd::Zero(rows, cols)};

    std::random_device r;
    std::default_random_engine engine(r());
    std::uniform_int_distribution<int> random_row(1, rows);
    std::uniform_int_distribution<int> random_column(1, cols);
    std::uniform_int_distribution<int> random_height(1, rows / 3);
    std::uniform_int_distribution<int> random_width(1, cols / 3);

    while (n_obstacles > 0)
    {
        const auto upper_left_obstacle_row = random_row(engine);
        const auto upper_left_obstacle_col = random_column(engine);
        const auto obstacle_height = random_height(engine);
        const auto obstacle_width = random_width(engine);

        for (size_t row_index = upper_left_obstacle_row, height = 0;
             row_index < rows && height < obstacle_height;
             row_index++, height++)
            for (size_t col_index = upper_left_obstacle_col, width = 0; col_index < cols &&
                                                                        width <
                                                                        obstacle_width; col_index++, width++)
            {
                grid_map(row_index, col_index) = 1;
            }

        n_obstacles--;
    }

    return grid_map;
}


}