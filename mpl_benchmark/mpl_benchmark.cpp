#include "mpl/planner.h"
#include "mpl/jps.h"
#include "mpl/astar.h"
#include "mpl/grid_map_generator/random_grid_map_generator.h"

#include <iostream>
#include <chrono>
#include <random>

static constexpr int n_iterations = 100;
static constexpr int dim_x = 2000;
static constexpr int dim_y = 2000;
static constexpr int n_obstacles = 4;

static constexpr bool multi_map = false;
static constexpr int n_map = 1;

int main()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> x_index_gen(0,dim_x-1);
    std::uniform_int_distribution<std::mt19937::result_type> y_index_gen(0,dim_y-1);

    int astar_iters = 0;
    int jps_iters = 0;
    std::chrono::milliseconds astar_time;
    std::chrono::milliseconds jps_time;

    std::vector<mpl::location_2d> start_indices;
    for(int i=0; i<1000; ++i)
    {
        start_indices.emplace_back(mpl::location_2d{x_index_gen(rng), y_index_gen(rng)});
    }

    std::vector<mpl::location_2d> end_indices;
    for(int i=0; i<1000; ++i)
    {
        end_indices.emplace_back(mpl::location_2d{x_index_gen(rng), y_index_gen(rng)});
    }

    if constexpr (!multi_map)
    {
        const auto grid_map = mpl::generate_map(dim_x, dim_y, n_obstacles);

        for(int i=0; i<n_iterations; i++)
        {
            auto planner1 = mpl::planner<mpl::astar>(grid_map);

            auto start = std::chrono::system_clock::now();
            const auto plan1 = planner1.get_plan(start_indices[i], end_indices[i]);
            auto end = std::chrono::system_clock::now();

            if(plan1 && plan1->size() > 50)
            {
                astar_time = astar_time + std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
                astar_iters++;
            }

            auto planner2 = mpl::planner<mpl::jps>(grid_map);

            start = std::chrono::system_clock::now();
            const auto plan2 = planner2.get_plan(start_indices[i], end_indices[i]);
            end = std::chrono::system_clock::now();

            if(plan2 && plan2->size() > 50)
            {
                jps_time = jps_time + std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
                jps_iters++;
            }
        }

        const std::chrono::milliseconds average_astar_time = astar_time/astar_iters;
        const std::chrono::milliseconds average_jps_time = jps_time/jps_iters;

        std::cout << "Average Time for A-Star in ms: "  << average_astar_time.count() << "\n";
        std::cout << "Average Time for JPS in ms: " << average_jps_time.count() << "\n";
    }

}