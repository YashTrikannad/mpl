//
// Created by yash on 9/14/19.
//

#pragma once

#include "mpl/astar.h"
#include "mpl/planner.h"

#include <gtest/gtest.h>

class PlannerFixture : public ::testing::Test
{
protected:
    PlannerFixture()
    {
        search_graph_ <<  0, 0, 0, 1 , 0,
                        0, 0, 1, 1 , 0,
                        0, 0, 0, 1 , 1,
                        0, 0, 1, 1 , 0,
                        1, 0, 0, 0 , 0;
    }

    Eigen::Matrix<double , 5, 5> search_graph_;
};

class AstarPlannerFixture : public PlannerFixture
{
protected:
    AstarPlannerFixture() : PlannerFixture(),
                            planner_(this->search_graph_)
    {}

    mpl::planner<mpl::astar> planner_;
};
