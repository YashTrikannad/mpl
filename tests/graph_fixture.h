//
// Created by yash on 9/14/19.
//

#pragma once

#include "mpl/planner.h"

#include <gtest/gtest.h>

class GraphFixture : public ::testing::Test
{
protected:
    GraphFixture()
    {
        standard_graph_ <<  0, 1, 0,
                            1, 0, 1,
                            0, 1, 0;
    }

    Eigen::Matrix3d standard_graph_;
};
