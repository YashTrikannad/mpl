//
// Created by yash on 9/14/19.
//

#include "planner_fixture.h"

#include <gtest/gtest.h>

TEST_F(AstarPlannerFixture, invalid_endpoints)
{
    auto plan = planner_.get_plan(mpl::location_2d(0, 0), mpl::location_2d(3, 3));
    if(!plan) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);

    plan = planner_.get_plan(mpl::location_2d(3, 3), mpl::location_2d(0, 0));
    if(!plan) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);

    plan = planner_.get_plan(mpl::location_2d(3, 3), mpl::location_2d(0, 3));
    if(!plan) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}

TEST_F(AstarPlannerFixture, shortest_path_size)
{
    auto plan = planner_.get_plan(mpl::location_2d(0, 0), mpl::location_2d(4, 4));
    if(!plan) EXPECT_TRUE(false);
    else EXPECT_EQ(plan->size(), 7);

    plan = planner_.get_plan(mpl::location_2d(2, 2), mpl::location_2d(4, 4));
    if(!plan) EXPECT_TRUE(false);
    else EXPECT_EQ(plan->size(), 5);

    plan = planner_.get_plan(mpl::location_2d(1, 0), mpl::location_2d(4, 3));
    if(!plan) EXPECT_TRUE(false);
    else EXPECT_EQ(plan->size(), 5);
}

TEST_F(AstarPlannerFixture, destination_unreachable)
{
    auto plan = planner_.get_plan(mpl::location_2d(0, 4), mpl::location_2d(4, 4));
    if(!plan) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);

    plan = planner_.get_plan(mpl::location_2d(2, 2), mpl::location_2d(1, 4));
    if(!plan) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}