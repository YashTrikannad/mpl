//
// Created by yash on 9/14/19.
//

#pragma once

#include "mpl/astar.h"
#include "mpl/jps.h"
#include "mpl/planner.h"

#include <gtest/gtest.h>

class GraphFixture : public ::testing::Test
{
protected:
    GraphFixture()
    {
        search_graph_ <<  0, 0, 0, 1 , 0,
                0, 0, 1, 1 , 0,
                0, 0, 0, 1 , 1,
                0, 0, 1, 1 , 0,
                1, 0, 0, 0 , 0;
    }

    Eigen::Matrix<double , 5, 5> search_graph_;
};

class AstarPlannerFixture : public GraphFixture
{
protected:
    AstarPlannerFixture() : GraphFixture(), planner_(search_graph_)
    {
    }

    mpl::planner<mpl::astar> planner_;
};


class JPSGraphFixture : public ::testing::Test
{
protected:
    JPSGraphFixture() :
            empty_graph_(Eigen::MatrixXd::Zero(10, 10))
    {
        single_obstacle_graph_up_ <<  0, 1, 0,
                                    0, 0, 0,
                                    0, 0, 0;
        single_obstacle_graph_right_ <<  0, 0, 0,
                                        0, 0, 1,
                                        0, 0, 0;
        single_obstacle_graph_down_ <<  0, 0, 0,
                                        0, 0, 0,
                                        0, 1, 0;
        single_obstacle_graph_left_ <<  0, 0, 0,
                                        1, 0, 0,
                                        0, 0, 0;
        single_obstacle_graph_top_left_ <<  1, 0, 0,
                                            0, 0, 0,
                                            0, 0, 0;
        single_obstacle_graph_top_right_ <<  0, 0, 1,
                                            0, 0, 0,
                                            0, 0, 0;
        single_obstacle_graph_bottom_left_ <<  0, 0, 0,
                                                0, 0, 0,
                                                1, 0, 0;
        single_obstacle_graph_bottom_right_ <<  0, 0, 0,
                                                0, 0, 0,
                                                0, 0, 1;
    }

    Eigen::MatrixXd empty_graph_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_up_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_right_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_down_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_left_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_top_left_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_top_right_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_bottom_left_;
    Eigen::Matrix<double , 3, 3> single_obstacle_graph_bottom_right_;
};

class JPSPlannerFixture : public JPSGraphFixture
{
protected:
    JPSPlannerFixture() : JPSGraphFixture(),
                          empty_planner_(this->empty_graph_),
                          single_obstacle_planner_up_(this->single_obstacle_graph_up_),
                          single_obstacle_planner_right_(this->single_obstacle_graph_right_),
                          single_obstacle_planner_down_(this->single_obstacle_graph_down_),
                          single_obstacle_planner_left_(this->single_obstacle_graph_left_),
                          single_obstacle_planner_top_left_(this->single_obstacle_graph_top_left_),
                          single_obstacle_planner_top_right_(this->single_obstacle_graph_top_right_),
                          single_obstacle_planner_bottom_left_(this->single_obstacle_graph_bottom_left_),
                          single_obstacle_planner_bottom_right_(this->single_obstacle_graph_bottom_right_)
    {}

    mpl::planner<mpl::jps> empty_planner_;
    mpl::planner<mpl::jps> single_obstacle_planner_up_;
    mpl::planner<mpl::jps> single_obstacle_planner_right_;
    mpl::planner<mpl::jps> single_obstacle_planner_down_;
    mpl::planner<mpl::jps> single_obstacle_planner_left_;
    mpl::planner<mpl::jps> single_obstacle_planner_top_left_;
    mpl::planner<mpl::jps> single_obstacle_planner_top_right_;
    mpl::planner<mpl::jps> single_obstacle_planner_bottom_left_;
    mpl::planner<mpl::jps> single_obstacle_planner_bottom_right_;
};
