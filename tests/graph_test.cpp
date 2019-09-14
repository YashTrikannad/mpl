//
// Created by yash on 9/14/19.
//

#include "graph_fixture.h"

#include <gtest/gtest.h>

TEST_F(GraphFixture, check_node_adjacency)
{
    auto create_adjacent_node_vectors = [&](mpl::location_2d&& location, auto graph_ptr)
    {
        std::vector<int> actual_rows_indices;
        std::vector<int> actual_cols_indices;
        mpl::for_all_adjacent_nodes(location, graph_ptr, [&](const auto adjacent_node){
            actual_rows_indices.emplace_back(adjacent_node.row);
            actual_cols_indices.emplace_back(adjacent_node.col);
        });
        return std::pair{actual_rows_indices, actual_cols_indices};
    };

    std::vector<int> ground_truth_row_indices = {0, 1, 1};
    std::vector<int> ground_truth_cols_indices = {1, 0, 1};
    auto actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(0, 0), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {0, 0, 1, 1, 1};
    ground_truth_cols_indices = {0, 2, 0, 1, 2};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(0, 1), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {0, 1, 1};
    ground_truth_cols_indices = {1, 1, 2};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(0, 2), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {0, 0, 1, 2, 2};
    ground_truth_cols_indices = {0, 1, 1, 0, 1};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(1, 0), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {0, 0, 0, 1, 1, 2, 2, 2};
    ground_truth_cols_indices = {0, 1, 2, 0, 2, 0, 1, 2};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(1, 1), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {0, 0, 1, 2, 2};
    ground_truth_cols_indices = {1, 2, 1, 1, 2};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(1, 2), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {1, 1, 2};
    ground_truth_cols_indices = {0 , 1, 1};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(2, 0), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {1, 1, 1, 2, 2};
    ground_truth_cols_indices = {0, 1, 2, 0, 2};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(2, 1), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);

    ground_truth_row_indices = {1, 1, 2};
    ground_truth_cols_indices = {1, 2, 1};
    actual_node_indices_pair = create_adjacent_node_vectors(mpl::location_2d(2, 2), &standard_graph_);
    EXPECT_EQ(ground_truth_row_indices, actual_node_indices_pair.first);
    EXPECT_EQ(ground_truth_cols_indices, actual_node_indices_pair.second);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

