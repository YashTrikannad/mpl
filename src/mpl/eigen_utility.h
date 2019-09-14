//
// Created by yash on 9/14/19.
//

#pragma once

#include <boost/functional/hash.hpp>
#include <Eigen/Dense>

namespace mpl
{

/// Used to represent the location of an element in the eigen matrix
struct location_2d
{
    location_2d() = default;
    location_2d(int row, int col) : row(row), col(col) {}
    int row;
    int col;

    friend std::size_t hash_value(location_2d const& location)
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, location.row);
        boost::hash_combine(seed, location.col);
        return seed;
    }

    friend std::ostream& operator<<(std::ostream& os, const location_2d& location)
    {
        os << "(" << location.row << ", " << location.col <<")";
    }
};

bool operator==(const location_2d& lhs, const location_2d& rhs)
{
    return (lhs.row == rhs.row) && (lhs.col == rhs.col);
}

bool operator!=(const location_2d& lhs, const location_2d& rhs)
{
    return lhs.row != rhs.row || lhs.col != rhs.col;
}

constexpr std::array<std::pair<int, int>, 8> neighbors_2d{{{-1, -1}, {-1, 0},{-1, 1},
                                                           {0, -1},            {0, 1},
                                                           {1, -1}, {1, 0}, {1, 1}}};

/// Gives Access to all the Adjacent Nodes of a point
/// @tparam Graph
/// @tparam Func
/// @param row
/// @param col
/// @param graph
/// @param func
template <typename Graph, typename Func>
void for_all_adjacent_nodes(const location_2d& loc, const Graph* graph, Func&& func)
{
    int rows = graph->rows();
    int cols = graph->cols();
    for(const auto& neighbor: neighbors_2d)
    {
        const auto new_row = loc.row + neighbor.first;
        const auto new_col = loc.col + neighbor.second;
        if(new_row < 0 || new_row > rows-1 || new_col < 0 || new_col > cols-1)
        {
            continue;
        }
        func(location_2d(new_row, new_col));
    }
}

}

namespace std
{
    template<> struct hash<mpl::location_2d> : boost::hash<mpl::location_2d> {};
}
