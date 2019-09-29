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

    bool is_within_boundary(int max_rows, int max_cols) const
    {
        return !(row < 0 || row > max_rows - 1 || col < 0 || col > max_cols - 1);
    }

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

inline bool operator==(const location_2d& lhs, const location_2d& rhs)
{
    return (lhs.row == rhs.row) && (lhs.col == rhs.col);
}

inline bool operator!=(const location_2d& lhs, const location_2d& rhs)
{
    return lhs.row != rhs.row || lhs.col != rhs.col;
}

inline bool operator==(const std::pair<location_2d, location_2d>& lhs, const std::pair<location_2d, location_2d>& rhs)
{
    return (lhs.first == rhs.first) && (lhs.second == rhs.second);
}

inline location_2d operator+(location_2d const& location1, location_2d const& location2)
{
    return location_2d(location1.row + location2.row, location1.col + location2.col);
}

inline location_2d operator-(location_2d const& location1, location_2d const& location2)
{
    return location_2d(location1.row - location2.row, location1.col - location2.col);
}

inline bool is_within_boundary(const location_2d& location, const int& rows, const int& cols)
{
    return !(location.row < 0 || location.row > rows - 1 || location.col < 0 || location.col > cols - 1);
}

constexpr std::array<std::pair<int, int>, 8> neighbors_2d{{ {-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1},{-1, 1},
                                                           {1, -1},  {1, 1}}};

/// Gives Access to all the Adjacent Nodes of a point (Does Boundary Condition Checking)
/// @tparam Graph
/// @tparam Func
/// @param row
/// @param col
/// @param graph
/// @param func
template <typename Graph, typename Func>
void for_all_adjacent_nodes(const location_2d& loc, const Graph& graph, Func&& func)
{
    for(const auto& neighbor: neighbors_2d)
    {
        const auto new_row = loc.row + neighbor.first;
        const auto new_col = loc.col + neighbor.second;
        if(new_row < 0 || new_row > graph.rows()-1 || new_col < 0 || new_col > graph.cols()-1 || graph(new_row, new_col) == 1)
        {
            continue;
        }
        func(location_2d(new_row, new_col));
    }
}

/// Get L2 distance
/// @param start - location_2d*
/// @param goal - location_2d*
/// @return l2 distance (double)
inline double get_distance(const location_2d* start, const location_2d* goal)
{
    return sqrt(pow(start->row - goal->row, 2) + pow(start->col - goal->col, 2));
}

} // namespace mpl

/// hash for location_2d
namespace std
{
    template<> struct hash<mpl::location_2d> : boost::hash<mpl::location_2d> {};
}
