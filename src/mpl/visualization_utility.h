//
// Created by yash on 9/14/19.
//

#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


namespace mpl
{

/// Display Eigen Graph
/// @tparam Graph
/// @param graph
template <typename Graph>
void display(const Graph& graph)
{
    using namespace cv;
    Mat image;
    eigen2cv(graph, image);
    imshow( "Current ContainerType", image );
    waitKey(0);
}

///
/// @tparam Path
/// @param graph
/// @param path
template <typename Graph, typename Path>
void display(Graph& graph, const Path& path)
{
    using namespace cv;
    Mat image;
    for(const auto& node:path)
    {
        graph(node.row, node.col) = 0.5;
    }

    eigen2cv(graph, image);

    imshow( "Current ContainerType", image );
    waitKey(0);
}

}
