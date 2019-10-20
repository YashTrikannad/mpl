//
// Created by yash on 9/14/19.
//

#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>


namespace mpl
{

/// Used for visualization (mpl_visualization must be set to true)
/// General Color Terminology
/// Explored Nodes: "blue"
/// Explored Nodes but not added to Open Set: "red"
/// Start: "green"
struct viz_location_2d : public location_2d
{
    viz_location_2d() = delete;
    viz_location_2d(const location_2d& loc, int r, int g, int b): location_2d(loc), r(r), g(g), b(b)
    {}
    viz_location_2d(const location_2d& loc, const std::string& color) : location_2d(loc)
    {
        if(color == "red")
        {
            r = 255;
            b = 0;
            g = 0;
        }
        else if(color == "blue")
        {
            r = 0;
            b = 255;
            g = 0;
        }
        else if(color == "green")
        {
            r = 0;
            b = 0;
            g = 255;
        }
        else if(color == "yellow")
        {
            r = 255;
            b = 0;
            g = 255;
        }
        else
        {
            std::__throw_invalid_argument("Color not available: Use red, green, blue, or pass r, g, b values");
        }
    }
    int r;
    int g;
    int b;
};


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


/// Displays only 2d visual information
/// @tparam Path
/// @param graph
/// @param path
template <typename Graph>
void display(Graph& graph, const std::vector<viz_location_2d>& visualization_info)
{
    for(int i=0; i<graph.rows(); i++)
    {
        for(int j=0; j<graph.cols(); ++j)
        {
            if(graph(i, j) == 1)
            {
                graph(i, j) = 255;
            }
        }
    }

    cv::Mat image;
    eigen2cv(graph, image);

    std::cout << "Mat type = " << image.type();
    cv::Mat image32;
    image.cv::Mat::convertTo(image32, CV_8UC1);

    cv::Mat color(image32.rows,image32.cols, CV_8UC3, cv::Scalar(0,0,0));

    cv::cvtColor(image32, color, CV_GRAY2BGR);

    for(const auto& viz_node:visualization_info)
    {
        if(viz_node.b == 0 && viz_node.g == 255 && viz_node.r == 255)
        {
            if(viz_node.row > 0 && viz_node.row<color.rows && viz_node.col > 0 && viz_node.col<color.cols)
            {
                color.at<cv::Vec3b>(viz_node.row+1, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row-1, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row, viz_node.col+1) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row, viz_node.col-1) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
            }
        }
        if(viz_node.b == 0 && viz_node.g == 255 && viz_node.r == 0)
        {
            if(viz_node.row > 0 && viz_node.row<color.rows && viz_node.col > 0 && viz_node.col<color.cols)
            {
                color.at<cv::Vec3b>(viz_node.row+1, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row-1, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row, viz_node.col+1) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row, viz_node.col-1) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
            }
        }
        if(viz_node.b == 0 && viz_node.g == 0 && viz_node.r == 255)
        {
            if(viz_node.row > 0 && viz_node.row<color.rows && viz_node.col > 0 && viz_node.col<color.cols)
            {
                color.at<cv::Vec3b>(viz_node.row+1, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row-1, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row, viz_node.col+1) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
                color.at<cv::Vec3b>(viz_node.row, viz_node.col-1) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
            }
        }
        else
        {
            color.at<cv::Vec3b>(viz_node.row, viz_node.col) = cv::Vec3b(viz_node.b, viz_node.g, viz_node.r);
        }
    }

    imshow( "Current ContainerType", color );
    cv::waitKey(0);
}

}
