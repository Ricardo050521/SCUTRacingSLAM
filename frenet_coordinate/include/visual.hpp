#ifndef VISUAL_HPP
#define VISUAL_HPP

#include "ros/ros.h"
#include "basic_struct.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace ns_fsd
{
    void visual(std::map<int, PathPoint> mid_set, SearchTree Path, PathStruct BestPath,
                visualization_msgs::Marker &visualTriangles,
                visualization_msgs::MarkerArray &visualTree,
                visualization_msgs::MarkerArray &visualBoundary,
                visualization_msgs::Marker &visualPath);
    // void visual(std::map<int, PathPoint> mid_set, SearchTree path, PathStruct BestPath,
    //             visualization_msgs::Marker &visualTriangles,
    //             visualization_msgs::MarkerArray &visualTree,
    //             visualization_msgs::MarkerArray &visualBoundary,
    //             visualization_msgs::Marker &visualPath,
    //             double x_car, double y_car, double phi);
} // namespace ns_fsd

#endif // visual.hpp