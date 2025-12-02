#include <ros/ros.h>
#include "frenet_coordinate_handle.hpp"

typedef ns_frenet_coordinate::FrenetCoordinateHandle FrenetCoordinateHandle;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "frenet_coordinate");
    ros::NodeHandle nodeHandle("~");
    FrenetCoordinateHandle myFrenetCoordinateHandle(nodeHandle);

    if(!myFrenetCoordinateHandle.is_start_switch_)
    {
        ROS_INFO("============Did not start frenet!!!===========");
        ros::shutdown();
        return 0;
    }
    
    // 默认为40Hz，config可调
    ros::Rate loop_rate(myFrenetCoordinateHandle.getNodeRate());
    while (ros::ok())
    {

        myFrenetCoordinateHandle.run();

        ros::spinOnce();   // Keeps node alive basically
        loop_rate.sleep(); // Sleep for loop_rate
    }
    return 0;
}
