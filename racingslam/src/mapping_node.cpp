#include "mapping.h"

using namespace racingslam;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"mapping");

    Mapping handle = Mapping("");

    handle.loadParam();
    handle.regTopic();

    int rate;
    if(handle.map_load_switch_ == false) rate = 100; //贯导发送消息最高频率为100
    if(handle.map_load_switch_ == true ) rate = 1;
    ros::Rate loop_rate(rate);

    while(ros::ok())
    {
        if(handle.map_load_switch_ == false) ros::spinOnce();
        handle.startSlam();

        loop_rate.sleep();
    }

    return 0;

}