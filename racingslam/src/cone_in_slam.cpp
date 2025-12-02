#include "cone_in_slam.h"

using namespace racingslam;

ConeInSlam::ConeInSlam(fsd_common_msgs::Cone cone)
{
    cone_ = cone;
    cone_time_  = 0;
    update_count_ = 0;
    r_count_ = 0;
    b_count_ = 0;
}


void ConeInSlam::update(fsd_common_msgs::Cone cone) 
{
    if(cone.color.data == "r") r_count_++;
    if(cone.color.data == "b") b_count_++;
    
    if(update_count_ == 0)
    {
        cone_.position           =    cone.position;
        if(r_count_ > b_count_) cone_.color.data = "r";
        if(r_count_ < b_count_) cone_.color.data = "b";
        if(r_count_ == b_count_) cone_.color.data == "unknow";
        //cone_.color              =    cone.color;
        cone_.poseConfidence     =    cone.poseConfidence;
        cone_.colorConfidence    =    cone.colorConfidence;
    }
    else
    {
        cone_.position.x         =     (cone.position.x + cone_.position.x) / 2;
        cone_.position.y         =     (cone.position.y + cone_.position.y) / 2;
        if(r_count_ > b_count_) cone_.color.data = "r";
        if(r_count_ < b_count_) cone_.color.data = "b";
        if(r_count_ == b_count_) cone_.color.data == "unknow";
        cone_.poseConfidence     =     cone.poseConfidence;
        cone_.colorConfidence    =     cone.colorConfidence;
    }

    update_count_++;
}