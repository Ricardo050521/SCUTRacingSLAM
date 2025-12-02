#pragma once

#include "fsd_common_msgs/Cone.h"

#include <geometry_msgs/Point.h>

namespace racingslam
{
    /**
     * @brief 用于记录建图过程中的椎桶信息，包含椎桶id以及椎桶更新次数，以及得到椎桶各个信息的接口
    */
    class ConeInSlam 
    {
    public:
        ConeInSlam(fsd_common_msgs::Cone cone);
        void update(fsd_common_msgs::Cone cone);

        int getUpdateCount() {return update_count_;}
        geometry_msgs::Point getPosition() const {return cone_.position;}
        fsd_common_msgs::Cone getCone() {return cone_;}
        void setPosition(double x, double y) {cone_.position.x = x; cone_.position.y = y;}
        void setUpdateCount(int num){update_count_ = num;}
        void setTime(double time){cone_time_ = time;};
        double getTime(){return cone_time_;};

    private:
        fsd_common_msgs::Cone cone_;
        int update_count_;
        int r_count_;
        int b_count_;
        double cone_time_;

    };
}