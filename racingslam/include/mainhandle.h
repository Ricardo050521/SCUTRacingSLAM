#pragma once

#include "handle.h"
#include "racingslam/CarTrack.h"

#include <ros/ros.h>

namespace racingslam {

    class MainHandle : public Handle {

        public:

            MainHandle();
            void trackCallBack(const racingslam::CarTrack::ConstPtr& msg);

        private:

            racingslam::CarTrack track_;

    };

}

