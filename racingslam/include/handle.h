#pragma once

#include <ros/ros.h>

#include <string>

namespace racingslam {

    class Handle {

        public:

            Handle(std::string ns) { node_ = ros::NodeHandle(ns); }

            // load param from config file
            virtual void loadParam() = 0;
            // used to register Topics and services
            virtual void regTopics() = 0;

        protected:
            ros::NodeHandle node_;

    };

}

