#include "mainhandle.h"

using namespace racingslam;

MainHandle::MainHandle() : Handle("~") {}



void MainHandle::trackCallBack(const racingslam::CarTrack::ConstPtr& msg) {

    track_.header = msg->header;
    track_.track = msg->track;

}