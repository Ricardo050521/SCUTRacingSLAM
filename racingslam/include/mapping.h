#pragma once

#include "cone_in_slam.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ConeDetections.h"
#include "fsd_common_msgs/ResAndAmi.h"
#include "fsd_common_msgs/SkidpadGlobalCenterLine.h"
#include "racingslam/CarTrack.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

namespace racingslam
{
    class Mapping
    {
    public:
        Mapping(std::string ns);
        void regTopic();
        void loadParam();

        std::vector<std::pair<ConeInSlam, double>> getMapWithTime();

        fsd_common_msgs::ConeDetections getMap();

        fsd_common_msgs::Map getMapWithColor();

        bool map_load_switch_;  //是否使用地图存储

        void startSlam();

    private:

        //sub & pub
        ros::NodeHandle node_;
        ros::Subscriber cone_sub_;
        ros::Subscriber state_sub_;
        ros::Subscriber track_sub_;
        ros::Subscriber radar_sub_;
        ros::Subscriber res_sub_;
        ros::Publisher map_pub_;
        ros::Publisher realtime_map_visualization_pub_;
        ros::Publisher global_gps_line_pub_;
        ros::Publisher gps_line_pub_;
        ros::Publisher map_visualization_pub_;
        ros::Publisher track_visualization_pub_;
        ros::Publisher current_visual_cone_detections_pub_;
        ros::Publisher realtime_map_pub_;
        ros::Publisher fix_acc_track_pub_;
        ros::Publisher skid_path_pub_;


        //以下参数会随着程序的运行发生改变，为中间量
        // std::vector<ConeInSlam> map_buffer_;
        std::vector<fsd_common_msgs::ConeDetections> cone_buffer_;
        fsd_common_msgs::CarState current_state_;
        fsd_common_msgs::CarState last_state_;
        fsd_common_msgs::CarState init_state_;
        racingslam::CarTrack track_;
        nav_msgs::Path gps_line_;
        nav_msgs::Path fix_acc_track_; //修复后的直线加速轨迹
        fsd_common_msgs::SkidpadGlobalCenterLine skid_path_; //八字绕环轨迹
        double odom_;
        double delta_dis_;
        double loop_detection_dis_;
        double track_fix_start_time_;
        double skid_start_time_;
        double fit_acc_track_length_; //修复的直线加速的轨迹长度
        int loop_detection_timer_; //为-1时，回环检测未启动 为0时，回环检测启动 为1时，回环检测完成
        bool is_state_init_; //用于回环检测
        bool is_track_line_init_; //用于记录轨迹
        bool is_loop_closed_;
        bool is_acc_end_;
        bool is_skid_start_;
        bool is_skid_end_;
        bool is_map_saved_;
        bool is_map_load_;
        bool is_track_fix_; //判断是否补全轨迹空缺
        bool is_acc_track_fit;//判断是否拟合直线加速轨迹
        bool is_skid_dealt;
        int cone_run_count_; //用于记录椎桶回调函数运行次数，每五次运行一次椎桶过滤器
        nav_msgs::Path track_line_;
        int res_state_; //用于检测res是否给出go信号

        bool is_center_load_;
        bool center_load_switch_;  // 中心线加载开关
        
        //以下参数是人为设置的参数，不会随着程序的运行而改变
        double reg_range_;
        int min_update_count_;
        double max_reg_dis_;
        double min_cone_distance_;
        double lidar2cog_length_; //雷达到质心的长度 A10为2.4
        double acc_distance_;
        double dis_to_earse_;
        int loop_detection_timer_count_;
        std::string cone_sub_topic_;
        std::string state_sub_topic_;
        std::string track_sub_topic_;
        std::string res_sub_topic_;
        std::string map_pub_topic_;
        std::string gps_line_pub_topic_;
        std::string global_gps_line_pub_topic_;
        std::string map_visualization_pub_topic_;
        std::string track_visualization_pub_topic_;
        std::string realtime_map_pub_topic_;
        std::string current_visual_cone_detections_pub_topic_;
        std::string fix_acc_track_pub_topic_;
        std::string skid_path_pub_topic_;

        std::string realtime_map_visualization_pub_topic;

        std::string frame_id_;
        std::string map_save_path_;
        std::string running_mode_; //赛项 acc为直线加速，track为循迹 会影响保存的地图
        bool odom_track_switch_;//是否使用里程计轨迹

        ros::Time latest_radar_stamp_; // 存储最新的雷达时间戳
        void radarCallback(const fsd_common_msgs::ConeDetections::ConstPtr& radar_msg);
        double dis_to_erase_;   // 锥桶到轨迹的最大允许距离（超过则过滤）
        void filterOutlierCones();
        void sendRealtimeMapVisualization(const fsd_common_msgs::ConeDetections& realtime_map, std::string frame_id);

        std::vector<geometry_msgs::Point> cubicSplineInterpolation(const std::vector<double>& x, const std::vector<double>& y, int num_points);

        // std::vector<std::pair<double, fsd_common_msgs::CarState>> cone_time_2pose_map_; // 全局映射表：存储「锥桶帧时间戳 → 对应匹配位姿pose_A」
        std::map<double, fsd_common_msgs::CarState> cone_time_2pose_map_;

        std::vector<std::pair<ConeInSlam, double>> cone_2time_map_;


        double time_match_tolerance_;   // 时间戳匹配允许的最大误差
        int max_cache_frames_;  // 映射表最大缓存帧数 
        bool findMatchedPose(double target_time, fsd_common_msgs::CarState& out_pose);

        fsd_common_msgs::ConeDetections translateConeState(const fsd_common_msgs::ConeDetections& original_cones, fsd_common_msgs::CarState& car_state);


        fsd_common_msgs::ConeDetections translate2Local(const std::vector<std::pair<ConeInSlam, double>>& original_cones_with_time, fsd_common_msgs::CarState& car_state);
        
        void coneFilter();
        bool dbscan(ConeInSlam cone); //根据距离剔除离群椎桶
        nav_msgs::Path fitLine(const nav_msgs::Path& path_to_fit, double fit_length); //使用最小二乘法拟合直线
        fsd_common_msgs::SkidpadGlobalCenterLine dealSkid(); //处理八字绕环最后一段线，防止控制规划线丢失
        void coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg);
        void stateCallback(const fsd_common_msgs::CarState &msg);
        void trackCallback(const racingslam::CarTrack::ConstPtr& msg);
        void resCallback(const fsd_common_msgs::ResAndAmi& msg);
        void regCones();
        void loopDetect(); //循迹回环检测
        double getDeltaDis();
        void sendMapVisualization(std::string frame_id);
        //void sendCarTrackVisualization(std::string frame_id, fsd_common_msgs::CarState car_state);
        void sendVisualCurrentConeDetections(std::string frame_id, fsd_common_msgs::ConeDetections cones);
        void saveMap();
        void loadMap();
        nav_msgs::Path carstateTrack2Path();

        //私有方法
        double dis(const fsd_common_msgs::Cone a){return sqrt(a.position.x*a.position.x + a.position.y*a.position.y);}
        double dis(const fsd_common_msgs::Cone a, const fsd_common_msgs::Cone b)
        {
            return sqrt((a.position.x - b.position.x)*(a.position.x - b.position.x) + (a.position.y - b.position.y)*(a.position.y - b.position.y));
        }
        double dis(const ConeInSlam& a, const ConeInSlam& b)
        {
            geometry_msgs::Point a_pos = a.getPosition();
            geometry_msgs::Point b_pos = b.getPosition();

            return sqrt((a_pos.x-b_pos.x)*(a_pos.x-b_pos.x) + (a_pos.y-b_pos.y)*(a_pos.y-b_pos.y));
        }
        double dis(const fsd_common_msgs::CarState a)
        {
            return sqrt((a.car_state.x - init_state_.car_state.x)*(a.car_state.x - init_state_.car_state.x) + (a.car_state.y - init_state_.car_state.y)*(a.car_state.y - init_state_.car_state.y));
        }
    };
}
